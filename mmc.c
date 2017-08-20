/* mmc.c: Emulation of the SD / MMC interface
   Copyright (c) 2017 Philip Kendall

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along
   with this program; if not, write to the Free Software Foundation, Inc.,
   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

   Author contact information:

   E-mail: Philip Kendall <philip-fuse@shadowmagic.org.uk>

*/

#include <config.h>

#include <stdio.h>
#include <string.h>

#include "internals.h"

#define IN_IDLE_STATE_MASK   0x01
#define ILLEGAL_COMMAND_MASK 0x04
#define PARAMETER_ERROR_MASK 0x40

/* The states while a command is being sent to the card */
enum command_state_t {
  WAITING_FOR_COMMAND,
  WAITING_FOR_DATA0,
  WAITING_FOR_DATA1,
  WAITING_FOR_DATA2,
  WAITING_FOR_DATA3,
  WAITING_FOR_CRC,
  WAITING_FOR_DATA_TOKEN,
  WAITING_FOR_DATA,
  WAITING_FOR_DATA_CRC1,
  WAITING_FOR_DATA_CRC2
};

/* The MMC commands we support */
enum command_byte {
  GO_IDLE_STATE = 0,
  SEND_IF_COND = 8,
  SEND_CSD = 9,
  SEND_CID = 10,
  READ_SINGLE_BLOCK = 17,
  WRITE_BLOCK = 24,
  APP_CMD = 55,
  READ_OCR = 58
};

/* Application specific commands, only some implemented
   but needed for command identification */
enum application_command_byte {
  SD_STATUS = 13,
  SEND_NUM_WR_BLOCKS = 22,
  SET_WR_BLK_ERASE_COUNT = 23,
  SD_SEND_OP_COND = 41,
  SET_CLR_CARD_DETECT = 42,
  SEND_SCR = 51,
};

typedef struct libspectrum_mmc_card {
  /* The actual "card" data */
  libspectrum_ide_drive drive;

  /* Cache of written sectors */
  GHashTable *cache;

  /* The C_SIZE field of the card CSD */
  libspectrum_word c_size;

  /* The number of sectors (512 bytes) */
  libspectrum_dword total_sectors;

  /* R1 status of current command */
  libspectrum_byte r1_status;

  /* The current state of the command being transmitted to the card */
  enum command_state_t command_state;

  /* The most recent command index sent to the MMC */
  libspectrum_byte current_command;

  /* The argument for the current MMC command */
  libspectrum_byte current_argument[4];

  /* How much data has been sent for the current command */
  size_t data_count;

  /* The data for the current command */
  libspectrum_byte send_buffer[512];

  /* The response to the most recent command */
  libspectrum_byte response_buffer[516];

  /* One past the last valid byte in response_buffer */
  libspectrum_byte *response_buffer_end;

  /* The next byte to be returned from response_buffer */
  libspectrum_byte *response_buffer_next;

  /* CMD8/SEND_IF_COND recieved while initialising */
  int cmd8_issued;

  /* CMD55/APP_CMD issued so next command is application specific */
  int cmd55_issued;

} libspectrum_mmc_card;

libspectrum_mmc_card*
libspectrum_mmc_alloc( void )
{
  libspectrum_mmc_card *card = libspectrum_new( libspectrum_mmc_card, 1 );

  card->drive.disk = NULL;
  card->cache = g_hash_table_new( g_int_hash, g_int_equal );

  libspectrum_mmc_reset( card );

  return card;
}

void
libspectrum_mmc_free( libspectrum_mmc_card *card )
{
  libspectrum_mmc_eject( card );

  g_hash_table_destroy( card->cache );

  libspectrum_free( card );
}

libspectrum_error
libspectrum_mmc_insert( libspectrum_mmc_card *card, const char *filename )
{
  libspectrum_error error;
  libspectrum_dword c_size;

  libspectrum_mmc_eject( card );
  if( !filename ) return LIBSPECTRUM_ERROR_NONE;

  error = libspectrum_ide_insert_into_drive( &card->drive, filename );
  if( error ) return error;

  card->total_sectors = (libspectrum_dword)card->drive.cylinders *
    card->drive.heads * card->drive.sectors;

  if( card->drive.sector_size != 512 || card->total_sectors % 1024 ) {
      libspectrum_print_error( LIBSPECTRUM_ERROR_UNKNOWN,
        "Image size not supported" );
    return LIBSPECTRUM_ERROR_UNKNOWN;
  }

  /* memory capacity = (C_SIZE+1) * 512K bytes
     we reduce the sector count by 1024. This gives us a minimum card size
     of 512 Kb. Not too worried about that. */
  c_size = ( card->total_sectors >> 10 ) - 1;

  /* We emulate a SDHC card which has a maximum size of (32 Gb - 80 Mb) */
  if( c_size >= 65375 ) {
    libspectrum_print_error( LIBSPECTRUM_ERROR_UNKNOWN,
      "Image size too big (>32 Gb)" );
    return LIBSPECTRUM_ERROR_UNKNOWN;
  }

  card->c_size = c_size;

  return LIBSPECTRUM_ERROR_NONE;
}

void
libspectrum_mmc_eject( libspectrum_mmc_card *card )
{
  libspectrum_ide_eject_from_drive( &card->drive, card->cache );
  libspectrum_mmc_reset( card );
}

void
libspectrum_mmc_reset( libspectrum_mmc_card *card )
{
  card->r1_status = IN_IDLE_STATE_MASK;
  card->command_state = WAITING_FOR_COMMAND;
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer;
  card->cmd8_issued = 0;
  card->cmd55_issued = 0;
}

int
libspectrum_mmc_dirty( libspectrum_mmc_card *card )
{
  return g_hash_table_size( card->cache ) != 0;
}

void
libspectrum_mmc_commit( libspectrum_mmc_card *card )
{
  libspectrum_ide_commit_drive( &card->drive, card->cache );
}

libspectrum_byte
libspectrum_mmc_read( libspectrum_mmc_card *card )
{
  libspectrum_byte r = card->response_buffer_next < card->response_buffer_end ?
    *(card->response_buffer_next)++ :
    0xff;

  return r;
}

static int
parse_command( libspectrum_mmc_card *card, libspectrum_byte b )
{
  /* All commands should have start bit == 0 and transmitter bit == 1 */
  if( ( b & 0xc0 ) != 0x40 ) return 0;  /* don't change current state */

  /* Unlike the SD Memory Card protocol, in the SPI mode, the card will always
     respond to a command. The response indicates acceptance or rejection of
     the command. Now wait for arguments and CRC */
  card->current_command = b & 0x3f;
  return 1;
}

static void
set_response_buffer_r1( libspectrum_mmc_card *card )
{
  card->response_buffer[ 0 ] = card->r1_status;
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + 1;
}

static void
set_response_buffer_r7( libspectrum_mmc_card *card, libspectrum_dword value )
{
  card->response_buffer[ 0 ] = card->r1_status;
  card->response_buffer[ 1 ] = ( value & 0xff000000 ) >> 24;
  card->response_buffer[ 2 ] = ( value & 0x00ff0000 ) >> 16;
  card->response_buffer[ 3 ] = ( value & 0x0000ff00 ) >>  8;
  card->response_buffer[ 4 ] = ( value & 0x000000ff );
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + 5;
}

static void
read_single_block( libspectrum_mmc_card *card )
{
  libspectrum_dword sector_number = 
    card->current_argument[ 3 ] +
    (card->current_argument[ 2 ] << 8) +
    (card->current_argument[ 1 ] << 16) +
    (card->current_argument[ 0 ] << 24);

  /* Sector out of range */
  if( sector_number >= card->total_sectors ) {
    card->r1_status |= PARAMETER_ERROR_MASK;
    set_response_buffer_r1( card );
    return;
  }

  int error = libspectrum_ide_read_sector_from_hdf(
      &card->drive, card->cache, sector_number, &card->response_buffer[ 2 ]
  );

  /* Data retrieval error */
  if( error ) {
    card->response_buffer[ 0 ] = card->r1_status; /* R1 command response */
    card->response_buffer[ 1 ] = 0x01;            /* data error token */
    card->response_buffer_next = card->response_buffer;
    card->response_buffer_end = card->response_buffer + 2;
    return;
  }

  card->response_buffer[ 0 ] = card->r1_status;
  card->response_buffer[ 1 ] = 0xfe;

  /* CRC */
  memset( &card->response_buffer[ 514 ], 0x00, 2 );

  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + 516;
}

static int
do_application_command( libspectrum_mmc_card *card )
{
  int is_acmd = 1;

  switch( card->current_command ) {
    case SD_STATUS:
    case SEND_NUM_WR_BLOCKS:
    case SET_WR_BLK_ERASE_COUNT:
    case SET_CLR_CARD_DETECT:
    case SEND_SCR:
      /* Application command not implemented, return illegal command for now */
      card->r1_status |= ILLEGAL_COMMAND_MASK;
      set_response_buffer_r1( card );

      libspectrum_print_error(
          LIBSPECTRUM_ERROR_UNKNOWN,
          "Unknown MMC application command %d received",
          card->current_command );
      break;

    case SD_SEND_OP_COND:
      /* SDHC cards return in_idle_state=1 when:
         1. CMD8 was not issued before ACMD41
         2. ACMD41 is issued with HCS=0 (host only supports SDSC)
      */
      if( card->cmd8_issued && ( card->current_argument[ 0 ] & 0x40 ) ) {
        card->r1_status &= ~IN_IDLE_STATE_MASK;
      }
      set_response_buffer_r1( card );
      break;

    default:
      is_acmd = 0;
      break;
  }

  return is_acmd;
}

static void
do_standard_command( libspectrum_mmc_card *card )
{
  switch( card->current_command ) {
    case GO_IDLE_STATE:
      card->r1_status = IN_IDLE_STATE_MASK;
      card->cmd8_issued = 0;
      set_response_buffer_r1( card );
      break;
    case SEND_IF_COND:
      card->cmd8_issued = 1;

      /* return echo back pattern */
      set_response_buffer_r7( card, 0x00000100 | card->current_argument[ 3 ] );
      break;
    case SEND_CSD:
      card->response_buffer[ 0 ] = card->r1_status; /* R1 command response */
      card->response_buffer[ 1 ] = 0xfe;            /* data token */

      memset( &card->response_buffer[ 2 ], 0x00, 16 );

      /* CSD_STRUCTURE version 2.0 */
      card->response_buffer[ 2 ] = 0x40;

      /* READ_BL_LEN = 9 => 2 ^ 9 = 512 byte sectors */
      card->response_buffer[ 2 +  5 ] = 0x09;

      /* C_SIZE (spread 6 bits, 8 bits, 8 bits across three bytes),
         first 6 bits set to zero */
      card->response_buffer[ 2 +  8 ] = ( card->c_size >> 8 ) & 0xff;
      card->response_buffer[ 2 +  9 ] = card->c_size & 0xff;

      /* WRITE_BL_LEN = 9 => 2 ^ 9 = 512 byte sectors
         (spread 2 bits, 2 bits across two bytes) */
      card->response_buffer[ 2 +  12 ] = 0x10;
      card->response_buffer[ 2 +  13 ] = 0x01;

      /* Bit 0, not used, always 1 */
      card->response_buffer[ 2 +  15 ] = 0x01;

      /* CRC */
      memset( &card->response_buffer[ 18 ], 0x00, 2 );

      card->response_buffer_next = card->response_buffer;
      card->response_buffer_end = card->response_buffer + 20;
      break;
    case SEND_CID:
      card->response_buffer[ 0 ] = card->r1_status; /* R1 command response */
      card->response_buffer[ 1 ] = 0xfe;            /* data token */

      /* For now, we return an empty CID. This seems to work. */
      memset( &card->response_buffer[ 2 ], 0x00, 16 );

      /* Set blank OID */
      memcpy( &card->response_buffer[ 2 + 1 ], "  ", 2 );

      /* Set product name */
      memcpy( &card->response_buffer[ 2 + 3 ], "FUSE", 4 );

      /* Bit 0, not used, always 1 */
      card->response_buffer[ 2 +  15 ] = 0x01;

      /* CRC */
      memset( &card->response_buffer[ 18 ], 0x00, 2 );

      card->response_buffer_next = card->response_buffer;
      card->response_buffer_end = card->response_buffer + 20;
      break;
    case READ_SINGLE_BLOCK:
      read_single_block( card );
      break;
    case WRITE_BLOCK:
      set_response_buffer_r1( card );
      break;
    case APP_CMD:
      card->cmd55_issued = 1;
      set_response_buffer_r1( card );
      break;
    case READ_OCR:
      /* We set only the card capacity status (CCS, bit 30) and card power up
         status bits (bit 31). CCS set indicates an SDHC card. */
      set_response_buffer_r7( card, 0xc0000000 );
      break;
    default:
      /* Command not implemented or illegal command */
      card->r1_status |= ILLEGAL_COMMAND_MASK;
      set_response_buffer_r1( card );

      libspectrum_print_error(
          LIBSPECTRUM_ERROR_UNKNOWN, "Unknown MMC command %d received",
          card->current_command );
      break;
  }
}

static void
do_command( libspectrum_mmc_card *card )
{
  /* Previous APP_CMD indicates to the card that the next command is an
     application specific command rather than a standard command */
  if( card->cmd55_issued ) {
    card->cmd55_issued = 0;

    /* If a non-ACMD is sent, then it is respected by the card as a normal
       SD Memory Card command */
    if( do_application_command( card ) )
      return;
  }

  do_standard_command( card );
}

static void
write_single_block( libspectrum_mmc_card *card )
{
  libspectrum_dword sector_number =
    card->current_argument[ 3 ] +
    (card->current_argument[ 2 ] << 8) +
    (card->current_argument[ 1 ] << 16) +
    (card->current_argument[ 0 ] << 24);

  /* Sector out of range */
  if( sector_number >= card->total_sectors ) {
    card->r1_status |= PARAMETER_ERROR_MASK;
    set_response_buffer_r1( card );
    return;
  }

  libspectrum_ide_write_sector_to_hdf( &card->drive, card->cache, sector_number,
                                       card->send_buffer );

  card->response_buffer[ 0 ] = 0x05; /* data response */
  card->response_buffer[ 1 ] = 0x01; /* busy flag (end of programming) */
  card->response_buffer_next = card->response_buffer;
  card->response_buffer_end = card->response_buffer + 2;
}

static void
do_command_data( libspectrum_mmc_card *card )
{
  switch( card->current_command ) {
    case WRITE_BLOCK:
      write_single_block( card );
      break;
    default:
      /* This should never happen as it indicates a failure in our state machine */
      libspectrum_print_error(
          LIBSPECTRUM_ERROR_LOGIC,
          "Attempting to execute unknown MMC data command %d\n",
          card->current_command );
      break;
  }
}

void
libspectrum_mmc_write( libspectrum_mmc_card *card, libspectrum_byte data )
{
  /* No card inserted => no change in state */
  if( !card->drive.disk ) return;

  switch( card->command_state ) {
    case WAITING_FOR_COMMAND:
      if( parse_command( card, data ) )
        card->command_state = WAITING_FOR_DATA0;
      break;
    case WAITING_FOR_DATA0:
    case WAITING_FOR_DATA1:
    case WAITING_FOR_DATA2:
    case WAITING_FOR_DATA3:
      card->current_argument[ card->command_state - 1 ] = data;
      card->command_state++;
      break;
    case WAITING_FOR_CRC:
      /* We ignore the CRC */
      do_command( card );

      /* reset command error flags except in_idle */
      card->r1_status &= IN_IDLE_STATE_MASK;

      card->command_state = card->current_command == WRITE_BLOCK ?
        WAITING_FOR_DATA_TOKEN :
        WAITING_FOR_COMMAND;
      break;
    case WAITING_FOR_DATA_TOKEN:
      if( data == 0xfe ) {
        card->command_state = WAITING_FOR_DATA;
        card->data_count = 0;
      }
      break;
    case WAITING_FOR_DATA:
      card->send_buffer[ card->data_count++ ] = data;
      if( card->data_count == 512 )
        card->command_state = WAITING_FOR_DATA_CRC1;
      break;
    case WAITING_FOR_DATA_CRC1:
      /* We ignore the data CRC as well */
      card->command_state = WAITING_FOR_DATA_CRC2;
      break;
    case WAITING_FOR_DATA_CRC2:
      do_command_data( card );
      card->command_state = WAITING_FOR_COMMAND;
      break;
    }
}
