/*
	This file is part of ttymidi.

	ttymidi is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	ttymidi is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with ttymidi.  If not, see <http://www.gnu.org/licenses/>.

	*new* by cchaussat
	Original ttymidi source code v0.60 (from Feb. 1st 2012)
	Taken from http://www.varal.org/ttymidi/ttymidi.tar.gz
	Using tutorial from http://www.qlcplus.org/forum/viewtopic.php?t=14337

	History of changes (at least as I can rebuild it from the various forums ...)

	****************************
	Modified 2014 by Johnty Wang
	****************************
	This is a slightly modified version of ttymidi, designed for a very
	specific purpose: to create a sysex bridge between a microcontroller
	that communicates sensor data via a USB serial port, and software
	running the same computer that communicates. It was designed to work with
	Infusion's ICubeX digitizers which can be configured and operated via sysex data

	The original ttymidi application only expected to parse and send MIDI data,
	and does not do anything when it receives sysex data. This version, on the other
	hand, *only* passes sysex messages back and forth between the serial and virtual midi ports

	The other change from original ttymidi code is that the MIDI por type being created:
	the bit SND_SEQ_PORT_TYPE_MIDI_GENERIC was added so that the virtual port
	shows up when searching for ports in ofxMidi

	to compile: gcc ttymidi.c -o ttymidi -lasound -pthread

	Created December 2014 by Johnty Wang [johntywang@infusionsystems.com]

	**************************
	Modified 2017 by sixeight7
	**************************
	The original ttymidi application did not support sysex messages. This version does.
	Based it on the work of johnty/ttymidi-icubex.c (see https://gist.github.com/johnty/de8b3d3041c7ee43accd)

	**************************
	Modified 2019 by ElBartoME
	**************************
	ttymidi with SysEx support (for MT-32) (see https://github.com/ElBartoME/ttymidi)
	I modified ttymidi in order to be able to process SysEx messages.
	This is crucial for some applications (like using Munt).
	I only implemented in one direction for now (Midi serial -> ALSA) as that is the only thing important for Munt.
	The code is based on sixeight7's code (https://github.com/sixeight7/ttymidi). His code unfortunately didn't work correctly.

	**************************
	Modified 2021 by cchaussat
	**************************
	Based on ElBartoME's code (based on sixeight7's code (based on Johnty Wang's code (based on original ttymidi code v0.60)))
	Now full operation midi + full bi-directional sysex operation, done by merging/tweaking some of JW's code into EB's new code
	Cleaned up all midi and sysex buffers and data casting to unsigned char (otherwise <0 char values sometimes appear as 4-byte FFFFFFnn data)
	All midi data prints formatted homogeneously and in hexadecimal (mix of decimal and hex was confusing)
	All prints to stdout flushed immediately
	Remaining issue : the non-midi text command FF 00 00 most of the times does not display the whole text message, seems to interfere with something

	See tags *new* on changes wrt original ttymidi code and/or JW's or EB's code (I did not use sixeight7's code at all)
	To compile: gcc ttymidi-sysex.c -o ttymidi-sysex -lasound -lpthread
*/


#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <argp.h>
#include <alsa/asoundlib.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <poll.h>
// Linux-specific
#include <linux/serial.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>
// Custom baud rates
#undef NCCS
#define termios termios_HIDE
#include <asm/termbits.h>
#undef termios
#undef NCCS

#define FALSE               0
#define TRUE                1

#define MAX_DEV_STR_LEN    32
#define BUF_SIZE         1024  // Size of the serial midi buffer - determines the maximum size of sysex messages *new*

/* change this definition for the correct port */
//#define _POSIX_SOURCE 1 /* POSIX compliant source */

int run;
int serial;
int port_out_id;
unsigned char running_status_out;

/* --------------------------------------------------------------------- */
// Program options

static struct argp_option options[] =
{
	{"serialdevice" , 's', "DEV" , 0, "Serial device to use. Default = /dev/ttyUSB0" },
	{"baudrate"     , 'b', "BAUD", 0, "Serial port baud rate. Default = 115200" },
	{"verbose"      , 'v', 0     , 0, "For debugging: Produce verbose output" },
	{"printonly"    , 'p', 0     , 0, "Super debugging: Print values read from serial -- and do nothing else" },
	{"quiet"        , 'q', 0     , 0, "Don't produce any output, even when the print command is sent" },
	{"name"		, 'n', "NAME", 0, "Name of the Alsa MIDI client. Default = ttymidi" },
	{ 0 }
};

typedef struct _arguments
{
	int  silent, verbose, printonly;
	char serialdevice[MAX_DEV_STR_LEN];
	speed_t baudrate, customrate;
	char name[MAX_DEV_STR_LEN];
} arguments_t;

void exit_cli(int sig)
{
	if (run) printf("\nttymidi closing down...");
	run = FALSE;
}

static error_t parse_opt (int key, char *arg, struct argp_state *state)
{
	/* Get the input argument from argp_parse, which we
	   know is a pointer to our arguments structure. */
	arguments_t *arguments = state->input;
	int baud_temp;

	switch (key)
	{
		case 'p':
			arguments->printonly = 1;
			break;
		case 'q':
			arguments->silent = 1;
			break;
		case 'v':
			arguments->verbose = 1;
			break;
		case 's':
			if (arg == NULL) break;
			strncpy(arguments->serialdevice, arg, MAX_DEV_STR_LEN - 1);
			break;
		case 'n':
			if (arg == NULL) break;
			strncpy(arguments->name, arg, MAX_DEV_STR_LEN - 1);
			break;
		case 'b':
			if (arg == NULL) break;
			baud_temp = strtol(arg, NULL, 0);
			if (baud_temp != EINVAL && baud_temp != ERANGE)
				switch (baud_temp)
				{
					case 1200   : arguments->baudrate = B1200  ; break;
					case 2400   : arguments->baudrate = B2400  ; break;
					case 4800   : arguments->baudrate = B4800  ; break;
					case 9600   : arguments->baudrate = B9600  ; break;
					case 19200  : arguments->baudrate = B19200 ; break;
					case 31250  : arguments->baudrate = B38400 ; arguments->customrate = 31250; break;
					case 38400  : arguments->baudrate = B38400 ; break;
					case 57600  : arguments->baudrate = B57600 ; break;
					case 115200 : arguments->baudrate = B115200; break;
					default: printf("Baud rate %i is not supported.\n",baud_temp); exit(1);
				}

		case ARGP_KEY_ARG:
		case ARGP_KEY_END:
			break;

		default:
			return ARGP_ERR_UNKNOWN;
	}

	return 0;
}

void arg_set_defaults(arguments_t *arguments)
{
	char *serialdevice_temp = "/dev/ttyUSB0";
	arguments->printonly    = 0;
	arguments->silent       = 0;
	arguments->verbose      = 0;
	arguments->baudrate     = B115200;
	arguments->customrate   = 0;
	char *name_tmp		= (char *)"ttymidi";
	strncpy(arguments->serialdevice, serialdevice_temp, MAX_DEV_STR_LEN - 1);
	strncpy(arguments->name, name_tmp, MAX_DEV_STR_LEN - 1);
}

const char *argp_program_version     = "ttymidi 0.60";
const char *argp_program_bug_address = "tvst@hotmail.com";
static char doc[]       = "ttymidi - Connect serial port devices to ALSA MIDI programs!";
static struct argp argp = { options, parse_opt, 0, doc };
arguments_t arguments;


/* --------------------------------------------------------------------- */
// MIDI stuff

int open_seq(snd_seq_t** seq)
{
	int port_out_id, port_in_id; // actually port_in_id is not needed nor used anywhere

	if (snd_seq_open(seq, "default", SND_SEQ_OPEN_DUPLEX, 0) < 0)
	{
		fprintf(stderr, "Error opening ALSA sequencer.\n");
		exit(1);
	}

	snd_seq_set_client_name(*seq, arguments.name);

	char nameInput[MAX_DEV_STR_LEN + 3];
	strcpy(nameInput, arguments.name);
	strcat(nameInput, " In");

	if ((port_out_id = snd_seq_create_simple_port(*seq, nameInput,
					SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ,
					SND_SEQ_PORT_TYPE_MIDI_GENERIC|SND_SEQ_PORT_TYPE_APPLICATION)) < 0)  // *new*
	{
		fprintf(stderr, "Error creating sequencer MIDI out port.\n");  // *new*
	}

	char nameOutput[MAX_DEV_STR_LEN + 4];
	strcpy(nameOutput, arguments.name);
	strcat(nameOutput, " Out");

	if ((port_in_id = snd_seq_create_simple_port(*seq, nameOutput,
					SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE,
					SND_SEQ_PORT_TYPE_MIDI_GENERIC|SND_SEQ_PORT_TYPE_APPLICATION)) < 0)  // *new*
	{
		fprintf(stderr, "Error creating sequencer MIDI in port.\n");  // *new*
	}

	return port_out_id;
}

void parse_midi_command(snd_seq_t* seq, int port_out_id, unsigned char *buf, int buflen)  // *new*
{
/*
	MIDI COMMANDS
	-------------------------------------------------------------------
	name                 status      param 1          param 2
	-------------------------------------------------------------------
	note off             0x80+C       key #            velocity
	note on              0x90+C       key #            velocity
	poly key pressure    0xA0+C       key #            pressure value
	control change       0xB0+C       control #        control value
	program change       0xC0+C       program #        --
	mono key pressure    0xD0+C       pressure value   --
	pitch bend           0xE0+C       range (LSB)      range (MSB)
	system               0xF0+C       manufacturer     model
	-------------------------------------------------------------------
	C is the channel number, from 0 to 15;
	-------------------------------------------------------------------
	source: http://ftp.ec.vanderbilt.edu/computermusic/musc216site/MIDI.Commands.html

	In this program the pitch bend range will be transmitter as
	one single 8-bit number. So the end result is that MIDI commands
	will be transmitted as 3 bytes, starting with the operation byte:

	buf[0] --> operation/channel
	buf[1] --> param1
	buf[2] --> param2        (param2 not transmitted on program change or key press)
*/

	snd_seq_event_t ev;
	snd_seq_ev_clear(&ev);
	snd_seq_ev_set_direct(&ev);
	snd_seq_ev_set_source(&ev, port_out_id);
	snd_seq_ev_set_subs(&ev);
	ev.type = SND_SEQ_EVENT_NONE;

	unsigned char operation, channel, param1, param2;  // *new* was int in original code
	int int_param1;  // *new*

	operation = buf[0] & 0xF0;
	channel   = buf[0] & 0x0F;
	param1    = buf[1] & 0xFF;  // *new* (protection ?)
	param2    = buf[2] & 0xFF;  // *new* (protection ?)

	switch (operation)
	{
		case 0x90:  // *new* handle noteon first to speed up the mostly used message
			if (!arguments.silent && arguments.verbose) {
				printf("Serial  %02X Note on            %02X %02X %02X\n", operation, channel, param1, param2);
				fflush(stdout);  // *new*
			}
			snd_seq_ev_set_noteon(&ev, channel, param1, param2);
			break;

		case 0x80:
			if (!arguments.silent && arguments.verbose) {
				printf("Serial  %02X Note off           %02X %02X %02X\n", operation, channel, param1, param2);
				fflush(stdout);  // *new*
			}
			snd_seq_ev_set_noteoff(&ev, channel, param1, param2);
			break;

		case 0xA0:
			if (!arguments.silent && arguments.verbose) {
				printf("Serial  %02X Pressure change    %02X %02X %02X\n", operation, channel, param1, param2);
				fflush(stdout);  // *new*
			}
			snd_seq_ev_set_keypress(&ev, channel, param1, param2);
			break;

		case 0xB0:
			if (!arguments.silent && arguments.verbose) {
				printf("Serial  %02X Controller change  %02X %02X %02X\n", operation, channel, param1, param2);
				fflush(stdout);  // *new*
			}
			snd_seq_ev_set_controller(&ev, channel, param1, param2);
			break;

		case 0xC0:
			if (!arguments.silent && arguments.verbose) {
				printf("Serial  %02X Program change     %02X %02X\n", operation, channel, param1);
				fflush(stdout);  // *new*
			}
			snd_seq_ev_set_pgmchange(&ev, channel, param1);
			break;

		case 0xD0:
			if (!arguments.silent && arguments.verbose) {
				printf("Serial  %02X Channel press      %02X %02X\n", operation, channel, param1);
				fflush(stdout);  // *new*
			}
			snd_seq_ev_set_chanpress(&ev, channel, param1);
			break;

		case 0xE0:
			int_param1 = (int) (param1 & 0x7F) + ((param2 & 0x7F) << 7);  // *new*
			if (!arguments.silent && arguments.verbose) {
				printf("Serial  %02X Pitch bend         %02X %04X\n", operation, channel, int_param1);  // *new*
				fflush(stdout);  // *new*
			}
			snd_seq_ev_set_pitchbend(&ev, channel, int_param1 - 8192); // in alsa MIDI we want signed int *new*
			break;

		case 0xF0:  // *new*
			switch (channel) {
				case 0x7: // Split Sysex
					buf++;
					buflen--;
					// fallthrough
				case 0x0:
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  %02X Sysex len = %04X   ", operation, buflen);  // *new*
						int i;
						for (i=0; i < buflen; i++) {
							printf("%02X ", buf[i]);
						}
						printf("\n");
						fflush(stdout);  // *new*
					}
					// Send sysex message
					snd_seq_ev_set_sysex(&ev, buflen, buf);
					break;
				case 0x1: // MTC Quarter Frame package
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  MTC Quarter Frame       %02x\n", param1);
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.data.control.value = param1;
					ev.type = SND_SEQ_EVENT_QFRAME;
					break;
				case 0x2: // Song Position
					int_param1 = (int) (param1 & 0x7F) + ((param2 & 0x7F) << 7);  // *new*
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Song Position           %04x\n", int_param1);
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.data.control.value = int_param1;
					ev.type = SND_SEQ_EVENT_SONGPOS;
					break;
				case 0x3: // Song Select
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Song Select             %02x\n", param1);
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.data.control.value = param1;
					ev.type = SND_SEQ_EVENT_SONGSEL;
					break;
				case 0x5: // Port Select (non-standard)
					if (!arguments.silent) {
						printf("Serial  Port Select             %02x (unsupported)\n", param1);
						fflush(stdout);  // *new*
					}
					break;
				case 0x6: // Tune Request
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Tune Request\n");
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.type = SND_SEQ_EVENT_TUNE_REQUEST;
					break;
				case 0x8: // Clock
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Clock\n");
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.type = SND_SEQ_EVENT_CLOCK;
					break;
				case 0x9: // Tick
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Tick\n");
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.type = SND_SEQ_EVENT_TICK;
					break;
				case 0xA: // Start
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Start\n");
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.type = SND_SEQ_EVENT_START;
					break;
				case 0xB: // Continue
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Continue\n");
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.type = SND_SEQ_EVENT_CONTINUE;
					break;
				case 0xC: // Stop
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Stop\n");
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.type = SND_SEQ_EVENT_STOP;
					break;
				case 0xE: // Active sense
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Active sense\n");
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.type = SND_SEQ_EVENT_SENSING;
					break;
				case 0xF: // Reset
					if (!arguments.silent && arguments.verbose) {
						printf("Serial  Reset\n");
						fflush(stdout);  // *new*
					}
					snd_seq_ev_set_fixed(&ev);
					ev.type = SND_SEQ_EVENT_RESET;
					break;

				default:
					if (!arguments.silent) {  // *new*
						printf("Serial  %02X Unknown MIDI System cmd\n", buf[0] & 0xFF);  // *new*
						fflush(stdout);  // *new*
					}
					break;
			}
			break;

		default:
			if (!arguments.silent) {  // *new*
				printf("Serial  %02X Unknown MIDI cmd   %02X %02X %02X\n", operation, channel, param1, param2);  // *new*
				fflush(stdout);  // *new*
			}
			break;
	}

	snd_seq_event_output_direct(seq, &ev);
	snd_seq_drain_output(seq);
}

void write_midi_action_to_serial_port(snd_seq_t* seq_handle)
{
	snd_seq_event_t* ev;
	unsigned char bytes[9], *bytes_data;  // *new*
	unsigned char *sysex_data = NULL;  // *new*
	int bytes_len, sysex_len, written;  // *new*

	do
	{
		snd_seq_event_input(seq_handle, &ev);

		bytes_len = 0;
		sysex_len = 0;

		switch (ev->type)
		{

			case SND_SEQ_EVENT_NOTEOFF:
				bytes[0] = 0x80 + ev->data.note.channel;
				bytes[1] = ev->data.note.note;
				bytes[2] = ev->data.note.velocity;
				bytes_len = 3;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Note off           %02X %02X %02X\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1], bytes[2]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_NOTEON:
				bytes[0] = 0x90 + ev->data.note.channel;
				bytes[1] = ev->data.note.note;
				bytes[2] = ev->data.note.velocity;
				bytes_len = 3;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Note on            %02X %02X %02X\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1], bytes[2]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_KEYPRESS:
				bytes[0] = 0xA0 + ev->data.note.channel;  // *new* was 90 in original code
				bytes[1] = ev->data.note.note;
				bytes[2] = ev->data.note.velocity;
				bytes_len = 3;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Pressure change    %02X %02X %02X\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1], bytes[2]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_CONTROLLER:
				bytes[0] = 0xB0 + ev->data.control.channel;
				bytes[1] = ev->data.control.param;
				bytes[2] = ev->data.control.value;
				bytes_len = 3;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Controller change  %02X %02X %02X\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1], bytes[2]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_CONTROL14:
				bytes[0] = 0xB0 + ev->data.control.channel;
				bytes[1] = ev->data.control.param;
				bytes[2] = (unsigned char)((ev->data.control.value >> 7) & 0x7F);
				bytes[3] = ev->data.control.param + 32;
				bytes[4] = (unsigned char)(ev->data.control.value & 0x7F);
				if (ev->data.control.param >= 0 && ev->data.control.param < 32)
				{
					bytes_len = 5;
					if (!arguments.silent && arguments.verbose) {
						printf("Alsa    %02X 14 bit Controller  %02X %04X %04X\n", bytes[0]&0xF0, bytes[0]&0xF, ev->data.control.param, ev->data.control.value);
						fflush(stdout);  // *new*
					}
				}
				else
				{
					if (!arguments.silent) {  // *new*
						printf("Alsa    %02X Unknown Controller %02X %04X %04X\n", bytes[0]&0xF0, bytes[0]&0xF, ev->data.control.param, ev->data.control.value);
						fflush(stdout);  // *new*
					}
				}
				break;

			case SND_SEQ_EVENT_NONREGPARAM:
				bytes[0] = 0xB0 + ev->data.control.channel;
				bytes[1] = 0x63; // NRPN MSB
				bytes[2] = (unsigned char)((ev->data.control.param >> 7) & 0x7F);
				bytes[3] = 0x62; // NRPN LSB
				bytes[4] = (unsigned char)(ev->data.control.param & 0x7F);
				bytes[5] = 0x06; // data entry MSB
				bytes[6] = (unsigned char)((ev->data.control.value >> 7) & 0x7F);
				bytes[7] = 0x26; // data entry LSB
				bytes[8] = (unsigned char)(ev->data.control.value & 0x7F);
				bytes_len = 9;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X 14 bit NRPN        %02X %04X %04X\n", bytes[0]&0xF0, bytes[0]&0xF, ev->data.control.param, ev->data.control.value);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_REGPARAM:
				bytes[0] = 0xB0 + ev->data.control.channel;
				bytes[1] = 0x65; // RPN MSB
				bytes[2] = (unsigned char)((ev->data.control.param >> 7) & 0x7F);
				bytes[3] = 0x64; // RPN LSB
				bytes[4] = (unsigned char)(ev->data.control.param & 0x7F);
				bytes[5] = 0x06; // data entry MSB
				bytes[6] = (unsigned char)((ev->data.control.value >> 7) & 0x7F);
				bytes[7] = 0x26; // data entry LSB
				bytes[8] = (unsigned char)(ev->data.control.value & 0x7F);
				bytes_len = 9;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X 14 bit RPN         %02X %04X %04X\n", bytes[0]&0xF0, bytes[0]&0xF, ev->data.control.param, ev->data.control.value);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_PGMCHANGE:
				bytes[0] = 0xC0 + ev->data.control.channel;
				bytes[1] = ev->data.control.value;
				bytes_len = 2;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Program change     %02X %02X\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_CHANPRESS:
				bytes[0] = 0xD0 + ev->data.control.channel;
				bytes[1] = ev->data.control.value;
				bytes_len = 2;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Channel press      %02X %02X\n", bytes[0]&0xF0, bytes[0]&0xF, bytes[1]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_PITCHBEND:
				bytes[0] = 0xE0 + (unsigned char)ev->data.control.channel;  // *new*
				ev->data.control.value += 8192;
				bytes[1] = (unsigned char)(ev->data.control.value & 0x7F);  // *new*
				bytes[2] = (unsigned char)(ev->data.control.value >> 7);  // *new*
				bytes_len = 3;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Pitch bend         %02X %04X\n", bytes[0]&0xF0, bytes[0]&0xF, ev->data.control.value);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_SYSEX:  // *new*
				sysex_len = ev->data.ext.len;
				sysex_data = (unsigned char*)ev->data.ext.ptr;
				if (!arguments.silent && arguments.verbose)
				{
					printf("Alsa    F0 Sysex len = %04X   ", sysex_len);
					int i;
					for (i=0; i<sysex_len; i++) {
						printf("%02X ", sysex_data[i]);  // *new* unsigned char cast suppressed
					}
					printf("\n");  // *new*
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_QFRAME:
				bytes[0] = 0xF1;
				bytes[1] = ev->data.control.value;
				bytes_len = 2;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X MTC Quarter Frame      %02X\n", bytes[0], bytes[1]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_SONGPOS:
				bytes[0] = 0xF2;
				ev->data.control.value += 8192;
				bytes[1] = (unsigned char)(ev->data.control.value & 0x7F);
				bytes[2] = (unsigned char)(ev->data.control.value >> 7);
				bytes_len = 3;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Song Position      %04X\n", bytes[0], ev->data.control.value);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_SONGSEL:
				bytes[0] = 0xF3;
				bytes[1] = ev->data.control.value;
				bytes_len = 2;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Song Select        %02X\n", bytes[0], bytes[1]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_TUNE_REQUEST:
				bytes[0] = 0xF6;
				bytes_len = 1;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Tune Request\n", bytes[0]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_CLOCK:
				bytes[0] = 0xF8;
				bytes_len = 1;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Clock\n", bytes[0]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_TICK:
				bytes[0] = 0xF9;
				bytes_len = 1;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Tick\n", bytes[0]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_START:
				bytes[0] = 0xFA;
				bytes_len = 1;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Start\n", bytes[0]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_CONTINUE:
				bytes[0] = 0xFB;
				bytes_len = 1;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Continue\n", bytes[0]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_STOP:
				bytes[0] = 0xFC;
				bytes_len = 1;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Stop\n", bytes[0]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_SENSING:
				bytes[0] = 0xFE;
				bytes_len = 1;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Active Sense\n", bytes[0]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_RESET:
				bytes[0] = 0xFF;
				bytes_len = 1;
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Reset\n", bytes[0]);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_PORT_SUBSCRIBED:
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Port connected     %i:%i -> %i:%i\n", bytes[0], ev->data.connect.sender.client, ev->data.connect.sender.port, ev->data.connect.dest.client, ev->data.connect.dest.port);
					fflush(stdout);  // *new*
				}
				break;

			case SND_SEQ_EVENT_PORT_UNSUBSCRIBED:
				if (!arguments.silent && arguments.verbose) {
					printf("Alsa    %02X Port disconnected  %i:%i -> %i:%i\n", bytes[0], ev->data.connect.sender.client, ev->data.connect.sender.port, ev->data.connect.dest.client, ev->data.connect.dest.port);
					fflush(stdout);  // *new*
				}
				break;

			default:
				if (!arguments.silent) {  // *new*
					printf("Alsa    %02X Unknown MIDI cmd   %02X\n", 0, ev->type);  // *new*
					fflush(stdout);  // *new*
				}
				break;
		}
/*
		bytes[0] = bytes[0] & 0xFF;  // *new* &0xFF (protection ?)
		bytes[1] = bytes[1] & 0xFF;  // *new* &0xFF (protection ?)
		bytes[2] = bytes[2] & 0xFF;  // *new* &0xFF (protection ?)
*/
		// *new* sysex addition
		if (sysex_len > 0) {
			running_status_out = 0;
			while (run && sysex_len > 0) {
				written = write(serial, sysex_data, sysex_len);
				if (written < 0) {
					if (errno != EINTR) {
						if (!arguments.silent) {
							printf("Alsa    Error sending data\n");
							fflush(stdout);
						}
						break;
					}
				} else {
					sysex_data += written;
					sysex_len -= written;
					tcdrain(serial);  // *new* (speed up ?)
				}
			}
		} else if (bytes_len > 0) {
			bytes[1] = (bytes[1] & 0x7F); // just to be sure that one bit is really zero
			bytes[2] = (bytes[2] & 0x7F);
			if (bytes[0] >= 0xF8) {
				/* RealTime messages */
				bytes_data = bytes;
			} else if (bytes[0] >= 0xF0) {
				/* System Common messages */
				running_status_out = 0;
				bytes_data = bytes;
			} else if (bytes[0] == running_status_out) {
				/* Don't send running status byte */
				bytes_data = bytes+1;
				bytes_len--;
			} else {
				/* Update running status */
				running_status_out = bytes[0];
				bytes_data = bytes;
			}
			while (run && bytes_len > 0) {
				written = write(serial, bytes_data, bytes_len);
				if (written < 0) {
					if (errno != EINTR) {
						running_status_out = 0;
						if (!arguments.silent) {
							printf("Alsa    Error sending data\n");
							fflush(stdout);
						}
						break;
					}
				} else {
					bytes_data += written;
					bytes_len -= written;
				}
			}
		}

		snd_seq_free_event(ev);

	} while (run && snd_seq_event_input_pending(seq_handle, 0) > 0);
}

void* read_midi_from_alsa(void* seq)
{
	int npfd;
	struct pollfd* pfd;
	snd_seq_t* seq_handle;

	seq_handle = seq;

	npfd = snd_seq_poll_descriptors_count(seq_handle, POLLIN);
	pfd = (struct pollfd*) alloca(npfd * sizeof(struct pollfd));
	snd_seq_poll_descriptors(seq_handle, pfd, npfd, POLLIN);

	running_status_out = 0;

	while (run)
	{
		if (poll(pfd,npfd, 100) > 0)
		{
			write_midi_action_to_serial_port(seq_handle);
		}
	}

	printf("\nStopping [PC]->[Hardware] communication...");

	return NULL;
}

void* read_midi_from_serial_port(void* seq)
{
	unsigned char buf[BUF_SIZE], running_status_in;  // *new*
	int i, bytesleft;  // *new* (buflen in JW's code not used)
	struct timespec u10ms;
	struct pollfd fds;

	/* set-up a small sleep in case fo error to avoid cpu hungry loops */
	u10ms.tv_sec  = 0;
	u10ms.tv_nsec = 10000000L;

	fds.fd = serial;
	fds.events = POLLIN;

	running_status_in = 0;
	buf[0] = 0;
	i = 1;

	while (run)
	{
		/*
		 * super-debug mode: only print to screen whatever
		 * comes through the serial port.
		 */

		if (arguments.printonly)
		{
			read(serial, buf, 1);
			printf("%02X ", buf[0]&0xFF);  // *new*
			fflush(stdout);
			continue;
		}

		/*
		 * so let's align to the beginning of a midi command.
		 */

		if ((running_status_in == 0) && ((buf[0] == 0xF0) || (buf[0] == 0xF7)) && (buf[i-1] != 0xF7))
		{
			/* Split Sysex */
			buf[0] = 0xF7;
		}
		else
		{
			buf[0] = running_status_in;
		}
		// int i = 1; *new*
		i = 1;  // *new* (i already declared at function start)
		bytesleft = (running_status_in) ? (((running_status_in & 0xF0) == 0xC0 || (running_status_in & 0xF0) == 0xD0) ? 2 : 3) : BUF_SIZE;  // *new*

		while (run && i < bytesleft) {  // *new*
			int ret = poll(&fds, 1, 1000);
			if (ret == 0) continue; // timeout
			if (ret > 0) {
				ret = read(serial, buf+i, 1);
			}
			if (ret <= 0) {
				/* serial error somewhere */
				printf("SerialIn error %02X %d %d\n", buf[0], ret, errno);
				nanosleep(&u10ms, NULL);
				continue;
			}
			buf[i] = buf[i] & 0xFF;  // *new* &0xFF (protection ?)

			if (buf[i] & 0x80) {
				/* Status byte received and will always be first bit!*/
				switch(buf[i] & 0xF0)
				{
					case 0x80: // Note off
					case 0x90: // Note on
					case 0xA0: // Pressure change
					case 0xB0: // Controller change
					case 0xE0: // Pitch bend
						running_status_in = buf[0] = buf[i];
						i = 1;
						bytesleft = 3;
						break;
					case 0xC0: // Program change
					case 0xD0: // Channel press
						running_status_in = buf[0] = buf[i];
						i = 1;
						bytesleft = 2;
						break;
					case 0xF0:
						/* System Common / RealTime messages */
						switch (buf[i] & 0x0F)
						{
							case 0x0: // Sysex start
								running_status_in = 0;
								buf[0] = buf[i];
								i = 1;
								bytesleft = BUF_SIZE;
								break;
							case 0x1: // MTC Quarter Frame package
							case 0x3: // Song Select
							case 0x4: // Undefined/Unknown
							case 0x5: // Port Select (non-standard)
								running_status_in = 0;
								buf[0] = buf[i];
								i = 1;
								bytesleft = 2;
								break;
							case 0x2: // Song Position
								running_status_in = 0;
								buf[0] = buf[i];
								i = 1;
								bytesleft = 3;
								break;
							case 0x6: // Tune Request
								running_status_in = 0;
								buf[0] = buf[i];
								i = 1;
								bytesleft = 1;
								break;
							case 0x7: // Sysex end
								running_status_in = 0;
								if ((buf[0] == 0xF0) || (buf[0] == 0xF7))
								{
									i++;
									bytesleft = i;
								}
								else
								{
									buf[0] = 0;
									i = 0;
									bytesleft = BUF_SIZE;
								}
								break;

							/* RealTime messages */
							case 0x8: // Clock
							case 0x9: // Tick
							case 0xA: // Start
							case 0xB: // Continue
							case 0xC: // Stop
							case 0xD: // Undefined/Unknown
							case 0xE: // Active sense
							case 0xF: // Reset
								/* RealTime message can arrive in the middle of another message */
								/* Process it immediately and continue reading the original message */
								parse_midi_command(seq, port_out_id, buf+i, 1);
								break;
						}
						break;
				}
			} else {
				/* Data byte received */
				if(buf[0] == 0)	// no status *new*
				{
					continue;
				}
				i++;
			}

		}

		if (!run) break;

		/* parse MIDI message */
		parse_midi_command(seq, port_out_id, buf, i);  // *new* (was i+1 in EB's code)
	}

	return NULL;
}


/* --------------------------------------------------------------------- */
// Main program

int main(int argc, char** argv)  // *new* int to remove compilation warning
{
	//arguments arguments;
	struct termios oldtio, newtio;
	//struct serial_struct ser_info;
	snd_seq_t *seq;

	arg_set_defaults(&arguments);
	argp_parse(&argp, argc, argv, 0, 0, &arguments);

	/*
	 * Open MIDI output port
	 */

	port_out_id = open_seq(&seq);

	/*
	 *  Open modem device for reading and not as controlling tty because we don't
	 *  want to get killed if linenoise sends CTRL-C.
	 */

	serial = open(arguments.serialdevice, O_RDWR | O_NOCTTY );

	if (serial < 0)
	{
		perror(arguments.serialdevice);
		exit(-1);
	}

	/* save current serial port settings */
	tcgetattr(serial, &oldtio);

	/* clear struct for new port settings */
	bzero(&newtio, sizeof(newtio));

	/*
	 * BAUDRATE : Set bps rate. You could also use cfsetispeed and cfsetospeed.
	 * CRTSCTS  : output hardware flow control (only used if the cable has
	 * all necessary lines. See sect. 7 of Serial-HOWTO)
	 * CS8      : 8n1 (8bit, no parity, 1 stopbit)
	 * CLOCAL   : local connection, no modem contol
	 * CREAD    : enable receiving characters
	 */
	newtio.c_cflag = arguments.baudrate | CS8 | CLOCAL | CREAD; // CRTSCTS removed

	/*
	 * IGNPAR  : ignore bytes with parity errors
	 * ICRNL   : map CR to NL (otherwise a CR input on the other computer
	 * will not terminate input)
	 * otherwise make device raw (no other input processing)
	 */
	newtio.c_iflag = IGNPAR;

	/* Raw output */
	newtio.c_oflag = 0;

	/*
	 * ICANON  : enable canonical input
	 * disable all echo functionality, and don't send signals to calling program
	 */
	newtio.c_lflag = 0; // non-canonical

	/*
	 * set up: we'll be reading 4 bytes at a time.
	 */
	newtio.c_cc[VTIME]    = 0;  /* inter-character timer unused */
	newtio.c_cc[VMIN]     = 1;  /* blocking read until n character arrives */

	/*
	 * now clean the modem line and activate the settings for the port
	 */
	tcflush(serial, TCIFLUSH);
	tcsetattr(serial, TCSANOW, &newtio);

	// Linux-specific: enable low latency mode (FTDI "nagling off")
//	ioctl(serial, TIOCGSERIAL, &ser_info);
//	ser_info.flags |= ASYNC_LOW_LATENCY;
//	ioctl(serial, TIOCSSERIAL, &ser_info);

	if (arguments.customrate)
	{
		struct termios2 term2;
		ioctl(serial, TCGETS2, &term2);
		term2.c_cflag &= ~CBAUD;
		term2.c_cflag |= BOTHER;
		term2.c_ispeed = arguments.customrate;
		term2.c_ospeed = arguments.customrate;
		ioctl(serial, TCSETS2, &term2);
	}

	if (arguments.printonly)
	{
		printf("Super debug mode: Only printing the signal to screen. Nothing else.\n");
	}

	/*
	 * read commands
	 */

	run = TRUE;
	struct sigaction signal_action;
	signal_action.sa_handler = exit_cli;
	sigemptyset(&signal_action.sa_mask);
	signal_action.sa_flags = 0;
	sigaction(SIGINT, &signal_action, NULL);
	sigaction(SIGTERM, &signal_action, NULL);

	/* Starting thread that is polling alsa midi in port */
	pthread_t midi_out_thread, midi_in_thread;
	pthread_attr_t thread_attr;
	int iret1, iret2;
	pthread_attr_init(&thread_attr);
	pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_JOINABLE);
	iret1 = pthread_create(&midi_out_thread, &thread_attr, read_midi_from_alsa, (void*) seq);
	/* And also thread for polling serial data. As serial is currently read in
		blocking mode, by this we can enable ctrl+c quiting and avoid zombie
		alsa ports when killing app with ctrl+z */
	iret2 = pthread_create(&midi_in_thread, &thread_attr, read_midi_from_serial_port, (void*) seq);
	pthread_attr_destroy(&thread_attr);

	while (run)
	{
		sleep(100);
	}

	void* status;
	if (iret1 == 0)
	{
		pthread_join(midi_out_thread, &status);
	}
	if ((iret2 == 0) && !arguments.printonly)
	{
		pthread_join(midi_in_thread, &status);
	}

	/* restore the old port settings */
	tcsetattr(serial, TCSANOW, &oldtio);
	close(serial);
	printf("\ndone!\n");
}
