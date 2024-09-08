/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 *
 * Main loop
 * Feb 2015 @ OpenSprinkler.com
 *
 * This file is part of the OpenSprinkler Firmware
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

//
// <JRJ> V2.2.1.0.100
// 	  2024-09-02 Added spdlog V1.14.1 for enhanced logging/debugging capability e.g. descriptive messages, log file management, enhanced message formatting
//

extern const char *__progname;

#include <limits.h>
// <JRJ>  #include <time.h>

#include "OpenSprinkler.h"
#include "program.h"
#include "weather.h"
#include "opensprinkler_server.h"
#include "mqtt.h"
// <JRJ> new in V2.2.1.0
#include "main.h"

// <JRJ> 20240902 "spdlog" logging solution https://github.com/gabime/spdlog V1.14.1
#include "spdlog/spdlog.h"
// #include <spdlog/sinks/dist_sink.h>
// #include <spdlog/sinks/null_sink.h>
#include "spdlog/sinks/stdout_color_sinks.h"
// #include <spdlog/sinks/basic_file_sink.h>
// #include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/daily_file_sink.h"

#if defined(ARDUINO)
#include <Arduino.h>
#endif

#include "ArduinoJson.hpp"

#if defined(ARDUINO)
#if defined(ESP8266)
ESP8266WebServer *update_server = NULL;
DNSServer *dns = NULL;
ENC28J60lwIP enc28j60(PIN_ETHER_CS); // ENC28J60 lwip for wired Ether
Wiznet5500lwIP w5500(PIN_ETHER_CS);	 // W5500 lwip for wired Ether
lwipEth eth;
bool useEth = false; // tracks whether we are using WiFi or wired Ether connection
static uint16_t led_blink_ms = LED_FAST_BLINK;
#else
EthernetServer *m_server = NULL;
EthernetClient *m_client = NULL;
SdFat sd; // SD card object
bool useEth = true;
#endif
unsigned long getNtpTime();
#else // header and defs for RPI/BBB

#endif

#if defined(USE_OTF)
OTF::OpenThingsFramework *otf = NULL;
#endif

void push_message(int type, uint32_t lval = 0, float fval = 0.f, const char *sval = NULL);
void manual_start_program(unsigned char, unsigned char);
void remote_http_callback(char *);

// Small variations have been added to the timing values below
// to minimize conflicting events
#define NTP_SYNC_INTERVAL 86413L			 // NTP sync interval (in seconds)
#define CHECK_NETWORK_INTERVAL 601			 // Network checking timeout (in seconds)
#define CHECK_WEATHER_TIMEOUT 21613L		 // Weather check interval (in seconds)
#define CHECK_WEATHER_SUCCESS_TIMEOUT 86400L // Weather check success interval (in seconds)
#define LCD_BACKLIGHT_TIMEOUT 15			 // LCD backlight timeout (in seconds))
#define PING_TIMEOUT 200					 // Ping test timeout (in ms)
#define UI_STATE_MACHINE_INTERVAL 50		 // how often does ui_state_machine run (in ms)
#define CLIENT_READ_TIMEOUT 5				 // client read timeout (in seconds)
#define DHCP_CHECKLEASE_INTERVAL 3600L		 // DHCP check lease interval (in seconds)
// Define buffers: need them to be sufficiently large to cover string option reading
char ether_buffer[ETHER_BUFFER_SIZE * 2]; // ethernet buffer, make it twice as large to allow overflow
char tmp_buffer[TMP_BUFFER_SIZE * 2];	  // scratch buffer, make it twice as large to allow overflow

// ====== Object defines ======
OpenSprinkler os; // OpenSprinkler object
ProgramData pd;	  // ProgramdData object

/* ====== Robert Hillman (RAH)'s implementation of flow sensor ======
 * flow_begin - time when valve turns on
 * flow_start - time when flow starts being measured (i.e. 2 mins after flow_begin approx
 * flow_stop - time when valve turns off (last rising edge pulse detected before off)
 * flow_gallons - total # of gallons+1 from flow_start to flow_stop
 * flow_last_gpm - last flow rate measured (averaged over flow_gallons) from last valve stopped (used to write to log file). */
ulong flow_begin, flow_start, flow_stop, flow_gallons;
ulong flow_count = 0;
unsigned char prev_flow_state = HIGH;
float flow_last_gpm = 0;

uint32_t reboot_timer = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// JRJ v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
byte current_program_pid = 0;
static char current_program_name[TMP_BUFFER_SIZE];
int current_program_state = 0;
int current_program_schedule_type = 0;
ulong current_program_start = 0;
ulong current_program_end = 0;
ulong current_program_duration = 0;
ulong current_program_remaining = 0;
float current_program_water_level = 0.0;
*/
//  (0==no override [default], 10==override to 0, 11=override to 1)
int sensor1_active_override = 0; // Override sensor1_active via WEB API http://os-ip/co?pw=xxx&sn1s=10&sn2s=11  sensor1 == no rain sensed, sensor2 == rain sensed
int sensor2_active_override = 0; // Override sensor1_active  via WEB API http://os-ip/co?pw=xxx&sn1s=11&sn2s=10 sensor1 == rain sensed, sensor2 == no rain sened

//  <JRJ> use the interrupt feature in WiringPi to capture the state changes of [5,6,12,13,19,16,26,20,  14,4,15,18,17,27,23,22,  21,24,25]

/*void gpioInterrupt5F  (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt5() INT_EDGE_FALLING Interrupt captured on GPIO: {0} value: {1}", 5, digitalRead(5)); }
void gpioInterrupt21F (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt21() INT_EDGE_FALLING Interrupt captured on GPIO: {0} value: {1}", 21, digitalRead(21)); }
void gpioInterrupt24F (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt24() INT_EDGE_FALLING Interrupt captured on GPIO: {0} value: {1}", 24, digitalRead(24)); }
void gpioInterrupt25F (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt25() INT_EDGE_FALLING Interrupt captured on GPIO: {0} value: {1}", 25, digitalRead(25)); }

void gpioInterrupt5R  (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt5() INT_EDGE_RISING Interrupt captured on GPIO: {0} value: {1}", 5, digitalRead(5)); }
void gpioInterrupt21R (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt21() INT_EDGE_RISING Interrupt captured on GPIO: {0} value: {1}", 21, digitalRead(21)); }
void gpioInterrupt24R (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt24() INT_EDGE_RISING Interrupt captured on GPIO: {0} value: {1}", 24, digitalRead(24)); }
void gpioInterrupt25R (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt25() INT_EDGE_RISING Interrupt captured on GPIO: {0} value: {1}", 25, digitalRead(25)); }

void gpioInterrupt5B  (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt5() INT_EDGE_BOTH Interrupt captured on GPIO: {0} value: {1}", 5, digitalRead(5)); }
void gpioInterrupt21B (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt21() INT_EDGE_BOTH Interrupt captured on GPIO: {0} value: {1}", 21, digitalRead(21)); }
void gpioInterrupt24B (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt24() INT_EDGE_BOTH Interrupt captured on GPIO: {0} value: {1}", 24, digitalRead(24)); }
void gpioInterrupt25B (void) { auto spdlogFileLogger = spdlog::get("multi_logger"); spdlogFileLogger->info("Main.cpp->gpioInterrupt25() INT_EDGE_BOTH Interrupt captured on GPIO: {0} value: {1}", 25, digitalRead(25)); }
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// JRJ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void flow_poll()
{
#if defined(ESP8266)
	if (os.hw_rev >= 2)
		pinModeExt(PIN_SENSOR1, INPUT_PULLUP); // this seems necessary for OS 3.2
#endif
	unsigned char curr_flow_state = digitalReadExt(PIN_SENSOR1);
	if (!(prev_flow_state == HIGH && curr_flow_state == LOW))
	{ // only record on falling edge
		prev_flow_state = curr_flow_state;
		return;
	}
	prev_flow_state = curr_flow_state;
	ulong curr = millis();
	flow_count++;

	/* RAH implementation of flow sensor */
	if (flow_start == 0)
	{
		flow_gallons = 0;
		flow_start = curr;
	} // if first pulse, record time
	if ((curr - flow_start) < 90000)
	{
		flow_gallons = 0;
	} // wait 90 seconds before recording flow_begin
	else
	{
		if (flow_gallons == 1)
		{
			flow_begin = curr;
		}
	}
	flow_stop = curr; // get time in ms for stop
	flow_gallons++;	  // increment gallon count for each poll
					  /* End of RAH implementation of flow sensor */
}

#if defined(ARDUINO)
// ====== UI defines ======
static char ui_anim_chars[3] = {'.', 'o', 'O'};

#define UI_STATE_DEFAULT 0
#define UI_STATE_DISP_IP 1
#define UI_STATE_DISP_GW 2
#define UI_STATE_RUNPROG 3

static unsigned char ui_state = UI_STATE_DEFAULT;
static unsigned char ui_state_runprog = 0;

bool ui_confirm(PGM_P str)
{
	os.lcd_print_line_clear_pgm(str, 0);
	os.lcd_print_line_clear_pgm(PSTR("(B1:No, B3:Yes)"), 1);
	unsigned char button;
	ulong start = millis();
	do
	{
		button = os.button_read(BUTTON_WAIT_NONE);
		if ((button & BUTTON_MASK) == BUTTON_3 && (button & BUTTON_FLAG_DOWN))
			return true;
		if ((button & BUTTON_MASK) == BUTTON_1 && (button & BUTTON_FLAG_DOWN))
			return false;
		delay(10);
	} while (millis() - start < 2500);
	return false;
}

void ui_state_machine()
{

	// to avoid ui_state_machine taking too much computation time
	// we run it only every UI_STATE_MACHINE_INTERVAL ms
	static uint32_t last_usm = 0;
	if (millis() - last_usm <= UI_STATE_MACHINE_INTERVAL)
	{
		return;
	}
	last_usm = millis();

#if defined(ESP8266)
	// process screen led
	static ulong led_toggle_timeout = 0;
	if (led_blink_ms)
	{
		if (millis() > led_toggle_timeout)
		{
			os.toggle_screen_led();
			led_toggle_timeout = millis() + led_blink_ms;
		}
	}
#endif

	if (!os.button_timeout)
	{
		os.lcd_set_brightness(0);
		ui_state = UI_STATE_DEFAULT; // also recover to default state
	}

	// read button, if something is pressed, wait till release
	unsigned char button = os.button_read(BUTTON_WAIT_HOLD);

	if (button & BUTTON_FLAG_DOWN)
	{ // repond only to button down events
		os.button_timeout = LCD_BACKLIGHT_TIMEOUT;
		os.lcd_set_brightness(1);
	}
	else
	{
		return;
	}

	switch (ui_state)
	{
	case UI_STATE_DEFAULT:
		switch (button & BUTTON_MASK)
		{
		case BUTTON_1:
			if (button & BUTTON_FLAG_HOLD)
			{ // holding B1
				if (digitalReadExt(PIN_BUTTON_3) == 0)
				{ // if B3 is pressed while holding B1, run a short test (internal test)
					if (!ui_confirm(PSTR("Start 2s test?")))
					{
						ui_state = UI_STATE_DEFAULT;
						break;
					}
					manual_start_program(255, 0);
				}
				else if (digitalReadExt(PIN_BUTTON_2) == 0)
				{ // if B2 is pressed while holding B1, display gateway IP
					os.lcd.clear(0, 1);
					os.lcd.setCursor(0, 0);
#if defined(ESP8266)
					if (useEth)
					{
						os.lcd.print(eth.gatewayIP());
					}
					else
					{
						os.lcd.print(WiFi.gatewayIP());
					}
#else
					{
						os.lcd.print(Ethernet.gatewayIP());
					}
#endif
					os.lcd.setCursor(0, 1);
					os.lcd_print_pgm(PSTR("(gwip)"));
					ui_state = UI_STATE_DISP_IP;
				}
				else
				{ // if no other button is clicked, stop all zones
					if (!ui_confirm(PSTR("Stop all zones?")))
					{
						ui_state = UI_STATE_DEFAULT;
						break;
					}
					reset_all_stations();
				}
			}
			else
			{ // clicking B1: display device IP and port
				os.lcd.clear(0, 1);
				os.lcd.setCursor(0, 0);
#if defined(ESP8266)
				if (useEth)
				{
					os.lcd.print(eth.localIP());
				}
				else
				{
					os.lcd.print(WiFi.localIP());
				}
#else
				{
					os.lcd.print(Ethernet.localIP());
				}
#endif
				os.lcd.setCursor(0, 1);
				os.lcd_print_pgm(PSTR(":"));
				uint16_t httpport = (uint16_t)(os.iopts[IOPT_HTTPPORT_1] << 8) + (uint16_t)os.iopts[IOPT_HTTPPORT_0];
				os.lcd.print(httpport);
				os.lcd_print_pgm(PSTR(" (ip:port)"));
				ui_state = UI_STATE_DISP_IP;
			}
			break;
		case BUTTON_2:
			if (button & BUTTON_FLAG_HOLD)
			{ // holding B2
				if (digitalReadExt(PIN_BUTTON_1) == 0)
				{ // if B1 is pressed while holding B2, display external IP
					os.lcd_print_ip((unsigned char *)(&os.nvdata.external_ip), 1);
					os.lcd.setCursor(0, 1);
					os.lcd_print_pgm(PSTR("(eip)"));
					ui_state = UI_STATE_DISP_IP;
				}
				else if (digitalReadExt(PIN_BUTTON_3) == 0)
				{ // if B3 is pressed while holding B2, display last successful weather call
					// os.lcd.clear(0, 1);
					os.lcd_print_time(os.checkwt_success_lasttime);
					os.lcd.setCursor(0, 1);
					os.lcd_print_pgm(PSTR("(lswc)"));
					ui_state = UI_STATE_DISP_IP;
				}
				else
				{ // if no other button is clicked, reboot
					if (!ui_confirm(PSTR("Reboot device?")))
					{
						ui_state = UI_STATE_DEFAULT;
						break;
					}
					os.reboot_dev(REBOOT_CAUSE_BUTTON);
				}
			}
			else
			{ // clicking B2: display MAC
				os.lcd.clear(0, 1);
				unsigned char mac[6];
				os.load_hardware_mac(mac, useEth);
				os.lcd_print_mac(mac);
				ui_state = UI_STATE_DISP_GW;
			}
			break;
		case BUTTON_3:
			if (button & BUTTON_FLAG_HOLD)
			{ // holding B3
				if (digitalReadExt(PIN_BUTTON_1) == 0)
				{ // if B1 is pressed while holding B3, display up time
					os.lcd_print_time(os.powerup_lasttime);
					os.lcd.setCursor(0, 1);
					os.lcd_print_pgm(PSTR("(lupt) cause:"));
					os.lcd.print(os.last_reboot_cause);
					ui_state = UI_STATE_DISP_IP;
				}
				else if (digitalReadExt(PIN_BUTTON_2) == 0)
				{ // if B2 is pressed while holding B3, reset to AP and reboot
#if defined(ESP8266)
					if (!ui_confirm(PSTR("Reset to AP?")))
					{
						ui_state = UI_STATE_DEFAULT;
						break;
					}
					os.reset_to_ap();
#endif
				}
				else
				{ // if no other button is clicked, go to Run Program main menu
					os.lcd_print_line_clear_pgm(PSTR("Run a Program:"), 0);
					os.lcd_print_line_clear_pgm(PSTR("Click B3 to list"), 1);
					ui_state = UI_STATE_RUNPROG;
				}
			}
			else
			{ // clicking B3: switch board display (cycle through master and all extension boards)
				os.status.display_board = (os.status.display_board + 1) % (os.nboards);
			}
			break;
		}
		break;
	case UI_STATE_DISP_IP:
	case UI_STATE_DISP_GW:
		ui_state = UI_STATE_DEFAULT;
		break;
	case UI_STATE_RUNPROG:
		if ((button & BUTTON_MASK) == BUTTON_3)
		{
			if (button & BUTTON_FLAG_HOLD)
			{
				// start
				manual_start_program(ui_state_runprog, 0);
				ui_state = UI_STATE_DEFAULT;
			}
			else
			{
				ui_state_runprog = (ui_state_runprog + 1) % (pd.nprograms + 1);
				os.lcd_print_line_clear_pgm(PSTR("Hold B3 to start"), 0);
				if (ui_state_runprog > 0)
				{
					ProgramStruct prog;
					pd.read(ui_state_runprog - 1, &prog);
					os.lcd_print_line_clear_pgm(PSTR(" "), 1);
					os.lcd.setCursor(0, 1);
					os.lcd.print((int)ui_state_runprog);
					os.lcd_print_pgm(PSTR(". "));
					os.lcd.print(prog.name);
				}
				else
				{
					os.lcd_print_line_clear_pgm(PSTR("0. Test (1 min)"), 1);
				}
			}
		}
		break;
	}
}

// ======================
// Setup Function
// ======================
void do_setup()
{
	/* Clear WDT reset flag. */
#if defined(ESP8266)
	WiFi.persistent(false);
	led_blink_ms = LED_FAST_BLINK;
#else
	MCUSR &= ~(1 << WDRF);
#endif

	DEBUG_BEGIN(115200);
	DEBUG_PRINTLN(F("started"));

	os.begin();			// OpenSprinkler init
	os.options_setup(); // Setup options

	pd.init(); // ProgramData init

	// set time using RTC if it exists
	if (RTC.exists())
		setTime(RTC.get());
	os.lcd_print_time(os.now_tz()); // display time to LCD
	os.powerup_lasttime = os.now_tz();

#if defined(OS_AVR)
	// enable WDT
	/* In order to change WDE or the prescaler, we need to
	 * set WDCE (This will allow updates for 4 clock cycles).
	 */
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	/* set new watchdog timeout prescaler value */
	WDTCSR = 1 << WDP3 | 1 << WDP0; // 8.0 seconds
	/* Enable the WD interrupt (note no reset). */
	WDTCSR |= _BV(WDIE);
#endif
	if (os.start_network())
	{ // initialize network
		os.status.network_fails = 0;
	}
	else
	{
		os.status.network_fails = 1;
	}

	os.status.req_network = 0;
	os.status.req_ntpsync = 1;

	os.mqtt.init();
	os.status.req_mqtt_restart = true;

	os.apply_all_station_bits(); // reset station bits

	// because at reboot we don't know if special stations
	// are in OFF state, here we explicitly turn them off
	for (unsigned char sid = 0; sid < os.nstations; sid++)
	{
		os.switch_special_station(sid, 0);
	}

	os.button_timeout = LCD_BACKLIGHT_TIMEOUT;
}

// Arduino software reset function
void (*sysReset)(void) = 0;

#if defined(OS_AVR)
volatile unsigned char wdt_timeout = 0;
/** WDT interrupt service routine */
ISR(WDT_vect)
{
	wdt_timeout += 1;
	// this isr is called every 8 seconds
	if (wdt_timeout > 15)
	{
		// reset after 120 seconds of timeout
		sysReset();
	}
}
#endif

#else
void initalize_otf();

void do_setup()
{

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// JRJ v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v //
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// DEBUG_PRINT(PSTR("Executing as user: "));
	// DEBUG_PRINTLN(getpwuid(getuid())->pw_name);

	// logger_reset_state();
	// logger_set_log_level(LOG_MAX_LEVEL_ERROR_WARNING_INFO_DEBUG);  // LOG_MAX_LEVEL_ERROR 0, LOG_MAX_LEVEL_ERROR_WARNING_INFO 1, LOG_MAX_LEVEL_ERROR_WARNING_INFO_DEBUG 2

	// logger_set_log_file();

	//
	// <JRJ> 20100608 Added "spdlog" logging solution V1.8.5   See: https://github.com/gabime/spdlog
	// <JRJ> Logging levels (low to high): trace, debug, info, warn, error, critical, off
	//

	// <JRJ> periodically flush all *registered* loggers every 3 seconds:
	spdlog::flush_every(std::chrono::seconds(3));

	// <JRJ> get the full spdlog logging file
	static char full_log_file_Path[PATH_MAX];
	static bool query = true;
	memset(full_log_file_Path, '\0', sizeof(char) * PATH_MAX);
	strcpy(full_log_file_Path, get_runtime_path()); // e.g. /home/james/OpenSprinkler-Firmware/
	strcat(full_log_file_Path, "logs/OpenSprinklerLog.txt");
	DEBUG_PRINT("Log file path set to: ");
	DEBUG_PRINTLN(full_log_file_Path);

	// <JRJ> Create an array of sinks
	std::vector<spdlog::sink_ptr> sinks;

	// <JRJ> Create a coloured console log sink, minimun logging level = info and above (i.e. not trace or debug)
	sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
	sinks[0]->set_pattern("%Y/%m/%d %H:%M:%S [%^%l%$] %v");
	// <JRJ> The following is lowest level in the "levels" heirarchy [spdlog:: -> sink -> logger]
	// <JRJ> also see spdlog::set_level && sinks->set_level below as they can ovveride[increase only] this setting
	sinks[0]->set_level(spdlog::level::trace); // set to "level::trace" allow trace and debug messages to show up in the color console for DEBUGGING
	// sinks[0]->set_level(spdlog::level::info);  // set to "level::info" to block trace and debug messages from showing up in the color console

	// <JRJ> Create a daily log file sink, minimum looging level = trace and above (i.e. all levels)
	// daily_file_sink(filename_t base_filename, int rotation_hour, int rotation_minute, bool truncate = false, uint16_t max_files = 0)
	sinks.push_back(std::make_shared<spdlog::sinks::daily_file_sink_mt>(full_log_file_Path, 23, 59, false, 7));
	sinks[1]->set_pattern("%Y/%m/%d %H:%M:%S.%e [%l] %v");
	// <JRJ> set the following line to "level::trace" allow all trace and debug messages to show up in the logging file
	// <JRJ> also set multi_logger->set_level(spdlog::level::trace); below else the logging file shows only "info" and above
	sinks[1]->set_level(spdlog::level::trace);

	// <JRJ> Create the multi sink logger
	auto multi_logger = std::make_shared<spdlog::logger>("multi_logger", begin(sinks), end(sinks));
	multi_logger->set_level(spdlog::level::trace); // Do NOT change... actual level controlled by "sink->set_level above"

	// <JRJ> Register logger for global access
	spdlog::register_logger(multi_logger); // multi_logger is type: ‘shared_ptr<spdlog::logger>’

	/*
	  // <JRJ> newer coding than above... works just as well
	  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
	  console_sink->set_level(spdlog::level::trace);
	  console_sink->set_pattern("%Y/%m/%d %H:%M:%S [%^%l%$] %v");

	  //auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/multisink.txt", true);
	  auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(full_log_file_Path, 23, 59, false, 7);
	  file_sink->set_level(spdlog::level::trace);
	  file_sink->set_pattern("%Y/%m/%d %H:%M:%S.%e [%l] %v");

	  // <JRJ> Create the multi sink logger
	  spdlog::logger logger("multi_sink", {console_sink, file_sink});

	  // <JRJ> Create an array of sinks
	  std::vector<spdlog::sink_ptr> sinks2 {console_sink, file_sink};

	  // <JRJ> Create the multi sink logger
	  auto multi_sink = std::make_shared<spdlog::logger>("multi_sink", begin(sinks2), end(sinks2));
	  multi_sink->set_level(spdlog::level::trace);

	  // <JRJ> Register logger for global access
	  spdlog::register_logger(multi_sink); //<-- this line registers logger for spdlog::get
	*/

	// <JRJ> enable the following line to allow trace and debug messages to show up in the color console and logging file else they only show in the logging file
	// spdlog::set_level(spdlog::level::info);  // overrides logger->set_level and sink->set_level but only if > logger->set_level and sink->set_level levels
	spdlog::set_level(spdlog::level::trace); // Do NOT change... actual level controlled by "sink->set_level above"

	// <JRJ> Test global logger access
	auto spdlogFileLogger = spdlog::get("multi_logger");

	spdlogFileLogger->info("========================================================================================================================");
	spdlogFileLogger->info("      Started=>  {0} Version: {1}.{2} Complied: {3} {4}", __progname, OS_FW_VERSION, OS_FW_MINOR, __DATE__, __TIME__);
	spdlogFileLogger->info("  Host Name: {0} User Name: {1} Home Directory: {2} Startup Directory: {3}", get_host_name(), get_user_name(), get_home_directory(), get_runtime_path());
	// wiringPi is NOT used in 2.2.1.0... see GPIO.cpp instead
	// int wiringPiMajor = 0;
	// int wiringPiMinor = 0;
	// wiringPiVersion (&wiringPiMajor, &wiringPiMinor);
	// spdlogFileLogger->info("  Root Topic: {0}, Using spdlog V{1}.{2}.{3} and wiringPi V{4}.{5}", get_mqtt_root_topic(), SPDLOG_VER_MAJOR, SPDLOG_VER_MINOR, SPDLOG_VER_PATCH, wiringPiMajor, wiringPiMinor);
	spdlogFileLogger->info("  Root Topic: {0}, Using spdlog V{1}.{2}.{3}", get_mqtt_root_topic(), SPDLOG_VER_MAJOR, SPDLOG_VER_MINOR, SPDLOG_VER_PATCH);
	spdlogFileLogger->info("========================================================================================================================");

	// <JRJ> Ensure wiringPi library is initialized
	// wiringPiSetupGpio();
	// int wiringPiMajor = 0;
	// int wiringPiMinor = 0;
	// spdlogFileLogger->info("Main.cpp->do_setup() Initialized wiringPi Library V{0}.{1}", wiringPiMajor, wiringPiMinor);

	// <JRJ> Configure interrupts in WiringPi to capture the state changes of [5,6,12,13,19,16,26,20,  14,4,15,18,17,27,23,22,  21,24,25]
	////wiringPiISR (5, INT_EDGE_BOTH, &gpioInterrupt5B) ;
	// wiringPiISR (24, INT_EDGE_BOTH, &gpioInterrupt24B) ;
	// wiringPiISR (25, INT_EDGE_BOTH, &gpioInterrupt25B) ;
	// wiringPiISR (21, INT_EDGE_BOTH, &gpioInterrupt21B) ;
	// spdlogFileLogger->info("Main.cpp->do_setup() Setup interrupts on [5(B), 24(B), 25(B), 21(B)]"); // JRJ
	// spdlogFileLogger->info("Main.cpp->do_setup() Setup interrupts on [25(B), 21(B)]"); // JRJ
	// spdlogFileLogger->info("Main.cpp->do_setup() Setup interrupts on [24(B), 25(B), 21(B)]"); // JRJ
	spdlogFileLogger->info("Main.cpp->do_setup() Setup interrupts on [*NONE*]"); // JRJ

	/*   INIReader reader("test.ini");

	  if (reader.ParseError() < 0)
	  {
		spdlogFileLogger->error("Main.cpp->do_setup() Unable to load config file 'test.ini'");
	  }
	  else
	  {
		// std::cout << "Config loaded from 'test.ini': version="
		// << reader.GetInteger("protocol", "version", -1) << ", name="
		// << reader.Get("user", "name", "UNKNOWN") << ", email="
		// << reader.Get("user", "email", "UNKNOWN") << ", pi="
		// << reader.GetReal("user", "pi", -1) << ", active="
		// << reader.GetBoolean("user", "active", true) << "\n";

		spdlogFileLogger->info("Main.cpp->do_setup() Config loaded from 'test.ini': version={0} name={1} email={2} pi={3} active={4}", reader.GetInteger("protocol", "version", -1), reader.Get("user", "name", "UNKNOWN"), reader.Get("user", "email", "UNKNOWN"), reader.GetReal("user", "pi", -1), reader.GetBoolean("user", "active", true));
	  } */

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// JRJ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ //
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	initialiseEpoch();	// initialize time reference for millis() and micros()
	os.begin();			// OpenSprinkler init
	os.options_setup(); // Setup options

	pd.init(); // ProgramData init

	if (os.start_network())
	{																		 // initialize network
																			 // DEBUG_PRINTLN("network established.");
		spdlogFileLogger->info("Main.cpp->do_setup() Network started okay"); // JRJ
		os.status.network_fails = 0;
	}
	else
	{
		// DEBUG_PRINTLN("network failed.");
		spdlogFileLogger->error("Main.cpp->do_setup() Network failed to start"); // JRJ
		os.status.network_fails = 1;
	}
	os.status.req_network = 0;

	// because at reboot we don't know if special stations are in OFF state, here we explicitly turn them off
	for (unsigned char sid = 0; sid < os.nstations; sid++)
	{
		os.switch_special_station(sid, 0);
	}

	// os.mqtt.init();
	// os.status.req_mqtt_restart = true;
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// JRJ v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v //
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// <JRJ> Check the network is ready for mqtt connectivity
	if (!os.network_connected())
		spdlogFileLogger->warn("Main.cpp->do_setup() Network NOT READY!"); // JRJ
	else
		spdlogFileLogger->info("Main.cpp->do_setup() Network is ready"); // JRJ

	// <JRJ> Ensure MQTT has started ASAP to ensure the following MQTT status updates get posted ok
	os.mqtt.init();
	os.status.req_mqtt_restart = true;
	os.mqtt.begin();
	os.status.req_mqtt_restart = false;
	os.mqtt.loop();
	spdlogFileLogger->info("Main.cpp->do_setup() MQTT started okay");

	// <JRJ> Check the network is ready for mqtt connectivity
	if (!os.mqtt.is_connected())
		spdlogFileLogger->warn("Main.cpp->do_setup() MQTT NOT READY!"); // JRJ
	else
		spdlogFileLogger->info("Main.cpp->do_setup() MQTT is ready"); // JRJ

	// <JRJ> Force a weather update
	os.checkwt_lasttime = 0;
	os.checkwt_success_lasttime = 0;

	// OSProgramCollection.Dump( cout );
	// OSProgramCollection.DumpMap( cout );

	// <JRJ> Send a NOTIFY_STATION_OFF for each Station
	for (unsigned char sid = 0; sid < os.nstations; sid++)
	{
		// Send a station off mqtt post
		push_message(NOTIFY_STATION_OFF, sid, 0.0);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// JRJ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ //
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	initalize_otf();

	spdlogFileLogger->info("Main.cpp->do_setup() Setup completed ok!");
}
#endif

void turn_on_station(unsigned char sid, ulong duration);
static void check_network();
void check_weather();
static bool process_special_program_command(const char *, uint32_t curr_time);
static void perform_ntp_sync();

#if defined(ESP8266)
bool delete_log_oldest();
void start_server_ap();
void start_server_client();
static Ticker reboot_ticker;
void reboot_in(uint32_t ms)
{
	if (os.state != OS_STATE_WAIT_REBOOT)
	{
		os.state = OS_STATE_WAIT_REBOOT;
		DEBUG_PRINTLN(F("Prepare to restart..."));
		reboot_ticker.once_ms(ms, ESP.restart);
	}
}
#else
void handle_web_request(char *p);
#endif

/** Main Loop */
void do_loop()
{
	// <JRJ> Aquire a pointer to the spdlog Logger
	auto spdlogFileLogger = spdlog::get("multi_logger");

	// handle flow sensor using polling every 1ms (maximum freq 1/(2*1ms)=500Hz)
	static ulong flowpoll_timeout = 0;
	if (os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_FLOW)
	{
		ulong curr = millis();
		if (curr != flowpoll_timeout)
		{
			flowpoll_timeout = curr;
			flow_poll();
		}
	}

	static time_os_t last_time = 0;
	static ulong last_minute = 0;

	unsigned char bid, sid, s, pid, qid, gid, bitvalue;
	ProgramStruct prog;

	os.status.mas = os.iopts[IOPT_MASTER_STATION];
	os.status.mas2 = os.iopts[IOPT_MASTER_STATION_2];
	time_os_t curr_time = os.now_tz();

	// ====== Process Ethernet packets ======
#if defined(ARDUINO) // Process Ethernet packets for Arduino
#if defined(ESP8266)
	static ulong connecting_timeout;
	switch (os.state)
	{
	case OS_STATE_INITIAL:
		if (useEth)
		{
			led_blink_ms = 0;
			os.set_screen_led(LOW);
			os.lcd.clear();
			os.save_wifi_ip();
			start_server_client();
			os.state = OS_STATE_CONNECTED;
			connecting_timeout = 0;
		}
		else if (os.get_wifi_mode() == WIFI_MODE_AP)
		{
			start_server_ap();
			dns->setErrorReplyCode(DNSReplyCode::NoError);
			dns->start(53, "*", WiFi.softAPIP());
			os.state = OS_STATE_CONNECTED;
			connecting_timeout = 0;
		}
		else
		{
			led_blink_ms = LED_SLOW_BLINK;
			if (os.sopt_load(SOPT_STA_BSSID_CHL).length() > 0 && os.wifi_channel < 255)
			{
				start_network_sta(os.wifi_ssid.c_str(), os.wifi_pass.c_str(), (int32_t)os.wifi_channel, os.wifi_bssid);
			}
			else
				start_network_sta(os.wifi_ssid.c_str(), os.wifi_pass.c_str());
			os.config_ip();
			os.state = OS_STATE_CONNECTING;
			connecting_timeout = millis() + 120000L;
			os.lcd.setCursor(0, -1);
			os.lcd.print(F("Connecting to..."));
			os.lcd.setCursor(0, 2);
			os.lcd.print(os.wifi_ssid);
		}
		break;

	case OS_STATE_TRY_CONNECT:
		led_blink_ms = LED_SLOW_BLINK;
		if (os.sopt_load(SOPT_STA_BSSID_CHL).length() > 0 && os.wifi_channel < 255)
		{
			start_network_sta_with_ap(os.wifi_ssid.c_str(), os.wifi_pass.c_str(), (int32_t)os.wifi_channel, os.wifi_bssid);
		}
		else
			start_network_sta_with_ap(os.wifi_ssid.c_str(), os.wifi_pass.c_str());
		os.config_ip();
		os.state = OS_STATE_CONNECTED;
		break;

	case OS_STATE_CONNECTING:
		if (WiFi.status() == WL_CONNECTED)
		{
			led_blink_ms = 0;
			os.set_screen_led(LOW);
			os.lcd.clear();
			os.save_wifi_ip();
			start_server_client();
			os.state = OS_STATE_CONNECTED;
			connecting_timeout = 0;
		}
		else
		{
			if (millis() > connecting_timeout)
			{
				os.state = OS_STATE_INITIAL;
				WiFi.disconnect(true);
				DEBUG_PRINTLN(F("timeout"));
			}
		}
		break;

	case OS_STATE_WAIT_REBOOT:
		if (dns)
			dns->processNextRequest();
		if (otf)
			otf->loop();
		if (update_server)
			update_server->handleClient();
		break;

	case OS_STATE_CONNECTED:
		if (os.get_wifi_mode() == WIFI_MODE_AP)
		{
			dns->processNextRequest();
			update_server->handleClient();
			otf->loop();
			connecting_timeout = 0;
			if (os.get_wifi_mode() == WIFI_MODE_STA)
			{
				// already in STA mode, waiting to reboot
				break;
			}
			if (WiFi.status() == WL_CONNECTED && WiFi.localIP() && reboot_timer != 0)
			{
				DEBUG_PRINTLN(F("STA connected, set up reboot timer"));
				reboot_timer = os.now_tz() + 10;
				// os.reboot_dev(REBOOT_CAUSE_WIFIDONE);
			}
		}
		else
		{
			if (useEth || WiFi.status() == WL_CONNECTED)
			{
				update_server->handleClient();
				otf->loop();
				connecting_timeout = 0;
			}
			else
			{
				// todo: better handling of WiFi disconnection
				DEBUG_PRINTLN(F("WiFi disconnected, going back to initial"));
				os.state = OS_STATE_INITIAL;
				WiFi.disconnect(true);
			}
		}
		break;
	}

#else // AVR

	static unsigned long dhcp_timeout = 0;
	if (curr_time > dhcp_timeout)
	{
		Ethernet.maintain();
		dhcp_timeout = curr_time + DHCP_CHECKLEASE_INTERVAL;
	}
	EthernetClient client = m_server->available();
	if (client)
	{
		ulong cli_timeout = now() + CLIENT_READ_TIMEOUT;
		while (client.connected() && now() < cli_timeout)
		{
			size_t size = client.available();
			if (size > 0)
			{
				if (size > ETHER_BUFFER_SIZE)
					size = ETHER_BUFFER_SIZE;
				int len = client.read((uint8_t *)ether_buffer, size);
				if (len > 0)
				{
					m_client = &client;
					ether_buffer[len] = 0; // properly end the buffer
					handle_web_request(ether_buffer);
					m_client = NULL;
					break;
				}
			}
		}
		client.stop();
	}

	wdt_reset(); // reset watchdog timer
	wdt_timeout = 0;

#endif

	ui_state_machine();

#else  // Process Ethernet packets for RPI/BBB
	if (otf)
		otf->loop();
#endif // Process Ethernet packets

	// Start up MQTT when we have a network connection
	if (os.status.req_mqtt_restart && os.network_connected())
	{
		DEBUG_PRINTLN(F("req_mqtt_restart"));
		os.mqtt.begin();
		os.status.req_mqtt_restart = false;
		os.mqtt.subscribe();
	}
	os.mqtt.loop();

	// The main control loop runs once every second
	if (curr_time != last_time)
	{
		// <JRJ> Send a app status message to the console/log file
		time_t now;
		time(&now);
		struct tm localTime;
		localtime_r(&now, &localTime);
		if ((localTime.tm_min == 0) && (localTime.tm_sec == 0))
		{
			spdlogFileLogger->info("--------------------------------------------------------------------------------------------------------------------");
      spdlogFileLogger->info("      Started=>  {0} Version: {1}.{2} Complied: {3} {4}", __progname, OS_FW_VERSION, OS_FW_MINOR, __DATE__, __TIME__);
			spdlogFileLogger->info("  Host Name: {0} User Name: {1} Home Directory: {2} Startup Directory: {3}", get_host_name(), get_user_name(), get_home_directory(), get_runtime_path());
			spdlogFileLogger->info("                              Root Topic: {0}", get_mqtt_root_topic());
			spdlogFileLogger->info("--------------------------------------------------------------------------------------------------------------------");
		}

#if defined(ESP8266)
		if (os.hw_rev >= 2)
		{
			pinModeExt(PIN_SENSOR1, INPUT_PULLUP); // this seems necessary for OS 3.2
			pinModeExt(PIN_SENSOR2, INPUT_PULLUP);
		}
#endif

		last_time = curr_time;
		if (os.button_timeout)
			os.button_timeout--;

#if defined(ARDUINO)
		if (!ui_state)
			os.lcd_print_time(curr_time); // print time
#endif

		// ====== Check raindelay status ======
		if (os.status.rain_delayed)
		{
			if (curr_time >= os.nvdata.rd_stop_time)
			{ // rain delay is over
				os.raindelay_stop();
			}
		}
		else
		{
			if (os.nvdata.rd_stop_time > curr_time)
			{ // rain delay starts now
				os.raindelay_start();
			}
		}

		// ====== Check controller status changes and write log ======
		if (os.old_status.rain_delayed != os.status.rain_delayed)
		{
			if (os.status.rain_delayed)
			{
				// rain delay started, record time
				os.raindelay_on_lasttime = curr_time;
				push_message(NOTIFY_RAINDELAY, LOGDATA_RAINDELAY, 1);
			}
			else
			{
				// rain delay stopped, write log
				write_log(LOGDATA_RAINDELAY, curr_time);
				push_message(NOTIFY_RAINDELAY, LOGDATA_RAINDELAY, 0);
			}
			os.old_status.rain_delayed = os.status.rain_delayed;
		}

		// ====== Check binary (i.e. rain or soil) sensor status ======
		os.detect_binarysensor_status(curr_time);

		if (os.old_status.sensor1_active != os.status.sensor1_active)
		{
			// send notification when sensor1 becomes active
			if (os.status.sensor1_active)
			{
				os.sensor1_active_lasttime = curr_time;
				push_message(NOTIFY_SENSOR1, LOGDATA_SENSOR1, 1);
			}
			else
			{
				write_log(LOGDATA_SENSOR1, curr_time);
				push_message(NOTIFY_SENSOR1, LOGDATA_SENSOR1, 0);
			}
		}
		os.old_status.sensor1_active = os.status.sensor1_active;

		if (os.old_status.sensor2_active != os.status.sensor2_active)
		{
			// send notification when sensor1 becomes active
			if (os.status.sensor2_active)
			{
				os.sensor2_active_lasttime = curr_time;
				push_message(NOTIFY_SENSOR2, LOGDATA_SENSOR2, 1);
			}
			else
			{
				write_log(LOGDATA_SENSOR2, curr_time);
				push_message(NOTIFY_SENSOR2, LOGDATA_SENSOR2, 0);
			}
		}
		os.old_status.sensor2_active = os.status.sensor2_active;

		// ===== Check program switch status =====
		unsigned char pswitch = os.detect_programswitch_status(curr_time);
		if (pswitch > 0)
		{
			reset_all_stations_immediate(); // immediately stop all stations
		}
		if (pswitch & 0x01)
		{
			if (pd.nprograms > 0)
				manual_start_program(1, 0);
		}
		if (pswitch & 0x02)
		{
			if (pd.nprograms > 1)
				manual_start_program(2, 0);
		}

		// ====== Schedule program data ======
		ulong curr_minute = curr_time / 60;
		boolean match_found = false;
		RuntimeQueueStruct *q;
		// since the granularity of start time is minute
		// we only need to check once every minute
		if (curr_minute != last_minute)
		{
			last_minute = curr_minute;

			apply_monthly_adjustment(curr_time); // check and apply monthly adjustment here, if it's selected

			// check through all programs
			for (pid = 0; pid < pd.nprograms; pid++)
			{
				pd.read(pid, &prog); // todo future: reduce load time
				if (prog.check_match(curr_time))
				{
					// program match found
					// check and process special program command
					if (process_special_program_command(prog.name, curr_time))
						continue;

					// process all selected stations
					for (sid = 0; sid < os.nstations; sid++)
					{
						bid = sid >> 3;
						s = sid & 0x07;
						// skip if the station is a master station (because master cannot be scheduled independently
						if ((os.status.mas == sid + 1) || (os.status.mas2 == sid + 1))
							continue;

						// if station has non-zero water time and the station is not disabled
						if (prog.durations[sid] && !(os.attrib_dis[bid] & (1 << s)))
						{
							// water time is scaled by watering percentage
							ulong water_time = water_time_resolve(prog.durations[sid]);
							// if the program is set to use weather scaling
							if (prog.use_weather)
							{
								unsigned char wl = os.iopts[IOPT_WATER_PERCENTAGE];
								water_time = water_time * wl / 100;
								if (wl < 20 && water_time < 10) // if water_percentage is less than 20% and water_time is less than 10 seconds
																// do not water
									water_time = 0;
							}

							if (water_time)
							{
								// check if water time is still valid
								// because it may end up being zero after scaling
								q = pd.enqueue();
								if (q)
								{
									q->st = 0;
									q->dur = water_time;
									q->sid = sid;
									q->pid = pid + 1;
									match_found = true;
								}
								else
								{
									// queue is full
								}
							} // if water_time
						} // if prog.durations[sid]
					} // for sid
					if (match_found)
					{
						push_message(NOTIFY_PROGRAM_SCHED, pid, prog.use_weather ? os.iopts[IOPT_WATER_PERCENTAGE] : 100);
					}
				} // if check_match
			} // for pid

			// calculate start and end time
			if (match_found)
			{
				schedule_all_stations(curr_time);

				// For debugging: print out queued elements
				/*DEBUG_PRINT("en:");
				for(q=pd.queue;q<pd.queue+pd.nqueue;q++) {
					DEBUG_PRINT("[");
					DEBUG_PRINT(q->sid);
					DEBUG_PRINT(",");
					DEBUG_PRINT(q->dur);
					DEBUG_PRINT(",");
					DEBUG_PRINT(q->st);
					DEBUG_PRINT("]");
				}
				DEBUG_PRINTLN("");*/
			}
		} // if_check_current_minute

		// ====== Run program data ======
		// Check if a program is running currently
		// If so, do station run-time keeping
		if (os.status.program_busy)
		{
			// first, go through run time queue to assign queue elements to stations
			q = pd.queue;
			qid = 0;
			for (; q < pd.queue + pd.nqueue; q++, qid++)
			{
				sid = q->sid;
				unsigned char sqi = pd.station_qid[sid];
				// skip if station is already assigned a queue element
				// and that queue element has an earlier start time
				if (sqi < 255 && pd.queue[sqi].st < q->st)
					continue;
				// otherwise assign the queue element to station
				pd.station_qid[sid] = qid;
			}
			// next, go through the stations and perform time keeping
			for (bid = 0; bid < os.nboards; bid++)
			{
				bitvalue = os.station_bits[bid];
				for (s = 0; s < 8; s++)
				{
					unsigned char sid = bid * 8 + s;

					// skip master stations and any station that's not in the queue
					if (os.status.mas == sid + 1)
						continue;
					if (os.status.mas2 == sid + 1)
						continue;
					if (pd.station_qid[sid] == 255)
						continue;

					q = pd.queue + pd.station_qid[sid];

					// if current station is not running, check if we should turn it on
					if (!((bitvalue >> s) & 1))
					{
						if (curr_time >= q->st && curr_time < q->st + q->dur)
						{
							turn_on_station(sid, q->st + q->dur - curr_time); // the last parameter is expected run time
						} // if curr_time > scheduled_start_time
					} // if current station is not running

					// check if this station should be turned off
					if (q->st > 0)
					{
						if (curr_time >= q->st + q->dur)
						{
							turn_off_station(sid, curr_time);
						}
					}
				} // end_s
			} // end_bid

			// finally, go through the queue again and clear up elements marked for removal
			int qi;
			for (qi = pd.nqueue - 1; qi >= 0; qi--)
			{
				q = pd.queue + qi;
				if (!q->dur || curr_time >= q->deque_time)
				{
					pd.dequeue(qi);
				}
			}

			// process dynamic events
			process_dynamic_events(curr_time);

			// activate / deactivate valves
			os.apply_all_station_bits();

			// check through runtime queue, calculate the last stop time of sequential stations
			memset(pd.last_seq_stop_times, 0, sizeof(ulong) * NUM_SEQ_GROUPS);
			time_os_t sst;
			unsigned char re = os.iopts[IOPT_REMOTE_EXT_MODE];
			q = pd.queue;
			for (; q < pd.queue + pd.nqueue; q++)
			{
				sid = q->sid;
				bid = sid >> 3;
				s = sid & 0x07;
				gid = os.get_station_gid(sid);
				// check if any sequential station has a valid stop time
				// and the stop time must be larger than curr_time
				sst = q->st + q->dur;
				if (sst > curr_time)
				{
					// only need to update last_seq_stop_time for sequential stations
					if (os.is_sequential_station(sid) && !re)
					{
						pd.last_seq_stop_times[gid] = (sst > pd.last_seq_stop_times[gid]) ? sst : pd.last_seq_stop_times[gid];
					}
				}
			}

			// if the runtime queue is empty
			// reset all stations
			if (!pd.nqueue)
			{
				// turn off all stations
				os.clear_all_station_bits();
				os.apply_all_station_bits();
				pd.reset_runtime();			// reset runtime
				os.status.program_busy = 0; // reset program busy bit
				pd.clear_pause();			// TODO: what if pause hasn't expired and a new program is scheduled to run?

				// log flow sensor reading if flow sensor is used
				if (os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_FLOW)
				{
					write_log(LOGDATA_FLOWSENSE, curr_time);
					push_message(NOTIFY_FLOWSENSOR, (flow_count > os.flowcount_log_start) ? (flow_count - os.flowcount_log_start) : 0);
				}

				// in case some options have changed while executing the program
				os.status.mas = os.iopts[IOPT_MASTER_STATION];	  // update master station
				os.status.mas2 = os.iopts[IOPT_MASTER_STATION_2]; // update master2 station
			}
		} // if_some_program_is_running

		// handle master
		for (unsigned char mas = MASTER_1; mas < NUM_MASTER_ZONES; mas++)
		{

			unsigned char mas_id = os.masters[mas][MASOPT_SID];

			if (mas_id)
			{ // if this master station is set
				int16_t mas_on_adj = os.get_on_adj(mas);
				int16_t mas_off_adj = os.get_off_adj(mas);

				unsigned char masbit = 0;

				for (sid = 0; sid < os.nstations; sid++)
				{
					// skip if this is the master station
					if (mas_id == sid + 1)
						continue;

					if (pd.station_qid[sid] == 255)
						continue; // skip if station is not in the queue

					q = pd.queue + pd.station_qid[sid];

					if (os.bound_to_master(q->sid, mas))
					{
						// check if timing is within the acceptable range
						if (curr_time >= q->st + mas_on_adj &&
							curr_time <= q->st + q->dur + mas_off_adj)
						{
							masbit = 1;
							break;
						}
					}
				}

				if (os.get_station_bit(mas_id - 1) == 0 && masbit == 1)
				{ // notify master on event
					push_message(NOTIFY_STATION_ON, mas_id - 1, 0);
				}

				os.set_station_bit(mas_id - 1, masbit);
			}
		}

		if (os.status.pause_state)
		{
			if (os.pause_timer > 0)
			{
				os.pause_timer--;
			}
			else
			{
				os.clear_all_station_bits();
				pd.clear_pause();
			}
		}
		// process dynamic events
		process_dynamic_events(curr_time);

		// activate/deactivate valves
		os.apply_all_station_bits();

#if defined(ARDUINO)
		// process LCD display
		if (!ui_state)
		{
			os.lcd_print_screen(ui_anim_chars[(unsigned long)curr_time % 3]);
		}

#endif

		// handle reboot request
		// check safe_reboot condition
		if (os.status.safe_reboot && (curr_time > reboot_timer))
		{
			// if no program is running at the moment
			if (!os.status.program_busy)
			{
				// and if no program is scheduled to run in the next minute
				bool willrun = false;
				for (pid = 0; pid < pd.nprograms; pid++)
				{
					pd.read(pid, &prog);
					if (prog.check_match(curr_time + 60))
					{
						willrun = true;
						break;
					}
				}
				if (!willrun)
				{
					os.reboot_dev(os.nvdata.reboot_cause);
				}
			}
		}
		else if (reboot_timer && (curr_time > reboot_timer))
		{
			os.reboot_dev(REBOOT_CAUSE_TIMER);
		}

		// real-time flow count
		static ulong flowcount_rt_start = 0;
		if (os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_FLOW)
		{
			if (curr_time % FLOWCOUNT_RT_WINDOW == 0)
			{
				os.flowcount_rt = (flow_count > flowcount_rt_start) ? flow_count - flowcount_rt_start : 0;
				flowcount_rt_start = flow_count;
			}
		}

		// perform ntp sync
		// instead of using curr_time, which may change due to NTP sync itself
		// we use Arduino's millis() method
		if (curr_time % NTP_SYNC_INTERVAL == 0)
			os.status.req_ntpsync = 1;
		// if((millis()/1000) % NTP_SYNC_INTERVAL==15) os.status.req_ntpsync = 1;
		perform_ntp_sync();

		// check network connection
		if (curr_time && (curr_time % CHECK_NETWORK_INTERVAL == 0))
			os.status.req_network = 1;
		check_network();

		// check weather
		check_weather();

		if (os.weather_update_flag & WEATHER_UPDATE_WL)
		{
			// at the moment, we only send notification if water level changed
			// the other changes, such as sunrise, sunset changes are ignored for notification
			push_message(NOTIFY_WEATHER_UPDATE, 0, os.iopts[IOPT_WATER_PERCENTAGE]);
			os.weather_update_flag = 0;
		}
		static unsigned char reboot_notification = 1;
		if (reboot_notification)
		{
#if defined(ESP266)
			if (useEth || WiFi.status() == WL_CONNECTED)
#endif
			{
				reboot_notification = 0;
				push_message(NOTIFY_REBOOT);
			}
		}
	}

#if !defined(ARDUINO)
	delay(1); // For OSPI/OSBO/LINUX, sleep 1 ms to minimize CPU usage
#endif
}

/** Check and process special program command */
static bool process_special_program_command(const char *pname, uint32_t curr_time)
{
	if (pname[0] == ':')
	{ // special command start with :
		if (strncmp(pname, ":>reboot_now", 12) == 0)
		{
			os.status.safe_reboot = 0;	   // reboot regardless of program status
			reboot_timer = curr_time + 65; // set a timer to reboot in 65 seconds
			// this is to avoid the same command being executed again right after reboot
			return true;
		}
		else if (strncmp(pname, ":>reboot", 8) == 0)
		{
			os.status.safe_reboot = 1;	   // by default reboot should only happen when controller is idle
			reboot_timer = curr_time + 65; // set a timer to reboot in 65 seconds
			// this is to avoid the same command being executed again right after reboot
			return true;
		}
	}
	return false;
}

/** Make weather query */
void check_weather()
{
	// <JRJ> Aquire a pointer to the spdlog Logger
	auto spdlogFileLogger = spdlog::get("multi_logger");
	// do not check weather if
	// - network check has failed, or
	// - the controller is in remote extension mode
	if (os.status.network_fails > 0 || os.iopts[IOPT_REMOTE_EXT_MODE])
		return;
	if (os.status.program_busy)
		return;

#if defined(ESP8266)
	if (!useEth)
	{ // todo: what about useEth==true?
		if (os.get_wifi_mode() != WIFI_MODE_STA || WiFi.status() != WL_CONNECTED || os.state != OS_STATE_CONNECTED)
			return;
	}
#endif

	time_os_t ntz = os.now_tz();
	if (os.checkwt_success_lasttime && (ntz > os.checkwt_success_lasttime + CHECK_WEATHER_SUCCESS_TIMEOUT))
	{
		// if last successful weather call timestamp is more than allowed threshold
		// and if the selected adjustment method is not one of the manual methods
		// reset watering percentage to 100
		// todo: the firmware currently needs to be explicitly aware of which adjustment methods, this is not ideal
		os.checkwt_success_lasttime = 0;
		unsigned char method = os.iopts[IOPT_USE_WEATHER];
		if (!(method == WEATHER_METHOD_MANUAL || method == WEATHER_METHOD_AUTORAINDELY || method == WEATHER_METHOD_MONTHLY))
		{
			os.iopts[IOPT_WATER_PERCENTAGE] = 100; // reset watering percentage to 100%
			wt_rawData[0] = 0;					   // reset wt_rawData and errCode
			wt_errCode = HTTP_RQT_NOT_RECEIVED;
		}
	}
	else if (!os.checkwt_lasttime || (ntz > os.checkwt_lasttime + CHECK_WEATHER_TIMEOUT))
	{
		os.checkwt_lasttime = ntz;
#if defined(ARDUINO)
		if (!ui_state)
		{
			os.lcd_print_line_clear_pgm(PSTR("Check Weather..."), 1);
		}
#endif
		GetWeather();
		spdlogFileLogger->info("Main::check_weather() water_level: {0}", os.iopts[IOPT_WATER_PERCENTAGE]); // JRJ
	}
}

/** Turn on a station
 * This function turns on a scheduled station
 */
void turn_on_station(unsigned char sid, ulong duration)
{
	// RAH implementation of flow sensor
	flow_start = 0;

	if (os.set_station_bit(sid, 1, duration))
	{
		push_message(NOTIFY_STATION_ON, sid, duration);
	}
}

// after removing element q, update remaining stations in its group
void handle_shift_remaining_stations(RuntimeQueueStruct *q, unsigned char gid, time_os_t curr_time)
{
	RuntimeQueueStruct *s = pd.queue;
	time_os_t q_end_time = q->st + q->dur;
	ulong remainder = 0;

	if (q_end_time > curr_time)
	{ // remainder is non-zero
		remainder = (q->st < curr_time) ? q_end_time - curr_time : q->dur;
		for (; s < pd.queue + pd.nqueue; s++)
		{

			// ignore station to be removed and stations in other groups
			if (s == q || os.get_station_gid(s->sid) != gid || !os.is_sequential_station(s->sid))
			{
				continue;
			}

			// only shift stations following current station
			if (s->st >= q_end_time)
			{
				s->st -= remainder;
				s->deque_time -= remainder;
			}
		}
	}
	pd.last_seq_stop_times[gid] -= remainder;
	pd.last_seq_stop_times[gid] += 1;
}

/** Turn off a station
 * This function turns off a scheduled station
 * writes a log record and determines if
 * the station should be removed from the queue
 */
void turn_off_station(unsigned char sid, time_os_t curr_time, unsigned char shift)
{

	unsigned char qid = pd.station_qid[sid];
	// ignore request if trying to turn off a zone that's not even in the queue
	if (qid >= pd.nqueue)
	{
		return;
	}
	RuntimeQueueStruct *q = pd.queue + qid;
	unsigned char force_dequeue = 0;
	unsigned char station_bit = os.is_running(sid);
	unsigned char gid = os.get_station_gid(q->sid);

	if (shift && os.is_sequential_station(sid) && !os.iopts[IOPT_REMOTE_EXT_MODE])
	{
		handle_shift_remaining_stations(q, gid, curr_time);
	}

	if (curr_time >= q->deque_time)
	{
		if (station_bit)
		{
			force_dequeue = 1;
		}
		else
		{ // if already off just remove from the queue
			pd.dequeue(qid);
			pd.station_qid[sid] = 0xFF;
			return;
		}
	}
	else if (curr_time >= q->st + q->dur)
	{ // end time and dequeue time are not equal due to master handling
		if (!station_bit)
		{
			return;
		}
	} // else { return; }

	os.set_station_bit(sid, 0);

	// RAH implementation of flow sensor
	if (flow_gallons > 1)
	{
		if (flow_stop <= flow_begin)
			flow_last_gpm = 0;
		else
			flow_last_gpm = (float)60000 / (float)((flow_stop - flow_begin) / (flow_gallons - 1));
	} // RAH calculate GPM, 1 pulse per gallon
	else
	{
		flow_last_gpm = 0;
	} // RAH if not one gallon (two pulses) measured then record 0 gpm

	// check if the current time is past the scheduled start time,
	// because we may be turning off a station that hasn't started yet
	if (curr_time >= q->st)
	{
		// record lastrun log (only for non-master stations)
		if (os.status.mas != (sid + 1) && os.status.mas2 != (sid + 1))
		{
			pd.lastrun.station = sid;
			pd.lastrun.program = q->pid;
			pd.lastrun.duration = curr_time - q->st;
			pd.lastrun.endtime = curr_time;

			// log station run
			write_log(LOGDATA_STATION, curr_time); // LOG_TODO
			push_message(NOTIFY_STATION_OFF, sid, pd.lastrun.duration);
		}
	}

	// make necessary adjustments to sequential time stamps
	int16_t station_delay = water_time_decode_signed(os.iopts[IOPT_STATION_DELAY_TIME]);
	if (q->st + q->dur + station_delay == pd.last_seq_stop_times[gid])
	{ // if removing last station in group
		pd.last_seq_stop_times[gid] = 0;
	}

	if (force_dequeue)
	{
		pd.dequeue(qid);
		pd.station_qid[sid] = 0xFF;
	}
}

/** Process dynamic events
 * such as rain delay, rain sensing
 * and turn off stations accordingly
 */
void process_dynamic_events(time_os_t curr_time)
{
	// <JRJ> Aquire a pointer to the spdlog Logger
	auto spdlogFileLogger = spdlog::get("multi_logger");
	// check if rain is detected
	bool sn1 = false;
	bool sn2 = false;
	bool rd = os.status.rain_delayed;
	bool en = os.status.enabled;

	if ((os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_RAIN || os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_SOIL) && os.status.sensor1_active)
		sn1 = true;

	if ((os.iopts[IOPT_SENSOR2_TYPE] == SENSOR_TYPE_RAIN || os.iopts[IOPT_SENSOR2_TYPE] == SENSOR_TYPE_SOIL) && os.status.sensor2_active)
		sn2 = true;

	// todo: handle sensor 2
	unsigned char sid, s, bid, qid, igs, igs2, igrd;
	for (bid = 0; bid < os.nboards; bid++)
	{
		igs = os.attrib_igs[bid];
		igs2 = os.attrib_igs2[bid];
		igrd = os.attrib_igrd[bid];

		for (s = 0; s < 8; s++)
		{
			sid = bid * 8 + s;

			// ignore master stations because they are handled separately
			if (os.status.mas == sid + 1)
				continue;
			if (os.status.mas2 == sid + 1)
				continue;
			// If this is a normal program (not a run-once or test program)
			// and either the controller is disabled, or
			// if raining and ignore rain bit is cleared
			// FIX ME
			qid = pd.station_qid[sid];
			if (qid == 255)
				continue;
			RuntimeQueueStruct *q = pd.queue + qid;

			if (q->pid >= 99)
				continue; // if this is a manually started program, proceed
			if (!en)
			{
				spdlogFileLogger->debug("Main::process_dynamic_events() [if system is disabled, turn off zone] sid: {0} pid: {1}", sid, q->pid); // JRJ
				q->deque_time = curr_time;
				turn_off_station(sid, curr_time);
			} // if system is disabled, turn off zone
			if (rd && !(igrd & (1 << s)))
			{
				spdlogFileLogger->debug("Main::process_dynamic_events() [if rain delay is on and zone does not ignore rain delay, turn it off] sid: {0} pid: {1}", sid, q->pid); // JRJ

				q->deque_time = curr_time;
				turn_off_station(sid, curr_time);
			} // if rain delay is on and zone does not ignore rain delay, turn it off
			if (sn1 && !(igs & (1 << s)))
			{
				spdlogFileLogger->debug("Main::process_dynamic_events() [if sensor1 is on and zone does not ignore sensor1, turn it off] sid: {0} pid: {1}", sid, q->pid); // JRJ

				q->deque_time = curr_time;
				turn_off_station(sid, curr_time);
			} // if sensor1 is on and zone does not ignore sensor1, turn it off
			if (sn2 && !(igs2 & (1 << s)))
			{
				spdlogFileLogger->debug("Main::process_dynamic_events() [if sensor2 is on and zone does not ignore sensor2, turn it off] sid: {0} pid: {1}", sid, q->pid); // JRJ

				q->deque_time = curr_time;
				turn_off_station(sid, curr_time);
			} // if sensor2 is on and zone does not ignore sensor2, turn it off
		}
	}
}

/* Scheduler
 * this function determines the appropriate start and dequeue times
 * of stations bound to master stations with on and off adjustments
 */
void handle_master_adjustments(time_os_t curr_time, RuntimeQueueStruct *q)
{

	int16_t start_adj = 0;
	int16_t dequeue_adj = 0;

	for (unsigned char mas = MASTER_1; mas < NUM_MASTER_ZONES; mas++)
	{

		unsigned char masid = os.masters[mas][MASOPT_SID];

		if (masid && os.bound_to_master(q->sid, mas))
		{

			int16_t mas_on_adj = os.get_on_adj(mas);
			int16_t mas_off_adj = os.get_off_adj(mas);

			start_adj = min(start_adj, mas_on_adj);
			dequeue_adj = max(dequeue_adj, mas_off_adj);
		}
	}

	// in case of negative master on adjustment
	// push back station's start time to allow sufficient time to turn on master
	if (q->st - curr_time < abs(start_adj))
	{
		q->st += abs(start_adj);
	}

	q->deque_time = q->st + q->dur + dequeue_adj;
}

/** Scheduler
 * This function loops through the queue
 * and schedules the start time of each station
 */
void schedule_all_stations(time_os_t curr_time)
{
	ulong con_start_time = curr_time + 1; // concurrent start time
	// if the queue is paused, make sure the start time is after the scheduled pause ends
	if (os.status.pause_state)
	{
		con_start_time += os.pause_timer;
	}
	int16_t station_delay = water_time_decode_signed(os.iopts[IOPT_STATION_DELAY_TIME]);
	ulong seq_start_times[NUM_SEQ_GROUPS]; // sequential start times
	for (unsigned char i = 0; i < NUM_SEQ_GROUPS; i++)
	{
		seq_start_times[i] = con_start_time;
		// if the sequential queue already has stations running
		if (pd.last_seq_stop_times[i] > curr_time)
		{
			seq_start_times[i] = pd.last_seq_stop_times[i] + station_delay;
		}
	}
	RuntimeQueueStruct *q = pd.queue;
	unsigned char re = os.iopts[IOPT_REMOTE_EXT_MODE];
	unsigned char gid;

	// go through runtime queue and calculate start time of each station
	for (; q < pd.queue + pd.nqueue; q++)
	{
		if (q->st)
			continue; // if this queue element has already been scheduled, skip
		if (!q->dur)
			continue; // if the element has been marked to reset, skip
		gid = os.get_station_gid(q->sid);

		// use sequential scheduling per sequential group
		// apply station delay time
		if (os.is_sequential_station(q->sid) && !re)
		{
			q->st = seq_start_times[gid];
			seq_start_times[gid] += q->dur;
			seq_start_times[gid] += station_delay; // add station delay time
		}
		else
		{
			// otherwise, concurrent scheduling
			q->st = con_start_time;
			// stagger concurrent stations by 1 second
			con_start_time++;
		}

		handle_master_adjustments(curr_time, q);

		if (!os.status.program_busy)
		{
			os.status.program_busy = 1; // set program busy bit
			// start flow count
			if (os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_FLOW)
			{ // if flow sensor is connected
				os.flowcount_log_start = flow_count;
				os.sensor1_active_lasttime = curr_time;
			}
		}
	}
}

/** Immediately reset all stations
 * No log records will be written
 */
void reset_all_stations_immediate()
{
	// <JRJ> Aquire a pointer to the spdlog Logger
	auto spdlogFileLogger = spdlog::get("multi_logger");
	os.clear_all_station_bits();
	os.apply_all_station_bits();
	pd.reset_runtime();
	pd.clear_pause();
	// <JRJ> Debug info
	spdlogFileLogger->info("Main.cpp->reset_all_stations_immediate() All Programs set to \"STATE_TYPE_OFF\"");
}

/** Reset all stations
 * This function sets the duration of
 * every station to 0, which causes
 * all stations to turn off in the next processing cycle.
 * Stations will be logged
 */
void reset_all_stations()
{
	// <JRJ> Aquire a pointer to the spdlog Logger
	auto spdlogFileLogger = spdlog::get("multi_logger");
	RuntimeQueueStruct *q = pd.queue;
	// go through runtime queue and assign water time to 0
	for (; q < pd.queue + pd.nqueue; q++)
	{
		q->dur = 0;
	}
	// <JRJ> Debug info
	spdlogFileLogger->info("Main.cpp->reset_all_stations() All Programs set to \"STATE_TYPE_OFF\"");
}

/** Manually start a program
 * If pid==0, this is a test program (1 minute per station)
 * If pid==255, this is a short test program (2 second per station)
 * If pid > 0. run program pid-1
 */
void manual_start_program(unsigned char pid, unsigned char uwt)
{
	// <JRJ> Aquire a pointer to the spdlog Logger
	auto spdlogFileLogger = spdlog::get("multi_logger");
	boolean match_found = false;
	reset_all_stations_immediate();
	ProgramStruct prog;
	ulong dur;
	unsigned char sid, bid, s;

	spdlogFileLogger->info("Main.cpp->manual_start_program() pid: {0} uwt: {1}", (pid - 1), uwt);
	if ((pid > 0) && (pid < 255))
	{
		pd.read(pid - 1, &prog);
		push_message(NOTIFY_PROGRAM_SCHED, pid - 1, uwt ? os.iopts[IOPT_WATER_PERCENTAGE] : 100, "");
	}
	for (sid = 0; sid < os.nstations; sid++)
	{
		bid = sid >> 3;
		s = sid & 0x07;
		// skip if the station is a master station (because master cannot be scheduled independently
		if ((os.status.mas == sid + 1) || (os.status.mas2 == sid + 1))
			continue;
		dur = 60;
		if (pid == 255)
			dur = 2;
		else if (pid > 0)
			dur = water_time_resolve(prog.durations[sid]);
		if (uwt)
		{
			dur = dur * os.iopts[IOPT_WATER_PERCENTAGE] / 100;
		}
		if (dur > 0 && !(os.attrib_dis[bid] & (1 << s)))
		{
			RuntimeQueueStruct *q = pd.enqueue();
			if (q)
			{
				q->st = 0;
				q->dur = dur;
				q->sid = sid;
				q->pid = 254;
				match_found = true;
			}
		}
	}
	if (match_found)
	{
		schedule_all_stations(os.now_tz());
	}
}

// ==========================================
// ====== PUSH NOTIFICATION FUNCTIONS =======
// ==========================================
void ip2string(char *str, size_t str_len, unsigned char ip[4])
{
	snprintf_P(str + strlen(str), str_len, PSTR("%d.%d.%d.%d"), ip[0], ip[1], ip[2], ip[3]);
}

#define PUSH_TOPIC_LEN 120
#define PUSH_PAYLOAD_LEN TMP_BUFFER_SIZE

void push_message(int type, uint32_t lval, float fval, const char *sval)
{
	// <JRJ> Aquire a pointer to the spdlog Logger
	auto spdlogFileLogger = spdlog::get("multi_logger");
	static char topic[PUSH_TOPIC_LEN + 1];
	static char payload[PUSH_PAYLOAD_LEN + 1];
	char *postval = tmp_buffer + 1; // +1 so we can fit a opening { before the loaded config
	uint32_t volume;

	// check if ifttt key exists and also if the enable bit is set
	os.sopt_load(SOPT_IFTTT_KEY, tmp_buffer);
	bool ifttt_enabled = ((os.iopts[IOPT_NOTIF_ENABLE] & type) != 0) && (strlen(tmp_buffer) != 0);

#define DEFAULT_EMAIL_PORT 465

// parse email variables
#if defined(SUPPORT_EMAIL)
	// define email variables
	ArduinoJson::JsonDocument doc; // make sure this has the same scope of email_x variables to prevent use after free
	const char *email_host = NULL;
	const char *email_username = NULL;
	const char *email_password = NULL;
	const char *email_recipient = NULL;
	int email_port = DEFAULT_EMAIL_PORT;
	int email_en = 0;

	os.sopt_load(SOPT_EMAIL_OPTS, postval);
	if (*postval != 0)
	{
		// Add the wrapping curly braces to the string
		postval = tmp_buffer;
		postval[0] = '{';
		int len = strlen(postval);
		postval[len] = '}';
		postval[len + 1] = 0;

		ArduinoJson::DeserializationError error = ArduinoJson::deserializeJson(doc, postval);
		// Test the parsing otherwise parse
		if (error)
		{
			DEBUG_PRINT(F("mqtt: deserializeJson() failed: "));
			DEBUG_PRINTLN(error.c_str());
		}
		else
		{
			email_en = doc["en"];
			email_host = doc["host"];
			email_port = doc["port"];
			email_username = doc["user"];
			email_password = doc["pass"];
			email_recipient = doc["recipient"];
		}
	}
#endif

#if defined(ESP8266)
	EMailSender::EMailMessage email_message;
#else
	struct
	{
		String subject;
		String message;
	} email_message;
#endif

	bool email_enabled = false;
#if defined(SUPPORT_EMAIL)
	if (!email_en)
	{
		email_enabled = false;
	}
	else
	{
		email_enabled = os.iopts[IOPT_NOTIF_ENABLE] & type;
	}
#endif

	// if none if enabled, return here
	if ((!ifttt_enabled) && (!email_enabled) && (!os.mqtt.enabled()))
		return;

	if (ifttt_enabled || email_enabled)
	{
		strcpy_P(postval, PSTR("{\"value1\":\"On site ["));
		os.sopt_load(SOPT_DEVICE_NAME, topic, PUSH_TOPIC_LEN);
		topic[PUSH_TOPIC_LEN] = 0;
		strcat(postval + strlen(postval), topic);
		strcat_P(postval, PSTR("], "));
		if (email_enabled)
		{
			strcat(topic, " ");
			email_message.subject = topic; // prefix the email subject with device name
		}
	}

	if (os.mqtt.enabled())
	{
		topic[0] = 0;
		payload[0] = 0;
	}

	switch (type)
	{
	case NOTIFY_STATION_ON:

		if (os.mqtt.enabled())
		{
			snprintf_P(topic, PUSH_TOPIC_LEN, PSTR("station/%d"), lval);
			if ((int)fval == 0)
			{
				snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"state\":1}")); // master on event does not have duration attached to it
			}
			else
			{
				snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"state\":1,\"duration\":%d}"), (int)fval);
			}
			spdlogFileLogger->info("Main.cpp->push_message() NOTIFY_STATION_ON mqtt_topic: {0} mqtt_payload: {1}", topic, payload);
		}

		// todo: add IFTTT and email support for this event as well.
		// currently no support due to the number of events exceeds 8 so need to use more than 1 byte
		break;

	case NOTIFY_STATION_STATUS_UPDATE:

		if (os.mqtt.enabled())
		{
			unsigned char qid = pd.station_qid[(int)lval];
			RuntimeQueueStruct *q = pd.queue + qid;
			//  <JRJ> Correct the auto program PIDs... exclude station manual=99 and program manual=254
			unsigned int program_id = 99;
			if ((q->pid > 0) && (q->pid != 99) && (q->pid != 254) && (q->pid < 255))
				program_id = (q->pid - 1);
			snprintf_P(topic, PUSH_TOPIC_LEN, PSTR("%s"), get_mqtt_station_topic());
			snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"sid\":%d, \"pid\":%d, \"state\":1, \"duration\":%d, \"remaining\":%d}"), (int)lval, program_id, q->dur, (int)fval);
			spdlogFileLogger->debug("Main.cpp->push_message() NOTIFY_STATION_STATUS_UPDATE mqtt_topic: {0} mqtt_payload: {1}", topic, payload);
		}
		break;

	case NOTIFY_STATION_OFF:

		if (os.mqtt.enabled())
		{
			snprintf_P(topic, PUSH_TOPIC_LEN, PSTR("station/%d"), lval);
			if (os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_FLOW)
			{
				snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"state\":0,\"duration\":%d,\"flow\":%d.%02d}"), (int)fval, (int)flow_last_gpm, (int)(flow_last_gpm * 100) % 100);
			}
			else
			{
				snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"state\":0,\"duration\":%d}"), (int)fval);
			}
			spdlogFileLogger->info("Main.cpp->push_message() NOTIFY_PROGRAM_OFF mqtt_topic: {0} mqtt_payload: {1}", topic, payload);
		}
		if (ifttt_enabled || email_enabled)
		{
			strcat_P(postval, PSTR("Station ["));
			os.get_station_name(lval, postval + strlen(postval));
			if ((int)fval == 0)
			{
				strcat_P(postval, PSTR("] closed."));
			}
			else
			{
				strcat_P(postval, PSTR("] closed. It ran for "));
				size_t len = strlen(postval);
				snprintf_P(postval + len, TMP_BUFFER_SIZE, PSTR(" %d minutes %d seconds."), (int)fval / 60, (int)fval % 60);
			}

			if (os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_FLOW)
			{
				size_t len = strlen(postval);
				snprintf_P(postval + len, TMP_BUFFER_SIZE, PSTR(" Flow rate: %d.%02d"), (int)flow_last_gpm, (int)(flow_last_gpm * 100) % 100);
			}
			if (email_enabled)
			{
				email_message.subject += PSTR("station event");
			}
		}
		break;

	case NOTIFY_PROGRAM_SCHED:

		if (ifttt_enabled || email_enabled)
		{
			if (sval)
				strcat_P(postval, PSTR("manually scheduled "));
			else
				strcat_P(postval, PSTR("automatically scheduled "));
			strcat_P(postval, PSTR("Program "));
			{
				ProgramStruct prog;
				pd.read(lval, &prog);
				if (lval < pd.nprograms)
					strcat(postval, prog.name);
			}
			size_t len = strlen(postval);
			snprintf_P(postval + len, TMP_BUFFER_SIZE, PSTR(" with %d%% water level."), (int)fval);
			if (email_enabled)
			{
				email_message.subject += PSTR("program event");
			}
		}
		break;

	case NOTIFY_SENSOR1:

		if (os.mqtt.enabled())
		{
			strcpy_P(topic, PSTR("sensor1"));
			snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"state\":%d}"), (int)fval);
			spdlogFileLogger->info("Main.cpp->push_message() NOTIFY_SENSOR1 mqtt_topic: {0} mqtt_payload: {1}", topic, payload);
		}
		if (ifttt_enabled || email_enabled)
		{
			strcat_P(postval, PSTR("sensor 1 "));
			strcat_P(postval, ((int)fval) ? PSTR("activated.") : PSTR("de-activated."));
			if (email_enabled)
			{
				email_message.subject += PSTR("sensor 1 event");
			}
		}
		break;

	case NOTIFY_SENSOR2:

		if (os.mqtt.enabled())
		{
			strcpy_P(topic, PSTR("sensor2"));
			snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"state\":%d}"), (int)fval);
		}
		if (ifttt_enabled || email_enabled)
		{
			strcat_P(postval, PSTR("sensor 2 "));
			strcat_P(postval, ((int)fval) ? PSTR("activated.") : PSTR("de-activated."));
			if (email_enabled)
			{
				email_message.subject += PSTR("sensor 2 event");
			}
		}
		break;

	case NOTIFY_RAINDELAY:

		if (os.mqtt.enabled())
		{
			strcpy_P(topic, PSTR("raindelay"));
			snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"state\":%d}"), (int)fval);
			spdlogFileLogger->info("Main.cpp->push_message() NOTIFY_SENSOR2 mqtt_topic: {0} mqtt_payload: {1}", topic, payload);
		}
		if (ifttt_enabled || email_enabled)
		{
			strcat_P(postval, PSTR("rain delay "));
			strcat_P(postval, ((int)fval) ? PSTR("activated.") : PSTR("de-activated."));
			if (email_enabled)
			{
				email_message.subject += PSTR("rain delay event");
			}
		}
		break;

	case NOTIFY_FLOWSENSOR:

		volume = os.iopts[IOPT_PULSE_RATE_1];
		volume = (volume << 8) + os.iopts[IOPT_PULSE_RATE_0];
		volume = lval * volume;
		if (os.mqtt.enabled())
		{
			strcpy_P(topic, PSTR("sensor/flow"));
			snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"count\":%u,\"volume\":%d.%02d}"), lval, (int)volume / 100, (int)volume % 100);
			spdlogFileLogger->info("Main.cpp->push_message() NOTIFY_FLOWSENSOR mqtt_topic: {0} mqtt_payload: {1}", topic, payload);
		}
		if (ifttt_enabled || email_enabled)
		{
			size_t len = strlen(postval);
			snprintf_P(postval + len, TMP_BUFFER_SIZE, PSTR("Flow count: %u, volume: %d.%02d"), lval, (int)volume / 100, (int)volume % 100);
			if (email_enabled)
			{
				email_message.subject += PSTR("flow sensor event");
			}
		}
		break;

	case NOTIFY_WEATHER_UPDATE:

		if (os.mqtt.enabled())
		{
			// sprintf_P(topic, "%s", get_mqtt_weather_topic());
			strcpy_P(topic, PSTR(get_mqtt_weather_topic()));

			time_t now;
			time(&now);
			// unsigned char ip[4];
			struct tm tm_info;
			localtime_r(&now, &tm_info); // Was gmtime
			char timestamp[21];
			memset(timestamp, 0, sizeof(timestamp) * sizeof(char));
			strftime(timestamp, 21, "%Y/%m/%d %H:%M:%S", &tm_info);

			strcpy_P(payload, PSTR("{"));
			snprintf_P(payload + strlen(payload), PUSH_PAYLOAD_LEN, PSTR("\"date_time\":\"%s\""), timestamp);

			if (lval > 0)
			{
				// strcat_P(postval, PSTR("External IP updated: "));
				strcat_P(payload, PSTR("\"updated_external_ip\":"));

				// IPAddress _ip;
				// unsigned char ip[4] = {_ip[0], _ip[1], _ip[2], _ip[3]};
				// ip2string(postval, TMP_BUFFER_SIZE, ip);

				unsigned char ip[4] = {(unsigned char)((lval >> 24) & 0xFF),
									   (unsigned char)((lval >> 16) & 0xFF),
									   (unsigned char)((lval >> 8) & 0xFF),
									   (unsigned char)(lval & 0xFF)};
				// ip2string(postval, TMP_BUFFER_SIZE, ip);
				strcat_P(payload, PSTR("\""));
				ip2string(payload, PUSH_PAYLOAD_LEN, ip);
				strcat_P(payload, PSTR("\""));
			}
			// sprintf_P(postval + strlen(postval), PSTR("Water level updated: %d%%."), (int)fval);
			if (fval > 0)
			{
				if (lval >= 0)
				{
					strcat_P(payload, PSTR(","));
				}
				snprintf_P(payload + strlen(payload), PUSH_PAYLOAD_LEN, PSTR("\"updated_water_level\":%d"), (int)fval);
			}

			strcat_P(payload, PSTR("}"));
			spdlogFileLogger->info("Main.cpp->push_message() NOTIFY_WEATHER_UPDATE mqtt_topic: {0} mqtt_payload: {1}", topic, payload);
		}

		break;

	case NOTIFY_REBOOT:
		if (os.mqtt.enabled())
		{
			// snprintf_P(topic, PUSH_TOPIC_LEN, PSTR("%s"), get_mqtt_system_topic());
			strcpy_P(topic, PSTR(get_mqtt_system_topic()));
			// strcpy_P(payload, PSTR("{\"state\":\"started\"}"));
			snprintf_P(payload, PUSH_PAYLOAD_LEN, PSTR("{\"state\":\"started\", \"version\":\"%d.%d\", \"date_time\":\"%s\"}"), OS_FW_VERSION, OS_FW_MINOR, getCurrentDateTime().c_str()); // JRJ
			spdlogFileLogger->info("Main.cpp->push_message() NOTIFY_REBOOT mqtt_topic: {0} mqtt_payload: {1}", topic, payload);
		}
		if (ifttt_enabled || email_enabled)
		{
#if defined(ARDUINO)
			strcat_P(postval, PSTR("rebooted. Device IP: "));
#if defined(ESP8266)
			{
				IPAddress _ip;
				if (useEth)
				{
					//_ip = Ethernet.localIP();
					_ip = eth.localIP();
				}
				else
				{
					_ip = WiFi.localIP();
				}
				unsigned char ip[4] = {_ip[0], _ip[1], _ip[2], _ip[3]};
				ip2string(postval, TMP_BUFFER_SIZE, ip);
			}
#else
			ip2string(postval, TMP_BUFFER_SIZE, &(Ethernet.localIP()[0]));
#endif
#else
			strcat_P(postval, PSTR("controller process restarted."));
#endif
			if (email_enabled)
			{
				email_message.subject += PSTR("reboot event");
			}
		}
		break;
	}

	if (os.mqtt.enabled() && strlen(topic) && strlen(payload))
		os.mqtt.publish(topic, payload);

	if (ifttt_enabled)
	{
		strcat_P(postval, PSTR("\"}"));

		BufferFiller bf = BufferFiller(ether_buffer, TMP_BUFFER_SIZE);
		bf.emit_p(PSTR("POST /trigger/sprinkler/with/key/$O HTTP/1.0\r\n"
					   "Host: $S\r\n"
					   "Accept: */*\r\n"
					   "Content-Length: $D\r\n"
					   "Content-Type: application/json\r\n\r\n$S"),
				  SOPT_IFTTT_KEY, DEFAULT_IFTTT_URL, strlen(postval), postval);

		os.send_http_request(DEFAULT_IFTTT_URL, 80, ether_buffer, remote_http_callback);
	}

	if (email_enabled)
	{
		email_message.message = strchr(postval, 'O'); // ad-hoc: remove the value1 part from the ifttt message
#if defined(ARDUINO)
#if defined(ESP8266)
		if (email_host && email_username && email_password && email_recipient)
		{ // make sure all are valid
			EMailSender emailSend(email_username, email_password);
			emailSend.setSMTPServer(email_host); // TODO: double check removing strdup
			emailSend.setSMTPPort(email_port);
			EMailSender::Response resp = emailSend.send(email_recipient, email_message);
			// DEBUG_PRINTLN(F("Sending Status:"));
			// DEBUG_PRINTLN(resp.status);
			// DEBUG_PRINTLN(resp.code);
			// DEBUG_PRINTLN(resp.desc);
		}
#endif
#else
		struct smtp *smtp = NULL;
		String email_port_str = to_string(email_port);
		// todo: check error?
		smtp_status_code rc;
		if (email_host && email_username && email_password && email_recipient)
		{ // make sure all are valid
			rc = smtp_open(email_host, email_port_str.c_str(), SMTP_SECURITY_TLS, SMTP_NO_CERT_VERIFY, NULL, &smtp);
			rc = smtp_auth(smtp, SMTP_AUTH_PLAIN, email_username, email_password);
			rc = smtp_address_add(smtp, SMTP_ADDRESS_FROM, email_username, "OpenSprinkler");
			rc = smtp_address_add(smtp, SMTP_ADDRESS_TO, email_recipient, "User");
			rc = smtp_header_add(smtp, "Subject", email_message.subject.c_str());
			rc = smtp_mail(smtp, email_message.message.c_str());
			rc = smtp_close(smtp);
			if (rc != SMTP_STATUS_OK)
			{
				DEBUG_PRINTF("SMTP: Error %s\n", smtp_status_code_errstr(rc));
			}
		}
#endif
	}
}

// ================================
// ====== LOGGING FUNCTIONS =======
// ================================
#if defined(ARDUINO)
char LOG_PREFIX[] = "/logs/";
#else
char LOG_PREFIX[] = "./logs/";
#endif

/** Generate log file name
 * Log files will be named /logs/xxxxx.txt
 */
void make_logfile_name(char *name)
{
#if defined(ARDUINO)
#if !defined(ESP8266)
	sd.chdir("/");
#endif
#endif
	strcpy(tmp_buffer + TMP_BUFFER_SIZE - 10, name); // hack: we do this because name is from tmp_buffer too
	strcpy(tmp_buffer, LOG_PREFIX);
	strcat(tmp_buffer, tmp_buffer + TMP_BUFFER_SIZE - 10);
	strcat_P(tmp_buffer, PSTR(".txt"));
}

/* To save RAM space, we store log type names
 * in program memory, and each name
 * must be strictly two characters with an ending 0
 * so each name is 3 characters total
 */
static const char log_type_names[] PROGMEM =
	"  \0"
	"s1\0"
	"rd\0"
	"wl\0"
	"fl\0"
	"s2\0"
	"cu\0";

/** write run record to log on SD card */
void write_log(unsigned char type, time_os_t curr_time)
{

	if (!os.iopts[IOPT_ENABLE_LOGGING])
		return;

	// file name will be logs/xxxxx.tx where xxxxx is the day in epoch time
	snprintf(tmp_buffer, TMP_BUFFER_SIZE, "%lu", curr_time / 86400);
	make_logfile_name(tmp_buffer);

	// Step 1: open file if exists, or create new otherwise,
	// and move file pointer to the end
#if defined(ARDUINO) // prepare log folder for Arduino

#if defined(ESP8266)
	File file = LittleFS.open(tmp_buffer, "r+");
	if (!file)
	{
		FSInfo fs_info;
		LittleFS.info(fs_info);
		// check if we are getting close to run out of space, and delete some oldest files
		if (fs_info.totalBytes < fs_info.usedBytes + fs_info.blockSize * 4)
		{
			// delete the oldest 7 files (1 week of log)
			for (unsigned char i = 0; i < 7; i++)
				delete_log_oldest();
		}
		file = LittleFS.open(tmp_buffer, "w");
		if (!file)
			return;
	}
	file.seek(0, SeekEnd);
#else
	sd.chdir("/");
	if (sd.chdir(LOG_PREFIX) == false)
	{
		// create dir if it doesn't exist yet
		if (sd.mkdir(LOG_PREFIX) == false)
		{
			return;
		}
	}
	SdFile file;
	int ret = file.open(tmp_buffer, O_CREAT | O_WRITE);
	file.seekEnd();
	if (!ret)
	{
		return;
	}
#endif

#else  // prepare log folder for RPI/BBB
	struct stat st;
	if (stat(get_filename_fullpath(LOG_PREFIX), &st))
	{
		if (mkdir(get_filename_fullpath(LOG_PREFIX), S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH))
		{
			return;
		}
	}
	FILE *file;
	file = fopen(get_filename_fullpath(tmp_buffer), "rb+");
	if (!file)
	{
		file = fopen(get_filename_fullpath(tmp_buffer), "wb");
		if (!file)
			return;
	}
	fseek(file, 0, SEEK_END);
#endif // prepare log folder

	// Step 2: prepare data buffer
	strcpy_P(tmp_buffer, PSTR("["));

	if (type == LOGDATA_STATION)
	{
		size_t size = strlen(tmp_buffer);
		snprintf(tmp_buffer + size, TMP_BUFFER_SIZE - size, "%d", pd.lastrun.program);
		strcat_P(tmp_buffer, PSTR(","));
		size = strlen(tmp_buffer);
		snprintf(tmp_buffer + size, TMP_BUFFER_SIZE - size, "%d", pd.lastrun.station);
		strcat_P(tmp_buffer, PSTR(","));
		// duration is unsigned integer
		size = strlen(tmp_buffer);
		snprintf(tmp_buffer + size, TMP_BUFFER_SIZE - size, "%lu", (ulong)pd.lastrun.duration);
	}
	else
	{
		ulong lvalue = 0;
		if (type == LOGDATA_FLOWSENSE)
		{
			lvalue = (flow_count > os.flowcount_log_start) ? (flow_count - os.flowcount_log_start) : 0;
		}

		size_t size = strlen(tmp_buffer);
		snprintf(tmp_buffer + size, TMP_BUFFER_SIZE - size, "%lu", lvalue);
		strcat_P(tmp_buffer, PSTR(",\""));
		strcat_P(tmp_buffer, log_type_names + type * 3);
		strcat_P(tmp_buffer, PSTR("\","));

		switch (type)
		{
		case LOGDATA_FLOWSENSE:
			lvalue = (curr_time > os.sensor1_active_lasttime) ? (curr_time - os.sensor1_active_lasttime) : 0;
			break;
		case LOGDATA_SENSOR1:
			lvalue = (curr_time > os.sensor1_active_lasttime) ? (curr_time - os.sensor1_active_lasttime) : 0;
			break;
		case LOGDATA_SENSOR2:
			lvalue = (curr_time > os.sensor2_active_lasttime) ? (curr_time - os.sensor2_active_lasttime) : 0;
			break;
		case LOGDATA_RAINDELAY:
			lvalue = (curr_time > os.raindelay_on_lasttime) ? (curr_time - os.raindelay_on_lasttime) : 0;
			break;
		case LOGDATA_WATERLEVEL:
			lvalue = os.iopts[IOPT_WATER_PERCENTAGE];
			break;
		}
		size = strlen(tmp_buffer);
		snprintf(tmp_buffer + size, TMP_BUFFER_SIZE - size, "%lu", lvalue);
	}
	strcat_P(tmp_buffer, PSTR(","));
	size_t size = strlen(tmp_buffer);
	snprintf(tmp_buffer + size, TMP_BUFFER_SIZE - size, "%lu", curr_time);
	if ((os.iopts[IOPT_SENSOR1_TYPE] == SENSOR_TYPE_FLOW) && (type == LOGDATA_STATION))
	{
		// RAH implementation of flow sensor
		strcat_P(tmp_buffer, PSTR(","));
#if defined(ARDUINO)
		dtostrf(flow_last_gpm, 5, 2, tmp_buffer + strlen(tmp_buffer));
#else
		size_t len = strlen(tmp_buffer);
		snprintf(tmp_buffer + len, TMP_BUFFER_SIZE - len, "%5.2f", flow_last_gpm);
#endif
	}
	strcat_P(tmp_buffer, PSTR("]\r\n"));

#if defined(ARDUINO)
#if defined(ESP8266)
	file.write((const uint8_t *)tmp_buffer, strlen(tmp_buffer));
#else
	file.write(tmp_buffer);
#endif
	file.close();
#else
	fwrite(tmp_buffer, 1, strlen(tmp_buffer), file);
	fclose(file);
#endif
}

#if defined(ESP8266)
bool delete_log_oldest()
{
	Dir dir = LittleFS.openDir(LOG_PREFIX);
	time_os_t oldest_t = ULONG_MAX;
	String oldest_fn;
	while (dir.next())
	{
		time_os_t t = dir.fileCreationTime();
		if (t < oldest_t)
		{
			oldest_t = t;
			oldest_fn = dir.fileName();
		}
	}
	if (oldest_fn.length() > 0)
	{
		DEBUG_PRINT(F("deleting "))
		DEBUG_PRINTLN(LOG_PREFIX + oldest_fn);
		LittleFS.remove(LOG_PREFIX + oldest_fn);
		return true;
	}
	else
	{
		return false;
	}
}
#endif

/** Delete log file
 * If name is 'all', delete all logs
 */
void delete_log(char *name)
{
	if (!os.iopts[IOPT_ENABLE_LOGGING])
		return;
#if defined(ARDUINO)

#if defined(ESP8266)
	if (strncmp(name, "all", 3) == 0)
	{
		// delete all log files
		Dir dir = LittleFS.openDir(LOG_PREFIX);
		while (dir.next())
		{
			LittleFS.remove(LOG_PREFIX + dir.fileName());
		}
	}
	else
	{
		// delete a single log file
		make_logfile_name(name);
		if (!LittleFS.exists(tmp_buffer))
			return;
		LittleFS.remove(tmp_buffer);
	}
#else
	if (strncmp(name, "all", 3) == 0)
	{
		// delete the log folder
		SdFile file;

		if (sd.chdir(LOG_PREFIX))
		{
			// delete the whole log folder
			sd.vwd()->rmRfStar();
		}
	}
	else
	{
		// delete a single log file
		make_logfile_name(name);
		if (!sd.exists(tmp_buffer))
			return;
		sd.remove(tmp_buffer);
	}
#endif

#else // delete_log implementation for RPI/BBB
	if (strncmp(name, "all", 3) == 0)
	{
		// delete the log folder
		rmdir(get_filename_fullpath(LOG_PREFIX));
		return;
	}
	else
	{
		make_logfile_name(name);
		remove(get_filename_fullpath(tmp_buffer));
	}
#endif
}

/** Perform network check
 * This function pings the router
 * to check if it's still online.
 * If not, it re-initializes Ethernet controller.
 */
static void check_network()
{
#if defined(OS_AVR)
	// do not perform network checking if the controller has just started, or if a program is running
	if (os.status.program_busy)
	{
		return;
	}

	// check network condition periodically
	if (os.status.req_network)
	{
		os.status.req_network = 0;
		// change LCD icon to indicate it's checking network
		if (!ui_state)
		{
			os.lcd.setCursor(LCD_CURSOR_NETWORK, 1);
			os.lcd.write('>');
		}

		boolean failed = false;
		// todo: ping gateway ip
		/*ether.clientIcmpRequest(ether.gwip);
		ulong start = millis();
		// wait at most PING_TIMEOUT milliseconds for ping result
		do {
			ether.packetLoop(ether.packetReceive());
			if (ether.packetLoopIcmpCheckReply(ether.gwip)) {
				failed = false;
				break;
			}
		} while(millis() - start < PING_TIMEOUT);*/
		if (failed)
		{
			if (os.status.network_fails < 3)
				os.status.network_fails++;
			// clamp it to 6
			// if (os.status.network_fails > 6) os.status.network_fails = 6;
		}
		else
			os.status.network_fails = 0;
		// if failed more than 3 times, restart
		if (os.status.network_fails == 3)
		{
			// mark for safe restart
			os.nvdata.reboot_cause = REBOOT_CAUSE_NETWORK_FAIL;
			os.status.safe_reboot = 1;
		}
		else if (os.status.network_fails > 2)
		{
			// if failed more than twice, try to reconnect
			if (os.start_network())
				os.status.network_fails = 0;
		}
	}
#else
	// nothing to do for other platforms
#endif
}

/** Perform NTP sync */
static void perform_ntp_sync()
{
#if defined(ARDUINO)
	// do not perform ntp if this option is disabled, or if a program is currently running
	if (!os.iopts[IOPT_USE_NTP] || os.status.program_busy)
		return;
	// do not perform ntp if network is not connected
	if (!os.network_connected())
		return;

	if (os.status.req_ntpsync)
	{
		os.status.req_ntpsync = 0;
		if (!ui_state)
		{
			os.lcd_print_line_clear_pgm(PSTR("NTP Syncing..."), 1);
		}
		DEBUG_PRINTLN(F("NTP Syncing..."));
		static ulong last_ntp_result = 0;
		ulong t = getNtpTime();
		if (last_ntp_result > 3 && t > last_ntp_result - 3 && t < last_ntp_result + 3)
		{
			DEBUG_PRINTLN(F("error: result too close to last"));
			t = 0; // invalidate the result
		}
		else
		{
			last_ntp_result = t;
		}
		if (t > 0)
		{
			setTime(t);
			RTC.set(t);
			DEBUG_PRINTLN(RTC.get());
		}
	}
#else
	// nothing to do here
	// Linux will do this for you
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// JRJ v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v v //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline string getCurrentDateTime(string s)
{
	time_t now = time(0);
	struct tm tstruct;
	char buf[80];
	tstruct = *localtime(&now);
	if (s == "now")
		strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
	else if (s == "date")
		strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
	return string(buf);
};

string getCurrentDateTime(void)
{
	time_t now;
	time(&now);
	struct tm tm_info;
	localtime_r(&now, &tm_info); // Was gmtime
	char timestamp[20];
	memset(timestamp, 0, sizeof(timestamp) * sizeof(char));
	strftime(timestamp, 20, "%Y/%m/%d %H:%M:%S", &tm_info);
	return string(timestamp);
}

string getCurrentDate(void)
{
	time_t now;
	time(&now);
	struct tm tm_info;
	localtime_r(&now, &tm_info); // Was gmtime
	char timestamp[9];
	memset(timestamp, 0, sizeof(timestamp) * sizeof(char));
	strftime(timestamp, 9, "%Y%m%d", &tm_info);
	return string(timestamp);
}

string getCurrentDateTimeFN(void)
{
	// <JRJ> Get current date/time
	time_t now;
	time(&now);
	struct tm tm_info;
	localtime_r(&now, &tm_info);
	char timestamp[16];
	memset(timestamp, 0, sizeof(timestamp) * sizeof(char));
	strftime(timestamp, 16, "%Y%m%d_%H%M%S", &tm_info);
	return string(timestamp);
}

string getDate(time_t now)
{
	struct tm tm_info;
	localtime_r(&now, &tm_info); // Was gmtime
	char timestamp[9];
	memset(timestamp, 0, sizeof(timestamp) * sizeof(char));
	strftime(timestamp, 9, "%Y%m%d", &tm_info);
	return string(timestamp);
}

string getDateTime(time_t now)
{
	struct tm tm_info;
	localtime_r(&now, &tm_info); // Was gmtime
	char timestamp[20];
	memset(timestamp, 0, sizeof(timestamp) * sizeof(char));
	strftime(timestamp, 20, "%Y/%m/%d %H:%M:%S", &tm_info);
	return string(timestamp);
}

// inline void Logger( string logMsg )
// {
// string filePath = "/home/pi/opensprinkler_log_"+getCurrentDateTime("date")+".txt";
// DEBUG_PRINT(F("Logging File: "));
// DEBUG_PRINTLN(filePath.c_str());
// string now = getCurrentDateTime("now");
// ofstream ofs(filePath.c_str(), std::ios_base::out | std::ios_base::app );
// ofs << now << PSTR("  ") << logMsg << '\n';
// ofs.close();
// }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// JRJ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if !defined(ARDUINO) // main function for RPI/BBB
int main(int argc, char *argv[])
{
	// Disable buffering to work with systemctl journal
	setvbuf(stdout, NULL, _IOLBF, 0);
	printf("Starting OpenSprinkler\n");

	int opt;
	while (-1 != (opt = getopt(argc, argv, "d:")))
	{
		switch (opt)
		{
		case 'd':
			set_data_dir(optarg);
			break;
		default:
			// ignore options we don't understand
			break;
		}
	}

	do_setup();

	while (true)
	{
		do_loop();
	}
	return 0;
}
#endif
