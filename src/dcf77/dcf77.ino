//#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts
//#define NO_PORTC_PINCHANGES // to indicate that port c will not be used for pin change interrupts
//#define NO_PIN_STATE // to indicate that you don't need the pinState
//#define DISABLE_PCINT_MULTI_SERVICE // to limit the handler to servicing a single interrupt per invocation.
#include <PinChangeInt.h>
#include <AdaEncoder.h>
#include <EEPROM.h>

#include <U8glib.h>
#include "MyGPS.h"
#include <TimeLib.h>
#include <Wire.h>
#include <DS3231.h>
#include <SPI.h>

#include "icons_16_8.h"
#include "monitor.h"

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

byte date_time_years;
byte date_time_months;
byte date_time_days;
byte date_time_hours;
byte date_time_minutes;
byte date_time_seconds;
byte date_time_days_of_week;

#define tz_offset 17
#define tz_size 2
#define start_bit_offset 20
#define start_bit_size 1

#define parity_size 1

#define minites_offset 21
#define minites_size 7
#define minites_parity_offset 28

#define hours_offset 29
#define hours_size 6
#define hours_parity_offset 35

#define days_offset 36
#define days_size 6

#define days_of_week_offset 42
#define days_of_week_size 3

#define months_offset 45
#define months_size 5

#define years_offset 50
#define years_size 8

#define date_parity_offset 58

/// dcf77 class begin
void write_to_bitmap(byte value, byte offset, byte size, byte* buf) {
  for (byte i = 0; i < size; ++i) {
    byte o = i + offset;
    buf[o / 8] |= ((value >> i) & 1) << (o % 8);
  }
}

byte get_byte(byte offset, byte* buf) {
  return (buf[offset / 8] >> (offset % 8)) & 1;
}

byte bcd(byte value) {
  return ((value / 10) << 4) | (value % 10);
}

byte calculate_parity(byte offset1, byte offset2, byte* buf) {
  byte result = 0;
  for (byte i = offset1; i <= offset2; ++i) {
    result ^= get_byte(i, buf);
  }
  return result;
}

void load_date_time(tmElements_t &d) {
  memset(&d, 0, sizeof(d));
  d.Year = date_time_years;
  d.Month = date_time_months;
  d.Day = date_time_days;
  d.Wday = date_time_days_of_week;
  d.Hour = date_time_hours;
  d.Minute = date_time_minutes;
  d.Second = date_time_seconds;
}

void add_seconds_to_date(tmElements_t &d, int secondsToAdd) {
  d.Year += 30;
  time_t t = makeTime(d) + secondsToAdd;
  breakTime(t, d);
  d.Year -= 30;
}

void todcf77(byte* dcf77) {
  memset(dcf77, 0, 8);
  tmElements_t d;
  load_date_time(d);
  add_seconds_to_date(d, SECS_PER_HOUR + 60); //+1 minute because of start on minute before. and two hour because of CEST
  int dow = (d.Wday != 1) ? (d.Wday - 1) : 7;

  write_to_bitmap(2, tz_offset, tz_size, dcf77);
  write_to_bitmap(1, start_bit_offset, start_bit_size, dcf77);

  write_to_bitmap(bcd(d.Minute), minites_offset, minites_size, dcf77);
  write_to_bitmap(calculate_parity(minites_offset, minites_parity_offset - 1, dcf77), minites_parity_offset, parity_size, dcf77);

  write_to_bitmap(bcd(d.Hour), hours_offset, hours_size, dcf77);
  write_to_bitmap(calculate_parity(hours_offset, hours_parity_offset - 1, dcf77), hours_parity_offset, parity_size, dcf77);

  write_to_bitmap(bcd(d.Day), days_offset, days_size, dcf77);
  write_to_bitmap(dow, days_of_week_offset, days_of_week_size, dcf77);//monday == 1, sunday == 7
  write_to_bitmap(bcd(d.Month), months_offset, months_size, dcf77);
  write_to_bitmap(bcd(d.Year), years_offset, years_size, dcf77);
  write_to_bitmap(calculate_parity(days_offset, date_parity_offset - 1, dcf77), date_parity_offset, parity_size, dcf77);
}

byte dcf77_blinker_buffer[8];

#define COUNT_OF_SIGNAL_PER_SECOND (1*1024)
#define SHORT_SIGNAL_COUNT (COUNT_OF_SIGNAL_PER_SECOND/10+10)
#define LONG_SIGNAL_COUNT (COUNT_OF_SIGNAL_PER_SECOND/5+10)
#define COUNT_OF_SIGNAL_PER_SECOND_MODE 1

#define MOSCOW_OFFSET 3
#define EEPROM_SIGNATURE 0x5501
volatile word clock_lo_counter = 0;
volatile byte clock_hi_counter = 0;
volatile byte reload_data = 0;
/// dcf77 class end
byte last_rendered_high = 0;
byte last_scheduller_work = 0;
Monitor<unsigned long> monitor;

#define MSG_PROCESS_INPUTS 0
#define MSG_BUTTON_1_PRESSED 1
#define MSG_BUTTON_2_PRESSED 2
#define MSG_POWER_NEXT 4
#define MSG_EXIT 5
#define MSG_DRAW 6
#define MSG_TIMEOUT 7
#define MSG_HAS_GPS_DATA 8
#define MSG_POWER_PREV 9
#define MSG_GPS_READY 10
#define MSG_GPS_FINISH 11
#define MSG_RELOAD_DATA 12
#define MSG_INIT_DATA 13
#define MSG_GUI_NEXT 14
#define MSG_GUI_RESET 15
#define MSG_GUI_CONTRAST 16
#define MSG_INCREASE_BRIGHT 17
#define MSG_DECREASE_BRIGHT 18
#define MSG_CONFIG_SAVE 19
#define MSG_CONFIG_LOAD 20
#define MSG_GPS_ON 21
#define MSG_RADIO_ON 22
#define MSG_RADIO_OFF 23
#define PROCESS_SCHEDULE 24
#define MSG_GPS_OFF 25
#define MSG_NEXT_POWER_STATISTIC 26
#define MSG_PREV_POWER_STATISTIC 27
#define MSG_IDLE 100

#define GUI_HOME 0
#define GUI_CONTRAST 1
#define GUI_POWER 2
#define GUI_CONFIG 3
//#define GUI_AUTO_DISPLAY_OFF
#define GUI_POWER_STATISTIC 4
#define GUI_SHEDULER_STATUS 5
#define GUI_LAST 6

#define CONFIG_NONE 0
#define CONFIG_SAVED 1
#define CONFIG_LOADED 2

#define SHOW_30_SEC_CURRENT 0
#define SHOW_4_MIN_CURRENT 1
#define SHOW_1_HOUR_CURRENT 2
#define SHOW_1_DAY_CURRENT 3
#define SHOW_CURRENT_LAST 3
unsigned long power_statistic_periods[] = {30, 300, 7200, 24LL * 3600};
char power_statistic_periods_text[] = "30\0""5m\0""2h\0""1d";
byte current_state_mode = SHOW_30_SEC_CURRENT;

#define OLED_SCK 13
#define OLED_MOSI 11
#define OLED_CS 10
#define OLED_DC 9
#define OLED_RESET 12

#define POWER_STATE_GPS 1 //first bit
#define POWER_STATE_RADIO 2 //second bit
#define POWER_STATE_NIGHT 4 //third bit

U8GLIB_SH1106_128X64 panel(OLED_CS, OLED_DC, OLED_RESET);
//U8GLIB_SSD1306_128X64 panel(OLED_CS, OLED_DC, OLED_RESET);
MyGPS gps(Serial);
DS3231 ds;

long gui_timeout = 0;
byte gui_state = GUI_HOME;
volatile bool transmitter = true;
bool gpscontrol = true;
bool night = true;
long last_fix_time = MyGPS::GPS_INVALID_FIX_TIME;
time_t last_fix_time_full = 0;
volatile byte push_button_state = 0;

struct config_t {
  byte gui_contrast;
  byte power_state;
} config_data;

byte config_state = CONFIG_NONE;

const unsigned  char * battery[]  = {battery_empty, battery_2, battery_4, battery_full, battery_plugged};

// todo move it to class

struct action {
  time_t next;
  byte hours;
  byte minutes;
  byte activity;
};

time_t nowTime() {
  tmElements_t d;
  load_date_time(d);
  return makeTime(d) + MOSCOW_OFFSET * SECS_PER_HOUR;
}

time_t nearest_at(byte hours, byte minutes) {
  time_t cur_time = nowTime();
  time_t candidate = cur_time - cur_time % SECS_PER_DAY + 60 * (hours * 60ll + minutes);
  if (candidate < cur_time)
    candidate += SECS_PER_DAY;
  return candidate;
}
// 01:00 gps on
// 01:20 gps off
// 2:55 radio on
// 3:15 radio off [watches try to do sync while 11 minutes if signal present but sync is unsuccess]
//[watches try to do sync while 2 minutes if signal does not present]
//[watches do sync approximately after 5 minutes if good signal]
action actions[4] = {{0, 1, 0, MSG_GPS_ON}, {0, 1, 20, MSG_GPS_OFF}, {0, 2, 55, MSG_RADIO_ON}, {0, 3, 15, MSG_RADIO_OFF}};

void initialize_everyday_activities() {
  load_date();
  for (byte i = 0; i < 4; ++i) {
    actions[i].next = nearest_at(actions[i].hours, actions[i].minutes);
  }
}

int scheduller() {
  if (night && last_scheduller_work != clock_hi_counter) { //consider do unschedule
    last_scheduller_work = clock_hi_counter;
    time_t cur_time = nowTime();
    for (byte i = 0; i < 4; ++i) {
      action& a = actions[i];
      if (cur_time > a.next) {
        a.next = nearest_at(a.hours, a.minutes);
        return a.activity;
      }
    }
  }
  return MSG_IDLE;
}

unsigned long seconds_from_start = 0;

unsigned long millis_t() {
  return clock_lo_counter + seconds_from_start * 1024;
}

unsigned long seconds_t() {
  return seconds_from_start;
}


bool filter_dizzy_press() {
  static int last = millis_t();
  if (millis_t() - last > 50) {
    last = millis_t();
    return true;
  }

  return false;
}

void set_contrast(byte c) {
  power_spi_enable();
  panel.setContrast(c);
  power_spi_disable();
}

void push_button() {
  if (filter_dizzy_press())
    ++push_button_state;
}

void setup() {
  DDRB |= B00000001; // enable pb0 pin for power on radio as output
  DDRC |= B00001000; // enable pc3 pin for power on device as output
  AdaEncoder::addEncoder('a', 6, 7);
  attachPinChangeInterrupt(4, push_button, RISING); // Any state change will trigger the interrupt.
  check_on_button();
  Wire.begin();
}

void check_on_button() {
  PORTB |= B00000001;
}

void transmitter_power(bool on) {
  if (on) {
    ds.enableOscillator(true, false, COUNT_OF_SIGNAL_PER_SECOND_MODE);
    PORTC |= B00001000;
  }
  else {
    ds.enableOscillator(true, false, 0);
    PORTC &= B11110111; // disable transmitter
    PORTD &= B11011111; // do prevent leaks
  }
  transmitter = on;
}

void GotoSleep() {
  set_sleep_mode(SLEEP_MODE_STANDBY/*SLEEP_MODE_ADC*/);
  sleep_mode();
}

void gps_power(bool on) {
  gpscontrol = on;
  if (gpscontrol)
    gps.cold_restart();//to avoid catch stored inside gps module information
  else
    gps.standby();
}

void save_config() {
  long signature = EEPROM_SIGNATURE;
  EEPROM.put(0, signature);
  EEPROM.put(4, config_data);
  config_state = CONFIG_SAVED;
}

void load_config() {
  long signature = 0;
  EEPROM.get(0, signature);
  if (signature == EEPROM_SIGNATURE)
    EEPROM.get(4, config_data);
  else
  {
    config_data.gui_contrast = 0;
    config_data.power_state = POWER_STATE_NIGHT;
  }
  set_contrast(config_data.gui_contrast);
  control_power();
  config_state = CONFIG_LOADED;
}

void switch_sheduller(bool on) {
  night = on;
  initialize_everyday_activities();
}

void control_power() {
  gps_power(config_data.power_state & POWER_STATE_GPS);
  transmitter_power(config_data.power_state & POWER_STATE_RADIO);
  switch_sheduller(config_data.power_state & POWER_STATE_NIGHT);
}

void process() {
  int8_t msg = MSG_INIT_DATA;
  int8_t clicks = 0;
  int8_t push_clicks = 0;
  while (1) {
    switch (msg) {
      case MSG_INIT_DATA:
        load_date();
        clock_hi_counter = date_time_seconds;
        msg = MSG_DRAW;
        break;
      case MSG_PROCESS_INPUTS:
        clicks = 0;
        AdaEncoder::genie(&clicks, 0);
        if (clicks > 0) {
          gui_timeout = seconds_t();
          switch (gui_state) {
            case GUI_HOME: case GUI_POWER: msg = MSG_POWER_NEXT; break;
            case GUI_CONTRAST: msg = MSG_INCREASE_BRIGHT; break;
            case GUI_CONFIG: msg = MSG_CONFIG_SAVE; break;
            case GUI_POWER_STATISTIC: msg = MSG_NEXT_POWER_STATISTIC; break;
            default: msg = MSG_GUI_RESET; break;
          }
        }
        else if (clicks  < 0) {
          gui_timeout = seconds_t();
          switch (gui_state) {
            case GUI_HOME: case GUI_POWER: msg = MSG_POWER_PREV; break;
            case GUI_CONTRAST: msg = MSG_DECREASE_BRIGHT; break;
            case GUI_CONFIG: msg = MSG_CONFIG_LOAD; break;
            case GUI_POWER_STATISTIC: msg = MSG_PREV_POWER_STATISTIC; break;
            default: msg = MSG_GUI_RESET; break;
          }
        }
        else if (push_button_state) {
          msg = MSG_GUI_NEXT;
          gui_timeout = seconds_t();
          push_button_state = 0;
        }
        else if ( gui_state != GUI_HOME && (seconds_t() - gui_timeout ) > 10) {
          msg = MSG_GUI_RESET;
        }
        else if (last_rendered_high != clock_hi_counter)
          msg = MSG_DRAW;
        else if (reload_data)
          msg = MSG_RELOAD_DATA;
        else if (gpscontrol && gps.has_data())
          msg = MSG_HAS_GPS_DATA;
        else
          msg = scheduller(); //msg could be planned work otherwise  MSG_IDLE
        break;
      case MSG_INCREASE_BRIGHT: {
          if (config_data.gui_contrast < 240)
            config_data.gui_contrast += 16;
          else
            config_data.gui_contrast = 255;
          set_contrast(config_data.gui_contrast);
          msg = MSG_DRAW;
          break;
        }
      case MSG_DECREASE_BRIGHT: {
          if (config_data.gui_contrast > 16)
            config_data.gui_contrast -= 16;
          else
            config_data.gui_contrast = 0;
          set_contrast(config_data.gui_contrast);
          msg = MSG_DRAW;
          break;
        }
      case MSG_CONFIG_SAVE: {
          save_config();
          msg = MSG_DRAW;
          break;
        }
      case MSG_CONFIG_LOAD: {
          load_config();
          msg = MSG_DRAW;
          break;
        }
      case MSG_HAS_GPS_DATA:
        if (gps.proeccess_one())
          msg = MSG_GPS_READY;
        else
          msg = MSG_PROCESS_INPUTS;
        break;
      case MSG_GUI_RESET: {
          gui_state = GUI_HOME;
          msg = MSG_DRAW;
          gui_timeout = 0;
          break;
        }
      case MSG_NEXT_POWER_STATISTIC: {
          if (current_state_mode == SHOW_CURRENT_LAST)
            current_state_mode = 0;
          else
            ++current_state_mode;
          msg = MSG_DRAW;
          break;
        }

      case MSG_PREV_POWER_STATISTIC: {
          if (current_state_mode == 0)
            current_state_mode = SHOW_CURRENT_LAST;
          else
            --current_state_mode;
          msg = MSG_DRAW;
          break;
        }
      case MSG_GUI_NEXT: {
          ++gui_state;
          config_state = CONFIG_NONE;
          msg = (gui_state == GUI_LAST) ? MSG_GUI_RESET : MSG_DRAW;
          gui_timeout = seconds_t();
          break;
        }
      case MSG_GPS_READY: {
          byte year = 0;
          byte month = 0;
          byte day = 0;
          byte hour = 0;
          byte minute = 0;
          byte second = 0;
          unsigned long fix = 0;

          gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, 0, &fix);
          if (fix == last_fix_time || year == 0)
          { // means that new sattelites is not found
            msg = MSG_PROCESS_INPUTS;
            break;
          }
          ds.setSecond(second); // closest position
          ds.setMinute(minute);
          last_fix_time = fix;
          date_time_years = year;
          date_time_months = month;
          date_time_days = day;
          date_time_hours = hour;
          date_time_minutes = minute;
          date_time_seconds = second;

          tmElements_t d;
          load_date_time(d);
          add_seconds_to_date(d, 0); //it will recalculate day of week
          last_fix_time_full = makeTime(d);
          date_time_days_of_week = d.Wday;
          ds.setHour(date_time_hours);
          ds.setDoW(date_time_days_of_week);
          ds.setDate(date_time_days);
          ds.setMonth(date_time_months);
          ds.setYear(date_time_years);
          clock_hi_counter = date_time_seconds;
          msg = MSG_GPS_OFF;
        }
        break;
      /*      case MSG_GPS_FINISH: // unused now
              gps_power(false);
              msg = MSG_INIT_DATA;//init -> to draw
              break;
      */
      case MSG_GPS_OFF:
        gps_power(false);
        msg = MSG_DRAW;
        break;
      case MSG_GPS_ON:
        gps_power(true);
        msg = MSG_DRAW;
        break;
      case MSG_RADIO_ON:
        transmitter_power(true);
        msg = MSG_DRAW;
        break;
      case MSG_RADIO_OFF:
        transmitter_power(false);
        msg = MSG_DRAW;
        break;
      case MSG_POWER_NEXT:
        gui_state = GUI_POWER;
        ++config_data.power_state;
        config_data.power_state &= 7;
        control_power();
        msg  = MSG_DRAW;
        break;
      case MSG_POWER_PREV:
        gui_state = GUI_POWER;
        --config_data.power_state;
        config_data.power_state &= 7;
        control_power();
        msg = MSG_DRAW;
        break;
      case MSG_RELOAD_DATA:
        todcf77(dcf77_blinker_buffer);
        reload_data = 0;
        msg = MSG_PROCESS_INPUTS;
        break;
      case MSG_DRAW:
        load_date();
        draw();
        last_rendered_high = clock_hi_counter;
        msg = MSG_PROCESS_INPUTS;
        break;
      case MSG_IDLE:
        if (!gpscontrol)
          GotoSleep();
        //delay(50);
        msg = MSG_PROCESS_INPUTS;
        break;
      default:
        msg = MSG_PROCESS_INPUTS;
        break;
    }
  }
}

void load_date() {
  DateTime tm = RTClib::now();
  date_time_years = tm.year();
  date_time_months = tm.month();
  date_time_days = tm.day();
  //date_time_days_of_week = tm.dayOfTheWeek();
  date_time_hours = tm.hour();
  date_time_minutes = tm.minute();
  date_time_seconds = tm.second();
  //ds.getTime(date_time_years, date_time_months, date_time_days, date_time_days_of_week, date_time_hours, date_time_minutes, date_time_seconds);
}

void print_two_digits(byte value) {
  if (value < 10)
    panel.print('0');
  panel.print(value, DEC);
}

void print_4_digits(unsigned long value) {
  print_two_digits((value / 100) % 100);
  print_two_digits(value % 100);
}

void print_voltage(unsigned long v) {
  panel.print(v / 100, DEC); panel.print('.');
  print_two_digits(v % 100); panel.print('v');
}

void print_current_short(unsigned long v) {
  panel.print(v / 10, DEC);
  if (v < 100) {
    panel.print('.'); panel.print(v % 10, DEC);
  }
}
void print_current(unsigned long v) {
  panel.print(v / 10, DEC);
  if (v < 100) {
    panel.print('.'); panel.print(v % 10, DEC);
  }
  panel.print(F("mA"));
}

void print_time(const tmElements_t& d) {
  print_two_digits(d.Hour);
  panel.print(':');
  print_two_digits(d.Minute);
}

void print_date(const tmElements_t& d) {
  print_two_digits(d.Day); panel.print('/'); print_two_digits(d.Month); panel.print('/'); print_two_digits(d.Year);
}

#define adc_resolution 1024
#define measurements 10

unsigned calculate_voltage(unsigned voltage_sum) { //in 10 mV
#define low_divider 100
#define high_divider 470
  //1.1 reference voltage. divider KOhm pulled up resitor. low_divider kOhm pulled down resistor. 100 scale (missed becuse of sum and reference voltage), 1024 resolution
  return voltage_sum * 11ll * (high_divider + low_divider) / low_divider / adc_resolution;
}

unsigned calculate_current(unsigned voltage_sum) { //in mA
  // RL 10kom
  // RS 1 om
  // IS = V0/(RS*RL/1000om)
  //#define reference_voltage 11/10
  //#define voltage_gain 10 //(RS*RL/1000om) == 1*10kOm/1000om
  //#define scale 1000
  //return (voltage_sum * scale * reference_voltage) / (measurements * voltage_gain * adc_resolution);
  return (voltage_sum * 110ll) / adc_resolution; // gain == 10 line above does not work because of overflow
}

const char g_dayShortNames_P[] = "ERR\0SUN\0MON\0TUE\0WED\0THU\0FRI\0SAT\0";

void draw() {
  unsigned volgage_sum_input = 0;
  unsigned volgage_sum_ext = 0;
  unsigned volage_current = 0;
  power_adc_enable();
  for (byte i = 0; i < measurements; ++i)
  {
    volgage_sum_input += analogRead(2); // parallel meauserement to make delay
    volage_current += analogRead(1);
    volgage_sum_ext += analogRead(0);
  }

  power_adc_disable();
  unsigned Vinput = calculate_voltage(volgage_sum_input);
  unsigned Vext = calculate_voltage(volgage_sum_ext);
  unsigned I = calculate_current(volage_current);//in mA
  monitor.PutData(nowTime(), volage_current);

  tmElements_t d;
  load_date_time(d);
  add_seconds_to_date(d, MOSCOW_OFFSET * SECS_PER_HOUR);


  byte hcnt = clock_hi_counter;
  word locnt = clock_lo_counter;
  byte v = get_byte(clock_hi_counter, dcf77_blinker_buffer);
  unsigned long tts =  seconds_t();
  unsigned battery_status = (4 * (Vinput - 300)) / (420 - 300);
  if (battery_status > 3)
    battery_status = 3;
  if (Vext != 0)
    battery_status = 4;

  tmElements_t last_sync_full_data;
  breakTime(last_fix_time_full, last_sync_full_data);
  add_seconds_to_date(last_sync_full_data, MOSCOW_OFFSET * SECS_PER_HOUR);
  power_spi_enable();
  panel.firstPage();
  panel.drawBitmapP(0, 0, 2, 8, battery[battery_status]);
  panel.setFont(u8g_font_helvR08r);//
  panel.setPrintPos(20, 8);
  print_voltage(Vinput); panel.print(' '); print_current(I);

  panel.drawBitmapP(112, 0, 2, 8, transmitter ? radio_on : radio_off);
  panel.drawBitmapP(97, 0, 2, 8, gpscontrol ? gps_on : gps_off);
  if (night)
    panel.drawBitmapP(88, 0, 1, 8, moon88);
  panel.nextPage();
  for (byte i = 1; i < 5; ++i) {
    panel.setFont(u8g_font_helvR24n);
    panel.setPrintPos(0, 35);
    print_time(d);

    panel.setFont(u8g_font_helvR10r);
    panel.setPrintPos(82, 22);
    print_two_digits(d.Second);
    panel.setPrintPos(100, 22);
    print_two_digits(hcnt); panel.print(' '); panel.print(v, DEC);
    /*
        panel.setFont(u8g_font_helvR08r);
        panel.setPrintPos(82, 34);
        panel.print('+'); panel.print(MOSCOW_OFFSET);
    */
    /*
        panel.setPrintPos(100, 34);
        print_4_digits(locnt);
    */
    panel.nextPage();
  }
  switch (gui_state) {
    case GUI_HOME: {
        panel.setFont(u8g_font_helvR10r);
        for (byte i = 5; i < 7; ++i) {
          panel.setPrintPos(0, 53);
          panel.print(&g_dayShortNames_P[d.Wday * 4]); panel.print(' '); print_date(d);
          panel.drawBitmapP(112, 42, 2, 12, home1216);
          panel.nextPage();
        }

        panel.setFont(u8g_font_helvR08r);
        panel.setPrintPos(0, 64); print_time(last_sync_full_data); panel.print(' '); print_date(last_sync_full_data); panel.print(' '); panel.print(tts); panel.print('s');
        panel.nextPage();
        break;
      }
    case GUI_CONTRAST: {
        panel.setFont(u8g_font_helvR10r);
        for (byte i = 0; i < 2; ++i) {
          panel.setPrintPos(0, 53);
          panel.print(config_data.gui_contrast);
          panel.drawBitmapP(112, 42, 2, 12, brightness1216);
          panel.nextPage();
        }
        panel.nextPage();
        break;
      }
    case GUI_CONFIG: {

        for (byte i = 5; i < 8; ++i) {
          panel.setPrintPos(0, 64);
          if (config_state != CONFIG_LOADED)
            panel.setFont(u8g_font_helvR08r);
          else
            panel.setFont(u8g_font_helvR10r);
          panel.print(F("load"));
          panel.setPrintPos(76, 64);
          if (config_state != CONFIG_SAVED)
            panel.setFont(u8g_font_helvR08r);
          else
            panel.setFont(u8g_font_helvR10r);
          panel.print(F("save"));
          panel.drawBitmapP(112, 42, 2, 12, update1216);
          panel.nextPage();
        }
        break;
      }
    case GUI_POWER: {
        panel.setFont(u8g_font_helvR08r);
        for (byte i = 5; i < 8; ++i) {
          panel.drawBitmapP(112, 42, 2, 12, electricity1216);
          panel.setPrintPos(0, 52); panel.print(F("radio ")); panel.print(transmitter ? F("on") : F("off")); if (night) panel.print(F(" night"));
          panel.setPrintPos(0, 62); panel.print(F("gps "));  panel.print(gpscontrol ? F("on") : F("off")); if (night) panel.print(F(" night"));
          panel.nextPage();
        }
        break;
      }
    case GUI_SHEDULER_STATUS: {
        panel.setFont(u8g_font_helvR08r);
        tmElements_t gps_time_on;
        breakTime(actions[0].next, gps_time_on);
        tmElements_t radio_time_on;
        breakTime(actions[2].next, radio_time_on);
        for (byte i = 5; i < 8; ++i) {
          panel.setPrintPos(0, 52); panel.print(F("gps   on:")); print_time(gps_time_on); panel.print(' '); print_date(gps_time_on);
          panel.setPrintPos(0, 64); panel.print(F("radio on:")); print_time(radio_time_on); panel.print(' '); print_date(radio_time_on);
          panel.drawBitmapP(112, 42, 2, 12, alert1216);
          panel.nextPage();
        }
        break;
      }
    case GUI_POWER_STATISTIC: {
        panel.setFont(u8g_font_helvR08r);
        unsigned long offset = 0;
        for (byte i = 0; i < 3; ++i) {
          panel.drawBitmapP(112, 42, 2, 12, battery1216);
          for (byte j = 0; j < 4; ++j) {
            unsigned long realPeriod = 0, lu = 0;
            unsigned long current = calculate_current(monitor.NextAlignedData(offset, power_statistic_periods[current_state_mode], realPeriod, lu));
            bool period_border = (monitor.GetLastUpdated() - offset) % ( power_statistic_periods[current_state_mode] * 12ll) == 0;
            if (period_border) {
              panel.setPrintPos(j * 24, 48 + i * 8); panel.print('>');
            }
            panel.setPrintPos(4 + j * 24, 48 + i * 8); print_current_short(current);
            offset += realPeriod;
          }
          panel.setPrintPos(97, 64); panel.print("mA/"); panel.print(&power_statistic_periods_text[3 * current_state_mode]);
          panel.nextPage();
        }
        break;
      }
  }
  power_spi_disable();
}

void setupInterrupts() {
  //  attachInterrupt(MEANDER_SIGNAL_INTERRUPT_PIN, clock, RISING);
  EICRA = (EICRA & ~((1 << ISC10) | (1 << ISC11))) | (RISING << ISC10);
  EIMSK |= (1 << INT1);
}

void loop() {
  Serial.begin(9600);
  setupInterrupts();
  analogReference(INTERNAL);
  ds.enable32kHz(false);
  ds.enableOscillator(true, false, COUNT_OF_SIGNAL_PER_SECOND_MODE);
  load_config();
  process();
}

ISR(INT1_vect) {
  if (transmitter) {
    clock_lo_counter++;
    switch (clock_lo_counter) {
      case 1:
        if (clock_hi_counter == 2)
          reload_data = 1;
        if (clock_hi_counter != 59)
          PORTD |= B00100000; //digitalWrite(5, HIGH);
        break;
      case SHORT_SIGNAL_COUNT+1:
        if (get_byte(clock_hi_counter, dcf77_blinker_buffer))
          break;
      case LONG_SIGNAL_COUNT+1:
        PORTD &= B11011111; //digitalWrite(5, LOW);
        break;
      case COUNT_OF_SIGNAL_PER_SECOND:
        ++seconds_from_start;
        ++clock_hi_counter;
        clock_lo_counter = 0;
        if (clock_hi_counter == 60)
          clock_hi_counter = 0;
        break;
      default:
        break;
    }
  }
  else
  {
    ++seconds_from_start;
    ++clock_hi_counter;
    clock_lo_counter = 0;
    if (clock_hi_counter == 60)
      clock_hi_counter = 0;
  }
}

