#include <Wire.h>

#define DEBUG 1

struct Time {
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

class ShiftRegister { // 74HC595
  public:
    ShiftRegister(uint8_t latch_pin, uint8_t data_pin, uint8_t clock_pin) {
      _latch_pin = latch_pin;
      _data_pin = data_pin;
      _clock_pin = clock_pin;
      pinMode(_latch_pin, OUTPUT);
      pinMode(_data_pin, OUTPUT);
      pinMode(_clock_pin, OUTPUT);
    }
    void write(uint8_t hour, uint8_t minute) {
      digitalWrite(_latch_pin, LOW);
      shiftOut(_data_pin, _clock_pin, MSBFIRST, minute);
      shiftOut(_data_pin, _clock_pin, MSBFIRST, hour);
      digitalWrite(_latch_pin, HIGH);
    }
  private:
    uint8_t _data_pin;
    uint8_t _latch_pin;
    uint8_t _clock_pin;
};

class Rtc { // DS1307
  public:
    Rtc(){};
    bool init() {
      /* Return value: true on successful initialization,
         false if couldn't get response from rtc */
      // Make sure that rtc is running
      Wire.beginTransmission(_address);
      Wire.write(0x00);
      Wire.endTransmission();
      Wire.requestFrom(_address, 0x01, true);
      while (Wire.available()) {
        uint8_t c = Wire.read();
        // If oscillator is off, turn it on
        if (c >> 7 == 1) {
            Serial.println(0x80 & c, BIN);
            Wire.beginTransmission(_address);
            Wire.write(0x00);
            Wire.write(0x78 & c);
            Wire.endTransmission();
        }
      return true;
      }
      return false;
    }
    // I don't care about the date, this clock only displays hours and minutes
    void set_time(Time time) {
      Wire.beginTransmission(_address);
      Wire.write(0x00);
      Wire.write(0x78 & _dec2bcd(time.second)); // Make sure not to turn off the rtc
      Wire.write(_dec2bcd(time.minute));
      Wire.write(0x3f & _dec2bcd(time.hour)); // Make sure the clock is in 24h mode
      Wire.endTransmission();
    }
    void get_time(Time *time) {
      time->second = 0; // When reading time I don't care about seconds
      Wire.beginTransmission(_address);
      Wire.write(0x01);
      Wire.endTransmission();
      Wire.requestFrom(_address, 0x02, true);
        if (Wire.available()) {
          uint8_t c = Wire.read();
          time->minute = _bcd2dec(c);
        }
        else // In case something goes wrong while reading
          time->minute = 0;
        if (Wire.available())  {
          uint8_t c = Wire.read();
          time->hour = _bcd2dec(c);
        }
        else
          time->hour = 0;
    }
  private:
    uint8_t _dec2bcd(uint8_t value) {
      return (value/10 << 4) | (value%10);
    }
    uint8_t _bcd2dec(uint8_t value) {
      return (value >> 4)*10 + (value & 0x0f);
    }
 // const byte _address = 0x68; // I2C address
  const int _address = 0b1101000;
  uint8_t _hour, _minute;
};

//#if DEBUG == 1
void serial_print(Time time) {
  Serial.print(time.hour);
  Serial.print(':');
  Serial.print(time.minute);
  Serial.print(':');
  Serial.print(time.second);
  Serial.println();
}
//#endif

// Globals defginition
ShiftRegister shiftRegister(8, 11, 12);
Rtc rtc;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  bool rtc_initialized;
  #if DEBUG == 1
  printf("Initializing RTC");
  #endif
  while (!rtc_initialized) {
    rtc_initialized = rtc.init();
  }
  #if DEBUG == 1
  printf("RTC initialized");
  #endif
  struct Time time = {
    21, 12, 32
  };
  rtc.set_time(time);
}

void loop() {
  static struct Time time;
  static uint8_t last_minute;
  rtc.get_time(&time);
  if (time.minute != last_minute) {// Only display new time when needed
    #if DEBUG == 1
    Serial.println("New time displayed");
    #endif
    shiftRegister.write(time.hour, time.minute);
  }
  last_minute = time.minute;
  #if DEBUG == 1
  serial_print(time);
  #endif
  if (Serial.available()) {
    uint8_t buffer[5];
    Serial.readBytes(buffer, 5);
    if (buffer[0] == 0xca && buffer[1] == 0xfe) { // Control bytes
      struct Time new_time;
      new_time.hour   = buffer[2];
      new_time.minute = buffer[3];
      new_time.second = buffer[4];
      rtc.set_time(new_time);
      Serial.write(0x00);
    }
    else // Wrong message was sent by host
      Serial.write(0xff);
  }
  delay(1000);
}
