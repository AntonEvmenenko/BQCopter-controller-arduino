#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <CircularBuffer.h>

#define NEED_CALIBRATION 0
#define BT_SERIAL Serial
#define DEBUG_SERIAL softwareSerial

#define AXIS_MAX 10000
#define AXIS_MIN 0
#define MIN_AXIS_DELTA 500
#define MIN_DELTA 50

#define NRF24L01_PACKAGE_SIZE 5

enum {
    CONTROL = 0
};

int LH_INV = 0, LV_INV = 0, RH_INV = 1, RV_INV = 0;

#if NEED_CALIBRATION == 1
int LH_MIN = AXIS_MAX, LV_MIN = AXIS_MAX, RH_MIN = AXIS_MAX, RV_MIN = AXIS_MAX;
int LH_MAX = AXIS_MIN, LV_MAX = AXIS_MIN, RH_MAX = AXIS_MIN, RV_MAX = AXIS_MIN;  
float RH_DELTA, RV_DELTA;
int calibration = 1;
#else
int LH_MIN, LV_MIN, RH_MIN, RV_MIN;
int LH_MAX, LV_MAX, RH_MAX, RV_MAX;
float RH_DELTA, RV_DELTA;
int calibration = 0;
#endif

int nrf24l01Initialized = 0;

SoftwareSerial softwareSerial(2, 3); // RX, TX
CircularBuffer<char, 128> outputBTSerialBuffer;
CircularBuffer<char, 128> inputBTSerialBuffer;

/*
struct Data {
    int16_t id;
    float data[n];
    int16_t checksum;
};
sizeof(Data) = 8 + n * 8
*/

struct Data {
    byte id;
    float data;
};

void printByteAsHex(byte x) {
    byte b1 = (x >> 4) & 0x0F;
    byte b2 = x & 0x0F;
    outputBTSerialBuffer.push_back((char)((b1 < 10) ? b1 + '0' : b1 - 10 + 'A'));
    outputBTSerialBuffer.push_back((char)((b2 < 10) ? b2 + '0' : b2 - 10 + 'A'));
}

void printInt16AsHex(int16_t x) {
    printByteAsHex(((byte*)&x)[1]);
    printByteAsHex(((byte*)&x)[0]);
}

void printFloatAsHex(float x) {
    for (int i = 3; i >= 0; --i) {
        printByteAsHex(((byte*)&x)[i]);
    }
}

void sendToBT(int16_t id, int count, float first, ...) {
    printInt16AsHex(id);
    int16_t checksum = id;
    float* pointer = &first;
    unsigned i = 0;

    while (count--) {
        int16_t* tempPointer = (int16_t*)pointer;
        checksum += tempPointer[0] + tempPointer[1];
        printFloatAsHex(*(pointer++));
    }

    printInt16AsHex(checksum);
    outputBTSerialBuffer.push_back('\n');
}

byte parseByte(char* x) {
    char b1 = x[0];
    char b2 = x[1];
    return ((((b1 >= '0' && b1 <= '9') ? b1 - '0' : b1 - 'A' + 10) << 4) & 0xF0) |
            (((b2 >= '0' && b2 <= '9') ? b2 - '0' : b2 - 'A' + 10) & 0x0F);
}

int16_t parseInt16(char* x) {
    int16_t result;
    byte* pointer = (byte*)&result;

    pointer[0] = parseByte(x + 2);
    pointer[1] = parseByte(x);

    return result;
}

float parseFloat(char* x) {
    float result;
    byte* pointer = (byte*)&result;

    pointer[0] = parseByte(x + 6);
    pointer[1] = parseByte(x + 4);
    pointer[3] = parseByte(x + 2);
    pointer[4] = parseByte(x);

    return result;
}

float range(float x, float minX, float maxX) {
    return max(min(x, maxX), minX);
}

void printThroughComma(int count, int first, ...) {
    int *pointer = &first;
    while(count--) {
        DEBUG_SERIAL.print(*(pointer++));
        DEBUG_SERIAL.print(' ');
    }
    DEBUG_SERIAL.println();
}

void EEPROM_float_write(int addr, float val) {
    byte *pointer = (byte *)&val;
    for(byte i = 0; i < 4; i++) {
        EEPROM.update(addr + i, pointer[i]);
    }
}

float EEPROM_float_read(int addr) {
    byte pointer[4];
    for(byte i = 0; i < 4; i++) {
        pointer[i] = EEPROM.read(addr + i);
    }
    return *((float*)&pointer);
}

void EEPROM_int16_write(int addr, int16_t val) {
    byte *pointer = (byte *)&val;
    for(byte i = 0; i < 2; i++) {
        EEPROM.update(addr + i, pointer[i]);
    }
}

int16_t EEPROM_int16_read(int addr) {
    byte pointer[2];
    for(byte i = 0; i < 2; i++) {
        pointer[i] = EEPROM.read(addr + i);
    }
    return *((int16_t*)&pointer);
}

void save_config() {
    EEPROM_int16_write(0, LH_MIN);
    EEPROM_int16_write(2, LV_MIN);
    EEPROM_int16_write(4, RH_MIN);
    EEPROM_int16_write(6, RV_MIN);
    EEPROM_int16_write(8, LH_MAX);
    EEPROM_int16_write(10, LV_MAX);
    EEPROM_int16_write(12, RH_MAX);
    EEPROM_int16_write(14, RV_MAX);
    EEPROM_float_write(16, RH_DELTA);
    EEPROM_float_write(20, RV_DELTA);
}

void load_config() {
    LH_MIN = EEPROM_int16_read(0);
    LV_MIN = EEPROM_int16_read(2);
    RH_MIN = EEPROM_int16_read(4);
    RV_MIN = EEPROM_int16_read(6);
    LH_MAX = EEPROM_int16_read(8);
    LV_MAX = EEPROM_int16_read(10);
    RH_MAX = EEPROM_int16_read(12);
    RV_MAX = EEPROM_int16_read(14);
    RH_DELTA = EEPROM_float_read(16);
    RV_DELTA = EEPROM_float_read(20);
}

void setup() {
    BT_SERIAL.begin( 115200 );
    DEBUG_SERIAL.begin( 9600 );

    //pinMode(2, INPUT_PULLUP);
    //pinMode(3, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
    pinMode(6, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
  
    pinMode(A0, INPUT_PULLUP);
    pinMode(A1, INPUT_PULLUP);
    pinMode(A2, INPUT_PULLUP);
    pinMode(A3, INPUT_PULLUP);

#if NEED_CALIBRATION == 0
    load_config();
#endif
}

void loop() {
    if (calibration) {
        int LH = analogRead(A4);
        int LV = analogRead(A5);
        int RH = analogRead(A6);
        int RV = analogRead(A7);

        LH_MIN = min(LH_MIN, LH);
        LV_MIN = min(LV_MIN, LV);
        RH_MIN = min(RH_MIN, RH);
        RV_MIN = min(RV_MIN, RV);
        LH_MAX = max(LH_MAX, LH);
        LV_MAX = max(LV_MAX, LV);
        RH_MAX = max(RH_MAX, RH);
        RV_MAX = max(RV_MAX, RV); 
    
        if (LH_MAX - LH_MIN > MIN_AXIS_DELTA && LV_MAX - LV_MIN > MIN_AXIS_DELTA && RH_MAX - RH_MIN > MIN_AXIS_DELTA && RV_MAX - RV_MIN > MIN_AXIS_DELTA &&
            abs(LH - (LH_MAX + LH_MIN)/2) < MIN_DELTA && abs(LV - LV_MIN) < MIN_DELTA && abs(RH - (RH_MAX + RH_MIN)/2) < MIN_DELTA && abs(RV - (RV_MAX + RV_MIN)/2) < MIN_DELTA){
            LV_MIN += 20;

            RH_DELTA = (RH - (RH_MAX + RH_MIN) / 2.0f);
            RV_DELTA = (RV - (RV_MAX + RV_MIN) / 2.0f);

            save_config();

            calibration = 0;
        }
    } else {
        if (!nrf24l01Initialized) {
            Mirf.spi = &MirfHardwareSpi;
            Mirf.init( );
            Mirf.setRADDR( ( byte* )"serv1" );
            Mirf.channel = 90;
            Mirf.payload = sizeof( byte ) * NRF24L01_PACKAGE_SIZE;
            Mirf.config( );
            nrf24l01Initialized = 1;
        }

        /*
        printThroughComma(8, LH_MIN, LV_MIN, RH_MIN, RV_MIN, LH_MAX, LV_MAX, RH_MAX, RV_MAX);
        DEBUG_SERIAL.print(RH_DELTA);
        DEBUG_SERIAL.print(' ');
        DEBUG_SERIAL.println(RV_DELTA);
        */

        byte package[NRF24L01_PACKAGE_SIZE];
        int packageGotFromBT = 0;

        while (BT_SERIAL.available()) {
            char current = BT_SERIAL.read();

            if (current == '\n') {
                if (inputBTSerialBuffer.get_size() > 4 && (inputBTSerialBuffer.get_size() - 8) % 8 == 0) {
                    char buffer[128];
                    for (unsigned i = 0; i < inputBTSerialBuffer.get_size(); ++i) {
                        buffer[i] = inputBTSerialBuffer.get(i);
                    }

                    int16_t id = parseInt16(buffer);
                    int16_t tempChecksum = id;

                    int n = (inputBTSerialBuffer.get_size() - 8) / 8;
                    float data[10];
                    for (unsigned i = 0; i < n; ++i) {
                        data[i] = parseFloat(buffer + 4 + i * 8);
                        tempChecksum += parseInt16(buffer + 4 + i * 8) + parseInt16(buffer + 4 + i * 8 + 4);
                    }

                    int16_t checksum = parseInt16(buffer + 4 + n * 8);

                    if (checksum == tempChecksum) {
                        // packages with only ONE data element now supported
                        if (n == 1) {
                            byte* pointer = (byte*)(data);

                            package[0] = id;
                            package[1] = pointer[0];
                            package[2] = pointer[1];
                            package[3] = pointer[2];
                            package[4] = pointer[3];

                            packageGotFromBT = 1;
                        }
                    }
                }
                inputBTSerialBuffer.clear();
            } else {
                inputBTSerialBuffer.push_back(current);
            }
        }

        byte LH = byte(range((analogRead(A4) - LH_MIN) / float(LH_MAX - LH_MIN), 0.0f, 1.0f) * 255);
        byte LV = byte(range((analogRead(A5) - LV_MIN) / float(LV_MAX - LV_MIN), 0.0f, 1.0f) * 255);
        byte RH = byte(range((analogRead(A6) - RH_MIN - RH_DELTA) / float(RH_MAX - RH_MIN), 0.0f, 1.0f) * 255);
        byte RV = byte(range((analogRead(A7) - RV_MIN - RV_DELTA) / float(RV_MAX - RV_MIN), 0.0f, 1.0f) * 255);
  
        LH = LH_INV ? (255 - LH) : LH;
        LV = LV_INV ? (255 - LV) : LV;
        RH = RH_INV ? (255 - RH) : RH;
        RV = RV_INV ? (255 - RV) : RV;

        //printThroughComma(4, LH, LV, RH, RV);

        if (!packageGotFromBT) {
            package[0] = CONTROL; // id
            package[1] = LV; // throttle
            package[2] = RH; // roll
            package[3] = RV; // pitch
            package[4] = 0; // control by camera
        }

        Mirf.setTADDR( ( byte* )"serv2" );
        Mirf.send(package); //268 us

        if (outputBTSerialBuffer.get_size() > 0) {
            BT_SERIAL.print(outputBTSerialBuffer.pop_front());
        } else {
            sendToBT(CONTROL, 3, float(LV - 128), float(RH - 128), float(RV - 128));
        }

    }
}

