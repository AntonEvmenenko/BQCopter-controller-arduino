#include <SoftwareSerial.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

#define AXIS_MAX 10000
#define AXIS_MIN 0
#define MIN_AXIS_DELTA 500
#define MIN_DELTA 50

int LH_MIN = AXIS_MAX, LV_MIN = AXIS_MAX, RH_MIN = AXIS_MAX, RV_MIN = AXIS_MAX;
int LH_MAX = AXIS_MIN, LV_MAX = AXIS_MIN, RH_MAX = AXIS_MIN, RV_MAX = AXIS_MIN;  
int LH_INV = 0, LV_INV = 0, RH_INV = 0, RV_INV = 0; 

int calibration = 1;

SoftwareSerial softwareSerial(2, 3); // RX, TX

struct Data {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t checksum;
};

void printByteAsHex(byte x) {
    byte b1 = (x >> 4) & 0x0F;
    byte b2 = x & 0x0F;
    softwareSerial.print((char)((b1 < 10) ? b1 + '0' : b1 - 10 + 'A'));
    softwareSerial.print((char)((b2 < 10) ? b2 + '0' : b2 - 10 + 'A'));
}

float range(float x, float minX, float maxX)
{
    return max(min(x, maxX), minX);
}

void printThroughComma(int count, int first, ...)
{
    int *pointer = &first;
    while(count--) {
        Serial.print(*pointer);
        Serial.print(' ');
        pointer++;
    }
    Serial.println();
}

void setup() {
    Serial.begin( 9600 ); 
    softwareSerial.begin( 9600 );

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
            calibration = 0;

            Mirf.spi = &MirfHardwareSpi;
            Mirf.init( );
            Mirf.setRADDR( ( byte* )"serv1" );
            Mirf.channel = 90;
            Mirf.payload = sizeof( byte ) * 3;
            Mirf.config( );
        }
    } else {
        byte LH = byte(range((analogRead(A4) - LH_MIN) / float(LH_MAX - LH_MIN), 0.0f, 1.0f) * 255);
        byte LV = byte(range((analogRead(A5) - LV_MIN) / float(LV_MAX - LV_MIN), 0.0f, 1.0f) * 255);
        byte RH = byte(range((analogRead(A6) - RH_MIN) / float(RH_MAX - RH_MIN), 0.0f, 1.0f) * 255);
        byte RV = byte(range((analogRead(A7) - RV_MIN) / float(RV_MAX - RV_MIN), 0.0f, 1.0f) * 255);
  
        LH = LH_INV ? (255 - LH) : LH;
        LV = LV_INV ? (255 - LV) : LV;
        RH = RH_INV ? (255 - RH) : RH;
        RV = RV_INV ? (255 - RV) : RV;
  
        byte data[ 3 ];
        data[ 0 ] = LV; // throttle
        data[ 1 ] = RH; // roll
        data[ 2 ] = RV; // pitch
    
        //Serial.println(data[0]);
        //Mirf.setTADDR( ( byte* )"serv2" );
        //Mirf.send( data );
      
        Data data1;
        data1.roll = LV - 128;
        data1.pitch = RH - 128;
        data1.yaw = RV - 128;
        data1.checksum = data1.roll + data1.pitch + data1.yaw;
      
        for(int i = 0; i < sizeof(data1) / 2; ++i) {
            printByteAsHex(((byte*)&data1)[2 * i + 1]);
            printByteAsHex(((byte*)&data1)[2 * i]);
        }
    
        softwareSerial.print('\n');
    }
}

