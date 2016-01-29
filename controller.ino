#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

void setup( ) {        
    Serial.begin( 9600 ); 
    Mirf.spi = &MirfHardwareSpi;
    Mirf.init( );  
    Mirf.setRADDR( ( byte* )"serv1" );
    Mirf.channel = 90;
    Mirf.payload = sizeof( byte ) * 4;
    Mirf.config( );

    pinMode(5, INPUT);
}

void loop() {
    byte data[ 3 ];
    data[ 0 ] = 255 - ( byte )( analogRead( A4 ) / 4 ); // throttle
    data[ 1 ] = 255 - ( byte )( analogRead( A1 ) / 4 ); // roll
    data[ 2 ] = ( byte )( analogRead( A3 ) / 4 ); // pitch
    data[ 3 ] = (digitalRead(5) == HIGH) ? 0 : 1;
    
    //Serial.println(data);
    Mirf.setTADDR( ( byte* )"serv2" );
    Mirf.send( data );
}
