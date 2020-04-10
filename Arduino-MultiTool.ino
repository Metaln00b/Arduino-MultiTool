#include <LiquidCrystal.h>

/*******************************************************************************
* Kapazität Messgerät 10nF bis 2000uF und Drehzahlmesser                       *
*******************************************************************************/


#define messPin 1            // Analog Messeingang
#define encoderPin 2         // Induktiver Sensor-Pin
#define pulsePerRound 1      // Pulse pro Umdrehung
#define ladePin 12           // Kondensator-Lade-Pin über einen 10kOhm Widerstand
#define entladePin 13        // Kondensator-Entlade-Pin über einen 220 Ohm Widerstand 
#define widerstand  9927.0F  // 10 kOhm > gemessen 9,927 kOhm

/* Kondensator */
unsigned long startZeit;
unsigned long vergangeneZeit;
float microFarad;
float nanoFarad;

/* Induktiver Sensor */
unsigned int rpm;
volatile unsigned int pulses;
unsigned long timeold;

/* LCD */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int lcd_key     = 0;
int adc_key_in  = 0;


/*******************************************************************************
* Die Knöpfe sind auf diese Werte zentriert: 0, 144, 329, 504, 741             *
* Wir addieren ca. 50 zu diesen Werten und prüfen, ob wir in der Nähe sind.    *
*******************************************************************************/

int read_LCD_buttons() {
    adc_key_in = analogRead(0);

    if (adc_key_in > 1000)
    {
        /* Wir machen dies aus Geschwindigkeitsgründen zur ersten Option,
        da es das wahrscheinlichste Ergebnis sein wird.*/
        return btnNONE;
    }

    // V1.1 vom Velleman VMA203
    /*
    if (adc_key_in < 50)
    {
        return btnRIGHT;
    }
    if (adc_key_in < 250)
    {
        return btnUP;
    }
    if (adc_key_in < 450)
    {
        return btnDOWN;
    }
    if (adc_key_in < 650)
    {
        return btnLEFT;
    }
    if (adc_key_in < 850)
    {
        return btnSELECT;
    }
    */
    // V1.0 vom Velleman VMA203

    if (adc_key_in < 50)
    {
        return btnRIGHT;
    }
    if (adc_key_in < 195)
    {
        return btnUP;
    }
    if (adc_key_in < 380)
    {
        return btnDOWN;
    }
    if (adc_key_in < 555)
    {
        return btnLEFT;
    }
    if (adc_key_in < 790)
    {
        return btnSELECT;
    }

    return btnNONE;
}

void read_CAP() {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Laden...");

    /* Kondensator laden */
    digitalWrite(ladePin, HIGH);                        // ladePin auf 5V, das Laden beginnt
    startZeit = micros();                               // Startzeit merken

    while(analogRead(messPin) < 648)
    {
        // 647 ist 63.2% von 1023 (5V)
    }

    vergangeneZeit = micros() - startZeit - 114;        // 0-Messung abziehen (112-116 us)

    if(vergangeneZeit > 4294960000)
    {
        vergangeneZeit = 0;                             // Minuswerte auf 0 setzen (ist long, deshalb der hohe Wert)
    }

    /* Umrechnung: us zu Sekunden ( 10^-6 ) und Farad zu mikroFarad ( 10^6 ),  netto 1 */
    microFarad = ( (float)vergangeneZeit / widerstand );
    Serial.print(vergangeneZeit);                       // Zeit ausgeben
    Serial.println(" nS");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(vergangeneZeit);
    lcd.print(" nS");
    delay(1000);

    if (microFarad > 1)
    {
        if(microFarad < 100)
        {
            Serial.print(microFarad, 2);                 // uF.x ausgeben
            Serial.println(" uF");

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(microFarad, 2);
            lcd.print(" uF");
        }
        else
        {
            Serial.print( (long)microFarad );           // uF ausgeben
            Serial.println(" uF");

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print( (long)microFarad );
            lcd.print(" uF");
        }
    }
    else
    {
        nanoFarad = microFarad * 1000.0;                // in nF umrechnen
        if(nanoFarad > 10)
        {
            Serial.print( (long)nanoFarad );            // nF ausgeben
            Serial.println(" nF");

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print( (long)nanoFarad );
            lcd.print(" nF");
        }
        else
        {
            Serial.println("kleiner 10 nF");

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("kleiner 10 nF");
        }
    }

    /* Kondensator entladen */
    digitalWrite(ladePin, LOW);                         // ladePin auf 0V 
    pinMode(entladePin, OUTPUT);                        // entladePin wird Ausgang 
    digitalWrite(entladePin, LOW);                      // entladePin auf 0V 

    lcd.setCursor(0, 1);
    lcd.print("Entaden...");

    while(analogRead(messPin) > 0)
    {
        // bis der Kondensator entladen ist (0V)
    }
    pinMode(entladePin, INPUT);                         // entladePin wird Eingang

    while( (micros() - startZeit) < 500000 )
    {
        // bis 500ms warten, d.h. max 2 Ausgaben pro Sekunde
    }
    lcd.setCursor(0, 1);
    lcd.print("          ");
}

void count_RPM()
{
    pulses++;
}

void setup_RPM()
{
    lcd.clear();
    lcd.print("Speed");

    pinMode(encoderPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPin), count_RPM, RISING);

    pulses = 0;
    rpm = 0;
    timeold = 0;
}

void loop_RPM()
{
    while(true) {
        if ( millis() - timeold >= 1000 ) {
            detachInterrupt(digitalPinToInterrupt(encoderPin));
            rpm = pulses * (60/pulsePerRound);
            pulses = 0;
            timeold = millis();
        
            lcd.setCursor (0, 1); 
            lcd.print(rpm);
            lcd.print("rpm         ");
        
            lcd.setCursor(8, 1);
        
            attachInterrupt(digitalPinToInterrupt(encoderPin), count_RPM, RISING);
        }
    }
}

void setup() {
    pinMode(ladePin, OUTPUT);                           // ladePin als Ausgang
    digitalWrite(ladePin, LOW);

    Serial.begin(9600);
    lcd.begin(16, 2);

    Serial.println("Kappa v1.0");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Kappa v1.0");
}

void loop() {
    lcd.setCursor(0, 1);
    lcd_key = read_LCD_buttons();

    switch (lcd_key)
    {
        case btnRIGHT:
        {
            lcd.print("RIGHT           ");
            setup_RPM();
            loop_RPM();
            break;
        }
        case btnLEFT:
        {
            lcd.print(adc_key_in);
            lcd.print(" v           ");
            break;
        }
        case btnUP:
        {
            lcd.print("UP              ");
            break;
        }
        case btnDOWN:
        {
            lcd.print("DOWN            ");
            break;
        }
        case btnSELECT:
        {
            lcd.print("SELECT          ");
            read_CAP();
            break;
        }
        case btnNONE:
        {
            lcd.print("Press SELECT    ");
            break;
        }
    }
}
