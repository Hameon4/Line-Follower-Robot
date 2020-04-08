#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/delay.h>
#include <math.h>
int val0, val1, val2, val3, val4, valColor, avg;
void setSpeedd(int speed1, int speed2) {
  TCCR0A = 0; //reset the register
  TCCR0B = 0; //reset  the register
  TCCR0A = 0b10100011; // fast pwm mode
  TCCR0B = 0b00000101; // prescaler 1024
  OCR0A = speed1; //duty cycle for pin 6
  OCR0B = speed2; //duty cycle for pin 5

}

void initADC(void) {
  ADMUX |= (1 << REFS0); /* reference voltage
  to AVCC */
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); /* ADC clock
  prescaler 128 */
  ADCSRA |= (1 << ADEN); /* enable ADC */
}
uint16_t readADC(uint8_t channel) {
  ADMUX = (0b01000000 & ADMUX) | channel; /* Select input Pin */
  ADMUX &= ~(1 << ADLAR); /* Right Adjust */
  ADCSRA |= (1 << ADSC); /* Start A to D conversion */
  // wait for measurement to finish
  // it is also possible to use this AVR macro : loop_until_bit_is_clear(ADCSRA,ADSC);
  while (ADCSRA & (1 << ADSC)) {}; /* ADSC is cleared to 0 when a
  conversion completes. */
  return (ADC); /* returns the conversion result */
}

  int color(void)
  {
  //s3 to HIGH (Already s2 in LOW) for BLUE values
  int blue;
  //Sensor set to detect blue color using the datasheet
  PORTB &= ~(1 << PINB3);
  PORTB |= (1 << PINB4);
  //input a pulse
  blue = pulseIn(10, digitalRead(10) == HIGH ? LOW : HIGH);
  return blue;

  }

// this function makes the right tires clockwise
// and the left tires counter hence going forward
void forward(void) {

  PORTD |= (1 << PORTD2);
  PORTD &= ~(1 << PORTD4);
  PORTD |= (1 << PORTD7);
  PORTB &= ~(1 << PORTB0);

}
// this function makes the right tires counter clockwise
// and the left tires counter hence going on reverse
void reverse(void) {

  PORTD &= ~(1 << PORTD2);
  PORTD |= (1 << PORTD4);
  PORTD &= ~(1 << PORTD7);
  PORTB |= (1 << PORTB0);
}
// this function makes the right tires work
// and the left tires not therefore making the car go to the left
void right(void) {

  PORTD |= (1 << PORTD2);
  PORTD &= ~(1 << PORTD4);
  PORTD &= ~(1 << PORTD7);
  PORTB &= ~(1 << PORTB0);
}
// this function makes the all tires
// go clockwise for the sharp left turns
void sright(void) {

  /*PORTD |= (1 << PORTD2);
    PORTD &= ~(1 << PORTD4);
    PORTD &= ~(1 << PORTD7);
    PORTB |= (1 << PORTB0);
  */
  PORTD  |= (1 << PORTD2);
  PORTD &= ~(1 << PORTD4);
  PORTD &= ~(1 << PORTD7);
  PORTB |=  (1 << PORTB0);
}
// this function makes the left tires work
// and the right tires not therefore making the car go to the right
void left(void) {

  PORTD &= ~(1 << PORTD2);
  PORTD &= ~(1 << PORTD4);
  PORTD |= (1 << PORTD7);
  PORTB &= ~(1 << PORTB0);
}
// this function makes the all tires
// go counter clockwise for the sharp right turns
void sleft(void) {

  PORTD &= ~(1 << PORTD2);
  PORTD |= (1 << PORTD4);
  PORTD |= (1 << PORTD7);
  PORTB &= ~(1 << PORTB0);
}
// this lets the car rotate 180 degrees
void fullturn() {

  PORTD |= (1 << PORTD2);
  PORTD &= ~(1 << PORTD4);
  PORTD &= ~(1 << PORTD7);
  PORTB |= (1 << PORTB0);
}
//this function stops the car
void stopp() {
  PORTD &= ~(1 << PORTD2);
  PORTD &= ~(1 << PORTD4);
  PORTD &= ~(1 << PORTD7);
  PORTB &= ~(1 << PORTB0);
  _delay_ms(0.5);
}
int main(void) {

  DDRD = 0xff;  // setting all ports D as outputs
  DDRC = 0x00;  // setting all ports C as inputs
  DDRB = 0xff;  // setting all ports B as outputs
  DDRB &= ~(1 << PORTB2); // setting Pin 2 on port B as input
  PORTB |= (1 << PORTB1); // Initialize Pin 1 on PortB to 1
  PORTB |= (1 << PORTB5); // Initialize Pin 5 on PortB to 1
  initADC();
  Serial.begin(9600);
  /*
    forward();
    set_Speed(100);
    _delay_ms(3000);
    reverse();
    set_Speed(100);
    _delay_ms(3000);
    right();
    set_Speed(100);
    _delay_ms(3000);
  */
  while (1) {
    //reading all the five sensors
    val0 = readADC(0);  // reading from channel 0
    val1 = readADC(1);  // reading from channel 1
    val2 = readADC(2);  // reading from channel 2
    val3 = readADC(3);  // reading from channel 3
    val4 = readADC(4);  // reading from channel 4
    //valColor = color(); // raeding from the color sensor


    //Serial.println(valColor);
    //if (valColor<=8 && valColor>=7) {

    //stopp();
    //sright();
    //setSpeedd(255, 255);
    //_delay_ms(500);

    //}
    if (val0 > 100 && val1 < 100 && val2 > 100 && val3 < 100 && val4 > 100) { // a condition to go forward
      forward();
      setSpeedd(200, 200);
    } else if (val1 > 100 && val2 > 100 && val3 < 100 && val4 > 100 && val0 > 100) { // a condition to go right
      left();
      setSpeedd(200, 200);
    } else if (val1 < 100 && val2 > 100 && val3 > 100 && val4 > 100 && val0 > 100) {  // a condition to go left
      right();
      setSpeedd(200, 200);
    } else if ((val0 > 100 && val1 > 100 && val2 > 100 && val3 < 100 && val4 < 100)
               || (val0 > 100 && val1 > 100 && val2 > 100 && val3 > 100 && val4 < 100)
               || (val0 > 100 && val1 < 100 && val2 > 100 && val3 < 100 && val4 < 100)
               || (val0 > 100 && val1 > 100 && val2 > 100 && val3 < 100 && val4 > 100)) { // a condition to turn on sharp rights
      sleft();
      setSpeedd(255, 200);
    } else if ((val4 > 100 && val3 > 100 && val2 > 100 && val1 < 100 && val0 < 100)
               || (val4 > 100 && val3 > 100 && val2 > 100 && val1 > 100 && val0 < 100)
               || (val4 > 100 && val3 < 100 && val2 > 100 && val1 < 100 && val0 < 100)
               || (val4 > 100 && val3 > 100 && val2 > 100 && val1 < 100 && val0 > 100)) { // a condition to turn on sharp left
      sright();
      setSpeedd(200, 255);
    }  else if (val2 < 100) { // a condition to avoid the obstacle
      stopp();
      _delay_ms(150);
      reverse();
      setSpeedd(255, 255);
      _delay_ms(150);
      stopp();
      _delay_ms(150);
      sright();
      setSpeedd(255, 255);
      _delay_ms(400);
      forward();
      setSpeedd(255, 255);
      _delay_ms(200);
      sleft();
      setSpeedd(255, 255);
      _delay_ms(500);
      forward();
      setSpeedd(255, 255);
      _delay_ms(800);
      sleft();
      setSpeedd(255, 255);
      _delay_ms(400);
      forward();
      setSpeedd(255, 255);
      _delay_ms(200);
      sright();
      setSpeedd(255, 255);
      _delay_ms(400);
    } else if (val4 > 100 && val3 > 100 && val2 > 100 && val1 > 100 && val0 > 100) { //Pass through black spot
      forward();
      _delay_ms(100);
    } else if (val4 < 100 && val3 < 100 && val2 < 100 && val1 < 100 && val0 < 100) {  // stops the car
      stopp();
    } else { //stops
      stopp();
      //setSpeedd(0);
    }
    //Serial.println(PORTD);
  }
  return 0;
}
