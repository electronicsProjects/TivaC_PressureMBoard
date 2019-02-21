/*ASDXPressureSensor
   Description:
   Reads an analog input pin. converts and calculates the corresponding
   output pressure. Also prints the results to the serial monitor.
*/

/*
 * The circuit:
 * ASDX pressure sensor pin2 connected to analog pin 0 (PE_3).
 * Pin 1 connected to Vcc (+5v for  ASDXACX030PAAA5) 30 psi, ps: +5 Vdc, Transfer Function limits: A --> 10% to 90%
 * Pin 3 connected to GND
 * The other pins are not connected
 */

/*
   Adapted 22 September 2018 by Juan Carlos Suárez
*/


/*
 * This version includes Serial Port Python Communication
 * /


/**** Constants and variables ***/

const int ADCFULLSCALE = 4095;                // ADC full scale of Tiva C is 12-bit (0-4095)
const float reference_voltage_mv = 3300.0;    // Vcc is 5000mv (5V) for  ASDXACX030PAAA5
const int Pmax = 30;                          // 30psi for ASDXACX030PAAA5
const int Pmin = 0;                           // 0 psi FOR ASDXACX030PAAA5
const int analogInPin = A0;                   // Analog input pin (PE_3) that the asdx pressure sensor is attached to
const int analogOutPin = BLUE_LED;            // Analog output pin that the LED is attached to
int sensorValue = 0;                          // value read from the pressure sensor
int outputValue = 0;                          // value output to the PWM (analog out)
float voltage_mv = 0.0;                       // pressure sensor voltage in mV
float voltage_v = 0.0;                        // pressure sensor voltage in volts
float output_pressure = 0.0;                  // output pressure in psi
float vacuum_pressure = 0.0;                  // (substract atmospheric pressure from output_preesure) // vacuum pressure in psi

/* */ 
unsigned long timer = 0;
long loopTime = 5000;   // microseconds

void setup() {
  /* initialize serial communications at 9600 bps: */
  Serial.begin(9600); 
  timer = micros();

}

void loop() {

  timeSync(loopTime);
  // change the resolution to 12 bits and read A0
  analogReadResolution(12);

  // read the analog in value:
  sensorValue = analogRead(analogInPin);                             // digital value of pressure sensor voltage
  voltage_mv = (sensorValue * reference_voltage_mv) / ADCFULLSCALE;  // pressure sensor voltage in mV
  voltage_v = voltage_mv / 1000;

  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 4095, 0, 1023); 

  //Blink led

  analogWrite(analogOutPin,outputValue);

  // print the results to the serial monitor:
  Serial.print("Voltage = ");
  Serial.println(voltage_v);
  //Serial.print("\r\n");
  double output_pressure = ( ( (voltage_v - (0.10 * (reference_voltage_mv/1000) )) * (Pmax - Pmin) ) / (0.8 * (reference_voltage_mv/1000) ) ) + Pmin;
  vacuum_pressure = output_pressure - 14.6; // subtract atmospheric pressure
  Serial.print("PressureAp = ");
  //Serial.print("\t\r\n");
  Serial.println(output_pressure);

  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  //////delay(500);     

  delay(500);
  sendToPC(&output_pressure);
}



void loop2() {
  timeSync(loopTime);
  //int val = analogRead(0) - 512;
  double val = (analogRead(0) -512) / 512.0;
  sendToPC(&val);
}





/*
 * timeSync(unsigned long deltaT):
 * 
 */


void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}


void sendToPC(int* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 2);
}
 
void sendToPC(double* data)
{
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}
