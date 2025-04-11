#include <LiquidCrystal.h>

int ThermistorPin = A0;
int Vo;
float R1 = 100000;
float logR2, R2, T, Tc, Tf;

//float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
float c1 = 8.24227928718280E-04, c2 = 2.09591365025075E-04, c3 = 8.27225454547728E-08;

int FSRPin = A1;
float Vfsr, Rfsr, Force, Pressure;
float R_M = 10000;  // Measuring resistor (10kΩ)
float fsrArea = 2.03e-5;  // 5.08mm diameter FSR sensor area in m²

const int rs = 2,
          en = 3,
          d4 = 6, 
          d5 = 7,
          d6 = 8, 
          d7 = 9;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
String TeamName = "Team 9     ";
String ProjectName = "Stroke Rehab Glove                ";
String Warning = "High Temp/Pressure!                ";

int GreenLED = 11; 
int RedLED = 10; 

float tempThreshold = 60.0; 
float presThreshold = 20.0; 

void setup() {
Serial.begin(9600);
lcd.begin(16, 2);
for (int i = 0; i < TeamName.length(); i++){
    lcd.print(TeamName.charAt(i));
    delay(300);
  }
  delay(700);
  lcd.clear();

lcd.setCursor(16, 1);
lcd.autoscroll();
for (int i = 0; i < ProjectName.length(); i++){
  lcd.print(ProjectName.charAt(i));
  delay(300);
}
delay(1000);
lcd.clear();
lcd.noAutoscroll();
}

void loop() {

// Temperature Reading
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;

  Serial.print("Vo: "); Serial.println(Vo);
  Serial.print("R2: "); Serial.println(R2);
  
  Serial.print("Temperature: "); Serial.print(Tc); Serial.println(" C");  
  Serial.println("-------------------------");
  
  // Pressure Reading
   int fsrADC = analogRead(FSRPin);
    Vfsr = (fsrADC * 5.0) / 1023.0;  // Convert ADC value to voltage

    if (Vfsr > 0 && Vfsr < 5.0) {
        Rfsr = (R_M * (5.0 - Vfsr)) / Vfsr;
    } else {
        Rfsr = 1000000;  // Large resistance when no pressure is applied
    }

 // Compute force 
    if (Rfsr < 100000) {  // Apply if within a valid range
        float logR = log10(Rfsr);
        Force = pow(10, (logR - 6.5) / -0.8);  // Convert resistance to force in grams (log scale)
        Force /= 1000.0;  // Convert g to N
    } else {
        Force = 0.0;  // No measurable force
    }

    // Compute pressure
    Pressure = (Force / fsrArea) / 1000.0;  // Convert Pa to kPa
    Pressure = Pressure * 7.50062; // Convert kPa to mmHg

    Serial.print("FSR Analog Reading: "); Serial.println(fsrADC);
    Serial.print("FSR Voltage: "); Serial.print(Vfsr); Serial.println(" V");
    Serial.print("FSR Resistance: "); Serial.print(Rfsr); Serial.println(" Ω");
    Serial.print("Force: "); Serial.print(Force); Serial.println(" N");
    Serial.print("Pressure: "); Serial.print(Pressure); Serial.println(" mmHg");
    Serial.println("-------------------------");

  // LCD display
  lcd.setCursor(0,0);
  lcd.print("Temp: "); lcd.print(Tc); lcd.print(" C ");  
  lcd.setCursor(0,1);
  lcd.print("Pres: "); lcd.print(Pressure); lcd.println(" mmHg  ");

  if (Tc > tempThreshold || Pressure > presThreshold) {
        digitalWrite(RedLED, HIGH);
        digitalWrite(GreenLED, LOW);
        lcd.clear();
        Serial.println("Warning! Overheat/High Pressure!");
        lcd.setCursor(0,0);
        lcd.print("Warning! ");
        lcd.setCursor(0,1);
        lcd.print("High Temp/Pressure!");
        delay(1000);
        lcd.clear();
        }

     else {
        digitalWrite(GreenLED, HIGH);
        digitalWrite(RedLED, LOW);
    }
    
  delay(1000);
  
}
