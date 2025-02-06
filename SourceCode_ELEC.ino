/*
  Code for the JM82 Smart Glove capstone Project
  Based off the pseudo-code document on google drive


  Link to Arduino micro pinout: https://content.arduino.cc/assets/Pinout-Micro_latest.png
*/


// Actuator group PWM pins
// const byte IndexPinky_PWM = 9;
// const byte MiddleRing_PWM = 10;
// const byte Thumb_PWM = 6;
// const byte Knuckle_PWM = 5;


const byte IndexPinky_PWM = 6;
const byte MiddleRing_PWM = 5;
const byte Thumb_PWM = 9;
const byte Knuckle_PWM = 10;


// FSR pins
const byte IndexPinky_FSR = A2;
const byte MiddleRing_FSR = A0;
const byte Thumb_FSR = A3;
const byte Knuckle_FSR = A1;


// SEL pins for Mux
const byte SEL_A = 12;
const byte SEL_B = 11;


// LED pins
const byte WEAK_LED = 8;
const byte MEDIUM_LED = 1;
const byte STRONG_LED = 0;
const byte ERROR_LED = A4;


// Interrupt pins for thermistor switch
const byte Therm1_interrupt = 2;
const byte Therm2_interrupt = 3;


// User control button - interrupt
const byte button_control = 7; // interrupt pin that changes power state


// Control variables
float Target_Pressure = 25; // Target pressure values (in mmHg)
enum state_definitions { // power states definitions
  OFF,
  WEAK,
  MEDIUM,
  STRONG,
};


state_definitions Power_state = WEAK; // State variable for power delivery


float Kp = 1; // Proportional coefficient
float Ki = 10; // Integral coefficient
float Ti = 0.01; // 10ms


//==========================================================================


// ISR - SHUT OFF ALL ACTUATORS
void shutoff(){
  analogWrite(IndexPinky_PWM, 0);
  analogWrite(MiddleRing_PWM, 0);
  analogWrite(Thumb_PWM, 0);
  analogWrite(Knuckle_PWM, 0);


  // Print to Serial Monitor
  Serial.print("\nSHUTTING OFF ACTUATORS\n");
}


//==========================================================================
// ISR - CHANGE POWER STATE
void pstate_change(){
  // This function changes the state of the glove depending on the value of the
  // state variable Power_state, added an off state in case of error
  switch(Power_state){
    case WEAK:
      // WEAK -> MEDIUM
      Power_state = MEDIUM;
      break;
    case MEDIUM:
      // MEDIUM -> STRONG
      Power_state = STRONG;
      break;
    case STRONG:
      // STRONG -> WEAK
      Power_state = WEAK;
      break;
    default:
      // CATCH STATE ERROR -> POWER OFF
      Power_state = OFF;
      break;
  }


  // Update targetPWM and LEDs
  UserInterface();


  // Print to Serial Monitor
  Serial.print("\nState changed to: ");
  Serial.print(Power_state);
}


//==========================================================================


float GetPressure(byte pin){
  float Pressure; // Pressure to be returned in mmHg
  float FSR_val = analogRead(pin); // Analog reading of FSR pin
  double TwoMohm = 2000000; // 2 Mohm Resistor in parallel
  double OneMohm = 1000000; // 1 Mohm Resistor in series
  float Force; // Equivalent force felt on FSR
  float correction_val = 0.3; // correction value to adjust the output value
  float offset = 0; // Linear offset up or down


  // Note: analogRead returns number between 0 and 1023
  // Range is equiv to 0-5V
  // based off 20240126 0.1mm/min tests


  float V_read = FSR_val * (5.0 / 1023.0); // map to 0-5V range
  float a = (V_read * OneMohm) / (5.0 - V_read); // Resistance of FSR in parallel with 2Mohm resistor
  float FSR_res = (a*TwoMohm)/(TwoMohm - a); // Resistance of FSR
  Serial.print("Analog:");
  Serial.print(FSR_val);


  // Serial.print(FSR_val);
  // Serial.print("\n\n\n\n");


  // Print to Serial Monitor
  // Serial.print("\nFSR_res: ");
  // Serial.print(FSR_res);


  // If FSR_res is negative, set to out of range value
  if( FSR_res < 0 ) FSR_res = TwoMohm;


  // Now convert FSR resistance to ranges
  // Note: values should be rounded into set ranges to account for fluctuation


  if( FSR_res > 500000 ){
    // The FSR has not received any force output yet
    Force = 0.0; // dummy value below minimum sensitivity
  }
  else if( FSR_res <= 500000 && FSR_res > 300000){
    // low t = 1590s, high t = 1605s
    Force = 0.225; // avg between approx 0.2 and 0.23
  }
  else if( FSR_res <= 300000 && FSR_res > 175000){
    // low t = 1605s, high t = 1620s
    Force = 0.25; // between 0.23 and 0.26
  }
  else if( FSR_res <= 175000 && FSR_res > 140000){
    // low t = 1620s, high t = 1639s
    Force = 0.27; // taking approx time of 1639s
  }
  else if( FSR_res <= 140000 && FSR_res > 50000){
    // low t = 1639s, high t = 1658s
    Force = 0.29; // taking approx time of 1658s
  }
  else if( FSR_res <= 50000 && FSR_res > 35000){
    // low t = 1658s, high t = 1693s
    Force = 0.33; // taking approx time of 1693s
  }
  else if( FSR_res <= 35000 && FSR_res > 30000){
    // low t = 1658s, high t = 1721s
    Force = 0.37; // taking approx time of 1721s
  }
  else if( FSR_res <= 30000 && FSR_res > 25000){
    // low t = 1721s, high t = 1768s
    Force = 0.45; // taking approx time of 1768s
  }
  else if( FSR_res <= 25000 && FSR_res > 20000){
    // low t = 1768s, high t = 1800s
    Force = 0.49; // taking approx time of 1800s
  }
  else if( FSR_res <= 20000 && FSR_res > 17000){
    // low t = 1800s, high t = 1829s
    Force = 0.53; // taking approx time of 1829s
  }
  else if( FSR_res <= 17000 && FSR_res > 15500){
    // low t = 1829s, high t = 1854s
    Force = 0.57; // taking approx time of 1854s
  }
  else if( FSR_res <= 15500 && FSR_res > 13000){
    // low t = 1854s, high t = 1878s
    Force = 0.62; // taking approx time of 1878s
  }
  else if( FSR_res <= 13000 && FSR_res > 12000){
    // low t = 1878s, high t = 1893s
    Force = 0.65; // taking approx time of 1893s
  }
  else if( FSR_res <= 12000 && FSR_res > 11000){
    // low t = 1893s, high t = 1907s
    Force = 0.68; // taking approx time of 1907s
  }
  else if( FSR_res <= 11000 && FSR_res > 10000){
    // low t = 1907s, high t = 1923s
    Force = 0.71; // taking approx time of 1923s
  }
  else Force = 0.73; // Dummy value - at this force, we are far out of range
  //   // Print to Serial Monitor
  // Serial.print("\nForce: ");
  // Serial.print(Force);


  // Convert to mmHg to match target pressure
  Pressure = (correction_val * Force) / (5.0  * pow(10, -5) * 133.3)  + offset;
  Pressure = constrain(Pressure, 0, 50.0);


  // Print to Serial Monitor
  Serial.print("\nPressure (mmHg): ");
  Serial.print(Pressure);


  // delay(500);  // delay added for debugging purposes
  return Pressure;
}


//==========================================================================


bool AbovePressure_Check(int group){
  /*
    This function checks if an actuator group is already above the target pressure, and if so,
    skips to the next group.
    1 = Above pressure
    0 = Below pressure (ideal)
  */
  float currentPressure = 0;


  switch (group){
    case 3:
      // Check Index and Pinky finger pressure
      currentPressure = GetPressure(IndexPinky_FSR);
      break;
    case 4:
      // Check Middle and Ring finger pressure
      currentPressure = GetPressure(MiddleRing_FSR);
      break;
    case 1:
      // Check Thumb pressure
      currentPressure = GetPressure(Thumb_FSR);
      break;
    case 2:
      // Check Knuckle pressure
      currentPressure = GetPressure(Knuckle_FSR);
      break;
    default:
      // If we are in the OFF state or there is an issue with the code - stop all actuation
      // Entering this branch will require a hard reset to
      shutoff();
      Power_state = OFF;
      break;
  }


  if(currentPressure > Target_Pressure)
    return 1;
  else
    return 0;


}


//==========================================================================


void UserInterface(){
  /*
    This function asserts the PWM setting associated with the Power_state.
  */
  switch(Power_state){
    case WEAK:
      Target_Pressure = 15;
      // Turn on WEAK LED and turn off all others
      digitalWrite(WEAK_LED, HIGH);
      digitalWrite(MEDIUM_LED, LOW);
      digitalWrite(STRONG_LED, LOW);
      digitalWrite(ERROR_LED, LOW);
      break;
    case MEDIUM:
      Target_Pressure = 20;
      // Turn on WEAK LED and turn off all others
      digitalWrite(WEAK_LED, LOW);
      digitalWrite(MEDIUM_LED, HIGH);
      digitalWrite(STRONG_LED, LOW);
      digitalWrite(ERROR_LED, LOW);
      break;
    case STRONG:
      Target_Pressure = 25;
      // Turn on WEAK LED and turn off all others
      digitalWrite(WEAK_LED, LOW);
      digitalWrite(MEDIUM_LED, LOW);
      digitalWrite(STRONG_LED, HIGH);
      digitalWrite(ERROR_LED, LOW);
      break;
    case OFF:
      Target_Pressure = 0;
      // Turn on the ERROR LED
      digitalWrite(WEAK_LED, LOW);
      digitalWrite(MEDIUM_LED, LOW);
      digitalWrite(STRONG_LED, LOW);
      digitalWrite(ERROR_LED, HIGH);
      break;
    default:
      Target_Pressure = 0;
      // Turn on the ERROR LED
      digitalWrite(WEAK_LED, LOW);
      digitalWrite(MEDIUM_LED, LOW);
      digitalWrite(STRONG_LED, LOW);
      digitalWrite(ERROR_LED, HIGH);
      break;
  }


  // Print to Serial Monitor
  Serial.print("\nState: ");
  Serial.print(Power_state);
}


//==========================================================================


void Warm(int group){
  /*
    This function performs the initial actuation
    Note: Not necessary to track temperature here - update
          also will grab pressure within the loop
  */


  int Group_output;
  int Group_PressurePin;
  float Group_Pressure;


  switch (group){
    case 3:
      // Actuate index finger and pinky finger
      Group_output = IndexPinky_PWM;
      Group_PressurePin = IndexPinky_FSR;


      // 10
      digitalWrite(SEL_B, HIGH);
      digitalWrite(SEL_A, LOW);


      break;
    case 4:
      // Actuate middle finger and ring finger
      Group_output = MiddleRing_PWM;
      Group_PressurePin = MiddleRing_FSR;


      // 01
      digitalWrite(SEL_B, LOW);
      digitalWrite(SEL_A, HIGH);


      break;
    case 1:
      // Actuate thumb
      Group_output = Thumb_PWM;
      Group_PressurePin = Thumb_FSR;


      // 00
      digitalWrite(SEL_B, LOW);
      digitalWrite(SEL_A, LOW);


      break;
    case 2:
      // Actuate knuckles
      Group_output = Knuckle_PWM;
      Group_PressurePin = Knuckle_FSR;


      // 11
      digitalWrite(SEL_B, HIGH);
      digitalWrite(SEL_A, HIGH);


      break;
    default:
      // If there is an issue with the code - stop all actuation
      // Entering this branch will require a hard reset
      shutoff();
      Power_state = OFF;
      break;
  }
 
  // Begin actuating
  // Note: AnalogWrite values range from 0 to 255
  // Note about unsigned int: range goes from 0 to 4294967295, which in ms equals ~1193 hours (well above our battery life)
  unsigned int StartTime = millis();
  unsigned int ElapsedTime = millis() - StartTime;


  // Variables used for tuning
  float Accumulated_Error = 0;
  float Group_Error = 0;
  float dutyCycle = 0;
  float dutyCycle_bounded = 0;


  // Variables used for checking if the FSR is responding to PWM
  float Base_Pressure = GetPressure(Group_PressurePin);
  unsigned int Period = 3000;
  bool Response_result = 0; // Updated after 3 seconds - 1 = response, 0 = no response
  bool Response_check_flag = 0; // Flag used to ensure that the FSR response is checked only once per period


  // Assure that the PWM is 0% at start
  analogWrite(Group_output, 0);


  while(ElapsedTime < 30000){ // non-blocking timer for 20 seconds
    ElapsedTime = millis() - StartTime;


    // Get the error
    Group_Pressure = GetPressure(Group_PressurePin);
    Group_Error = Target_Pressure - Group_Pressure;


    // Check if the FSR is connected
    if(ElapsedTime % Period < 10 && ElapsedTime > Period + 7000 && !Response_check_flag){ // We want to check the elapsed time every period+10ms after the first period
      Serial.print("\nChecking for FSR response");
      // COMMENT / UNCOMMENT FOLLOWING TWO LINES DEPENDING ON IF RESPONSE CHECK IS DESIRED
      //Response_result = FSR_response_check(Base_Pressure, Group_Pressure, Group_Error);
      Response_result = 0;


      if(Response_result){
        Power_state = OFF; // Since there was no response, we want to set the power state to off to avoid future actuation
        break; // Break out of the while loop to end actuation
      }
      Base_Pressure = Group_Pressure; // Update the Base_pressure value
      Response_check_flag = 1; // Update the response check flag to ensure we don't run the check function without giving the control loop enough time to adjust output
    }
    else{
      if((ElapsedTime % Period) > (Period - 10)){
        // Set the flag back to 0 within the last 10ms of the period to run the FSR_response_check function at the beginning of the following period
        Response_check_flag = 0;
      }
      else Response_check_flag = 1;
    }


    // Print to Serial Monitor
    Serial.print("\nGroup Error:");
    Serial.print(Group_Error);
    Accumulated_Error += Group_Error * Ti;
    dutyCycle = Kp * Group_Error + Ki * Accumulated_Error;
    dutyCycle_bounded = constrain(dutyCycle, 0, 255);


    // dutyCycle = map(ElapsedTime, 0, 20000, 0, 255 ); // ramp, do not use
    analogWrite(Group_output, dutyCycle_bounded); // NORMAL DELIVERY
    // analogWrite(Group_output, 255); // FULL PWM DELIVERY


    // Print to Serial Monitor
    Serial.print("\nduty cycle:");
    Serial.print(dutyCycle_bounded);
    // Print to Serial Monitor
    Serial.print("\nTarget Pressure:");
    Serial.print(Target_Pressure);
    Serial.print("\nTime elapsed:");
    Serial.print(ElapsedTime);
    Serial.print("\nWarming Group: ");
    Serial.print(group);
    // Serial.print("\n\n\n\n\n\n\n\n\n");
    delay(50);


    // Serial Plotter testing
    Serial.print("\nPressure:");
    Serial.print(Group_Pressure);
  }


  // After completing the 20 second cycle, turn the actuators off
  shutoff();


}


//==========================================================================


bool FSR_response_check(float Base_Pressure, float Group_Pressure, float Group_Error){
  /*
    This function detects if there are any changes in the readings of the FSR
    when PWM is increasing. This is implemented to avoid danger of constantly
    rising PWM in the event of an FSR losing connection.


    input values:
      Base_Pressure: The pressure measured at the beginning of the previous period


    return values:
      0: There was a response detected within the last 3 seconds, system can continue operation
      1: There was NO response detected within the last 3 seconds, system should enter the error/off state


    This function should only be called every 3 seconds. We only want to check if there is no FSR response if the system is outside the
    error tolerance to account for fluctuation
  */


  float Error_tolerance = 1.0; // The absolute error tolerance to hover around the target pressure value (mmHg)
  float Min_Pressure_Change = 0.5; // The minimum expected pressure change over the period (mmHg)
  bool FSR_response = 0;


  // Check if the FSR is completely disconnected
  if(Base_Pressure == 0.0 && Group_Pressure == 0.0){
    FSR_response = 1;
    Serial.print("\nFSR not connected");
    //delay(20000);
  }


  // Check if we are outside the pressure tolerance zone
  if(abs(Group_Error) > Error_tolerance){
    // The measured pressure is outside the tolerance
    if( abs(Group_Pressure - Base_Pressure) > Min_Pressure_Change){
      // There was more than the minimum pressure change so the FSR is working in the control loop
      FSR_response = 0;
     
    }
    else{
      FSR_response = 1; // There was less than the minimum pressure change, so the FSR isn't working in the control loop
      Serial.print("\nFSR not working properly\n");
      //delay(20000);
    }
  }
  else{
    // The measured pressure is within the error tolerance of the target pressure so the control loop is working
    FSR_response = 0;
  }
  return FSR_response;
}


//==========================================================================


void Calibrate(){
  /*
    This function runs through the calibration process for the glove upon startup.
    The user will interface with the glove and determine when to proceed.
   
    There should also be a passthrough function for the user to proceed without having to calibrate in the event of redundant reboot.
  */


  // Turn on all LEDs to signify calibration
  digitalWrite(WEAK_LED, HIGH);
  digitalWrite(MEDIUM_LED, HIGH);
  digitalWrite(STRONG_LED, HIGH);
  digitalWrite(ERROR_LED, HIGH);


  delay(1000); // delay for debugging


  // Turn off all LEDs to signify completion of calibration sequence
  digitalWrite(WEAK_LED, LOW);
  digitalWrite(MEDIUM_LED, LOW);
  digitalWrite(STRONG_LED, LOW);
  digitalWrite(ERROR_LED, LOW);
}


//==========================================================================


//==========================================================================


void setup() {
  // Set PWM outputs
  pinMode(IndexPinky_PWM, OUTPUT);
  pinMode(MiddleRing_PWM, OUTPUT);
  pinMode(Thumb_PWM, OUTPUT);
  pinMode(Knuckle_PWM, OUTPUT);


  // Set LEDs to digital outputs
  pinMode(WEAK_LED, OUTPUT);
  pinMode(MEDIUM_LED, OUTPUT);
  pinMode(STRONG_LED, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);


  // Set SEL pins A and B to digital outputs
  pinMode(SEL_A, OUTPUT);
  pinMode(SEL_B, OUTPUT);


  // Thermistor switch interrupt pins
  pinMode(Therm1_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Therm1_interrupt), shutoff, FALLING);


  pinMode(Therm2_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Therm2_interrupt), shutoff, FALLING);


  // User interface button interrupt pin
  pinMode(button_control, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_control), pstate_change, FALLING);


  // Begin serial monitor and calibration sequence
  Serial.begin(9600);
  Calibrate();
}


//==========================================================================


void loop() {
  // Cycle through the finger groups


  for(int i = 1; i <= 4; i++){
    // Change target delivery based off user setting
    UserInterface();


    // Check if the group we want to test is already above target pressure
    if(!AbovePressure_Check(i)){


      // Print to serial monitor
      Serial.print("\nWarming group ");
      Serial.print(i);
      // Begin warming
      Warm(i);
    }
   
  }


//  Warm(3); // Warm only one group continuously


  // Add delay after finishing cycling through each finger
  delay(500);


}


