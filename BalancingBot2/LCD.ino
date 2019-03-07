

#define rs 7
#define en 8
#define d4 9 
#define d5 10
#define d6 11
#define d7 12
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

uint8_t LCDsetup()
{
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.write("P:");
  lcd.setCursor(7,0);
  lcd.write("I:");  
  lcd.setCursor(0,1);
  lcd.write("D:"); 
  lcd.setCursor(7,1);
  lcd.write("A:");
  
}

uint8_t updateLCD()
{

  int cycle = 0;
  static int ButtonState = ButtonState_Up;
  static float sensorValue1 = 0;        // value read from the pot
  static float sensorValue2 = 0;        // value read from the pot
  static float sensorValue3 = 0;        // value read from the pot


// read the analogIn value:
 while(cycle < 4)  { 
    ButtonState = UpdateButtonState(ButtonState, checkButton(buttonPin));
    if(ButtonState == ButtonState_Pressed){
        cycle = 1;
    }
  while(cycle == 1){
    sensorValue1 = analogRead(A0);
    Kp = map(sensorValue1, 0, 1023, 0.00, 255);
    Serial.print(cycle);
     Serial.print("\t");
     Serial.println(Kp);
    lcd.setCursor(3,0);
    lcd.print(Kp,0);
    delay(10);
    ButtonState = UpdateButtonState(ButtonState, checkButton(buttonPin));
    if(ButtonState == ButtonState_Pressed){
        cycle = 2;
    }
  }
  while(cycle == 2){
    sensorValue2 = analogRead(A0);
    Ki = map(sensorValue2, 0, 1023, 0.00, 255);
     Serial.print(cycle);
     Serial.print("\t");
     Serial.print(Kp);
     Serial.print("\t");
     Serial.println(ButtonState);
    lcd.setCursor(10,0);
    lcd.print(Ki,0);  
    delay(10);
    ButtonState = UpdateButtonState(ButtonState, checkButton(buttonPin));
    if(ButtonState == ButtonState_Pressed){
        cycle = 3;
    }
  }
  while(cycle == 3){
    sensorValue3 = analogRead(A0);
    Kd = map(sensorValue3, 0, 1023, 0.00, 255);
    Serial.print(cycle);
     Serial.print("\t");
     Serial.println(Kp);
    lcd.setCursor(3,1);
    lcd.print(Kd,0); 
    delay(10);
    ButtonState = UpdateButtonState(ButtonState, checkButton(buttonPin));
    if(ButtonState == ButtonState_Pressed){
        cycle = 4;
    }
  }
  delay(10);
}
}
