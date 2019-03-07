//button states
#define ButtonState_Up      1
#define ButtonState_Pressed 2 
#define ButtonState_Down    3
#define ButtonState_Released 4

//map button high/low to up/down
#define ButtonRead_Up 1
#define ButtonRead_Down 2
#define buttonPin 6


int UpdateButtonState(int iButtonState, int iNewButtonRead)
{
  if(iButtonState == ButtonState_Up && iNewButtonRead == ButtonRead_Down)
    return ButtonState_Pressed;
  else if(iButtonState == ButtonState_Pressed)
    return ButtonState_Down;
  else if(iButtonState == ButtonState_Down && iNewButtonRead == ButtonRead_Up)
    return ButtonState_Released;
  else if(iButtonState == ButtonState_Released)
    return ButtonState_Up;
    
  return iButtonState;
}

int checkButton(int iButtonPin)
{
  int newButtonRead = digitalRead(iButtonPin);
  delay(1);
  int debounceButtonRead = digitalRead(iButtonPin);
  if(newButtonRead != debounceButtonRead)
    return -1;//button is not stable

  if(newButtonRead == LOW) //Button is active low, ie using pullup
    return ButtonRead_Down;
  return ButtonRead_Up;
}
