

uint8_t prints(int printType, double accX, double accY, double accZ, double gyroX, double gyroY, double gyroZ, double pitch, double gyroYangle, double compAngleY, double dOutput)  
{
  if(printType == accelgyro){
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");
  
  }
  else if(printType == yaxis){
    Serial.print(pitch); Serial.print("\t");
    Serial.print(gyroYangle); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");
    
  }
  else if(printType == filters){
    Serial.print(compAngleY); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");
    
  }
  else if(printType == output){
    Serial.print(kalAngleY); Serial.print("\t");
    Serial.print(dOutput); Serial.print("\t");
  }
  
  
  Serial.print("\r\n");


}
