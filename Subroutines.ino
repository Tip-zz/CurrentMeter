 
//**************************************************************************
// Print formatted default value
//**************************************************************************
void Default( int dflt)
  {
  Serial.print(F(" ["));
  Serial.print(dflt);
  Serial.print(F("]: "));
  }
void Default( float dflt)
  {
  Serial.print(F(" ["));
  Serial.print(dflt,4);
  Serial.print(F("]: "));
  }
void Default( uint8_t dflt)
  {
  Serial.print(F(" ["));
  Serial.print(dflt);
  Serial.print(F("]: "));
  }
void Default( char * dflt)
  {
  Serial.print(F(" ["));
  Serial.print(dflt);
  Serial.print(F("]: "));
  }
