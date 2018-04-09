#ifndef SETUP_H
#define SETUP_H

void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

void BLEsetup()
{
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  Serial.println( F("OK!") );
  ble.echo(false);
  ble.info();
  ble.verbose(false);
  //  while (! ble.isConnected()) {
  //    delay(500);
  //  }
  // ble.setMode(BLUEFRUIT_MODE_DATA);
  delay(3000);
}

void Sensorsetup(void)
{
  if (!bno1.begin())
  {
    /* There was a problem detecting the BNO055 at 0x28 ... check your connections */
    Serial.print("Ooops, no BNO055 detected at 0x28 ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Initialise the second sensor */
  if (!bno2.begin())
  {
    /* There was a problem detecting the BNO055 at 0x29 ... check your connections */
    Serial.print("Ooops, no BNO055 detected at 0x29 ... Check your wiring or I2C ADDR!");
    while (1);
  }

  bno1.setExtCrystalUse(true);
  delay(1000);
  bno2.setExtCrystalUse(true);
  delay(1000);
}

#endif
