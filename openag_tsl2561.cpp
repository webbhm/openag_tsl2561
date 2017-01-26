/*
 * openag_tsl2561.cpp
 * A library for TSL2561
 *
 * Copyright (c) 2017 Howard Webb
 * Author     : Howard Webb
 * Create Time:
 * Change Log :  Jack Shao, Nov 2014, bug fix and update for user experience
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <openag_tsl2561.h>

//......................Public....................................
/*
* Empty Constructor
*/
TSL2561::TSL2561(){}

/*
* Standard constructor, not used by Arduino 
* IC2 address passes as parameter
*/
TSL2561::TSL2561(int address){
   _TSL2561_Address = address;
}

//allow Arduino to pass constructor info during setup
void TSL2561::setAddress(int address){
  _TSL2561_Address = address;
}

/*
* Std OpenAg sensor functions
*/
 void TSL2561::begin() {
   Wire.begin(); // enable i2c port
   writeRegister(TSL2561_Address,TSL2561_Control,0x03);  // POWER UP
   writeRegister(TSL2561_Address,TSL2561_Timing,0x00);  //No High Gain (1x), integration time of 13ms
   writeRegister(TSL2561_Address,TSL2561_Interrupt,0x00);
   writeRegister(TSL2561_Address,TSL2561_Control,0x00);  // POWER Down

//OpenAg boilerplate
   status_level = OK;
   status_msg = "";
   _send_lux = false;
   _send_ir = false;
   _send_full = false;
   _time_of_last_reading = 0;
 }

/*
* Retreive sensor data and store in private variable 
* throttle readings so give sensor time to absorb light
*/
 void TSL2561::update() {
   if (millis() - _time_of_last_reading > _min_update_interval) {
     unsigned long lux = readVisibleLux();
     _time_of_last_reading = millis();
      if (lux > 0){
        _send_lux = true;
	_send_ir = true;
	_send_full = true;
        status_level = OK;
        status_msg = "";
      } else {
        status_level = ERROR;
        status_msg = "31";
      }
	 }
 } 

/*
* Access to data variables
*/
  bool TSL2561::get_Lux(std_msgs::UInt16 &msg){
   msg.data = lux;
   bool res = _send_lux;
  _send_lux = false;
  return res;
}

 bool TSL2561::get_Channel1(std_msgs::UInt16 &msg){
   msg.data = channel1;
   bool res = _send_lux;
  _send_lux = false;
  return res;
}
 
 bool TSL2561::get_Channel0(std_msgs::UInt16 &msg){
   msg.data = channel0;
   bool res = _send_lux;
  _send_lux = false;
  return res;
}
 
//.........................Private............................
 

/*
* Get register data for light
*/
signed long TSL2561::readVisibleLux()
{
   writeRegister(TSL2561_Address,TSL2561_Control,0x03);  // POWER UP
   delay(14);
   getRegisters();

   writeRegister(TSL2561_Address,TSL2561_Control,0x00);  // POWER Down
   if(ch1 == 0)
   { 
     return 0;
   }
   if(ch0/ch1 < 2 && ch0 > 4900)
   {
     return -1;  //ch0 out of range, but ch1 not. the lux is not valid in this situation.
   }
   return calculateLux(0, 0, 0);  //T package, no gain, 13ms
}

unsigned long TSL2561::calculateLux(unsigned int iGain, unsigned int tInt,int iType)
{
 switch (tInt)
 {
  case 0:  // 13.7 msec
  chScale = CHSCALE_TINT0;
  break;
  case 1: // 101 msec
  chScale = CHSCALE_TINT1;
  break;
  default: // assume no scaling
  chScale = (1 << CH_SCALE);
  break;
}
if (!iGain)  chScale = chScale << 4; // scale 1X to 16X
// scale the channel values
channel0 = (ch0 * chScale) >> CH_SCALE;
channel1 = (ch1 * chScale) >> CH_SCALE;

  ratio1 = 0;
 if (channel0!= 0) ratio1 = (channel1 << (RATIO_SCALE+1))/channel0;
// round the ratio value
 unsigned long ratio = (ratio1 + 1) >> 1;

 switch (iType)
 {
 case 0: // T package
   if ((ratio >= 0) && (ratio <= K1T))
    {b=B1T; m=M1T;}
   else if (ratio <= K2T)
    {b=B2T; m=M2T;}
   else if (ratio <= K3T)
    {b=B3T; m=M3T;}
   else if (ratio <= K4T)
    {b=B4T; m=M4T;}
   else if (ratio <= K5T)
    {b=B5T; m=M5T;}
   else if (ratio <= K6T)
    {b=B6T; m=M6T;}
   else if (ratio <= K7T)
    {b=B7T; m=M7T;}
   else if (ratio > K8T)
    {b=B8T; m=M8T;}
 break;
  case 1:// CS package
   if ((ratio >= 0) && (ratio <= K1C))
    {b=B1C; m=M1C;}
   else if (ratio <= K2C)
    {b=B2C; m=M2C;}
  else if (ratio <= K3C)
   {b=B3C; m=M3C;}
  else if (ratio <= K4C)
   {b=B4C; m=M4C;}
  else if (ratio <= K5C)
   {b=B5C; m=M5C;}
  else if (ratio <= K6C)
   {b=B6C; m=M6C;}
  else if (ratio <= K7C)
    {b=B7C; m=M7C;}
 }
  temp=((channel0*b)-(channel1*m));
  if(temp<0) temp=0;
  temp+=(1<<(LUX_SCALE-1));
  // strip off fractional portion
  lux=temp>>LUX_SCALE;
  return (lux);
 }
 
//Low level chip utilities

void TSL2561::getRegisters(void)
{
    CH0_LOW=readRegister(TSL2561_Address,TSL2561_Channal0L);
    CH0_HIGH=readRegister(TSL2561_Address,TSL2561_Channal0H);
    //read two bytes from registers 0x0E and 0x0F
    CH1_LOW=readRegister(TSL2561_Address,TSL2561_Channal1L);
    CH1_HIGH=readRegister(TSL2561_Address,TSL2561_Channal1H);

    ch0 = (CH0_HIGH<<8) | CH0_LOW;
    ch1 = (CH1_HIGH<<8) | CH1_LOW;
}

uint8_t TSL2561::readRegister(int deviceAddress, int address)
{

    uint8_t value;
     Wire.beginTransmission(deviceAddress);
     Wire.write(address);                // register to read
     Wire.endTransmission();
     Wire.requestFrom(deviceAddress, 1); // read a byte
     while(!Wire.available());
     value = Wire.read();
     //delay(100);
     return value;
}

void TSL2561::writeRegister(int deviceAddress, int address, uint8_t val)
{
     Wire.beginTransmission(deviceAddress);  // start transmission to device
     Wire.write(address);                    // send register address
     Wire.write(val);                        // send value to write
     Wire.endTransmission();                 // end transmission
     //delay(100);
}
