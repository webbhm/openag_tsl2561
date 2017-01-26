/*
 * Digital_Light_Sensor.ino
 * A library for TSL2561
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : zhangkun
 * Create Time:
 * Change Log :
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

#include <Wire.h>
#include <openag_tsl2561.h>
#include <openag_module.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

std_msgs::UInt16 msg;
unsigned long lux;
unsigned long channel0;
unsigned long channel1;
TSL2561 tsl2561;

void setup()
{
//  Wire.begin();
  Serial.begin(9600);
  tsl2561.setAddress(29);
  tsl2561.begin();
}

void loop()
{
  bool old_data;

  tsl2561.update();
  old_data = tsl2561.get_Lux(msg);
  if (old_data){
      Serial.println("Old Lux Data");
   } else {
      Serial.println("Fresh Lux Data");
   }

   lux = msg.data;
    
  old_data = tsl2561.get_Channel0(msg);
  if (old_data){  Serial.print("The LUX value is: ");
  Serial.println(lux);

      Serial.println("Old Channel0 Data");
   } else {
      Serial.println("Fresh Channel0 Data");
   }

   channel0 = msg.data;
    
  old_data = tsl2561.get_Channel1(msg);
  if (old_data){
      Serial.println("Old Channel1 Data");
   } else {
      Serial.println("Fresh Channel1 Data");
   }

   channel1 = msg.data; 

  
  Serial.print("The LUX value is: ");
  Serial.println(lux);
  Serial.print("The Channel0 value is: ");
  Serial.println(channel0);
  Serial.print("The Channel1 value is: ");
  Serial.println(channel1);

  delay(1000);
}


