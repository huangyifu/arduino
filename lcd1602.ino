

#include <DHT.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

//红外感应
#define PIRPIN 11
//声纳
#define ECHOPIN 9
#define TRIGPIN 10

//温湿度感应器
#define DHTPIN 8 //你需要更改这里，改成你SIGN所用的PIN
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);//
LiquidCrystal lcd(7,6,5,4,3,2);  //定义LCD脚位
SoftwareSerial mySerial(12, 13);

unsigned char ucRxBuffer[32] = {0};
  
void setup(){
  lcd.begin(16,2); //设置LCD显示的数目。16 X 2：16格2行。
  lcd.print("hello,world!"); //将hello,world!显示在LCD上
  dht.begin();
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(PIRPIN, INPUT);
  
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);

}
void loop() {
  //delay(100);
  ProcessSerialData();
  for (int i=0;i<32;i++) {Serial.print(ucRxBuffer[i]);Serial.print(" ");}
  Serial.println(" "); 
  
  long pmcf10=(float)ucRxBuffer[4]*256+(float)ucRxBuffer[5];Serial.print("PM1.0_CF1:");Serial.print(pmcf10);Serial.print("   "); 
  int pmcf25=(float)ucRxBuffer[6]*256+(float)ucRxBuffer[7];Serial.print("PM2.5_CF1:");Serial.print(pmcf25);Serial.print("   ");
  int pmcf100=(float)ucRxBuffer[8]*256+(float)ucRxBuffer[9];Serial.print("PM10_CF1:");Serial.print(pmcf100);Serial.println("   ");
  int pmat10=(float)ucRxBuffer[10]*256+(float)ucRxBuffer[11];  Serial.print("PM1.0_AT:");Serial.print(pmat10);Serial.print("   ");
  int pmat25=(float)ucRxBuffer[12]*256+(float)ucRxBuffer[13];  Serial.print("PM2.5_AT:");Serial.print(pmat25);Serial.print("   ");
  int pmat100=(float)ucRxBuffer[14]*256+(float)ucRxBuffer[15];  Serial.print("PM10_AT:");Serial.print(pmat100);Serial.println("   ");
  int pmcount03=(float)ucRxBuffer[16]*256+(float)ucRxBuffer[17];  Serial.print("PMcount0.3:");Serial.print(pmcount03);Serial.print("   ");
  int pmcount05=(float)ucRxBuffer[18]*256+(float)ucRxBuffer[19];  Serial.print("PMcount0.5:");Serial.print(pmcount05);Serial.print("   ");
  int pmcount10=(float)ucRxBuffer[20]*256+(float)ucRxBuffer[21];  Serial.print("PMcount1.0:");Serial.print(pmcount10);Serial.println("   ");
  int pmcount25=(float)ucRxBuffer[22]*256+(float)ucRxBuffer[23];  Serial.print("PMcount2.5:");Serial.print(pmcount25);Serial.print("   ");
  int pmcount50=(float)ucRxBuffer[24]*256+(float)ucRxBuffer[25];  Serial.print("PMcount5.0:");Serial.print(pmcount50);Serial.print("   ");
  int pmcount100=(float)ucRxBuffer[26]*256+(float)ucRxBuffer[27]; Serial.print("PMcount10:");Serial.print(pmcount100);Serial.println("   ");
  int hcho=((float)ucRxBuffer[28]*256+(float)ucRxBuffer[29]);  Serial.print("HCHO: ");Serial.print(hcho);Serial.println("ug/m3   ");
  Serial.println("*****************************************************************  ");
  Serial.println("   ");
  Serial.flush();
  // Wait a few seconds between measurements.
  

  //PIR
  int pir = digitalRead(PIRPIN);
  //Serial.println(pir);
  //
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  float distance = pulseIn(ECHOPIN, HIGH)*0.034/2;
  //Serial.println(distance);
  if(distance>3000){
    return;
  }
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
/*
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
*/
  lcd.setCursor(0,0); 
  lcd.print(t); //显示读取到的温度，Float类型
  lcd.print("\xdf");
  lcd.print("C  ");//显示摄氏度的C
  lcd.print(h); //显示读取到的湿度，Float类型
  lcd.print("%"); //显示百分百符号
  lcd.setCursor(0,1);
  
  lcd.print(pir==0?" ":"*");
  
  lcd.print((int)round(distance));
  lcd.print("cm ");
  lcd.print(pmat25);
  lcd.print("ug ");
  lcd.print(hcho);
  lcd.print("ug        ");
}



bool ProcessSerialData()//读取PMS1003的数据。并根据通信协议转化成有效的值。
{
  uint8_t mData = 0;
  uint8_t mPkt[32]={0};
  int mCheck = 0;
while (mySerial.available() > 0) 
  {  
    //Basing on the protocol of Plantower PMS1003
    mData = mySerial.read();     
    delay(2);//wait until packet is received
    if(mData == 0x42)//head1 ok
     {
        mPkt[0] =  mData;
        mData = mySerial.read();
        if(mData ==0x4d)//head2 ok
        {
          mPkt[1] =  mData;
          mCheck = 66+77;
          for(int i=2;i < 30;i++)//data recv and crc calc
          {
             delay(2);
             mPkt[i] = mySerial.read();
             mCheck += mPkt[i];
          }
          delay(2);
          mPkt[30] = mySerial.read();
          delay(2);
          mPkt[31] = mySerial.read();
          
          Serial.println();
          Serial.print(mCheck);
          Serial.print("  ");
          Serial.println(mPkt[30]*256+mPkt[31]);
          if(mCheck == mPkt[30]*256+mPkt[31])//crc ok
          {

            int xyz = mPkt[28]*256+mPkt[29];
            Serial.print("HCHO ==");
            Serial.println(xyz);
            Serial.flush();
            for (int i=0;i<32;i++) {ucRxBuffer[i]=mPkt[i];}
            return true;
          }
          else{
            Serial.println(" CRC Error!");
          }
        }else{
          Serial.println(" Head[2] != 0x4d");
        }
     }else{
      Serial.println(" Head[1]!=0x42");
     }
   } 
   return false;
}
