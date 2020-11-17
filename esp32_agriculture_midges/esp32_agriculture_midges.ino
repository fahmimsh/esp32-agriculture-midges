  #include <WiFi.h>
  #include <FirebaseESP32.h>
  #include <OneWire.h>
  #include <DallasTemperature.h>
  #include <Arduino.h>
  #include <analogWrite.h>
  
  #define FIREBASE_HOST "rplsenorita.firebaseio.com" 
  #define FIREBASE_AUTH "J3L8fXYcYWqWAVNieLIu1WOAiEbFVsQve0ukKgFA"
  #define WIFI_SSID "Kiiddding"
  #define WIFI_PASSWORD "qwertasdf"
  
  #define Pin_SenMoisture     36     // Analog Pin ADC0
  #define Pin_SenRainAng      33     // Analog Pin ADC1/5
  #define Pin_SenLdr          34     // Analog Pin ADC2
  #define Pin_SenVolt         35     // Analog Pin ADC3
  #define Pin_SenRainDig      7      // Digital Pin 7
  #define Pin_SenSuhu         15     // Digital Pin 15
  #define Pin_RelayElektrik   5      // Digital Pin 5 
  #define Pin_RelayLedStrip   18      // Digital Pin 4 
  #define Pin_InMtr1          16     // Digital Pin 16
  #define Pin_InMtr2          27     // Digital Pin 27
  #define Pin_PwmMtr          17     // Digital Pin 17 ENABLE
  
    OneWire oneWire(Pin_SenSuhu); //  Setup Sensor
    DallasTemperature sensorSuhu(&oneWire);  // Initailize Sensor
    DeviceAddress sensor1 = { 0x28, 0xAA, 0xFF, 0xBD, 0x38, 0x14, 0x1, 0x3A };
    float sensorValueFix_Suhu, sensor_Moisture=0, sensorValueFix_Moisture;
    float Sensor_Ldr, sensorValueFix_LDR, sensorValueFix_Rain, Sensor_Rain;
    float Voltage = 0, vRead = 0, R1  = 30000, R2  = 7500, sensorValueFix_Voltage;
    boolean bIsRaining = false;
    int Cuaca, RElektrik,RLStrip, nilai_pwm, SwElectric, SwIrigasi, SwLamp, ModeAuto;
    unsigned long prevmillis = 0, sendDataPrevMillis = 0;
    char buff[100]; 
    FirebaseData firebaseData1;
    String path1_control = "/Control", path2_sensor = "/Sensor";

  float Sen_Moisture(){
      sensor_Moisture = analogRead(Pin_SenMoisture); 
      sensor_Moisture = map(sensor_Moisture, 0, 4095, 0, 255);
      float moisture = sensor_Moisture;
      if(moisture >= 255){moisture=255;}
      else if (moisture <= 0){moisture=0;}
      moisture = moisture/255*100;
      return moisture;
    }
  float Sen_Rain(){
    Sensor_Rain = analogRead(Pin_SenRainAng);
    Sensor_Rain = map(Sensor_Rain, 0, 4095, 0, 255);
    float rain = Sensor_Rain;
    if(rain >= 255){rain=255;}
    else if (rain <= 0){rain=0;}
    
    bIsRaining = !(digitalRead(Pin_SenRainDig));
    if (bIsRaining)
    {
       Cuaca=1; // HUJAN
    }
    else{
       Cuaca = 0;   //CERAH
    }
    rain = rain/255*100;
    return rain;
  }
  float Sen_Suhu()
  {
     sensorSuhu.requestTemperatures();
     float suhu = sensorSuhu.getTempC(sensor1);
     return suhu;   
  }
  float Sen_Ldr(){
     Sensor_Ldr = analogRead(Pin_SenLdr);
     Sensor_Ldr = map(Sensor_Ldr, 0, 4095, 0, 255);
     float Ldr = Sensor_Ldr;
     if(Ldr >= 255){Ldr=255;}
     else if (Ldr <= 0){Ldr=0;}
     Ldr = Ldr/255*100;
     return Ldr;
  }
  float Sen_Voltage(){
     vRead  = analogRead(Pin_SenVolt) * 3.5 /4095;
     float volt = vRead / (R2 / (R1+R2));
     volt = volt/12*100;
     return volt;
  }
  void Kontrol_Relay(){
    digitalWrite(Pin_RelayElektrik, HIGH);
    digitalWrite(Pin_RelayLedStrip, HIGH);
  }
  void Kontrol_Motor(){
    analogWriteResolution(12);   // RESOLUTION 12BIT
    nilai_pwm=1026;
    nilai_pwm = map(nilai_pwm, 0, 4095, 0, 255);
    if(nilai_pwm >= 255){nilai_pwm=255;}
    else if (nilai_pwm <= 0){nilai_pwm=0;}
    analogWrite(Pin_PwmMtr, nilai_pwm);
    digitalWrite(Pin_InMtr1, HIGH);             
    digitalWrite(Pin_InMtr2, LOW);    
    delay(1000);  
  }
  void Send_Serial(){
    if (millis() - prevmillis > 250){
      prevmillis = millis();
      sprintf(buff, "KELEMBAPAN : %.2f | CUACA : %.2f %d | SUHU : %.2f | INTENSITAS CAHAYA : %.2f | Voltage : %.2f | pwm : %d", 
      sensorValueFix_Moisture,sensorValueFix_Rain,Cuaca,sensorValueFix_Suhu,sensorValueFix_LDR,sensorValueFix_Voltage, nilai_pwm);  
      Serial.println(buff);
     }
  }
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(Pin_SenRainDig, INPUT);
  pinMode(Pin_RelayLedStrip, OUTPUT);
  pinMode(Pin_RelayElektrik, OUTPUT);
  pinMode(Pin_InMtr1, OUTPUT);
  pinMode(Pin_InMtr2, OUTPUT);
  pinMode(Pin_PwmMtr, OUTPUT);
  sensorSuhu.begin();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  if (!Firebase.beginStream(firebaseData1, path1_control))
  {
    Serial.println("------------------------------------");
    Serial.println("Can't begin stream connection...");
    Serial.println("REASON: " + firebaseData1.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }
   Firebase.getInt(firebaseData1, path1_control + "/SwitchMode", ModeAuto);
}
void loop() {
  if(ModeAuto == 1){      // Mode Auto
    if(millis() - sendDataPrevMillis > 1000){
      sendDataPrevMillis = millis();
      //GET DATA MODE
      Firebase.getInt(firebaseData1, path1_control + "/SwitchMode", ModeAuto);
      // SEND TO BASE MONITORING
      sensorValueFix_Moisture = Sen_Moisture();    
      Firebase.setInt(firebaseData1, path2_sensor + "/Soil",sensorValueFix_Moisture); 
      sensorValueFix_Rain     = Sen_Rain();        
      Firebase.setInt(firebaseData1, path2_sensor + "/Rain", sensorValueFix_Rain);     
      sensorValueFix_Suhu     = Sen_Suhu();         
      Firebase.setInt(firebaseData1, path2_sensor + "/Temperature", sensorValueFix_Suhu);    
      sensorValueFix_LDR      = Sen_Ldr();          
      Firebase.setInt(firebaseData1, path2_sensor + "/Intensity", sensorValueFix_LDR);     
      sensorValueFix_Voltage  = Sen_Voltage();      
      Firebase.setInt(firebaseData1, path2_sensor + "/Voltage", sensorValueFix_Voltage);
      Send_Serial();
    }
  }
  else if (ModeAuto == 0){  // Mode Manual
      Kontrol_Motor();
      Kontrol_Relay();
      Send_Serial();
  }
}
