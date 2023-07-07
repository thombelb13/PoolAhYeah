// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID           "TMPLQ3bwOqV5"
#define BLYNK_DEVICE_NAME           "Quickstart Device"
#define BLYNK_AUTH_TOKEN     "iqa3k49AyZyUOBLW0Mt7Q7EGDr4BUdXm"    
#define ONE_WIRE_BUS 9  //digital 7
#define W5100_CS  10
#define SDCARD_CS 4

#include <Controllino.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include "Nextion.h"
#include <SPI.h>
#include <Ethernet.h>
#include <BlynkSimpleEthernet.h>

char auth[] = BLYNK_AUTH_TOKEN;
BlynkTimer timer;
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

int A0measurementResultVoltage,A1measurementResultVoltage;
float ph_calc = 7, ph_calc_new, ph_decimal, ORP_calc = 0.400, ORP_new, ORP_decimal;
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
const float tau_ph = 0.2, tau_ORP = 0.2;

float ph_target = 7.3, K_PH = 160 , ORP_target = 700, K_ORP = 0.04, PHmoins=0, ORPplus = 0;
float PH_pwm = 0, ORP_pwm;

int mode_auto = 0, mode_manu = 1, mode_manu_on = 0, skimmer_on = 0, mode_reglage = 0;
int mode_skimmer = 0; // 10 mode automatique, 0 mode manuel off, 1 mode manuel forcé ON
int robot_on = 0; // en mode automatique
int regul_PH_auto =1, regul_ORP_auto =1;

int temperature = 0;
char entete[16];
char content[160];
int heure = 1, heure_new = 0;
int minute = 0, minute_new = 0;
int seconde = 0, seconde_new = 0;
int comp = 0, page = 0;
int top1s = 0, top1min = 0, go10s = 0, top10s = 0, top1h = 0, uptime_s = 0, uptime_h=-1, top5s =0, cpt5s=0, cpt_boucle=0;
int heure_temp, minute_temp, start_heure = 6, start_minute = 0, duree_heure = 14, duree_minute = 0, stop_heure = 20, stop_minute = 0;
int duree_robot_heure = 6 ;

int Calib_ph_400 = 310, Calib_ph_918 = 535, Calib_ph_400_temp = 310, compensation_temperature=0;

// (page id, component id, component name)

NexButton b_off = NexButton(0, 23, "b2");
NexButton b_auto = NexButton(0, 10, "b0");
NexButton b_manu = NexButton(0, 11, "b1");
NexNumber tempbox = NexNumber(0, 18, "n7"); 
NexNumber PH_entier_box = NexNumber(0, 12, "n3"); 
NexNumber PH_dec_box = NexNumber(0, 13, "n4"); 
NexNumber ORP_entier_box = NexNumber(0, 15, "n5"); 
NexNumber ORP_dec_box = NexNumber(0, 16, "n6"); 
NexNumber heurebox = NexNumber(0, 1, "n0"); 
NexNumber minutebox = NexNumber(0, 2, "n1"); 
NexNumber secondebox = NexNumber(0, 3, "n2"); 
NexNumber skimmerbox = NexNumber(0, 22, "n9"); 
NexTouch *nex_listen_list[] = {
  &b_auto,
  &b_manu,
  &b_off,
  NULL
};


// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
}

void setup() {
   // start serial port 
 Serial.begin(115200); 
 Serial.println("start");
 Blynk.begin(auth);
 // Start up the library 
 sensors.begin(); 
 pinMode(CONTROLLINO_A0, INPUT);
 pinMode(CONTROLLINO_A1, INPUT);
 pinMode(CONTROLLINO_D0, OUTPUT);
 pinMode(CONTROLLINO_D1, OUTPUT);

//  pinMode(SDCARD_CS, OUTPUT);
//  digitalWrite(SDCARD_CS, HIGH); // Deselect the SD card

 Serial.println("done");
 nexInit();
 Controllino_RTC_init(0);
 Controllino_SetTimeDate(1,1,1,23,12,00,00); // set initial values to the RTC chip
 b_auto.attachPush(b_auto_PushCallback, &b_auto);
 b_manu.attachPush(b_manu_PushCallback, &b_manu);
 b_off.attachPush(b_off_PushCallback, &b_off);
}

void b_auto_PushCallback(void *ptr) {
   mode_auto = 1;
   mode_manu = 0;
   mode_manu_on = 0;
 Serial.println("b_auto");
}

void b_manu_PushCallback(void *ptr) {
   mode_auto = 0;
   mode_manu = 1;
   mode_manu_on = 1;
 Serial.println("b_manu");
}

void b_off_PushCallback(void *ptr) {
   mode_auto = 0;
   mode_manu = 1;
   mode_manu_on = 0;
 Serial.println("b_manu_off");
}

BLYNK_WRITE(V0)
{
  int value = param.asInt();
  Serial.println("mode auto");
 if (value == 0) {
   mode_auto = 0;
   mode_manu = 1;
   mode_manu_on = 0;
 }
 else {
   mode_auto = 1;
   mode_manu = 0;
   mode_manu_on = 0;
 }
}

BLYNK_WRITE(V1)
{
  int value = param.asInt();

  // Update state
//  Blynk.virtualWrite(V8, value);
 if (value == 0) {
    Serial.println("mode manu OFF");
   mode_auto = 0;
   mode_manu = 1;
   mode_manu_on = 0;
 }
 else {
      Serial.println("mode manu ON");
   mode_auto = 0;
   mode_manu = 1;
   mode_manu_on = 1;
 }
}

BLYNK_WRITE(V9)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();
  Controllino_SetTimeDate(31,2,1,17,value,minute,seconde);
}

BLYNK_WRITE(V10)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();
  Controllino_SetTimeDate(31,2,1,17,heure,value,seconde);
}

BLYNK_WRITE(V13)
{
  int value = param.asInt();
  start_heure = value  ;
  if (start_heure > 23) (start_heure = 0);
  int stop_time;
  stop_time = start_heure * 60 + start_minute + duree_heure * 60 + duree_minute;
  if (stop_time > 1439) (stop_time = 1439);
  stop_heure = stop_time / 60 ;
  stop_minute = stop_time - (stop_heure * 60);
  //  sprintf(content, "start %d:%d - durée %d:%d - stop %d:%d  - %d min",start_heure, start_minute, duree_heure, duree_minute, stop_heure, stop_minute,stop_time);
  //  Serial.println(content);
}


BLYNK_WRITE(V14)
{
  int value = param.asInt();
  start_minute = value  ;
   if (start_minute > 59) (start_minute = 0);
  int stop_time;
  stop_time = start_heure * 60 + start_minute + duree_heure * 60 + duree_minute;
  if (stop_time > 1439) (stop_time = 1439);
  stop_heure = stop_time / 60 ;
  stop_minute = stop_time - (stop_heure * 60);
}

BLYNK_WRITE(V15)
{
  int value = param.asInt();
  duree_heure = value;
  if (duree_heure > 23) (duree_heure = 0);
  int stop_time;
  stop_time = start_heure * 60 + start_minute + duree_heure * 60 + duree_minute;
  if (stop_time > 1439) (stop_time = 1439);
  stop_heure = stop_time / 60 ;
  stop_minute = stop_time - (stop_heure * 60);
 }
 
 BLYNK_WRITE(V16)
{
  int value = param.asInt();
  duree_heure = value;
   if (duree_heure > 23) (duree_heure = 0);
  int stop_time;
  stop_time = start_heure * 60 + start_minute + duree_heure * 60 + duree_minute;
  if (stop_time > 1439) (stop_time = 1439);
  stop_heure = stop_time / 60 ;
  stop_minute = stop_time - (stop_heure * 60);
}

BLYNK_WRITE(V17)
{
  int value = param.asInt();
  duree_minute = value;
   if (duree_minute > 59) (duree_minute = 0);
  int stop_time;
  stop_time = start_heure * 60 + start_minute + duree_heure * 60 + duree_minute;
  if (stop_time > 1439) (stop_time = 1439);
  stop_heure = stop_time / 60 ;
  stop_minute = stop_time - (stop_heure * 60);
}

BLYNK_WRITE(V25)
{
  // Calib at PH = 9.18
  Serial.print("Calibration manuelle du PH = 9.18 - précédente valeur en bit =");
  Serial.println(Calib_ph_918);
  Calib_ph_918 = analogRead(CONTROLLINO_A0);
  Serial.print("Nouvelle mesure en bit = ");
  Serial.println(Calib_ph_918);
  Blynk.virtualWrite(V25,0);
}

BLYNK_WRITE(V26)
{
  // Calib at PH = 4.00
  Serial.print("Calibration manuelle du PH = 4.00 - précédente valeur en bit =");
  Serial.println(Calib_ph_400);
  Calib_ph_400 = analogRead(CONTROLLINO_A0);
  Serial.print("Nouvelle mesure en bit = ");
  Serial.println(Calib_ph_400);
  Blynk.virtualWrite(V26,0);
}

BLYNK_WRITE(V20)
{
  float value = param.asFloat();
  Serial.print("Nouvelle consigne PH =");
  Serial.println(value);
  ph_target = (float)value;
}

BLYNK_WRITE(V21)
{
  int value = param.asInt();
  ORP_target = (float)value;
}

BLYNK_WRITE(V22)
{
  float value = param.asFloat();
  Serial.print("Nouveau gain ORP =");
  Serial.println(value);
  K_ORP = (float)value;
}

BLYNK_WRITE(V29)
{
  float value = param.asFloat();
  Serial.print("Nouveau gain PH =");
  Serial.println(value);
  K_PH = (float)value;
}
BLYNK_WRITE(V27)
{
  int value = param.asInt();
  regul_PH_auto = value;
}

BLYNK_WRITE(V28)
{
  int value = param.asInt();
  regul_ORP_auto = value;
}

BLYNK_WRITE(V11)
{
  int value = param.asInt();
  duree_robot_heure = value;
}

void loop() {
  Blynk.run();
//  nexLoop(nex_listen_list);
//  timer.run();
//  delay(10);
/***** Horloge *********************************************/  
  seconde_new=Controllino_GetSecond();
  minute_new=Controllino_GetMinute();
  heure_new=Controllino_GetHour();
  top1s = 0;
  top5s = 0;
  top10s = 0;
  cpt_boucle++;
 // cpt5s = 0;
  if (seconde_new != seconde) {
    top1s = 1;  
    }
  seconde = seconde_new;
  if (((seconde_new >= cpt5s) && (cpt5s > 0)) || ((cpt5s == 0 ) && (seconde_new <=55))) {
    top5s = 1;
    cpt5s =(int)(seconde_new / 5) * 5;
    cpt5s += 5;
    if (cpt5s >= 60) {
        cpt5s=0;
    }
    if (cpt5s % 10 == 0) {
      top10s = 1;
    }
//    Serial.print("Top 5s - cpt ");
//    Serial.print(cpt5s);
//    Serial.print(" - time seconde ");
//    Serial.print(seconde_new);
//    Serial.print(" - nb boucle ");
//    Serial.println(cpt_boucle);
    secondebox.setValue(seconde);
  }

 if (heure_new != heure) {
  top1h = 1;  
  uptime_h++;
  }
  heure = heure_new;
    
  top1min = 0;
  if (minute_new != minute) {
    top1min = 1;
  };
  minute = minute_new;

if (top10s == 1) {

    /***** Temperature *********************************************/  
      //Serial.print(" Requesting temperatures..."); 
      sensors.requestTemperatures(); // Send the command to get temperature readings 
    
      //Serial.print("Temperature is: "); 
      temperature = sensors.getTempCByIndex(0);
          tempbox.setValue(temperature);
      //Serial.println(temperature); 
      sprintf(content, "%d:%d:%d - skimmer %d - robot %d - modeauto %d - mode manu %d - mode manu on %d - mode reglage %d", heure, minute, seconde, skimmer_on, robot_on, mode_auto, mode_manu, mode_manu_on, mode_reglage); 
      Serial.println(content);
      sprintf(content, "Mesure PH=%d bit - ORP=%d bit - Calcul PH=%d.%1d - ORP=%d.%3d ",A0measurementResultVoltage,A1measurementResultVoltage,(int)ph_calc,(int)ph_decimal,(int)ORP_calc,(int)ORP_decimal);
      Serial.println(content);
      sprintf(content, "Controle PH PWM=%d - timer =%d",(int)PH_pwm,(int)(PH_pwm * 60 / 100));
      Serial.println(content);
      sprintf(content, "Controle ORP=%d, target=%d, PWM=%d - timer =%d",(int)(ORP_calc*1000),(int)ORP_target,(int)ORP_pwm,(int)(ORP_pwm * 60 / 100));
      Serial.println(content);

}


  if (top5s == 1) {
      if (comp > 0) {
        comp = 0;
        digitalWrite(CONTROLLINO_D0, LOW);
      }
      else {
        comp++;
        digitalWrite(CONTROLLINO_D0, HIGH);
      }
   

    
/**** PH **  ORP ***************************************************/
// mesure PH          
      A0measurementResultVoltage = analogRead(CONTROLLINO_A0);
      if (compensation_temperature==1) {
        // avec compensation de température
        Calib_ph_400_temp=Calib_ph_400 + ((float)temperature - 22)/(8-22)*(338-330);
      }
      else {
        // sans compensation de température
        Calib_ph_400_temp=Calib_ph_400;
      }
      // calcul avec approx linéaire
      ph_calc_new = 4.00 + ((float)A0measurementResultVoltage - Calib_ph_400_temp ) / (Calib_ph_918 - Calib_ph_400_temp) * (9.18 - 4.00);
      
      // filtrage temporel 1er ordre
      ph_calc = tau_ph * ph_calc_new + (1 - tau_ph) * ph_calc ;
      ph_decimal= (ph_calc - (float)(int)ph_calc ) * 100;

// mesure ORP
      A1measurementResultVoltage = analogRead(CONTROLLINO_A1);
      ORP_calc = ( 2.5 - (float)A1measurementResultVoltage / 1024 * 5 ) / 1.037;
      ORP_decimal= (ORP_calc - (float)(int)ORP_calc ) * 1000;

//      sprintf(content, "Mesure PH=%d bit - ORP=%d bit - Calcul PH=%d.%1d - ORP=%d.%3d ",A0measurementResultVoltage,A1measurementResultVoltage,(int)ph_calc,(int)ph_decimal,(int)ORP_calc,(int)ORP_decimal);
//      Serial.println(content);

  }
    /************* gestion des modes ************/
  if (top1s == 1) {
    if (mode_auto ==1) {
      mode_skimmer = 10;
      Blynk.virtualWrite(V0, 1 );  
      Blynk.virtualWrite(V1, 0 ); 
//      sprintf(content, "%d:%d:%d - lapse %d - start %d - stop %d",heure,minute,seconde,heure * 60 + minute,start_heure*60 + start_minute,stop_heure * 60 + stop_minute);
//      Serial.println(content);
      if ((heure * 60 + minute >= start_heure*60 + start_minute) && (heure * 60 + minute <= stop_heure * 60 + stop_minute))
      {
        Serial.println("mode skimmer auto = periode OK => skimmer on");
        skimmer_on = 1;
        if (heure * 60 + minute <= (start_heure + duree_robot_heure) * 60 + start_minute) {
          robot_on = 1;
        }
        else {
          robot_on = 0;
        }
        
      }
      else
      {
//        Serial.println("mode skimmer auto = hors periode => skimmer off");
        skimmer_on = 0;
        robot_on = 0;
      }
    }
    else {
      Blynk.virtualWrite(V0, 0 );  

      if (mode_manu_on ==1 ) {
        Blynk.virtualWrite(V1, 1 ); 
//        Serial.println("mode skimmer manu force => skimmer on");
        skimmer_on = 1;
        mode_skimmer = 1;
        robot_on = 1;

      }
      else
      {
        Blynk.virtualWrite(V1, 0 ); 
//        Serial.println("mode skimmer manu OFF => skimmer off");
        skimmer_on = 0;        
        mode_skimmer = 0;
        robot_on = 0;
      }
    }
 }
 if (skimmer_on ==1) {
   digitalWrite(CONTROLLINO_R2, HIGH);
   digitalWrite(CONTROLLINO_D0, HIGH);
 }
 else
 {
  digitalWrite(CONTROLLINO_R2, LOW);
   digitalWrite(CONTROLLINO_D0, LOW);
 }
 if (robot_on ==1) {
  digitalWrite(CONTROLLINO_R3, HIGH);
  digitalWrite(CONTROLLINO_D1, HIGH);
 }
 else
 {
  digitalWrite(CONTROLLINO_R3, LOW);
  digitalWrite(CONTROLLINO_D1, LOW);
 }
   

  
    /************* affichage *************/
  if (top5s ==1 ){ 
      sprintf(content, "page %d - skimmer %d - modeauto %d - mode manu %d - mode manu on %d - mode reglage %d", page, skimmer_on, mode_auto, mode_manu, mode_manu_on, mode_reglage); 
      Serial.println(content);
      uptime_s = (millis() / 1000) % 3600;
      
      Blynk.virtualWrite(V2, uptime_s);
      Blynk.virtualWrite(V3, uptime_h);
      Blynk.virtualWrite(V4, robot_on);
      Blynk.virtualWrite(V6, ph_calc);
      Blynk.virtualWrite(V5, PH_pwm);
      Blynk.virtualWrite(V7, ORP_decimal);    
      Blynk.virtualWrite(V8, skimmer_on );  
      Blynk.virtualWrite(V12, mode_skimmer );  
      Blynk.virtualWrite(V9, heure ); 
      Blynk.virtualWrite(V10, minute ); 
      Blynk.virtualWrite(V13, start_heure ); 
      Blynk.virtualWrite(V14, start_minute ); 
      Blynk.virtualWrite(V15, duree_heure ); 
      Blynk.virtualWrite(V16, duree_minute ); 
      Blynk.virtualWrite(V11, duree_robot_heure);
      Blynk.virtualWrite(V19, ORP_pwm);
      Blynk.virtualWrite(V20, ph_target);
      Blynk.virtualWrite(V21, ORP_target);
      Blynk.virtualWrite(V23, (float)temperature);
      Blynk.virtualWrite(V24, A0measurementResultVoltage);
      Blynk.virtualWrite(V17, PHmoins ); 
      Blynk.virtualWrite(V18, ORPplus );
      Blynk.virtualWrite(V29, K_PH ); 
      Blynk.virtualWrite(V22, K_ORP );
      switch (page) { 
        case 0:
          PH_entier_box.setValue((int)ph_calc);
          PH_dec_box.setValue((int)ph_decimal);
          ORP_entier_box.setValue((int)ORP_calc);
          ORP_dec_box.setValue((int)ORP_decimal);
          secondebox.setValue(seconde);
          minutebox.setValue(minute);
          heurebox.setValue(heure);
          skimmerbox.setValue(skimmer_on);
           break;
      }
  }

  
    /******** control *******/
  if (top1s ==1 && skimmer_on ==1 ) {

    // calcul de la consigne PWM de control du PH (% de temps d'allumage du moteur)
    // si PH est < à la consigne, on met du PH plus.
    // la pompe fait 3 l/h pour 100% d'opération
    // conso : 1L pour 0.1 de pH pour 100 m3
    // pour un delta de 0.5 pH => ajout de 5L => à faire en 20 h => 0.25 l/h => PWM = 0.25/3 =env 0.08
    // K_PH = 0.08 / 0.5 *100= 0.16*100 = 160
    PH_pwm = (ph_calc - ph_target) * K_PH;
    if (PH_pwm < 0 ) { PH_pwm = 0;}
    if (PH_pwm > 100 ) { PH_pwm = 100;}
    if (regul_PH_auto ==1) {
      // Application de la consigne
      sprintf(content, "Controle PH PWM=%d - timer =%d",(int)PH_pwm,(int)(PH_pwm * 60 / 100));
      Serial.println(content);
      if (seconde < (int)(PH_pwm * 60 / 100)) {
         digitalWrite(CONTROLLINO_R0, HIGH);
         PHmoins = 1;
         
      }
      else
      {
        digitalWrite(CONTROLLINO_R0, LOW);
        PHmoins = 0;       
      
      }
    }
    else
    {
      digitalWrite(CONTROLLINO_R0, LOW);
      PHmoins = 0;       

    }
    

    // calcul de la consigne PWM de control du CL (% de temps d'allumage du moteur)
    // si ORP est < à la consigne, on met du Cl.
    // si 200 mV d'écart => ajout de 5 l en 20h
    // => 0.25 l/h => PWM = 0.25/3 =env 0.08
    // K_ORP = 0.08 / 200 * 100 = 0.04
    ORP_pwm = -(ORP_calc*1000 - ORP_target) * K_ORP;
    if (ORP_pwm < 0 ) { ORP_pwm = 0;}
    if (ORP_pwm > 100 ) { ORP_pwm = 100;}
    // Application de la consigne
    if (regul_ORP_auto ==1) {
      sprintf(content, "Controle ORP=%d, target=%d, PWM=%d - timer =%d",(int)(ORP_calc*1000),(int)ORP_target,(int)ORP_pwm,(int)(ORP_pwm * 60 / 100));
      Serial.println(content);
      if (seconde < (int)(ORP_pwm * 60 / 100)) {
         digitalWrite(CONTROLLINO_R1, HIGH);
         ORPplus = 1;
          
      }
      else
      {
        digitalWrite(CONTROLLINO_R1, LOW);
        ORPplus = 0;       

      }
    }
    else
    {
      digitalWrite(CONTROLLINO_R1, LOW);
      ORPplus = 0;       

    }    
  }  
  if (skimmer_on == 0 ) {
    ORPplus = 0;
    PHmoins = 0; 
    digitalWrite(CONTROLLINO_R0, LOW);
    digitalWrite(CONTROLLINO_R1, LOW);

  }


}
