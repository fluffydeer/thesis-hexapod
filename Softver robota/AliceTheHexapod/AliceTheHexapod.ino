#include <math.h>                     //sin, cos, math_pi... 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>  //na adafruit 16-channel servo driver
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

//preskusat tie prazdne pakety
//mergenut walkForwardstart a end
//upratat komentare
//zbavit sa serial printov
//upravit rychlost chodze

//predtym ako zacnem posielat inkerement alebo nastavujem inkrement by som mohla kontrolovat ze ci nohy nahodou uz nie su na danom mieste alebo vypocitat ten inkrement na zaklade tooh, lebo teraz
//ak sa tie nohy uz nachadzaju na danej pozicii tak sa dalej priratavaju inkrementy k cureentPosition az dovtedy dokym v podtstate nevyprsi cas/interval v setNewPositions
//dalo by sa, ak nechceme davat pozor na to ako s nim hybeme, ale momentalne by sa nemalo stat ze by sa dvakrat spustila funckia standUp ktora by vyustila do problemov




//=======================================================================
//                    Variables
//=======================================================================
double coxaL = 1.7, femurL = 5.6711, tibiaL = 9.29;   //dlzky budu v cm
double zFemBody = 2.1;
double femurDesignAngle = 45;                         //45 stupnovy uhol v trojuholniku stehennej kosti
double axisRotAngFemur = 22, axisRotAngTibia = 57;    //axis rotatio angle - uhol potrebny na "otocenie" osi pre 0-180 stupnov, konstanty, ktore vznikli kvoli tomu, aby napr suradnicova sustava tibie bola kolmo na zem
double angBetweenLegPlanes = 42.6;                    //uhol, ktory zviera rovina strednej nohy so svojimi krajnymi nahomi je fi = 42.6 stupna

//pri zmene hodnot v cm treba mysliet na to, aby nohy so sebou nekolidovali a preto je vhodne vzdy otestovat nove hodnoty
double xSitPosition = 2.1, ySitPosition = 0.0, zSitPosition = 3.6;                        //pozicie nohy nohy v kartezianskej sustave, ked "sedi"
double xStandPosition = /*6.10*/ 5, yStandPosition = 0.0, zStandPosition = 6.6, stride = 3.5;   //zakladna stojacia poloha
double yForwardPosition = stride, zForwardPosition =  3.6;                                //premenne pre chodzu vpred
double updateTime = 50;

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();            //PCA9685 doska s adresou 0x40
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

//uint16_t sequence[] = {0,3,2,5,1,4};    //sekvencia pre chodzu typu 5+1
//double interval = 2500;                 //odporucana rychlost akrualizacie serv pre 5+1
uint16_t sequence[] = {0, 2, 1};        //sekvencia pre chodzu typu 4+2
double interval = 1500;                 //odporucana rychlost akrualizacie serv pre 4+2
//uint16_t sequence[] = {0,1};              //sekvencia pre chodzu typu 3+3
//double interval = 2000;                   //odporucana rychlost akrualizacie serv pre 3+3

uint8_t numberOfLegs = 6;
uint8_t length = sizeof(sequence) / sizeof(sequence[0]);


//Network variables
const char* ssid = "Alice_AP";          //nazov vytvoreneho pristupoveho bodu
const char* password = "87080";         //mozne heslo

boolean wifiConnected = false;          //pomozne premenne
boolean udpConnected = false;
WiFiUDP UDP;
unsigned int port = 8889;                     //port na ktorom aplikacia komunikuje
char commandBuffer[UDP_TX_PACKET_MAX_SIZE];   //buffer na ulozenie prichadzajucich sprav

IPAddress local_IP(192, 168, 4, 22);          //adresa, ktoru ma robot v novovytvorenej sieti
IPAddress gateway(192, 168, 4, 9);
IPAddress subnet(255, 255, 255, 0);

//Handle command variables
boolean standing = false;                     //robot na zaciatku sedi, cize nestoji
boolean walkingForward = false;
boolean walkingBackward = false;
boolean walkingLeft = false;
boolean walkingRight = false;




//=======================================================================
//                    Setup
//=======================================================================
void setup() {
  Serial.begin(115200);                       //baud rate pre seriovy monitor
  pinMode(2, OUTPUT);                         //LED je na pine 2
  Serial.println("I am Alice the Hexapod!");

  wifiConnected = createAccessPoint();        //inicializuj wifi pripojenie - pristupovy bod

  if (wifiConnected) {                        //nalinkovanie portu
    udpConnected = connectUDP();
    if (udpConnected) {
      turnOnLED();
      blinkLED();
    }
  }

  pwm1.begin();                               //pripojime dosky
  pwm1.setPWMFreq(60);                        //analogove servos bezia ~60 Hz aktualizaciach
  pwm2.begin();
  pwm2.setPWMFreq(60);
  delay(10);

  initializeLegs();
  delay(1000);
}




//=======================================================================
//                    Loop
//=======================================================================
//prijimem paket, odoslem spat paket, spracujem paket, idem na dalsi paket
//ak nemam paket, tak spustim tu istu funkciu, pokial nebolo poslane stop
//kontrolujem ci prijaty paket nie je prazdny, ak ano, tak tam necham povodnu spravu a spracuje ju
void loop() {
  if (wifiConnected) {
    if (udpConnected) {
      handleCommunication();
    }
  }
  handleCommand();
}





//=======================================================================
//                    Communication
//=======================================================================
boolean createAccessPoint() {
  Serial.print("Setting soft-AP configuration ... ");
  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    return false;
  }
  Serial.print("ready. Setting soft-AP ... ");
  if (!WiFi.softAP(ssid/*,password*/)) {
    return false;
  }
  Serial.print("ready. Soft-AP IP address = ");
  Serial.print(WiFi.softAPIP());
  Serial.print(" on port = ");
  Serial.println(port);
  return true;
}


boolean connectUDP() {                            //pripoji sa k UDP
  Serial.println("Connecting to UDP");
  if (UDP.begin(port) == 1) {
    Serial.println("Connection successful");
    return true;
  }
  else {
    Serial.println("Connection failed");
    return false;
  }
}


void handleCommunication() {                //ak su pristupne data, tak prijmi datagram, neblokujuca funkcia
  int packetSize = UDP.parsePacket();
  if (packetSize) {
    /*Serial.println("");                   //vypise info o odosielatelovi
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = UDP.remoteIP();
    for (int i = 0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(UDP.remotePort());*/

    commandBuffer[0] = (char)0;                         //precistime buffer
    memset(commandBuffer, 0, sizeof(commandBuffer));
    UDP.read(commandBuffer, UDP_TX_PACKET_MAX_SIZE);    //precitame datagram do commandBuffer
    Serial.print("Recieved datagram: ");
    Serial.println(commandBuffer);

    UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());  //odosleme datagram naspat
    UDP.write(commandBuffer);
    UDP.endPacket();
  }
  delay(10);
}


void turnOnLED() {        //na WemosD1 Mini je LED na pine 2
  digitalWrite(2, LOW);   //zapne LEDku
  delay(1000);
}

void blinkLED() {         //LEDka na mikropocitaci blikne
  digitalWrite(2, HIGH);  //vypne
  delay(10);
  digitalWrite(2, LOW);   //zapne
}






//=======================================================================
//                    Hexapod classes
//=======================================================================
class Joint {                      //trieda definujuca 1 servo na 1 pine
  private:
    uint8_t id;
    uint8_t pin;
    Adafruit_PWMServoDriver pwm; 
    uint16_t minPwm;              //0 stupnov
    uint16_t middlePwm;           //90 stupnov
    uint16_t maxPwm;              //max mozna hodnota dosahu
    double  angle;                //aktualny nastaveny uhol v serve
  public:
    Joint() {
    }

    //tato funckia bude necakane zavolana len raz na zaciatku programu
    void setJoint(uint8_t id, uint8_t pin, Adafruit_PWMServoDriver pwm, uint16_t minPwm, uint16_t middlePwm, uint16_t maxPwm) { 
      this->id = id;
      this->pin = pin;
      this->pwm = pwm;
      this->minPwm = minPwm;
      this->middlePwm = middlePwm;
      this->maxPwm = maxPwm;
    }

    void printJoint() {
      char printString[50];
      sprintf(printString, "id: %d pin: %d min: %d middle: %d max: %d\n", id, pin, minPwm, middlePwm, maxPwm);
      Serial.print(printString);
    }

    //reverse moze byt 0 alebo 180, ak je to lava strana tak uhol sa nemeni, pri pravej strane angle je odcitany od 180
    void setJointPwm(double angle, uint8_t reverseMovement) { 
      this->angle = angle;
 
      if (reverseMovement == 180) {          //kvoli tomu, ze prava a lava strana su od seba vlastne otocene o 180 stupnov                        
        angle = reverseMovement - angle;
      }
      uint16_t  pulselength;
      pulselength = degreesToPulse(angle, 0, 90, minPwm, middlePwm);

      //ak je uhol pod 10 alebo nad 170 stupnov, tak ho nesmie nastavit -> nicenie serva
      if (pulselength < degreesToPulse(10, 0, 90, minPwm, middlePwm) || pulselength > degreesToPulse(170, 0, 90, minPwm, middlePwm)) { 
        Serial.print("error");
        Serial.print(pulselength);
        Serial.print(" id nohy: ");
        Serial.println(this->id);
        return;
      }
      pwm.setPWM(pin, 0, pulselength);
    }

    double degreesToPulse(double d, double d1, double d2, double p1, double p2) {   //prekonvertuje stupne na PWM signal
      return d * ((p2 - p1) / (d2 - d1)) + p1;
    }

    double pulseToDegrees(double p, double p1, double p2, double d1, double d2) {
      return (p - p1) / ((p2 - p1) / (d2 - d1));
    }

    double getAngle(){
      return this->angle;
    }
};

class Leg {
  private:
    uint8_t id;
    uint8_t reverseMovement;    //serva lavej a pravej strany su od seba otocene o 180 stupnov, pozor rozsah je len 0..255!!
    Joint joints[3];            //toto zavola default konstruktor, cize uz nemozno zavolat parametricky

    double coxaAngle, femurAngle, tibiaAngle;
    double xCurrentPosition, yCurrentPosition, zCurrentPosition;    //aktualna pozicia nohy v kartezianskej sustave
    double fi;

    double xInc;                //inkrement podla ktoreho sa bude noha pohybovat
    double yInc;
    double zInc;
    //double stride;            //toto mozem vymazat, nepouzivam to

  public:
    Leg() {                     //konstruktor pre joints sa zavola automaticky
    }

    void setLeg(uint8_t id, uint8_t reverseMovement, double fi) {             //tato funckia bude zavolana len raz na zaciatku programu!
      this->id = id;
      this->reverseMovement = reverseMovement;
      this->fi = fi;
      
      this->xInc = this->yInc = this->zInc = 0; 
      this->xCurrentPosition = xSitPosition;
      this->yCurrentPosition = ySitPosition;
      this->zCurrentPosition = zSitPosition;
      
      this->getAngles(xCurrentPosition, yCurrentPosition, zCurrentPosition);  //najprv sa nastavia serva/jointy a potom sa nastavi noha
      this->setAngles();                                                      //nastavi hodnoty sadnutia si
    }

    void setXIncrement(double xInc) {
      this->xInc = xInc;
    }

    double getXIncrement() {
      return xInc;
    }

    void setYIncrement(double yInc) {
      this->yInc = yInc;
    }

    double getYIncrement() {
      return yInc;
    }

    void setZIncrement(double zInc) {
      this->zInc = zInc;
    }

    double getZIncrement() {
      return zInc;
    }

    double getCoxaAngle() {
      return this->coxaAngle;
    }

    double getTibiaAngle() {
      return this->tibiaAngle;
    }

    double getFemurAngle() {
      return this->femurAngle;
    }

    void setCoxaAngle(double a) {
      this->coxaAngle = a;
    }

    void setTibiaAngle(double a) {
      this->tibiaAngle = a;
    }

    void setFemurAngle(double a) {
      this->femurAngle = a;
    }


    boolean isLegInBasicPosition() {    //funkcia skontroluje ci je dana noha v zakladnej pozicii
      //je potrebne premenne zaokruhlit, kedze nam to vyhadzovalo, ze sa nerovnaju pricom boli rovnake
      if ((roundf(xCurrentPosition * 100) / 100) != (roundf(xStandPosition * 100) / 100)) {   
        return false;
      }
      if ((roundf(yCurrentPosition * 100) / 100) != (roundf(yStandPosition * 100) / 100)) {
        return false;
      }
      if ((roundf(zCurrentPosition * 100) / 100) != (roundf(zStandPosition * 100) / 100)) {
        return false;
      }

      //pozrieme aj uhly v servo motoroch
      getAngles(xStandPosition, yStandPosition, zStandPosition);                                //vypocita stupne pre uhly
      if ((roundf(joints[0].getAngle() * 100) / 100) != (roundf(coxaAngle * 100) / 100)+90) {
        return false;
      }
      if ((roundf(joints[1].getAngle() * 100) / 100) != (roundf(femurAngle * 100) / 100)) {
        return false;
      }
      if ((roundf(joints[2].getAngle() * 100) / 100) != (roundf(tibiaAngle * 100) / 100)) {
        return false;
      }
      return true;
    }

    Joint& getJoint(uint8_t number) {             //treba poslat referenciu, adresu
      return joints[number]; 
    }

    double getXForwardPosition(double stride) {   //kazda noha ma inu vzdialnost panvicky od konca nohy podla typu umiestnenia na tele
      double xForwardPosition = sqrt(pow(xStandPosition, 2) + pow(stride, 2) - 2 * stride * xStandPosition * cos((90 + fi) * (PI / 180)));
      return xForwardPosition;
    }


    void getAngles(double x, double y, double z) {            //vypocita uhly, implementacia rovnic inverznej kinematiky
      double xFemTib = x - coxaL;
      double zBodyGround = z - zFemBody;
      double HF = sqrt(pow(xFemTib, 2) + pow(z, 2));
      double A1 = atan2(xFemTib, z) * (180 / PI);
      double A2 = acos((pow(femurL, 2) + pow(HF, 2) - pow(tibiaL, 2)) / (2 * HF * femurL)) * (180 / PI);
      double b1 = acos((-pow(HF, 2) + pow(tibiaL, 2) + pow(femurL, 2)) / (2 * femurL * tibiaL)) * (180 / PI);

      femurAngle = 180 - A1 - A2 + femurDesignAngle - axisRotAngFemur;
      tibiaAngle = b1 - femurDesignAngle + axisRotAngTibia;
      coxaAngle = asin((y * sin(atan2(xStandPosition * sin((90 - fi) * (PI / 180)), y + xStandPosition * sin((fi) * (PI / 180))))) / xStandPosition ) * (180 / PI);
      delay(0);                                                 //kvoli swt reset stack
    }

    void setAngles() {                                          //nastav serva na vypocitane hodnoty
      joints[0].setJointPwm(90 + coxaAngle, reverseMovement);
      joints[1].setJointPwm(femurAngle, reverseMovement);
      joints[2].setJointPwm(tibiaAngle, reverseMovement);
    }

    void setNewPosition(double xInc, double yInc, double zInc) {  //posunie nohu o kusok - o nastaveny inkrement na novu poziciu
      xCurrentPosition += xInc;
      yCurrentPosition += yInc;
      zCurrentPosition += zInc;
      getAngles(xCurrentPosition, yCurrentPosition, zCurrentPosition);
      setAngles();
    }
};

Leg legs[6];                                                                         //deklaracia noh, je potrebne to mat tuto

void setNewPositions(uint8_t  first, uint8_t  last, uint8_t  advance) {               //predtym ako je tato funckia pouzita, treba vzdy nastavit vsetky tri inkrmenety!        
                                                                                      //nastavi vsetky nohy o dany inkeremeny za dany cas, NARAZ
  for (double timeCounter = 1; timeCounter <= interval / updateTime; timeCounter++) { //v tento cas je potrebne tu istu poziciu
    for (uint8_t i = first; i < last; i += advance) {                                 //ktora noha bude nastavovana prva -> first, ktora bude nastavovana posledna->last, "v akom poradi/s akym krokom" ->advance 
      legs[i].setNewPosition(legs[i].getXIncrement(), legs[i].getYIncrement(), legs[i].getZIncrement());               
    }
  }
}


double getIncrement(double d1, double d2) {             //d1 a d2 su v cm, vypocita nam jeden krok
  return (d2 - d1) / (interval / updateTime);
}

void setXIncrements(double from, double to) {         //nastavi rovnaky x inkrement vsetkym noham
  for (uint8_t i = 0; i < 6; i++) {                   //pouzivame ak menime vzdialenost nohy od tela po xovej osi
    legs[i].setXIncrement(getIncrement(from, to));
  }
}

void setYIncrements(double from, double to) {       //pouzivame ak sa noha hybe v y osi cize napr robi krok dopredu
  for (uint8_t i = 0; i < 6; i++) {
    legs[i].setYIncrement(getIncrement(from, to));
  }
}


void setZIncrements(double from, double to) {       //pouzivame ak sa telo zdviha alebo klesa
  for (uint8_t i = 0; i < 6; i++) {
    legs[i].setZIncrement(getIncrement(from, to));
  }
}


void resetIncrements() {                            //nastavi inkrementy vsetkym noham na 0
  setXIncrements(0, 0);
  setYIncrements(0, 0);
  setZIncrements(0, 0);
}


boolean areLegsInBasicPosition() {                  //kontroluje, ci su nohy v zakladnej stojacej polohe
  for (uint8_t i = 0; i < 6; i++) {
    //legs[i].printCurrentPositions();
    if (!legs[i].isLegInBasicPosition()) {
      return false;
    }
  }
  return true;
}




//=======================================================================
//                    Hexapod movement
//=======================================================================
void initializeLegs() {                         //naplni triedy, nastavi polohu robota na sedenie
  Serial.println("Initialize legs");
  uint16_t minPwms[] = {140, 125, 130, 140, 131, 135, 150, 125, 140, 140, 139, 125, 145, 135, 157, 150, 145, 123};
  uint16_t middlePwms[] = {385, 360, 354, 375, 360, 357, 395, 379, 382, 380, 358, 353, 385, 365, 402, 387, 391, 327};
  uint16_t maxPwms[] = {602, 595, 578, 590, 580, 565, 625, 640, 605, 593, 570, 580, 600, 580, 620, 600, 610, 560};
  double fis[] = { -42.6, 0.0, 42.6, 42.6, 0.0, -42.6};

  //naplnenie tried
  for (uint8_t i = 0; i < 3; i++) {             //lava strana
    for (uint8_t j = 0; j < 3; j++) {           //mame 3 klby
      legs[i].getJoint(j).setJoint(i * 3 + j, i * 3 + j, pwm1, minPwms[i * 3 + j], middlePwms[i * 3 + j], maxPwms[i * 3 + j]);
    }
    legs[i].setLeg(i, 0, fis[i]);               //najprv musia byt nastavena serva k pinom, az potom nastavime nohu, kde sa nastavi pozicia sit
  }

  for (uint8_t i = 3; i < 6; i++) {             //prava strana
    for (uint8_t j = 0; j < 3; j++) {           //lebo mame 3 klby
      legs[i].getJoint(j).setJoint(i * 3 + j, i * 3 + j - 9, pwm2, minPwms[i * 3 + j], middlePwms[i * 3 + j], maxPwms[i * 3 + j]);
    }
    legs[i].setLeg(i, 180, fis[i]);             //najprv musia byt nastavena serva k pinom, az potom nastavime nohu, kde sa nastavi pozicia sit
  }
}


void standUp() {
  setXIncrements(0, 0);
  setYIncrements(0, 0);                         //zostane 0 celu dobu az dovtedy dokym sa nebudeme snazit spravit krok dopredu
  setZIncrements(zSitPosition, zStandPosition); //zdvihne sa telo/vsetky nohy na rovnakom x a y ako bolo nastavene v inicializacii
  setNewPositions(0, numberOfLegs, 1);

  for (uint8_t i = 0; i < length; i++) {        //cyklus na sekvenciu
    //je vhodne si uvedomit ze nastavujem inkrement pre vsetky nohy naraz 18x, lepsie by bolo to nastavovat zvlast pre kazdu nohu
    setXIncrements(xSitPosition, ((xStandPosition - xSitPosition) / 2) + xSitPosition);   //posunie sa noha do police x
    setZIncrements(-zSitPosition, -zStandPosition);                                       //zdvihne sa noha do polovice x - treba znizit telo
    setNewPositions(sequence[0 + i], numberOfLegs, length);

    setXIncrements(xSitPosition, ((xStandPosition - xSitPosition) / 2) + xSitPosition);   //polozi sa noha z 1/2x na cele x
    setZIncrements(zSitPosition, zStandPosition);                                         //polozime nohu -> treba zvysit "telo"
    setNewPositions(sequence[0 + i], numberOfLegs, length);
  }
}



//vymazat
void walkForwardStart() {                         //zdvihne nohy v roznych sekvenciach a polozi ich dopredu, cize zaklad -> vsetky nohy vpredu
  for (uint8_t i = 0; i < numberOfLegs; i++) {    //zaklad telo->dopredu telo
    legs[i].setXIncrement(getIncrement(xStandPosition, legs[i].getXForwardPosition(-stride)));    //posielame -stride!! lebo chceme dlzku nohy pre krok dopredu, posielame plusovy inkrement - zvysujeme dlzku kroku
  }
  setYIncrements(-yStandPosition, -yForwardPosition);
  setZIncrements(0, 0);
  setNewPositions(0, numberOfLegs, 1) ;
}


void walkPrepare(double prepare) {                //ak zaciname chodzu +1, ak koncime chodzu -1
  for (uint8_t i = 0; i < numberOfLegs; i++) {    //telo je posunute dopredu -> presunie nohy do zakladnej pozicie
    legs[i].setXIncrement(prepare*getIncrement(xStandPosition, legs[i].getXForwardPosition(-stride)));
  }
  setYIncrements(-prepare*yStandPosition, -prepare*yForwardPosition);
  setZIncrements(0, 0);
  setNewPositions(0, numberOfLegs, 1);
}


void walkForwardEnd() {                           //nohy su posunute dopredu -> posunie telo do zakladnej pozicie
  for (uint8_t i = 0; i < numberOfLegs; i++) {    //dopredu->zaklad
    legs[i].setXIncrement(-getIncrement(xStandPosition, legs[i].getXForwardPosition(-stride)));
  }
  setYIncrements(yStandPosition, yForwardPosition);
  setZIncrements(0, 0);
  setNewPositions(0, numberOfLegs, 1);
}

//+stride -> krok dozadu -stride -> krok dopredu
void walk() {
  for (uint8_t i = 0; i < numberOfLegs; i++) {                      //dopredu->zaklad
    legs[i].setXIncrement(-getIncrement(xStandPosition, legs[i].getXForwardPosition(-stride)));    //posielame -stride!! lebo chceme dlzku nohy pre krok dopredu, //posielame minusovy inkrement! zmensujeme dlzku kroku
  }
  setYIncrements(yStandPosition, yForwardPosition);     //plusovy inkrement -> vracia nohu z y = -6 na 0
  setZIncrements(0, 0);
  setNewPositions(0, numberOfLegs, 1);                  //vsetky nohy naraz!!!

  for (uint8_t i = 0; i < numberOfLegs; i++) {          //zaklad -> dozadu
    legs[i].setXIncrement(getIncrement(xStandPosition, legs[i].getXForwardPosition(stride)));
  }
  setYIncrements(yStandPosition, yForwardPosition);
  setZIncrements(0, 0);
  setNewPositions(0,  numberOfLegs, 1);

  for (uint8_t i = 0; i < length; i++) {                      //dozadu -> zaklad so zdvihnutou nohou
    for (uint8_t i = 0; i < numberOfLegs; i++) {
      legs[i].setXIncrement(-getIncrement(xStandPosition, legs[i].getXForwardPosition(stride)));  //posielame minusovy inkrement! zmensujeme dlzku kroku
    }
    setYIncrements(-yStandPosition, -yForwardPosition);
    setZIncrements(zStandPosition, zForwardPosition);         //zaporny inkrement
    setNewPositions(sequence[0 + i], numberOfLegs, length);
        
    for (uint8_t i = 0; i < numberOfLegs; i++) {              //zaklad so zdvihnutou nohou -> dopredu
      legs[i].setXIncrement(getIncrement(xStandPosition, legs[i].getXForwardPosition(-stride)));
    }
    setYIncrements(-yStandPosition, -yForwardPosition);
    setZIncrements(-zStandPosition, -zForwardPosition);       //zvysi nohu, je to plusovy inkrement
    setNewPositions(sequence[0 + i], numberOfLegs, length);
  }
}




void rotate(double typeOfTurn) {
  //otoci vpravo prave vtedy ak coxaAngleStep je kladne
  //otoci vlavo prave vtedy ak coxaAngleStep je zaporne
  double oldInterval = interval;            //ulozime si povodny interval
  interval = interval*3;
  double coxaAngleStep = typeOfTurn * getIncrement(0, 60);
  //double tibiaAngleStep = getIncrement(0, 80);
  double femurAngleStep = getIncrement(0, 100);
  
  for (uint8_t j = 0; j < length; j++) {    //zdvihne nohy podla sekvencie do vzduchu a polozi
                                            //zdvihne nohu
    for (float timeCounter = 1; timeCounter <= (interval / updateTime) / 2; timeCounter++) {        //"cas" za ktory sa nastavia nohy v tej istej sekvencii bud sa zdvihne 1 noha/2 nohy/3 nohy
      for (uint8_t i = 0; i < numberOfLegs; i += length) {
        if (sequence[j] + i < 3) {                                                                  //kvoli otoceniu noh prava strana pricituje coxaAngleStep
          legs[sequence[j] + i].setCoxaAngle(legs[sequence[j] + i].getCoxaAngle() + coxaAngleStep); //rozdielne na zaklade strany
        }
        else {                  //lava strana ho odcituje
          legs[sequence[j] + i].setCoxaAngle(legs[sequence[j] + i].getCoxaAngle() - coxaAngleStep); //rozdielne na zaklade strany
        }
        //legs[sequence[j] + i].setTibiaAngle(legs[sequence[j] + i].getTibiaAngle() + tibiaAngleStep); //je ok
        legs[sequence[j] + i].setFemurAngle(legs[sequence[j] + i].getFemurAngle() - femurAngleStep); //je ok
        legs[sequence[j] + i].setAngles();
      }
    }

    for (float timeCounter = 1; timeCounter <= (interval / updateTime) / 2; timeCounter++) {        //polozi nohu
      for (uint8_t i = 0; i < numberOfLegs; i += length) {
        if (sequence[j] + i < 3) { //kvoli otoceniu noh prava strana pricituje coxaAngleStep
          legs[sequence[j] + i].setCoxaAngle(legs[sequence[j] + i].getCoxaAngle() + coxaAngleStep); //rozdielne na zaklade strany
        }
        else {                  //lava strana ho odcituje
          legs[sequence[j] + i].setCoxaAngle(legs[sequence[j] + i].getCoxaAngle() - coxaAngleStep); //rozdielne na zaklade strany
        }
        //legs[sequence[j] + i].setTibiaAngle(legs[sequence[j] + i].getTibiaAngle() - tibiaAngleStep);
        legs[sequence[j] + i].setFemurAngle(legs[sequence[j] + i].getFemurAngle() + femurAngleStep); //je ok
        legs[sequence[j] + i].setAngles();
      }
    }
    delay(0);
  }

  for (float timeCounter = 1; timeCounter <= (interval / updateTime); timeCounter++) {      //otoci telo spat
    for (uint8_t i = 0; i < numberOfLegs / 2; i++) {                                        //najprv prava strana
      legs[i].setCoxaAngle(legs[i].getCoxaAngle() - coxaAngleStep);
      legs[i].setAngles();
    }
    for (uint8_t i = numberOfLegs / 2; i < numberOfLegs; i++) {                             //vsetky nohy otacame spat
      legs[i].setCoxaAngle(legs[i].getCoxaAngle() + coxaAngleStep);
      legs[i].setAngles();
    }
    delay(0);
  }

  resetIncrements();                    //resetne inkrementy
  setNewPositions(0,numberOfLegs ,1);   //nastavi zakladnu polohu, je to nutne!
  interval = oldInterval;
}


void walkSidewayPrepare(double xLeft, double xRight, double prepare) {        //ak xLeft < xRight tak pohyb bude dolava, prepare bud nohy zosynchronizuje rovnobezne so strednou (1) alebo ich vrati do basic pozicie (-1)
  double y1 = xStandPosition * sin((90 - angBetweenLegPlanes) * (PI / 180));  //hodnota pre nohy 0 a 3 aby boli rovnobezne so stredovou nohou
  double y2 = -xStandPosition * sin(angBetweenLegPlanes * (PI / 180));        //hodnota pre nohy 2 a 5 aby boli rovnobezne so stredovou nohou
  double yIncrements[] = {y1, 0, y2, y2, 0, y1};

  resetIncrements();                                                          //pre istotu, aby sa nam nahodou nepohlo nieco co sa nema pohnut
  double oldInterval = interval; 
  interval = interval*2;    

  for (uint8_t j = 0; j < length; j++) {                                      //nastavime inkrementy pre sekvenciu noh, ktora sa bude hybat spolu
    for (uint8_t i = 0; i < numberOfLegs; i += length) {                      //zdvihne nohu od basic do polovice
      if ((sequence[j] + i) < 3) {                                            //lava strana
        legs[sequence[j] + i].setXIncrement(prepare * getIncrement(xStandPosition, xLeft));
      }
      else {                                                                  //prava strana
        legs[sequence[j] + i].setXIncrement(prepare * getIncrement(xStandPosition, xRight));
      }
      legs[sequence[j] + i].setYIncrement(prepare * getIncrement(yStandPosition, yIncrements[sequence[j] + i] / 2));    //otoci nohu do polovice cesty
      legs[sequence[j] + i].setZIncrement(getIncrement(zStandPosition, zForwardPosition));                              //zdvihne nohu
    }
    setNewPositions(sequence[j], numberOfLegs, length);                       //najprv nohy v jednej sekvencii spolu zdvihne do polovice

    for (uint8_t i = 0; i < numberOfLegs; i += length) {                      //polozi nodu od polovice na poziciu rovnobezne so stredovou nohou
      if ((sequence[j] + i) < 3) {                                            //lava strana
        legs[sequence[j] + i].setXIncrement(prepare * getIncrement(xStandPosition, xLeft));
      }
      else {                                                                  //prava strana
        legs[sequence[j] + i].setXIncrement(prepare * getIncrement(xStandPosition, xRight));
      }
      legs[sequence[j] + i].setYIncrement(prepare * getIncrement(yStandPosition, yIncrements[sequence[j] + i] / 2));  //otoci nohu do polovice cesty
      legs[sequence[j] + i].setZIncrement(-getIncrement(zStandPosition, zForwardPosition));                           //polozi nohu
    }
    setNewPositions(sequence[j], numberOfLegs, length);                       //potom nohy v jednej sekvencii spolu pusti na konecnu polohu
  }
  interval = oldInterval;
}


//komentare su pre pripad chodze vpravo ak sa na hexapoda pozerame spredu
void walkSideway(double myDirection) {                //myDirection vyjadruje, ci sa robot pohne dolava (-1), alebo doprava (1)
  resetIncrements();                                  //treba vyresetovat y
  double oldInterval = interval; 
  interval = interval*2;
  
  for (uint8_t j = 0; j < length; j++) {                                                    //nastavime inkrementy pre sekvenciu noh, ktora sa bude hybat spolu
    for (uint8_t i = 0; i < numberOfLegs; i += length) {                                    //zdvihne nohu od basic do polovice
      if ((sequence[j] + i) < 3) {                                                          //lava strana
        legs[sequence[j] + i].setXIncrement(myDirection * (+getIncrement(xStandPosition, ((xStandPosition - xSitPosition) / 2 + xSitPosition)))); //x na lavej strane zmensujeme, preto je inkrement - (netreba pridavat znamienko, hodnoty zadane v danom poradi vyvoria minusovy inkrement)
      }
      else {                                                                                //prava strana
        legs[sequence[j] + i].setXIncrement(myDirection * (-getIncrement(xStandPosition, ((xStandPosition - xSitPosition) / 2 + xSitPosition)))); //x na pravej strane zvacsujeme, preto je inkrement +  (treba pridat -, aby bol konecny ink +)
      }
      legs[sequence[j] + i].setZIncrement(getIncrement(zStandPosition, zForwardPosition));  //zdvihne nohu
    }
    setNewPositions(sequence[j], numberOfLegs, length);                                     //najprv nohy v jednej sekvencii spolu zdvihne do polovice

    //polozenie noh
    for (uint8_t i = 0; i < numberOfLegs; i += length) {                                    //polozi nohu od polovice na konecnu poziciu
      if ((sequence[j] + i) < 3) {                                                          //lava strana
        legs[sequence[j] + i].setXIncrement(myDirection * (+getIncrement(xStandPosition, ((xStandPosition - xSitPosition) / 2 + xSitPosition)))); //x na lavej strane zmensujeme, preto je inkrement -
      }
      else {                                                                                //prava strana
        legs[sequence[j] + i].setXIncrement(myDirection * (-getIncrement(xStandPosition, ((xStandPosition - xSitPosition) / 2 + xSitPosition)))); //x na pravej strane zvacsujeme, preto je inkrement +
      }
      legs[sequence[j] + i].setZIncrement(-getIncrement(zStandPosition, zForwardPosition)); //polozi nohu
    }
    setNewPositions(sequence[j], numberOfLegs, length);                                     //nohy v jednej sekvencii spolu polozi
  }

  //prenesenie vahy na druhu stranu, v pripade chodze je to prenesenie vahy na pravu stranu ak sa pozerama na hexapoda spredu
  for (uint8_t i = 0; i < numberOfLegs / 2; i++) {                                          //lava strana
    legs[i].setXIncrement(myDirection * (-getIncrement(xStandPosition, xSitPosition)));     //ak je konecny inkrement +, tak zvysujeme vzdialenost od tela pre x
  }
  for (uint8_t i = numberOfLegs / 2; i < numberOfLegs; i++) {                               //prava strana
    legs[i].setXIncrement(myDirection * (+getIncrement(xStandPosition, xSitPosition)));     //ak je konecny inkrement -, tak znizujeme vzdialenost od tela pre x
  }
  setYIncrements(0, 0);     //nemenime
  setZIncrements(0, 0);     //takisto zotrvava rovnake
  setNewPositions(0, numberOfLegs, 1);

  interval = oldInterval;
}


void sitDown() {
  for (uint8_t i = 0; i < length; i++) {  //cyklus na sekvenciu
    //zdvihne nohu
    setXIncrements(0, 0);                 //zotrvava na povodnom mieste
    setYIncrements(0, 0);                 //zostane 0 celu dobu az dovtedy dokym sa nebudeme snazit spravit krok dopredu
    setZIncrements(-3.6, -6.6);
    setNewPositions(sequence[0 + i], numberOfLegs, length);

    //znizi nohu a polozi ju pod telo
    setXIncrements(-xSitPosition, -xStandPosition);
    setYIncrements(0, 0);                 //zostane 0 celu dobu az dovtedy dokym sa nebudeme snazit spravit krok dopredu
    setZIncrements(3.6, 6.6);
    setNewPositions(sequence[0 + i], numberOfLegs, length);
  }
  //znizi telo - "sadne si"
  setXIncrements(0, 0);
  setYIncrements(0, 0);                   //zostane 0 celu dobu az dovtedy dokym sa nebudeme snazit spravit krok dopredu
  setZIncrements(-3.6, -6.6);
  setNewPositions(0, numberOfLegs, 1);
}


void setLegsToBasicPosition() {           //nastavi na zakladnu stojacu poziciu na zaklade toho aky pohyb vykonaval
  if (walkingForward) {
    //Serial.println("walk forward end");
    //walkForwardEnd();
    walkPrepare(-1);
    walkingForward = false;
    return;
  }
  else if (walkingBackward) {
    //Serial.println("walk backward end");
    //walkForwardEnd();
    walkPrepare(-1);
    stride = -stride;
    yForwardPosition = -yForwardPosition;
    walkingBackward = false;
    return;
  }
  else if (walkingRight) {
    //Serial.println("walk right end");
    walkSidewayPrepare(xStandPosition, (xStandPosition-xSitPosition)/2 + xSitPosition, -1);
    walkingRight = false;
    return;
  }
  else if (walkingLeft) {
    //Serial.println("walk left end");
    walkSidewayPrepare((xStandPosition-xSitPosition)/2 + xSitPosition, xStandPosition,  -1);
    walkingLeft = false;
    return;
  }
  //sem by sa nemal nikto dostat lebo, to znamena ze nohy stale neboli nastavene do povodnej pozicie
}



void checkLegs() {
  if (!areLegsInBasicPosition()) {
    setLegsToBasicPosition();
  }
}


//predtym ako sa spusti kazda funkcia, je potrebne aby bol hexapod v zakladnej stojacej polohe, cize treba spustit checkLegs();
void handleCommand() {
  if (standing) {                                 //ak robot stoji
    if (strcmp(commandBuffer, "FORWARD") == 0) {  //ak opatovne prijme tento prikaz, tak by robot nemal nijako reagovat
      if (walkingForward) {
        //Serial.println("step forward");
        walk();
      }
      else {
        //Serial.println("walk forward start");
        checkLegs();                              //toto tu je potrebne vtedy ak z jedneho typu chodze hned prepneme do ineho
        //walkForwardStart();                       //nastavi nohy dopredu
        walkPrepare(1);
        walkingForward = true;
      }
    }
    else if (strcmp(commandBuffer, "BACKWARD") == 0) {
      if (walkingBackward) {
        //Serial.println("step backwards");
        walk();
      }
      else {
        //Serial.println("walk backward start");
        checkLegs();
        stride = -stride;
        yForwardPosition = -yForwardPosition;
        //walkForwardStart();                       //nastavi nohy dopredu
        walkPrepare(1);
        walkingBackward = true;
      }
    }
    else if (strcmp(commandBuffer, "LEFT") == 0) {
      if (walkingLeft) {
        //Serial.println("step left");
        walkSideway(-1);                          //vyjadruje vchodzu vlavo
      }
      else {
        //Serial.println("walk left start");
        checkLegs();
        //mensia hodnota znamena, ze sa robot bude hybat danym smerom, 1 znamena ze nohy treba umiestnit rovnobezne so strednou
        walkSidewayPrepare((xStandPosition - xSitPosition) / 2 + xSitPosition, xStandPosition, 1); 
        walkingLeft = true;
      }
    }
    else if (strcmp(commandBuffer, "RIGHT") == 0) {
      if (walkingRight) {
        //Serial.println("step right");
        walkSideway(+1);                            //vyjadruje vchodzu vpravo
      }
      else {
        //Serial.println("walk right start");
        checkLegs();
        //mensia hodnota znamena, ze sa robot bude hybat danym smerom, 1 znamena ze nohy treba umiestnit rovnobezne so strednou
        walkSidewayPrepare(xStandPosition, (xStandPosition - xSitPosition) / 2 + xSitPosition, 1); 
        walkingRight = true;
      }
    }
    else if (strcmp(commandBuffer, "TURN_LEFT") == 0) {
      //Serial.println(" z funckie rotate left");
      checkLegs();
      rotate(-1);                                             //otoci vlavo
    }
    else if (strcmp(commandBuffer, "TURN_RIGHT") == 0) {
      //Serial.println(" z funckie rotate right");
      checkLegs();
      //Serial.println("1");
      rotate(+1);                                             //otoci vpravo
    }
    else if (strcmp(commandBuffer, "STOP") == 0) {            //netreba implementovat ziadnu zvlast funcku?
      //Serial.println(" z fukncie stop");
      checkLegs();
      delay(500);
      commandBuffer[0] = (char)0;                             //vycistit ten buffer aby sa funkcia stop nespustala neustale
    }
    else if (strcmp(commandBuffer, "SIT_DOWN") == 0) {
      //Serial.print(" z funckie sit down");
      checkLegs();
      sitDown();
      standing = false;
    }
  }
  else {                                                      //ak nestoji, tak sedi, cize sa moze postavit
    if (strcmp(commandBuffer, "STAND_UP") == 0) {
      //Serial.print(" z funckie stand up");
      standUp();                                              //tu netreba kontrolovat ci nohy v zakladnej pozicii statia, vsak lebo robot sedi ;-)
      standing = true;
    }
  }
}
