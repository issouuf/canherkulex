#include "mbed.h"
#include "herkulex.h"
#include "CAN.h"

#define SEC 1000
#define PAUSE 100

// ! Constantes pour les acknoledges de fin d'action et de réception de trame
#define IDCARD 0x01A0
#define GRABACK 0x85
#define COMBACK 0x86
#define STEPPERACK 0x87
#define SIGNACK 0x88
// #define IDCARD 0x01A1
// #define GRABACK 0x95
// #define COMBACK 0x96
// #define STEPPERACK 0x97
// #define SIGNACK 0x98

// ! Constantes pour les servos des pinces droites avant & arrière
#define LGRABBERMAX 0x03AD    // 941
#define LGRABBERMIN 0x01CD    // 461
#define LGRABBERCENTER 0x02A0 // 672
#define LGRABBERHELP 0x0335   // 821

// ! Constantes pour les servos des pinces gauches avant & arrière
#define RGRABBERMAX 0x0042    // 66
#define RGRABBERMIN 0x0227    // 551
#define RGRABBERCENTER 0x0158 // 344
#define RGRABBERHELP 0x00BA   // 186

// ! Constantes pour la levée et la descente des peignes
#define COMBUP 0x028C       // 652
#define COMBTAKING 0x1E2   // 462 //? test de 482 -> 462
#define COMBDROPPING 0x0200 // 512




// ! Constantes pour les servos des peignes avant & arrière
#define RHERKULEX 0x01 // HerkuleX Droit
#define LHERKULEX 0x02 // HerkuleX Gauche
#define COMB 0x03      // Peigne
#define SIGN 0x04      // Panneau solaire

// ! Constantes pour le servo dirigeant les panneaux solaires
#define SIGNUPPER 0x00E8 // 232
#define SIGNLOW 0x027D   // 637

// ! Constantes pour appeler lse fonctions CAN
#define CLOSE 0x01        // ? Ferme les pinces
#define OPEN 0x02         // ? Ouvre les pinces
#define PLANT 0x03        // ? Ferme les pinces en position "Prise de plantes"
#define UP 0x04           // ? Monte le peigne
#define POS 0x05          // ? Renvoie la position du servo
#define HOMING 0x06       // ? Renvoie le moteur pas-à-pas à sa position de départ
#define ELEVATORUP 0x07   // ? Monte l'ascenseur
#define ELEVATORMID 0x08  // ? Monte l'ascenseur à mi-hauteur
#define ELEVATORDOWN 0x09 // ? Descend l'ascenseur
#define SHAKING 0x0A      // ? Secoue le peigne
#define CLEAR 0x0B        // ? Réinitialise le microcontrôleur
#define COMBTAKE 0x0C     // ? Prend le pot de fleur
#define COMBDROPOFF 0x0D  // ? Dépose le pot de fleur
#define COMBHELP 0x0E     // ? Aide le peigne à la prise de pot de fleur
#define GRABDROP 0x0F     // ? Dépose le pot de fleur dans les zones de culture
#define SIGNUP 0x10       // ? Monte le système de panneau solaire
#define SIGNDOWN 0x11     // ? Baisse le système de panneau solaire

// ! Constantes pour le moteur pas-à-pas
#define CLOCKWISE 1        // Monte l'ascenseur
#define COUNTERCLOCKWISE 0 // Descend l'ascenseur

DigitalOut STBY(D7);
DigitalOut STEP(D6);
DigitalOut DIr(D3);
DigitalOut EN(A2);
DigitalOut M0(D5);
DigitalOut M1(D9);
DigitalOut M2(D8);
DigitalOut led(LED1);

// ! Initialisation du FDC
DigitalIn FDC(PA_1);

Herkulex servo(PB_6, PB_7, 115200);

CAN can(PA_11, PA_12, 1000000); // CAN Rx pin name, CAN Tx pin name

int etat;

// TODO : Fonction pour faire une pause comme sur Arduino
void delay(int ms);

//* Envoie un message CAN
void CANFill(CANMessage &msg, char length, int id, char data0h, char data1h, char data2h, char data3h, char data4h, char data5h, char data6h, char data7h);

//* Affiche un message CAN reçu
void printCANMsg(CAN *can, CANMessage &msg);

//* Fonction pour ouvrir les pinces avant
void openingGrabber();

//* Fonction pour fermer les pinces avant
void closingGrabber();

//* Fonction pour fermer les pinces avant en position plant
void plantClosingGrabber();

//* Fonction pour fermer les pinces avant en position plant
void grabberHelp();

//* Fonction pour lever le peigne avant
void CombUp();

//* Fonction pour baisser le peigne avant
void Position(int HerkuleXID);

//* Fonction pour baisser le peigne avant à mi-hauteur
void CombMid();

//* Fonction pour secouer le peigne avant
void CombShaking();

//* Fonction pour monter l'ascenseur
void elevatorUp();

//* Fonction pour descendre l'ascenseur
void elevatorDown();

//* Fonction pour descendre l'ascenseur à mi-hauteur
void elevatorMid();

//* Fonction pour activer le couple du moteur pas-à-pas de l'ascenseur
void blockStepper();

//* Fonction pour le retour à la position de départ du moteur pas-à-pas
void homingStepper();

//* Fonction pour initialiser les servos
void initServo();

//* Fonction pour initialiser le moteur pas-à-pas
void initStepper();

//* Fonction pour prendre le pot de fleur
void CombTaking();

//* Fonction pour déposer le pot de fleur
void CombDropping();

//* Fonction pour déposer le pot de fleur dans les zones de culture
void grabberDrop();

//* Fonction pour la prise de pots de fleur avec peignes
void grabberHelp();

//* Fonction pour lever le système de panneau solaire
void signUp();

//* Fonction pour baisser le système de panneau solaire
void signDown();

//* Fonction pour la mise en route du moteur pas-à-pas
int stepper(int nsteps, bool m0, bool m1, bool m2, bool direction, float time);

char* ptrGPIO = 0b0000000;

int main()
{
  initServo();
  initStepper();
  homingStepper();

  ptrGPIO = (char *)0x40021000;

  CANMessage TXMsg;
  CANMessage RXMsg;

  while (1)
  {
    printf("Rherkulex %d",servo.getPos(RHERKULEX));
    printf("             Lherkulex %d ",servo.getPos(LHERKULEX));
    printf("                          comb %d",servo.getPos(COMB));
    printf("                                  SIGN %d \n",servo.getPos(SIGN));
    if (can.read(RXMsg) && (RXMsg.id == IDCARD))
    {
      switch (RXMsg.data[0])
      {
      case OPEN:
        openingGrabber();
        break;

      case CLOSE:
        closingGrabber();
        break;

      case PLANT:
        plantClosingGrabber();
        break;

      case UP:
        CombUp();
        break;

      case POS:
        Position(RXMsg.data[2]);
        break;

      case HOMING:
        homingStepper();
        break;

      case ELEVATORUP:
        elevatorUp();
        break;

      case ELEVATORDOWN:
        elevatorDown();
        break;

      case ELEVATORMID:
        elevatorMid();
        break;
      case SHAKING:
        CombShaking();
        break;

      case CLEAR:
        __NVIC_SystemReset();
        break;

      case COMBTAKE:
        CombTaking();
        break;

      case COMBDROPOFF:
        CombDropping();
        break;

      case COMBHELP:
        grabberHelp();
        break;

      case GRABDROP:
        grabberDrop();
        break;

      case SIGNUP:
        signUp();
        break;

      case SIGNDOWN:
        signDown();
        break;

      default:
        break;
      }
      delay(PAUSE);
    }
  }
}

void CANFill(CANMessage &msg, char length, int id, char data0h, char data1h, char data2h, char data3h, char data4h, char data5h, char data6h, char data7h)
{
  msg.len = length;
  msg.type = CANData;
  msg.format = CANStandard;
  msg.id = id;
  msg.data[0] = data0h;
  msg.data[1] = data1h;
  msg.data[2] = data2h;
  msg.data[3] = data3h;
  msg.data[4] = data4h;
  msg.data[5] = data5h;
  msg.data[6] = data6h;
  msg.data[7] = data7h;
}

void printCANMsg(CAN *can, CANMessage &msg)
{
  printf("  ID      = 0x%.3x\r\n", msg.id);
  printf("  Type    = %d\r\n", msg.type);
  printf("  format  = %d\r\n", msg.format);
  printf("  Length  = %d\r\n", msg.len);
  printf("  Data    =");
  for (int i = 0; i < msg.len; i++)
  {
    printf(" 0x%.2X", msg.data[i]);
  }
  printf("\r\n");
}

int stepper(int swpulse, int m0, int m1, int m2, int dir, int dur, bool up = false)
{
  M0 = m0;
  M1 = m1;
  M2 = m2;
  DIr = dir;
  EN = 1;
  // step generator
  for (int i = 0; i < swpulse; i++)
  {
    STEP = 1;
    ThisThread::sleep_for(2ms);
    STEP = 0;
    ThisThread::sleep_for(2ms);
    if ((FDC.read()) == 1 && up)
    {
      EN = 0;
      return 1;
    }
  }
  EN = 0;
  return 0;
}

void delay(int ms)
{
  wait_us((int)(ms * 1000.0));
}

void initServo(void)
{
  servo.clear(BROADCAST_ID);
  servo.setTorque(BROADCAST_ID, TORQUE_ON);
  servo.clear(BROADCAST_ID);
}
void blockStepper(void)
{
  STBY = 1;
  EN = 1;
  M0 = 1;
  M1 = 1;
  M2 = 1;
}

void initStepper(void)
{
  STBY = 1;
  EN = 0;
  M0 = 0;
  M1 = 0;
  M2 = 0;
}

void openingGrabber()
{
  CANMessage TXMsg;
  servo.clear(BROADCAST_ID);
  servo.positionControl_Mul_ensemble(RHERKULEX, RGRABBERMAX, 35, BLED_ON, LHERKULEX, LGRABBERMAX, GLED_ON);
  servo.clear(BROADCAST_ID);
  TXMsg.id = IDCARD;
  TXMsg.len = 1;
  TXMsg.data[0] = GRABACK;
  can.write(TXMsg);
}

void closingGrabber()
{
  CANMessage TXMsg;
  servo.clear(BROADCAST_ID);
  servo.positionControl_Mul_ensemble(RHERKULEX, RGRABBERMIN, 35, BLED_ON, LHERKULEX, LGRABBERMIN, GLED_ON);
  servo.clear(BROADCAST_ID);
  TXMsg.data[0] = GRABACK;
  can.write(TXMsg);
}

void plantClosingGrabber()
{
  CANMessage TXMsg;
  servo.clear(BROADCAST_ID);
  servo.positionControl_Mul_ensemble(RHERKULEX, RGRABBERCENTER, 75, BLED_ON, LHERKULEX, LGRABBERCENTER, GLED_ON);
  servo.clear(BROADCAST_ID);
  TXMsg.data[0] = GRABACK;
  can.write(TXMsg);
}

void CombUp()
{
  CANMessage TXMsg;
  servo.clear(COMB);
  servo.positionControl(COMB, COMBTAKING + 26, 60, GLED_ON);
  servo.positionControl(COMB, COMBTAKING + 52, 45, GLED_ON);
  servo.positionControl(COMB, COMBTAKING + 78, 45, GLED_ON);
  servo.positionControl(COMB, COMBTAKING + 114, 60, GLED_ON);
  servo.positionControl(COMB, COMBUP, 75, GLED_ON);
  servo.clear(COMB);
  TXMsg.data[0] = COMBACK;
  can.write(TXMsg);
}

void Position(int HerkuleXID)
{
  CANMessage TXMsg;
  uint16_t pos = 0;
  servo.clear(HerkuleXID);
  pos = servo.getPos(HerkuleXID);
  servo.clear(HerkuleXID);
  TXMsg.len = 3;
  TXMsg.data[0] = HerkuleXID;
  TXMsg.data[1] = (pos >> 8) & 0x00FF;
  TXMsg.data[2] = (pos << 8) & 0xFF00;
  can.write(TXMsg);
}

void CombShaking()
{
  CANMessage TXMsg;
  int count = 20;
  int pos = 0;
  pos = servo.getPos(COMB);
  for (int i = 0; i < count; i++)
  {
    servo.positionControl(COMB, pos - 3, 1, RLED_ON);
    servo.positionControl(COMB, pos + 3, 1, RLED_ON);
  }
  TXMsg.data[0] = COMBACK;
  can.write(TXMsg);
}

void CombTaking()
{
  CANMessage TXMsg;
  servo.clear(BROADCAST_ID);
  servo.positionControl(COMB, COMBTAKING, 45, BLED_ON);
  servo.clear(BROADCAST_ID);
  TXMsg.data[0] = COMBACK;
  can.write(TXMsg);
}

void CombDropping()
{
  CANMessage TXMsg;
  servo.clear(COMB);
  servo.positionControl(COMB, COMBUP - 26, 60, GLED_ON);
  servo.positionControl(COMB, COMBUP - 52, 45, GLED_ON);
  servo.positionControl(COMB, COMBUP - 78, 45, GLED_ON);
  servo.positionControl(COMB, COMBUP - 114, 60, GLED_ON);
  servo.positionControl(COMB, COMBDROPPING, 75, GLED_ON);
  servo.clear(COMB);
  TXMsg.data[0] = COMBACK;
  can.write(TXMsg);
}

void grabberHelp(void)
{
  CANMessage TXMsg;
  servo.clear(BROADCAST_ID);
  servo.positionControl_Mul_ensemble(RHERKULEX, RGRABBERHELP, 90, BLED_ON, LHERKULEX, LGRABBERHELP, GLED_ON);
  servo.clear(BROADCAST_ID);
  TXMsg.data[0] = GRABACK;
  can.write(TXMsg);
}

void grabberDrop(void)
{
  CANMessage TXMsg;
  servo.clear(BROADCAST_ID);
  servo.positionControl_Mul_ensemble(RHERKULEX, RGRABBERCENTER + 62, 60, GLED_ON, LHERKULEX, LGRABBERCENTER - 66, BLED_ON);
  servo.positionControl_Mul_ensemble(RHERKULEX, RGRABBERCENTER + 124, 45, GLED_ON, LHERKULEX, LGRABBERCENTER - 132, BLED_ON);
  servo.positionControl_Mul_ensemble(RHERKULEX, RGRABBERCENTER + 186, 45, GLED_ON, LHERKULEX, LGRABBERCENTER - 198, BLED_ON);
  servo.positionControl_Mul_ensemble(RHERKULEX, RGRABBERCENTER + 246, 60, GLED_ON, LHERKULEX, LGRABBERCENTER - 264, BLED_ON);
  servo.positionControl_Mul_ensemble(RHERKULEX, RGRABBERMAX, 75, GLED_ON, LHERKULEX, LGRABBERMAX, BLED_ON);
  servo.clear(BROADCAST_ID);
  TXMsg.data[0] = GRABACK;
  can.write(TXMsg);
}

void signUp(void)
{
  CANMessage TXMsg;
  servo.clear(BROADCAST_ID);
  servo.positionControl(SIGN, SIGNUPPER, 45, GLED_ON);
  servo.clear(BROADCAST_ID);
  TXMsg.data[0] = SIGNACK;
  can.write(TXMsg);
}

void signDown(void)
{
  CANMessage TXMsg;
  servo.clear(BROADCAST_ID);
  servo.positionControl(SIGN, SIGNLOW, 45, BLED_ON);
  servo.clear(BROADCAST_ID);
  TXMsg.data[0] = SIGNACK;
  can.write(TXMsg);
}

void homingStepper(void)
{
  CANMessage TXMsg;
  while (FDC.read() == 0)
  {
    stepper(20, 0, 0, 0, 1, 1);
  }
  etat = 1;
  TXMsg.data[0] = STEPPERACK;
  can.write(TXMsg);
}

void elevatorDown(void)
{
  CANMessage TXMsg;
  if (etat == 1)
  {
    stepper(610, 0, 0, 0, 0, 1);
  }
  else if (etat == 2)
  {
    stepper(305, 0, 0, 0, 0, 1);
  }
  etat = 3;
  TXMsg.data[0] = STEPPERACK;
  can.write(TXMsg);
}

void elevatorMid(void)
{
  CANMessage TXMsg;
  if (etat == 1)
  {
    stepper(305, 0, 0, 0, 0, 1);
  }
  else if (etat == 3)
  {
    stepper(305, 0, 0, 0, 1, 1, true);
  }
  etat = 2;
  TXMsg.data[0] = STEPPERACK;
  can.write(TXMsg);
}

void elevatorUp(void)
{
  CANMessage TXMsg;
  if (etat == 3)
  {
    stepper(610, 0, 0, 0, 1, 1, true);
  }
  else if (etat == 2)
  {
    stepper(305, 0, 0, 0, 1, 1, true);
  }
  etat = 1;
  TXMsg.data[0] = STEPPERACK;
  can.write(TXMsg);
}
