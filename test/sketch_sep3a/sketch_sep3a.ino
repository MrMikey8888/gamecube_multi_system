#define CONSOLE 3 // to change
#define PORTCONDITION 4 // status of the data is in portdata
#define PORTDATA1 8 // data being transfered
#define PORTDATA2 9 // data being transfered
#define PORTDATA3 10 // data being transfered
#define PORTDATA4 11 // data being transfered

uint8_t bitmask = 0x00;
volatile uint8_t* inport;
void setup() {


  // put your setup code here, to run once:
  Serial.begin(9600);


  uint8_t AportData1 = digitalPinToPort(PORTDATA1);
  uint8_t AportData2 = digitalPinToPort(PORTDATA2);
  uint8_t AportData3 = digitalPinToPort(PORTDATA3);
  uint8_t AportData4 = digitalPinToPort(PORTDATA4);

  uint8_t BportData1 = digitalPinToBitMask(PORTDATA1);
  uint8_t BportData2 = digitalPinToBitMask(PORTDATA2);
  uint8_t BportData3 = digitalPinToBitMask(PORTDATA3);
  uint8_t BportData4 = digitalPinToBitMask(PORTDATA4);

  bitmask |= BportData1;
  bitmask |= BportData2;
  bitmask |= BportData3;
  bitmask |= BportData4;

  inport = portInputRegister(AportData1);


  Serial.print("\n1:");
  Serial.print(BportData1, BIN);
  Serial.print("\n2:");
  Serial.print(BportData2, BIN);
  Serial.print("\n3:");
  Serial.print(BportData3, BIN);
  Serial.print("\n4:");
  Serial.print(BportData4, BIN);

  Serial.print("\n bitmask:");
  Serial.print(bitmask, BIN);

  volatile uint8_t* portData1 = portOutputRegister(AportData1);
  volatile uint8_t* portData2 = portOutputRegister(AportData2);
  volatile uint8_t* portData3 = portOutputRegister(AportData3);
  volatile uint8_t* portData4 = portOutputRegister(AportData4);

  Serial.print("\n1:");
  Serial.print(*portData1, BIN);
  Serial.print("\n2:");
  Serial.print(*portData2, BIN);
  Serial.print("\n3:");
  Serial.print(*portData3, BIN);
  Serial.print("\n4:");
  Serial.print(*portData4, BIN);

  volatile uint8_t* portmode = portModeRegister(AportData1);
  // set as outport
  *portmode &= ~bitmask;
  // enable the pullup resistor
  *portData1 |= bitmask;
}

void loop() {
  uint8_t pin_status = *inport;
  pin_status &= bitmask;
  // put your main code here, to run repeatedly:
  Serial.print("\npin data:");
  Serial.print(pin_status, BIN);
  delay(500);
}
