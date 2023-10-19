#define CONTOLLER 3 // to change
//these 5 pins need to be on the same port, the data ones work best if on pin 1 - 4 but can be on others,k just edit the asm
#define PORTCONDITION 12 // status of the data is in portdata TODO: work out a better pin, if memory is a problem put in same buss
#define PORTDATA1 8 // data being transfered
#define PORTDATA2 9 // data being transfered
#define PORTDATA3 10 // data being transfered
#define PORTDATA4 11 // data being transfered



// data for the pin on the console
uint8_t bitMaskContoller;

volatile uint8_t* modePortContoller;
volatile uint8_t* outPortContoller;
volatile uint8_t* inPortContoller;

// bitmask for the info and the status
uint8_t bitMaskStatus;
uint8_t bitMaskInfo;

// outport for the comunicatiuon of other boards
volatile uint8_t* outPortComm;


// data to send 
uint8_t data1[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
uint8_t data2[8] = {0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t data3[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t data4[8] = {0x30, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t data5[8] = {0x40, 0xA4, 0xD9, 0xE1, 0x8F, 0xAD, 0x27, 0x11};
uint8_t data6[8] = {0x50, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};


void setup() {
  setupports();
  Serial.begin(9600);
  Serial.print("starting\n");
  delay(1000);
  //Serial.print("\n");
  //Serial.print(bitMaskStatus, BIN);
  //Serial.print("\n");
  //Serial.print(bitMaskInfo, BIN);
  //Serial.print("\n");
  //Serial.print(*modePortStatus, BIN);
  //Serial.print("\n");
  //Serial.print(*outPortStatus, BIN);
  //Serial.print("\n");
  //Serial.print(*modePortInfo, BIN);
  //Serial.print("\n");
  //Serial.print(*outPortInfo, BIN);
  //Serial.print("\n");
  //Serial.print(*modePortStatus, BIN);
  //Serial.print("\n");
  //Serial.print(*outPortStatus, BIN);
  //Serial.print("\n");
  //Serial.print(*modePortInfo, BIN);
  //Serial.print("\n");
  //Serial.print(*outPortInfo, BIN);
  //Serial.print("\n");
}

void loop() {
  uint8_t oldSREG = SREG;
  cli();
  // put your main code here, to run repeatedly:
  //// set pin to output and ensure that it is high, we have different ones since they may be on differnet ports
  stream_data(data1, 0x08);
  stream_data(data2, 0x08);
  stream_data(data3, 0x08);
  stream_data(data4, 0x08);
  stream_data(data5, 0x08);
  stream_data(data6, 0x08);
  Serial.print("sent data\n");
  SREG = oldSREG;
}

void setupports() {
  // data for the information status pin
  bitMaskContoller = digitalPinToBitMask(CONTOLLER);
  uint8_t portContoller = digitalPinToPort(CONTOLLER);

  modePortContoller = portModeRegister(portContoller); // need to swap from in and out
  outPortContoller = portOutputRegister(portContoller); // need to set the data to send
  inPortContoller = portInputRegister(portContoller); // need to 

  // data for the information status pin
  bitMaskStatus = digitalPinToBitMask(PORTCONDITION);
  uint8_t portStatus = digitalPinToPort(PORTCONDITION);

  //volatile uint8_t* modePortStatus = portModeRegister(portStatus);
  //outPortStatus = portOutputRegister(portStatus);
  //inPortStatus = portInputRegister(portStatus);


  // data for the information pin 

  bitMaskInfo = digitalPinToBitMask(PORTDATA1);
  bitMaskInfo |= digitalPinToBitMask(PORTDATA2);
  bitMaskInfo |= digitalPinToBitMask(PORTDATA3);
  bitMaskInfo |= digitalPinToBitMask(PORTDATA4);

  uint8_t portInfo = digitalPinToPort(PORTDATA1);

  //volatile uint8_t* modePortInfo = portModeRegister(portInfo);
  //outPortInfo = portOutputRegister(portInfo);
  outPortComm = portOutputRegister(portInfo);

  // set information pins to output, (since all data are on the same rail we only need these 2)
  *portModeRegister(portStatus) |= bitMaskStatus;
  *portModeRegister(portInfo) |= bitMaskInfo;

  // set the pins to low
  *outPortComm &= ~bitMaskStatus;
  *outPortComm &= ~bitMaskInfo;
}

// make logic to track the data from the gamecube and then send stored data to the console
void stream_data(const uint8_t* buff, uint8_t len)
{
    // temporary register values used as "clobbers"
    register uint8_t temp_data;
    //register uint8_t send_buffer;

    asm volatile (
        "; Start of assembly to read gamecube input then respomd\n"

        // Instruction cycles are noted in parentheses
        // branch instructions have two values, one if the branch isn't
        // taken and one if it is

        // passed in the state of the pins 
        // passed in the location to set pins
        // passed in is the number of bytes to send

        // needs a register to use as temp storage for bit
        // needs register sending buffer 
        // needs register to count the bytes sent

        // sudocode for this 
        // place to jump to that sets all the lines low apart from the controle line (want to have )

        // **set all the lines high**
        // and the sending buffer with 0x10
        // wait needed nops

        // **set line low**
        // load the bit into a temp register
        // right shift by 4
        // set the 5th bit to be 1 (this is where the sending loop starts if uninitalised)
        // or the sending buffer with the temp register
        // wait needed nops

        // **set the line to the data**
        // or the buffer with 0x01
        // wait needed nops

        // **set all the lines high**
        // and the sending buffer with 0x10
        // wait needed nops

        // **set line low**
        // load the bit into a temp register
        // and with 0x01
        // or the sending buffer with the temp register
        // wait needed nops

        // **set the line to the data**
        // or the buffer with 0x01
        // decrement the bytes sent counter
        // jump to setting the lines low if it is not 0
        // 
        // program exit

        "ori %[send_buffer], 0x0F\n" // (1) send buffer will have the lower bits set to high and upper bits unchanged

        ".L%=_bit_start:\n"

        // set the pins low
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "andi %[send_buffer], 0xF0\n" // (1) sets the lower pins in the buffer to be 0
        // 16 cycles for the bit - 3 above 
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\n" //(3) (13)

        // sets the pins high
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "ld %[temp_data], %a[buff]\n" // (2) load the next byte into the temp register
        "lsr %[temp_data]\n" // (1)
        "lsr %[temp_data]\n" // (1)
        "lsr %[temp_data]\n" // (1)
        "lsr %[temp_data]\n" // (1)
        "ori %[temp_data], 0x10\n" // (1) (sets the bit to indicate that data is being sent)
        "or %[send_buffer], %[temp_data]\n" // (1)
        // we are 10/16 cycles into the data
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\n" //(1) (6)

        // set the pins to the data
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "ori %[send_buffer], 0x0F\n" // (1) sets lower pins to high
        // we are 3/16 cycles into the data
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\n" //(3) (13) 


        // sets the pins high again
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "andi %[send_buffer], 0xF0\n" // (1) sets the lower pins in the buffer to be 0
        // we are 3/16 cycles into the data 
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\n" //(3) (13)

        // sets the pins low again
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "ld %[temp_data], %a[buff]+\n" // (2) load the next byte into the temp register
        "andi %[temp_data], 0x0F\n" // (1) only keeps the lower 4 bits
        "or %[send_buffer], %[temp_data]\n" // (1) sets the lower 4 bits to the data
        // we are 6/16 cycles into the data 
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)

        // set the pins to the data
        "st %a[outPort],%[send_buffer]\n" // (2) set pins to send buffer
        "ori %[send_buffer], 0x0F\n" // (1) sets lower pins to high
        "dec %[len]\n" // (1) decrements the len by 1
        "breq .L%=_exit\n" // (1/2) jumps if the value is 0
        // we are 5/16 in but have 2 from the jump
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\n" //(4) (9)
        "rjmp .L%=_bit_start\n" // (2)

        ".L%=_exit:\n"
        "andi %[send_buffer], 0xE0\n" // (1) set the send buffer to leep the top 3 ports and lower 5 to be 0
        // we are 7/16 in, here
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\n" //(4) (9)
        "st %a[outPort],%[send_buffer]\n" // (2) set pins in the

        // ----------
        // outputs:
        : [buff] "+e" (buff), // (read and write) TODO: check if writes / can it move to inputs
        [outPort] "+e" (outPortComm), // (read and write)
        [temp_data] "=&d" (temp_data) // ()
        //[send_buffer] "=&r" (send_buffer) // (output only)

        // inputs:
        : [len] "r" (len),
        [send_buffer] "r" (*outPortComm) // gets the pinstates
        // no clobbers
        ); // end of asm volatile
}
