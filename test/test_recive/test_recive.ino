#define CONSOLE 3 // to change
//these 5 pins need to be on the same port, the data ones work best if on pin 1 - 4 but can be on others,k just edit the asm
#define PORTCONDITION 12 // status of the data is in portdata TODO: work out a better pin, if memory is a problem put in same buss
#define PORTDATA1 8 // data being transfered
#define PORTDATA2 9 // data being transfered
#define PORTDATA3 10 // data being transfered
#define PORTDATA4 11 // data being transfered

uint8_t command[3];
// data for the 
uint8_t initaldata[8]; 
uint8_t console_data[8];

// data for the pin on the console
uint8_t bitMaskConsole;

volatile uint8_t* modePortConsole;
volatile uint8_t* outPortConsole;
volatile uint8_t* inPortConsole;

// bitmask for the info and the status
uint8_t bitMaskStatus;
uint8_t bitMaskInfo;
// inport for the comunicatiuon of other boards 
volatile uint8_t* inPortComm;


void setup() {
  Serial.begin(9600);
  setupports();
  Serial.print("starting\n");


  //Serial.println(*inPortComm, BIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t oldSREG = SREG;
  cli();
  uint8_t len = getmaininput(console_data, 0x08);
  if (len != 0x08) {
    Serial.print(len, HEX);
    printdata();
  } else {
    //Serial.println(len);
  }
  SREG = oldSREG;
  //Serial.print(getmaininput(console_data, 0x08), HEX);
  //printdata();
}

void printdata() {
  Serial.print("\nData starting:");
  Serial.print("\ndata[0], 0x");
  Serial.print(console_data[0], HEX);
  Serial.print("\ndata[1], 0x");
  Serial.print(console_data[1], HEX);
  Serial.print("\ndata[2], 0x");
  Serial.print(console_data[2], HEX);
  Serial.print("\ndata[3], 0x");
  Serial.print(console_data[3], HEX);
  Serial.print("\ndata[4], 0x");
  Serial.print(console_data[4], HEX);
  Serial.print("\ndata[5], 0x");
  Serial.print(console_data[5], HEX);
  Serial.print("\ndata[6], 0x");
  Serial.print(console_data[6], HEX);
  Serial.print("\ndata[7], 0x");
  Serial.print(console_data[7], HEX);
  Serial.print("\n\n");
  //delay(1234);
}

void setupports() {
  // data for the information status pin
  bitMaskConsole= digitalPinToBitMask(CONSOLE);
  uint8_t portConsole = digitalPinToPort(CONSOLE);

  modePortConsole = portModeRegister(portConsole); // need to swap from in and out
  outPortConsole = portOutputRegister(portConsole); // need to set the data to send
  inPortConsole = portInputRegister(portConsole); // need to 

  // data for the information status pin
  bitMaskStatus = digitalPinToBitMask(PORTCONDITION);
  uint8_t portStatus = digitalPinToPort(PORTCONDITION);

  //volatile uint8_t* modePortStatus = portModeRegister(portStatus);
  //outPortStatus = portOutputRegister(portStatus);
  //inPortStatus = portInputRegister(portStatus);

  //Serial.print(bitMaskStatus, DEC);
  //Serial.print("\n");
  // data for the information pin 

  bitMaskInfo = digitalPinToBitMask(PORTDATA1);
  bitMaskInfo |= digitalPinToBitMask(PORTDATA2);
  bitMaskInfo |= digitalPinToBitMask(PORTDATA3);
  bitMaskInfo |= digitalPinToBitMask(PORTDATA4);

  uint8_t portInfo = digitalPinToPort(PORTDATA1);

  //volatile uint8_t* modePortInfo = portModeRegister(portInfo);
  //outPortInfo = portOutputRegister(portInfo);
  inPortComm = portInputRegister(portInfo);

  // set information pins to input, (since all data are on the same rail we only need these 2)
  *portModeRegister(portStatus) &= ~bitMaskStatus;
  *portModeRegister(portInfo) &= ~bitMaskInfo;

  // enable internel pullup resistor
  *portOutputRegister(portStatus) |= bitMaskStatus;
  *portOutputRegister(portInfo) |= bitMaskInfo;
}



uint8_t getmaininput(const uint8_t* buff, uint8_t len)
{
    // hard coded to only read 8 bits


    // set pin to input, we have different ones since they may be on differnet ports
    //*modePortStatus &= ~bitMaskStatus;
    //*modePortInfo &= ~bitMaskInfo;

    // enable internel pullup resistor
    //*outPortStatus |= bitMaskStatus;
    //*outPortInfo |= bitMaskInfo;

    // temporary register values used as "clobbers"

    register uint8_t data_temp; // temp for bytes recived
    register uint8_t bytecount;
    //register uint8_t waitloop2;
    register uint8_t inputVal; // temp to store the pinstates when reading. 
    register uint8_t timeout; // timeout for the loops while reading the data

    // first time compleatly writing asm code with no predefined skeleton to watch for. 
    asm volatile (
        "; Start of assembly to read input\n"
        // passed in to this block are: TODO: 
        // the %a[buff] register is the buffer pointer for the responce to the console
        // %[len] is the register holding the length of the buffer in bytes


        // basic sudocode of this function:
        // start with a loop until the contol wire goes low
        //        this waits for the previous data to finish sending
        // loop until the wire goes high
        //        this indicates its time to start reading
        // start of byte loop
        // check that all the wires are low,
        //        start of the bit
        // data is being sent for 2us, we want to try read at the lowest of 10 cycles into it (highest will be about 18)
        //        this ensures that the data has updated
        // and the data with the bitmask 
        // put into the data buffer 
        // shift the data buffer left by 4 !!!!! can be moved down
        // enter a wait loop for the data to go high signeling the end of the bit
        // enter a loop for the data to go low 
        // wait time to read the data since it may not be set yet
        // read pins into temp register
        // and the data with the bitmask
        // or it with the data in the buffer      // save the new data into the buff and incrmeent buff to new sopt
        // check that was not the last byte
        // wait for line to go high 
        // jump to start of the loop
        // 
        // jump here when last byte
        // early mark, no need to read that data was high in last bit we are free


        // Instruction cycles are noted in parentheses
        // branch instructions have two values, one if the branch isn't
        // taken and one if it is

        "mov %[bytecount], %[len]\n" // (1) sets the number of bytes we expect to the len
        // initally lets check if the pin is already low: 
        ".L%=_wait_low_loop:\n"
        "ld %[inputVal], %a[inPortComm]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskStatus]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "rjmp .L%=_wait_low_loop\n" // (2) loop

        ".L%=_found_low:\n"
        ".L%=_wait_high_loop:\n"
        "ld %[inputVal], %a[inPortComm]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskStatus]\n" // (1) compares the input value to the bitmask
        "brne .L%=_start_reading\n" // (1/2) jumps if the value is 1 from the and (the result was high)
        "rjmp .L%=_wait_high_loop\n" // (2) loop 


        // **Logic to read the data**


        ".L%=_wait_prebits:\n"
        "ldi %[timeout],%[timeout_length]\n" /// (1)
        // logic to wait for line to go low
        ".L%=_wait_high_prebit:\n"
        "ld %[inputVal], %a[inPortComm]\n" // (2) reads pin then saves to input value 
        "andi %[inputVal],  0x01\n" // (1) compares the input value to the bitmask
        "brne .L%=_wait_low_prebit\n" // (1/2) jumps if the value is not 0 from the and (the result was high)
        "dec %[timeout]\n" // (1) decrements timeout 
        "brne .L%=_wait_high_prebit\n" // (1/2) loop if the timeout is not 0
        "rjmp .L%=_program_exit\n" // (2) 
        
        // logic to wait for line to go high 
        ".L%=_wait_low_prebit:\n"
        "ld %[inputVal], %a[inPortComm]\n" // (2) reads pin then saves to input value 
        "andi %[inputVal], 0x01\n" // (1) compares the input value to the bitmask
        "breq .L%=_bits_1\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[timeout]\n" // (1) decrements timeout 
        "brne .L%=_wait_low_prebit\n" // (1/2) loop if the timeout is not 0
        "rjmp .L%=_program_exit\n" // (2)

        

        ".L%=_bits_1:\n"
        // TODO: add nops

        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(2) (15)
        "nop\n" // (1) (16)

        // time to program the recording of results
        ".L%=_start_reading:\n"
        // worst case time is that we took 7 cycles to identity the high and then a further 5 to get here
        // best case is that we took 0 to identify the high so we are 5 cycles into it
        // hence are 5 to 12 instructions into the inital bits of the data, this is 32 cycles long
        // we want to check etween the 12th and 19th cycle so we wait 7 cycles
        "ld %[data_temp], %a[inPortComm]\n" // (2)
        // we are between 12 and 19 instructions into the data bit
        // set a timeout for these to prevent problems

        ".L%=_wait_midbits:\n"
        "ldi %[timeout],%[timeout_length]\n"
        // need to wait for line to go 0
        ".L%=_wait_high_loop_middle:\n"
        "ld %[inputVal], %a[inPortComm]\n" // (2) reads pin then saves to input value 
        "andi %[inputVal], 0x01\n" // (1) compares the input value to the bitmask
        "brne .L%=_wait_low_loop_middle\n" // (1/2) jumps if the value is not 0 from the and (the result was high)
        "dec %[timeout]\n" // (1) decrements timeout 
        "brne .L%=_wait_high_loop_middle\n" // (1/2) loop if the timeout is not 0
        "rjmp .L%=_program_exit\n" // (2)

        ".L%=_wait_low_loop_middle:\n"
        "ld %[inputVal], %a[inPortComm]\n" // (2) reads pin then saves to input value 
        "andi %[inputVal], 0x01\n" // (1) compares the input value to the bitmask
        "breq .L%=_bits_2\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[timeout]\n" // (1) decrements timeout 
        "brne .L%=_wait_low_loop_middle\n" // (1/2) loop if the timeout is not 0
        "rjmp .L%=_program_exit\n" // (2)

        ".L%=_bits_2:\n"
        // we are between 5 and 12 nops into the high bit, it is 16long and we want to read 
        // between the 5th and 12th cycle of the data bit, this means that we need 16 cycles
        "lsl %[data_temp]\n" // (1)
        "lsl %[data_temp]\n" // (1) (2)
        "lsl %[data_temp]\n" // (1) (3)
        "lsl %[data_temp]\n" // (1) (4)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (9)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (14)
        "nop\nnop\n" // (4) (16)
        "ld %[inputVal], %a[inPortComm]\n" // (2)
        "and %[inputVal], %[bitMaskInfo]\n" // (1)
        "or %[data_temp], %[inputVal]\n" // (1)
        "st %a[buff]+,%[data_temp]\n" // (2) save %[data] back to memory and increment byte pointer
        "dec %[bytecount]\n" // (1) 
        "breq .L%=_program_exit\n" // (1/2) loop if the counter isn't 0
        "rjmp .L%=_wait_prebits\n" // (2)

        ".L%=_program_exit:\n"

        // ----------
        // outputs:
        : [buff] "+e" (buff), // (read and write) // where to save the bytes to
        [data_temp] "=&r" (data_temp), // (output only) // temp for storing bits recived when constructing bytes, ldi needs the upper registers)
        [bytecount] "=&r" (bytecount), // (output only, ldi needs the upper registers) (also the number of bits)
        [inputVal] "=&d" (inputVal), // temp storager for pin value 
        [timeout] "=&d" (timeout) // used to timeout internel loops if they continue too long
        // inputs:
        : [len] "r" (len),
        [inPortComm] "e" (inPortComm), // where to read from
        [bitMaskStatus] "r" (bitMaskStatus), // for identifying the port to check state
        [bitMaskInfo] "r" (bitMaskInfo), // for identifying the port to check state
        [timeout_length] "M" (0x40) // if we loop 20 times waiting between data then we exit, max should be 32 cycles, this exits when we reach 7x that

        // no clobbers
        ); // end of asm volatile

        return (len - bytecount);
}

