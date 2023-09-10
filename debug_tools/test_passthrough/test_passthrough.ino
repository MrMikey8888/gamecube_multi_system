#define CONTOLLER 4
#define CONSOLE 5

uint8_t start[1] = {0x00};
uint8_t startresp[3];

uint8_t getdata[3] = {0x40, 0x03, 0x00};
uint8_t byteseq[8];

uint8_t init_send[1] = {0x41};
uint8_t init_data[10];

uint8_t cmdbuffer[3];
uint8_t bitMaskContoller;

volatile uint8_t* modePortContoller;
volatile uint8_t* outPortContoller;
volatile uint8_t* inPortContoller;

uint8_t bitMaskConsole;

volatile uint8_t* modePortConsole;
volatile uint8_t* outPortConsole;
volatile uint8_t* inPortConsole;


void setup() {
  Serial.begin(9600);
  uint8_t oldSREG = SREG;
  cli();
  bitMaskContoller = digitalPinToBitMask(CONTOLLER);
  uint8_t portContoller = digitalPinToPort(CONTOLLER);

  modePortContoller = portModeRegister(portContoller); // need to swap from in and out
  outPortContoller = portOutputRegister(portContoller); // need to set the data to send
  inPortContoller = portInputRegister(portContoller); 

  bitMaskConsole = digitalPinToBitMask(CONSOLE);
  uint8_t portConsole = digitalPinToPort(CONSOLE);

  modePortConsole = portModeRegister(portContoller); // need to swap from in and out
  outPortConsole = portOutputRegister(portContoller); // need to set the data to send
  inPortConsole = portInputRegister(portContoller);  
  send(start, 1, bitMaskContoller, modePortContoller, outPortContoller);
  uint8_t len = read(startresp, 3, bitMaskContoller, modePortContoller, outPortContoller, inPortContoller);
  while (len != 3) {
    send(start, 1, bitMaskContoller, modePortContoller, outPortContoller);
    len = read(startresp, 3, bitMaskContoller, modePortContoller, outPortContoller, inPortContoller);
    Serial.print("geting init failed, last len was: ");
    Serial.println(len, DEC);
  }
  SREG = oldSREG;
  Serial.println("starting cmds");
  Serial.println(startresp[0], HEX);
  Serial.println(startresp[1], HEX);
  Serial.println(startresp[2], HEX);
  cli();
  send(init_send, 1, bitMaskContoller, modePortContoller, outPortContoller);
  len = read(init_data, 10, bitMaskContoller, modePortContoller, outPortContoller, inPortContoller);
  while (len != 10) {
    send(init_send, 1, bitMaskContoller, modePortContoller, outPortContoller);
    len = read(init_data, 10, bitMaskContoller, modePortContoller, outPortContoller, inPortContoller);
    Serial.print("geting inital data failed, last len was: ");
    Serial.println(len, DEC);
  }
  SREG = oldSREG;
  
  Serial.println("init_resp");
  Serial.println(init_data[0], BIN);
  Serial.println(init_data[1], BIN);
  Serial.println(init_data[2], BIN);
  Serial.println(init_data[3], BIN);
  Serial.println(init_data[4], BIN);
  Serial.println(init_data[5], BIN);
  Serial.println(init_data[6], BIN);
  Serial.println(init_data[7], BIN);
  Serial.println(init_data[8], BIN);
  Serial.println(init_data[9], BIN);
  cli();

  send(getdata, 3, bitMaskContoller, modePortContoller, outPortContoller);
  read(byteseq, 8, bitMaskContoller, modePortContoller, outPortContoller, inPortContoller);
  SREG = oldSREG;
}


void loop() {
  uint8_t oldSREG = SREG;
  cli();
  uint8_t len = read(cmdbuffer, 3, bitMaskConsole, modePortConsole, outPortConsole, inPortConsole);

  if (len == 1 && (cmdbuffer[0] == 0x00 || cmdbuffer[0] == 0xFF)) {
    send(startresp, 3, bitMaskConsole, modePortConsole, outPortConsole);
  } else if (len == 1 && (cmdbuffer[0] == 0x41 || cmdbuffer[0] == 0x42)) {
    send(init_data, 10, bitMaskConsole, modePortConsole, outPortConsole);
  } else if (len == 3 && cmdbuffer[0] == 0x40) {
    //report_convert();
    report_convert();
    send(byteseq, 8, bitMaskConsole, modePortConsole, outPortConsole);
    send(getdata, 3, bitMaskContoller, modePortContoller, outPortContoller);
    read(byteseq, 8, bitMaskContoller, modePortContoller, outPortContoller, inPortContoller);
    //Serial.println("a");
  //} else {
  //  SREG = oldSREG;
  //  Serial.println("oi, mate. you F*cked up");
  //  Serial.println(len, DEC);
  //  Serial.println(cmdbuffer[0], HEX);
  //  Serial.println(cmdbuffer[1], HEX);
  //  Serial.println(cmdbuffer[2], HEX);
  }
  SREG = oldSREG;
  
}



void send(const uint8_t* buff, uint8_t len, uint8_t bitMask, volatile uint8_t* modePort, volatile uint8_t* outPort)
{
    // set pin to high
    *outPort |= bitMask;
    // set pin to output
    *modePort |= bitMask;

    // temporary register values used as "clobbers"
    register uint8_t bitCount;
    register uint8_t data;


    // TODO: need to make nicer and easyer to read 
    asm volatile (
        "; Start of assembly to send data to the pin\n"

        // passed in to this block are:
        // the %a[buff] register is the buffer pointer for the responce to the console
        // %[len] is the register holding the length of the buffer in bytes

        // Instruction cycles are noted in parentheses
        // branch instructions have two values, one if the branch isn't
        // taken and one if it is


        // Start sending the consoles return to the console. 

        // %[data] will be the current buffer byte loaded from memory
        // %[bitCount] will be the bit counter for the current byte. when this
        // reaches 0, we need to decrement the length counter, load
        // the next buffer byte, and loop. (if the length counter becomes
        // 0, that's the next byte condition)
        // %[len] if it reaches 0 then exit

        // This label starts sending the data to the console


        // sudocode
        // load byte into data
        // set bitcount to 8
        // start bitloop
        // set the line low
        // wait nops (12)
        // shift left the data
        // branch to data branch,
        // set the data then wait
        // decerement bit count
        // loop if nmot 0
        // decrment byte count
        // loop if not 0
        // send stop bit

        // This label starts the outer loop, which sends a single byte
        ".L%=_byte_loop:\n"
        "ld %[data], %a[buff]+\n" // (2) load the next byte and increment byte pointer
        "ldi %[bitCount],0x08\n" // (1) set bitcount to 8 bits
        // This label starts the inner loop, which sends a single bit
        ".L%=_bit_loop:\n"
        // we want low for 1us to start sending a bit
        "st %a[outPort],%[low]\n" // (2) pull the line low

        // line needs to stay low for 1us
        // 16 cycles, 2 from st above, 1 from lsl below and 2 from brcc, 
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\n" //(1) (11)

        "lsl %[data]\n" // (1) shift left. MSB goes into carry bit of status reg
        // brcc branches if the carry bit (output from previous line) is a 0
        "brcc .L%=_zero_bit\n" // (1/2) branch if carry is cleared
        "nop\n" // (1)

        "st %a[outPort],%[high]\n" // (2) set the line high again
        // Now stay high for 2us = 30 cycles
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (25)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "rjmp .L%=_finish_bit\n" // (2) jumps to end of the bit

        
        // this block is the timing for a 0 bit (3us low, 1us high)
        ".L%=_zero_bit:\n"
        // Need to go high in 2us, (32 cycles)
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (25)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\n" // (2) (32)

        
        // The two branches meet up here.
        ".L%=_finish_bit:\n"

        "st %a[outPort],%[high]\n" // (2) set the line high

        // logic about ending the bit
        "dec %[bitCount]\n" // (1) subtract 1 from our bit counter
        "breq .L%=_load_next_byte\n" // (1/2) branch if we've sent all the bits of this byte

        // 16 cycles for the high, - 2 for the st, -2 for the logic - 2 for the jump
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "rjmp .L%=_bit_loop\n" // (2)


        // we are 5 cycles into the high bit
        ".L%=_load_next_byte:\n"
        "dec %[len]\n" // (1) len--
        "breq .L%=_loop_exit\n" // (1/2) if the byte counter is 0, exit
        // needs to go high after 1us or 16 cycles
        // 16 - 7 (above) - 2 (the jump itself) - 3 (after jump) = 4
        "nop\nnop\nnop\nnop\n" //(4) 
        "rjmp .L%=_byte_loop\n" // (2)


        // Loop exit
        ".L%=_loop_exit:\n"

        // final task: send the stop bit, which is a 1 (1us low 3us high)
        // the line goes low in:
        // 16 - 8 (above since line went high) = 8 cycles
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\n" //(3) (8)

        "st %a[outPort],%[low]\n" // (2) pull the line low
        // stay low for 1us
        // 16 - 2 (below st) = 14
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\n" //(4) (14)

        "st %a[outPort],%[high]\n" // (2) set the line high again
        // just stay high. no need to wait 3us before returning


        // exit asm
        //".L%=_program_exit:\n"

        // ----------
        // outputs:
        : [buff] "+e" (buff), // (read and write)
        [outPort] "+e" (outPort), // (read and write)
        [bitCount] "=&d" (bitCount), // (output only, ldi needs the upper registers)
        [data] "=&r" (data) // (output only)

        // inputs:
        : [len] "r" (len),
        [high] "r" (*outPort | bitMask), // precalculate new pin states
        //[inPortConsole] "e" (inPortContoller), // where to read from
        [low] "r" (*outPort & ~bitMask) // this works because we turn interrupts off
        // no clobbers
        ); // end of asm volatile
}



uint8_t read(const uint8_t* buff, uint8_t len, uint8_t bitMask, volatile uint8_t* modePort, volatile uint8_t* outPort, volatile uint8_t* inPort)
{
    // set pin to input
    *modePort &= ~bitMask;
    // enable internel pullup resistor
    *outPort |= bitMask;

    // temporary register values used as "clobbers"
    register uint8_t bitCount;
    register uint8_t receivedBytes; // bytes found

    register uint8_t data; // temp for bytes recived

    register uint8_t waitloop1;

    register uint8_t lastbit; 
    register uint8_t inputVal; // temp to store the pinstates when reading. 


    asm volatile (
        "; Start of assembly to read input\n"

        // passed in to this block are:
        // the %a[buff] register is the buffer pointer for the responce to the console
        // %[len] is the register holding the length of the buffer in bytes

        //  buff - pointer to place to store data
        //  bitCount - bits left to read in current byte
        //  data - current byte
        //  waitloop1 - outer loop for waiting for comand
        //  waitloop2 - inner loop for waiting for comand
        //  inputVal - temp to store value of pin
        //  receivedBytes - number of bytes recived
        //
        // can merge recived bytes into the len
        // can merge the bitcount into one of the loops



        // ASM overview
        // start big loop, (255 loops max)
        // start smaller loop(255 loops max)
        // 
        //  LOOP INFORMATION
        // get status of the pin that we talk on
        // compare to bitmast
        // break to foundlow if a 0 from the and
        // decrement number of times that loop will operate
        // jump to start of loop if not 0
        // 
        // loop twice more
        // jump to start of big loop
        //
        // search for low loop
        // read pin and work out if it is low
        // have a preset max number of loops that will be done before error
        // 
        //
        // found low
        // preform a number of nops so we read data from the middle of the 2us of data being sent
        // depeding on what was found break
        //
        // if high 
        // shift left
        // then we set the bit in the data buffer to high 
        // check the status of the number of bits found
        // jump to the wait for low loop
        //
        // if low
        // shift left
        // check number of bits found
        // enter wait for high loop
        //
        // wait for high loop
        // read pin, break when high
        //
        // exit error
        // exit

        // set the outer loop to loop 256 times
        "ldi %[waitloop1],0x00\n" // (1)
        "ldi %[receivedBytes],0x00\n" // (1) yet to get a byte
        "ldi %[bitCount],0x08\n" // (1) set bitcount to 8 bits


        ".L%=_externel_loop:\n"
        // start the internal loop to loop 256 times
        "ldi %[data],0x00\n" // (1)

        ".L%=_internel_loop1:\n"
        "ld %[inputVal], %a[inPort]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMask]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop1\n" // (1/2) loop if the counter isn't 0


        // repeat all the code second time to increase instructions 
        "ldi %[inputVal],0x00\n" // (1)

        ".L%=_internel_loop2:\n"
        "ld %[inputVal], %a[inPort]\n" // (2) reads pin then saves to input value
        "and %[inputVal], %[bitMask]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop2\n" // (1/2) loop if the counter isn't 0

        // 3rd loop so we are sure that we wait long enough
        "ldi %[data],0x00\n" // (1)

        ".L%=_internel_loop3:\n"
        "ld %[inputVal], %a[inPort]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMask]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop3\n" // (1/2) loop if the counter isn't 0

        // checking here reduces the worst case senario that the input remains undetected for to 8 (from 10)
        "ld %[inputVal], %a[inPort]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMask]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)

        // decrement the externel loop
        "dec %[waitloop1]\n" // (1) decrease outer counter by 1
        "brne .L%=_externel_loop\n" // (1/2) loop if the counter isn't 0
        

        // loop has timed out since console has not queried for the port yet. 
        // outer loop loops 255 times and all 3 of the inner loops loop 255 times with 7 instructions for each loop
        // this totals 85.345ms
        "rjmp .L%=_program_exit\n" // (2) timeout, jump to the end


        // ** Reading the pin now**





        // checks the pin every 7 cycles
        ".L%=_wait_for_low:\n"
        "ldi %[waitloop1],0x10\n" // (1) max 16 loops so that will provide time to know that there is smth wrong
        ".L%=_wait_for_low_loop:\n" 
        "ld %[inputVal], %a[inPort]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMask]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[waitloop1]\n" // (1) 
        "brne .L%=_wait_for_low_loop\n" // (1/2) loop if the counter isn't 0
        // timeout if the loop did not find low
        "rjmp .L%=_program_exit\n" // (2) jump to end
        
        // logic for identifying the data 
        // worst case it has been 8 instructions since last pin check and it takes 5 instrctions to jump here
        // this means that between 3 instrctions and 11 instructions left in the start of this bit
        ".L%=_found_low:\n"
        // to check in the most optimal way we want to check in the middle of the 32 bits of data, 
        // this will be between the 12 and the 20th bit. 
        // if the 3 left goes to the 20 then we need 23 nops, meaning that the one with 11 left checks on the 12th cycle as e wanted. 
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\n" //(3) (23)

        // logic to check what the bit is. 
        ".L%=_read_bit:\n"
        "ld %[inputVal], %a[inPort]\n" // (2) reads pin then saves to input value (happens before the 2 cycles)

        "lsl %[data]\n" // (1) moves data recived left so the new data can fit in 
        "and %[inputVal], %[bitMask]\n" // (1) compares the input value to the bitmask
        "breq .L%=_decrement_bit\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "sbr %[data],0x01\n" // (1) sets the first bit in the data to be a 1
        ".L%=_decrement_bit:\n"
        "dec %[bitCount]\n" // (1) decrement 1 from our bit counter
        "brne .L%=_wait_for_high\n" // (1/2) branch if we've not received the full byte
        
        "inc %[receivedBytes]\n" // (1) increase byte count
        "ldi %[bitCount],0x08\n" // (1) set bitcount to 8 bits
        "st %a[buff]+,%[data]\n" // (2) save %[data] back to memory and increment byte pointer
        "cp %[len],%[receivedBytes]\n" // (1) %[len] == %[receivedBytes] ?
        "breq .L%=_stop_bit\n" // (1/2) jump to w
        
        ".L%=_wait_for_high:\n"
        "ldi %[waitloop1],%[timeout_length]\n" // (1) set the timeout to 16 loops, if it has not gone high then there is error
        ".L%=_wait_for_high_loop:\n" // 7 cycles if loop fails
        "ld %[inputVal], %a[inPort]\n" // (2) read the pin
        "and %[inputVal], %[bitMask]\n" // (1) compare pinstate with bitmask
        "brne .L%=_wait_for_low\n" // (1/2) if the line is high then jump
        "dec %[waitloop1]\n" // (1) decrease timeout by 1
        "brne .L%=_wait_for_high_loop\n" // (1/2) loop if the counter isn't 0

        ".L%=_stop_bit:\n"
        // can be removed since we dont talk imediatly after read

      //  // want to wait for the last bit, to do this we get here with at most 21 bits in this byte , 
      //  "nop\nnop\nnop\nnop\nnop\n" //(5)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
      //  "nop\n" //(3) (21)
      //  // now we want to wait the 16 cycles for the last low to end
      //  "nop\nnop\nnop\nnop\nnop\n" //(5)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
      //  "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
      //  "nop\n" //(3) (16)
      //  // should be high or turning high rn, 

        
        // we return the number of bits filled 
        ".L%=_program_exit:\n"

        // ----------
        // outputs:
        : [buff] "+e" (buff), // (read and write) // where to save the bytes to
        [bitCount] "=&d" (bitCount), // (output only, ldi needs the upper registers)
        [data] "=&d" (data), // (output only) // temp for storing bits recived when constructing bytes, ldi needs the upper registers)
        [waitloop1] "=&d" (waitloop1), // (output only, ldi needs the upper registers)
        [inputVal] "=&r" (inputVal), // temp storager for pin value 
        [receivedBytes] "=&d" (receivedBytes) // (ldi needs the upper registers)
        // if memory is a problem then we can remove input value and use the free registers when it is used 

        // inputs:
        : [len] "r" (len), // max bytes to get
        [inPort] "e" (inPort), // where to read from
        [bitMask] "r" (bitMask), // for identifying the port to check state
        [timeout_length] "M" (0x10)


        // no clobbers
        ); // end of asm volatile
        //input_data.origin = 1;
        return receivedBytes;
}

void report_convert() {
  if (cmdbuffer[1] == 0x03) {
  } else if (cmdbuffer[1] == 0x01) { 
    byteseq[4] = byteseq[4] >> 4 || (byteseq[5] && 0xF0); // need to check that we dont have horosontal and vertical swapped
    byteseq[5] = byteseq[6];
    byteseq[6] = byteseq[7];
    byteseq[7] = 0x00;
  } else if (cmdbuffer[1] == 0x02) { // needs testing
    byteseq[4] = byteseq[4] >> 4 || (byteseq[5] && 0xF0);
    byteseq[5] = byteseq[6] >> 4 || (byteseq[7] && 0xF0);
    byteseq[6] = 0x00;
    byteseq[7] = 0x00;
  } else if (cmdbuffer[1] = 0x04) { // want to have a look into this, seans like an odd format to use
    byteseq[6] = 0x00;
    byteseq[7] = 0x00;
  } else { // mode 0, 5, 6, 7
    byteseq[6] = byteseq[6] >> 4 || (byteseq[7] && 0xF0);
    byteseq[7] = 0x00;
  }
  return;
}


void wait0x(uint8_t wait_loops_outer, uint8_t wait_loops_inner) {
  register uint8_t outer; 
  register uint8_t inner; // temp to store the pinstates when reading. 
  asm volatile (
    "mov %[outer], %[wait_loops_outer]\n" // (1)
    ".L%=outer_loop:\n"
    "mov %[inner], %[wait_loops_inner]\n" // (1)
    ".L%=inner_loop:\n"
    "nop\nnop\nnop\nnop\nnop\n" // (5)
    "nop\nnop\nnop\nnop\nnop\n" // (5) (10)
    "nop\nnop\nnop\n" // (3) (13)
    "dec %[inner]\n" // (1) (14)
    "brne .L%=inner_loop\n"// (1/2) (16)
    "dec %[outer]\n" // (1)
    "brne .L%=inner_loop\n"// (1/2)
    : [outer] "=&d" (outer),
    [inner] "=&d" (inner)

    : [wait_loops_outer] "r" (wait_loops_outer),
    [wait_loops_inner] "r" (wait_loops_inner)
  );
}
