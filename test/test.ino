#define CONSOLE 4 // to change
uint8_t bitMaskConsole;

volatile uint8_t* modePortConsole;
volatile uint8_t* outPortConsole;
volatile uint8_t* inPortConsole;

uint8_t command[8];
void setup() {
  Serial.begin(9600);
  pinMode(CONSOLE, INPUT);


  bitMaskConsole= digitalPinToBitMask(CONSOLE);
  uint8_t portConsole = digitalPinToPort(CONSOLE);

  modePortConsole = portModeRegister(portConsole); // need to swap from in and out
  outPortConsole = portOutputRegister(portConsole); // need to set the data to send
  inPortConsole = portInputRegister(portConsole); // need to 

}

void loop() {
  cli();
  //if (digitalRead(CONSOLE) == 0) {
  //  Serial.println("hi");
  //} else {
  //  Serial.println("2");
  //}
  uint8_t datalen = read(command, 0x03);
  Serial.println(":");
  Serial.println(datalen, HEX);
  Serial.println(command[0], HEX);
  //Serial.println(command[1], HEX);
  //Serial.println(command[2], HEX);
  //command[0] = 0xFF;
  //command[1] = 0xFF;
  //command[2] = 0xFF;
}


uint8_t read(const uint8_t* buff, uint8_t len)
{
    // set pin to input
    *modePortConsole &= ~bitMaskConsole;
    // enable internel pullup resistor
    *outPortConsole &= ~bitMaskConsole;

    // temporary register values used as "clobbers"
    register uint8_t bitCount;
    register uint8_t receivedBytes; // bytes found

    register uint8_t data; // temp for bytes recived

    register uint8_t waitloop1;
 
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
        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop1\n" // (1/2) loop if the counter isn't 0
        //"rjmp .L%=_internel_loop1\n"


        // repeat all the code second time to increase instructions 
        "ldi %[data],0x00\n" // (1)

        ".L%=_internel_loop2:\n"
        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value
        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop2\n" // (1/2) loop if the counter isn't 0

        // 3rd loop so we are sure that we wait long enough
        "ldi %[data],0x00\n" // (1)

        ".L%=_internel_loop3:\n"
        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop3\n" // (1/2) loop if the counter isn't 0

        // checking here reduces the worst case senario that the input remains undetected for to 8 (from 10)
        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)

        // decrement the externel loop
        "dec %[waitloop1]\n" // (1) decrease outer counter by 1
        "brne .L%=_externel_loop\n" // (1/2) loop if the counter isn't 0
        

        // loop has timed out since console has not queried for the port yet. 
        // outer loop loops 255 times and all 3 of the inner loops loop 255 times with 7 instructions for each loop
        // this totals 85.345ms
        "ldi %[receivedBytes],0x30\n"
        "rjmp .L%=_program_exit\n" // (2) timeout, jump to the end


        // ** Reading the pin now**





        // checks the pin every 7 cycles
        ".L%=_wait_for_low:\n"
        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[waitloop1]\n" // (1) 
        "brne .L%=_wait_for_low\n" // (1/2) loop if the counter isn't 0
        // timeout if the loop did not find low
        "ori %[receivedBytes],0x40\n"
        //"or %[receivedBytes], %[bitCount]\n"
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
        "nop\n" //(1) (21)
        "lsl %[data]\n" // (1) moves data recived left so the new data can fit in 
        "ldi %[waitloop1],%[timeout_length]\n" // (1) (23) set the timeout to 16 loops, if it has not gone high then there is error, do it in advance

        // logic to check what the bit is. 
        ".L%=_read_bit:\n"
        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value (happens before the 2 cycles)
        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
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
        "ld %[inputVal], %a[inPortConsole]\n" // (2) read the pin
        "and %[inputVal], %[bitMaskConsole]\n" // (1) compare pinstate with bitmask
        "brne .L%=_wait_for_low\n" // (1/2) if the line is high then jump
        "dec %[waitloop1]\n" // (1) decrease timeout by 1
        "brne .L%=_wait_for_high\n" // (1/2) loop if the counter isn't 0
        "ori %[receivedBytes],0x50\n"
        ".L%=_stop_bit:\n"
        // want to wait for the last bit, to do this we get here with at most 21 bits in this byte , 
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\n" //(3) (21)
        // now we want to wait the 16 cycles for the last low to end
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (15)
        "nop\n" //(3) (16)
        // should be high or turning high rn, 
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
        [inPortConsole] "e" (inPortConsole), // where to read from
        [bitMaskConsole] "r" (bitMaskConsole), // for identifying the port to check state
        [timeout_length] "M" (0x10) // if we loop 64 times waiting between data then we exit, max should be 32 cycles, this exits when we reach 7x that



        // no clobbers
        ); // end of asm volatile

        return receivedBytes;
}