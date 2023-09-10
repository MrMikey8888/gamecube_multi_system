/*
Taking huge insperation from NicoHood's gamecube library, (used as a resorce for learning the asm and how the input is formatted)
ik i am rewriting alot, but since this is my first project using the arduino ide i wanted to only use code 
i understood and could modify in the future. 
i will try add support for other contollers in future maybe?

feel free to add stuff to this or whatever 
*/

#define CONTOLLER 4 // to change
//these 5 pins need to be on the same port, the data ones work best if on pin 1 - 4 but can be on others,k just edit the asm

typedef union {
    uint8_t byteseq[8];
    struct {
      // first data byte (bitfields are sorted in LSB order)
      uint8_t a : 1;
      uint8_t b : 1;
      uint8_t x : 1;
      uint8_t y : 1;
      uint8_t start : 1;
      uint8_t origin : 1; // Indicates if GetOrigin(0x41) was called (LOW)
      uint8_t errlatch : 1;
      uint8_t errstat : 1;

      // second data byte
      uint8_t dleft : 1;
      uint8_t dright : 1;
      uint8_t ddown : 1;
      uint8_t dup : 1;
      uint8_t z : 1;
      uint8_t r : 1;
      uint8_t l : 1;
      uint8_t high1 : 1;

      // 3rd-8th data byte
      uint8_t xAxis;
      uint8_t yAxis;
      uint8_t cxAxis;
      uint8_t cyAxis;
      uint8_t left;
      uint8_t right;
    }; 
} contoller_data;

// data to send
uint8_t getdata[3] = {0x40, 0x03, 0x00};
contoller_data input_data;

// data for the pin on the console
uint8_t bitMaskContoller;

volatile uint8_t* modePortContoller;
volatile uint8_t* outPortContoller;
volatile uint8_t* inPortContoller;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setupports();
  //Serial.print("test:\n");
  // saves the interupts
  uint8_t oldSREG = SREG;
  // clears interupts
  cli();




  //input_data.byteseq[0] = 0x00;
  uint8_t temp[1] = {0x00};
  send(temp, 0x01);
  uint8_t test[3];
  uint8_t status = read(test, 0x03);
  Serial.println("0x00");
  Serial.println(test[0], BIN);
  Serial.println(test[1], BIN);
  Serial.println(test[2], BIN);

  
  //uint8_t status = read(input_data.byteseq, 0x03);
  //Serial.println("0x00");
  //Serial.println(input_data.byteseq[0], BIN);
  //Serial.println(input_data.byteseq[1], BIN);
  //Serial.println(input_data.byteseq[2], BIN);

  temp[0] = 0x00;
  send(temp, 0x01);
  status = read(input_data.byteseq, 0x03);
  Serial.println("0x00att2");
  Serial.println(input_data.byteseq[0], BIN);
  Serial.println(input_data.byteseq[1], BIN);
  Serial.println(input_data.byteseq[2], BIN);

  temp[0] = 0x41;
  uint8_t origin_a[10];
  send(temp, 0x01);
  status = read(origin_a, 0x0A);
  Serial.println("0x41");
  Serial.println(origin_a[0], BIN);
  Serial.println(origin_a[1], BIN);
  Serial.println(origin_a[2], BIN);
  Serial.println(origin_a[3], BIN);
  Serial.println(origin_a[4], BIN);
  Serial.println(origin_a[5], BIN);
  Serial.println(origin_a[6], BIN);
  Serial.println(origin_a[7], BIN);
  Serial.println(origin_a[8], BIN);
  Serial.println(origin_a[9], BIN);

  //uint8_t status = send_get(input_data.byteseq, 0x01, 0x03);
  //while(status != 3) {
  //  cli();
  //  send(0x00, 0x01);
  //Serial.println(status, DEC);
  //input_data.origin = 1;
  //input_data.byteseq[0] = getdata[0];
  //input_data.byteseq[1] = getdata[1];
  //input_data.byteseq[2] = getdata[2];
  //  uint8_t status = read(input_data.byteseq, 0x03);
  //  SREG = oldSREG;
  //  delay(10);
  //}
  
  //while (read(input_data.byteseq, 0x08)!=8); for the other program
  //this program can deal with any data as long as it is less then 8 bytes long, the other side can deal with the issue when connecting it to the cube. 
  
  
  // dont stream data here 
  //stream_data(input_data.byteseq, 0x08);
  SREG = oldSREG;
  //Serial.print("len:");
  //Serial.println(status, DEC);
  //Serial.println(input_data.byteseq[0], BIN);
  //Serial.println(input_data.byteseq[1], BIN);
  //Serial.println(input_data.byteseq[2], BIN);
  //Serial.println(input_data.byteseq[3], BIN);
  //Serial.println(input_data.byteseq[4], BIN);
  //Serial.println(input_data.byteseq[5], BIN);
  //Serial.println(input_data.byteseq[6], BIN);
  //Serial.println(input_data.byteseq[7], BIN);
}


void loop() {
  // put your main code here, to run repeatedly:

  // saves the interupts
  uint8_t oldSREG = SREG;
  // clears interupts
  cli();
  //input_data.byteseq[0] = getdata[0];
  //input_data.byteseq[1] = getdata[1];
  //input_data.byteseq[2] = getdata[2];
  //uint8_t status = send_get(input_data.byteseq, 0x03, 0x08);
  
  //uint8_t temp[1] = {0x69};
  //send(temp, 0x01);
  

  //uint8_t status = gc_n64_get(input_data.byteseq, 0x08, modePortContoller, outPortContoller, inPortContoller, bitMaskContoller);
  // lets start by getting the console status
  //contoller_macro();
  send(getdata, 0x03);
  uint8_t status = read(input_data.byteseq, 0x08);
  //Serial.println("usual data");
  
  //Serial.println(input_data.byteseq[0], BIN);
  //Serial.println(input_data.byteseq[1], BIN);
  //Serial.println(input_data.byteseq[2], BIN);
  //Serial.println(input_data.byteseq[3], BIN);
  //Serial.println(input_data.byteseq[4], BIN);
  //Serial.println(input_data.byteseq[5], BIN);
  //Serial.println(input_data.byteseq[6], BIN);
  //Serial.println(input_data.byteseq[7], BIN);

  if (status != 8) {
    Serial.println(input_data.byteseq[0], BIN);
  }

  SREG = oldSREG;
  //Serial.println(input_data.byteseq[0], BIN);
  //wait0x(0x02, 0x02);

  //delay(1);
  //Serial.print("A:");
  //Serial.println(status, DEC);
  //Serial.println(input_data.byteseq[0], BIN);
  //Serial.println(input_data.byteseq[1], BIN);
  //Serial.println(input_data.byteseq[2], BIN);
  //Serial.println(input_data.byteseq[3], BIN);
  //Serial.println(input_data.byteseq[4], BIN);
  //Serial.println(input_data.byteseq[5], BIN);
  //Serial.println(input_data.byteseq[6], BIN);
  //Serial.println(input_data.byteseq[7], BIN);

}

void setupports() {
  // data for the information status pin
  bitMaskContoller = digitalPinToBitMask(CONTOLLER);
  uint8_t portContoller = digitalPinToPort(CONTOLLER);

  modePortContoller = portModeRegister(portContoller); // need to swap from in and out
  outPortContoller = portOutputRegister(portContoller); // need to set the data to send
  inPortContoller = portInputRegister(portContoller); // need to 
}

void contoller_macro() {
  // blank for now, will be added in future

  // want one to reset game
}


void send(const uint8_t* buff, uint8_t len)
{
    // set pin to high
    *outPortContoller |= bitMaskContoller;
    // set pin to output
    *modePortContoller |= bitMaskContoller;

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
        [outPort] "+e" (outPortContoller), // (read and write)
        [bitCount] "=&d" (bitCount), // (output only, ldi needs the upper registers)
        [data] "=&r" (data) // (output only)

        // inputs:
        : [len] "r" (len),
        [high] "r" (*outPortContoller | bitMaskContoller), // precalculate new pin states
        //[inPortConsole] "e" (inPortContoller), // where to read from
        [low] "r" (*outPortContoller & ~bitMaskContoller) // this works because we turn interrupts off
        // no clobbers
        ); // end of asm volatile
}

uint8_t read(const uint8_t* buff, uint8_t len)
{
    // set pin to input
    *modePortContoller &= ~bitMaskContoller;
    // enable internel pullup resistor
    *outPortContoller |= bitMaskContoller;

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
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop1\n" // (1/2) loop if the counter isn't 0


        // repeat all the code second time to increase instructions 
        "ldi %[inputVal],0x00\n" // (1)

        ".L%=_internel_loop2:\n"
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop2\n" // (1/2) loop if the counter isn't 0

        // 3rd loop so we are sure that we wait long enough
        "ldi %[data],0x00\n" // (1)

        ".L%=_internel_loop3:\n"
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[data]\n" // (1) decrease loop iterations by 1
        "brne .L%=_internel_loop3\n" // (1/2) loop if the counter isn't 0

        // checking here reduces the worst case senario that the input remains undetected for to 8 (from 10)
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
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
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
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
        "ld %[inputVal], %a[inPortContoller]\n" // (2) reads pin then saves to input value (happens before the 2 cycles)

        "lsl %[data]\n" // (1) moves data recived left so the new data can fit in 
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compares the input value to the bitmask
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
        "ld %[inputVal], %a[inPortContoller]\n" // (2) read the pin
        "and %[inputVal], %[bitMaskContoller]\n" // (1) compare pinstate with bitmask
        "brne .L%=_wait_for_low\n" // (1/2) if the line is high then jump
        "dec %[waitloop1]\n" // (1) decrease timeout by 1
        "brne .L%=_wait_for_high_loop\n" // (1/2) loop if the counter isn't 0

        ".L%=_stop_bit:\n"
        // can be removed since we dont talk imediatly after read

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
        ".L%=_program_exit:\n"
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (100)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (200)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (300)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (400)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (500)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (100)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (200)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (300)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (400)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (1000)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (100)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (200)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (300)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (70)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (80)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (90)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (400)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (20)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (30)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (40)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (50)
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (60)
        //1460 nops
        // we return the number of bits filled 
        

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
        [inPortContoller] "e" (inPortContoller), // where to read from
        [bitMaskContoller] "r" (bitMaskContoller), // for identifying the port to check state
        [timeout_length] "M" (0x10)


        // no clobbers
        ); // end of asm volatile
        //input_data.origin = 1;
        return receivedBytes;
}

void wait10() {
  asm volatile (
    "; we disabel interupts so this a a delay of 10us\n"
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 16 nops or 1 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 32 nops or 2 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 3 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 4 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 5 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 6 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 7 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 8 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 9 us now

    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    "nop\nnop\nnop\nnop\n" //(4)
    // we have 10 us now
  );
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
