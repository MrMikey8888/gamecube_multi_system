/*
Taking huge insperation from NicoHood's gamecube library, (used as a resorce for learning the asm and how the input is formatted)
ik i am rewriting alot, but since this is my forst project using the arduino ide i wanted to only use code 
i understood and could modify in the future. 
i will try add support for other contollers in future maybe?  

feel free to add stuff to this or whatever

dm me on disc @mikey8888
*/

// functions
//void setsupports();
//void send(const uint8_t* buff, uint8_t len);
//uint8_t read(const uint8_t* buff, uint8_t len);
//uint8_t getmaininput(const uint8_t* buff, uint8_t len);
//void report_convert();
//void set_origin_status();


// ports to use
#define CONSOLE 4 // to change
//these 5 pins need to be on the same port, the data ones work best if on pin 1 - 4 but can be on others,k just edit the asm
#define PORTCONDITION 12 // status of the data is in portdata TODO: work out a better pin, if memory is a problem put in same buss
#define PORTDATA1 8 // data being transfered
#define PORTDATA2 9 // data being transfered
#define PORTDATA3 10 // data being transfered
#define PORTDATA4 11 // data being transfered

// so we can access the data in either a structured way or unstructured way
//typedef union{
//    uint8_t byteseq[8];
//    struct {
//      // first data byte (bitfields are sorted in LSB order)
//      uint8_t A : 1;
//      uint8_t B : 1;
//      uint8_t X : 1;
//      uint8_t Y : 1;
//      uint8_t start : 1;
//      uint8_t origin : 1; // Indicates if GetOrigin(0x41) was called (LOW)
//      uint8_t errlatch : 1; // unsure
//      uint8_t errstat : 1;  // unsure
//
//      // second data byte
//      uint8_t dleft : 1;
//      uint8_t dright : 1;
//      uint8_t ddown : 1;
//      uint8_t dup : 1;
//      uint8_t Z : 1;
//      uint8_t R : 1;
//      uint8_t L : 1;
//      uint8_t high1 : 1; // always high
//
//      // 3rd-8th data byte, these may be changed depending on the data format that the cube wants
//      uint8_t xAxis;  
//      uint8_t yAxis;
//      uint8_t cxAxis;
//      uint8_t cyAxis;
//      uint8_t left;
//      uint8_t right;
//    }; 
//} contoller_data;
uint8_t precmd[3];
// data read from console
uint8_t command[3];
// data for the initalisation
uint8_t initaldata[10]; 
// data from contoler
uint8_t contoller_data[8];
// idk correct input
uint8_t contoller_init[3];
//bool origin_called = 1; 

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

//uint8_t send[] = {0x40, 0x03, 0x02}

void setup() {
  // put your setup code here, to run once

  setsupports();


  Serial.begin(9600);
  // saves the interupts
  uint8_t oldSREG = SREG;
  // clears interupts
  cli();

  //while(true) {
    //getmaininput(initaldata, 0x08);
    //Serial.println("a");
  //}
  //while (true) {
  //  Serial.println(getmaininput(initaldata, 0x08), DEC);
  //}
  contoller_init[0] = 0x09;
  contoller_init[1] = 0x00;
  contoller_init[2] = 0x03;

  //getmaininput(initaldata, 0x08);
  while (getmaininput(initaldata, 0x08) != 8);
  initaldata[8] = 0x02;
  initaldata[9] = 0x02;
  //initaldata[0] &= 0xDF;

  //contoller_data[0] = 0x00;
  //contoller_data[1] = 0x00;
  //contoller_data[2] = 0x00;
  //contoller_data[3] = 0x00;
  //contoller_data[4] = 0x00;
  //contoller_data[5] = 0x00;
  //contoller_data[6] = 0x00;
  //contoller_data[7] = 0x00;

  //getmaininput(contoller_data, 0x08);
  while (getmaininput(contoller_data, 0x08) != 8);
  SREG = oldSREG;
  Serial.println("init_resp");
  Serial.println(initaldata[0], BIN);
  Serial.println(initaldata[1], BIN);
  Serial.println(initaldata[2], BIN);
  Serial.println(initaldata[3], BIN);
  Serial.println(initaldata[4], BIN);
  Serial.println(initaldata[5], BIN);
  Serial.println(initaldata[6], BIN);
  Serial.println(initaldata[7], BIN);
  Serial.println(initaldata[8], BIN);
  Serial.println(initaldata[9], BIN);
}

//void loop() {
//  // saves the interupts
//  //uint8_t oldSREG = SREG;
//  // clears interupts
//  //cli();
//  
//  //while(getmaininput(button_stats.byteseq, 0x08)!= 8); //{
//    //Serial.println("0x");
//  //}
//  
//  uint8_t datalen = read(command, 0x03);
//  //while (datalen < 1) {
//  //  datalen = read(command, 0x03);
//  //  Serial.println(command[0], HEX);
//  //}
//  //Serial.print("0x");
//  //Serial.println(command[0], HEX);
//  //Serial.println("0x");
//  //Serial.println(command[1], HEX);
//  //Serial.println("0x");
//  //Serial.println(command[2], HEX);
//  if (datalen == 0x01 && (command[0] == 0x00 ||command[0] ==  0xFF)) {
//    send(contoller_init, 0x03);
//    //origin_called = 0;
//    //datalen = read(command, 0x03);
//    //Serial.println(":");
//    //Serial.println(datalen, DEC);
//    //Serial.println(":");
//    //Serial.println(datalen, HEX);
//    //Serial.println(command[0], HEX);
//    //Serial.println(command[1], HEX);
//    //Serial.println(command[2], HEX);
//
//  } else if (datalen == 1 && (command[0] == 0x41 ||command[0] ==  0x42)) {
//    send(initaldata, 0x0A);
//    //origin_called == 1;
//    //Serial.println("b");
//  //} else {
//  //  Serial.println(":");
//  //  Serial.println(datalen);
//  //  Serial.println(command[0], HEX);
//  } else if (datalen == 3 ) {
//    //report_convert();
//    send(button_stats.byteseq, 0x08);
//    getmaininput(button_stats.byteseq, 0x08);
//  } else {
//    Serial.println("a");
//  }
//  //SREG = oldSREG;
//  //Serial.println("start");
//  //Serial.println(datalen);
//  //Serial.println(command[0]);
//  //Serial.println(button_stats.byteseq[0], BIN);
//  //Serial.println(button_stats.byteseq[1], BIN);
//  //Serial.println(button_stats.byteseq[2], BIN);
//  //Serial.println(button_stats.byteseq[3], BIN);
//  //Serial.println(button_stats.byteseq[4], BIN);
//  //Serial.println(button_stats.byteseq[5], BIN);
//  //Serial.println(button_stats.byteseq[6], BIN);
//  //Serial.println(button_stats.byteseq[7], BIN);
//  //command[0] = 0x00;
//  //command[1] = 0x00;
//  //command[2] = 0x00;
//}
//
void loop() {
  uint8_t oldSREG = SREG;
  cli();
  uint8_t len = read(command, 0x03);
  if (len == 1 && (command[0] == 0x00 || command[0] == 0xFF)) {
    //uint8_t len = 0x03;
    send(contoller_init, 0x03);
    //Serial.println(contoller_init[0], HEX);
    //Serial.println(contoller_init[1], HEX);
    //Serial.println(contoller_init[2], HEX);
    //if (contoller_data[1] != 0) {
    //  SREG = oldSREG;
    //  contoller_data[1] &= 0x00;
    //}
  } else if (len == 1 && (command[0] == 0x41 || command[0] == 0x42)) {
    send(initaldata, 0x0A);
    //SREG = oldSREG;
    //Serial.println(precmd[0], HEX);
  } else if (len == 3 && command[0] == 0x40) {
    //report_convert();
    report_convert();
    send_get_new(contoller_data, 8);
    //send(contoller_data, 8);
    //getmaininput(contoller_data, 0x08);
    //Serial.println(contoller_data[0], BIN);
    //send(contoller_data, 8);
    //getmaininput(contoller_data, 0x08);
    //send(getdata, 3, bitMaskContoller, modePortContoller, outPortContoller);
    //read(byteseq, 8, bitMaskContoller, modePortContoller, outPortContoller, inPortContoller);
    //SREG = oldSREG;
    //Serial.println(contoller_data[0], BIN);
  //} else {
  //  SREG = oldSREG;
  //  Serial.println("sad");
  //  Serial.println(len, DEC);
  //  Serial.println(command[0], HEX);
  //  Serial.println(command[1], HEX);
  //  Serial.println(command[2], HEX);
  //} else if (len == 0) {
    //SREG = oldSREG;
    //Serial.println("hmm");
  //  Serial.println(len, DEC);
  //  Serial.println(command[1], HEX);
  }
  //SREG = oldSREG;
  //precmd[0] = command[0];
  //precmd[1] = command[1];
  //precmd[2] = command[2];
}

void setsupports() {
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

// make logic to track the data from the gamecube and then send stored data to the console
void send(const uint8_t* buff, uint8_t len)
{
    // set pin to high
    *outPortConsole |= bitMaskConsole;
    // set pin to output
    *modePortConsole |= bitMaskConsole;

    // temporary register values used as "clobbers"
    register uint8_t bitCount;
    register uint8_t data;
    register uint8_t bytecount;

    // TODO: need to make nicer and easyer to read 
    asm volatile (
        "; Start of assembly to read gamecube input then respomd\n"

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
        "mov %[bytecount], %[len]\n"

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
        "dec %[bytecount]\n" // (1) len--
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
        [outPort] "+e" (outPortConsole), // (read and write)
        [bitCount] "=&d" (bitCount), // (output only, ldi needs the upper registers)
        [data] "=&d" (data), // (output only)
        [bytecount] "=&d" (bytecount) // (output only) (i think we need upper registers for mov)

        // inputs:
        : [len] "r" (len),
        [high] "r" (*outPortConsole | bitMaskConsole), // precalculate new pin states
        [low] "r" (*outPortConsole & ~bitMaskConsole) // this works because we turn interrupts off
        // no clobbers
        ); // end of asm volatile
}


uint8_t read(const uint8_t* buff, uint8_t len)
{
    // set pin to input
    *modePortConsole &= ~bitMaskConsole;
    // enable internel pullup resistor
    *outPortConsole |= bitMaskConsole;

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
        //"ldi %[data],0x00\n" // (1)

        ".L%=_internel_loop1:\n"
        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        //"dec %[data]\n" // (1) decrease loop iterations by 1
        //"brne .L%=_internel_loop1\n" // (1/2) loop if the counter isn't 0
        "rjmp .L%=_internel_loop1\n"


        // repeat all the code second time to increase instructions 
//        "ldi %[data],0x00\n" // (1)
//    
//        ".L%=_internel_loop2:\n"
//        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value
//        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
//        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
//        "dec %[data]\n" // (1) decrease loop iterations by 1
//        "brne .L%=_internel_loop2\n" // (1/2) loop if the counter isn't 0
//
//        // 3rd loop so we are sure that we wait long enough
//        "ldi %[data],0x00\n" // (1)
//
//        ".L%=_internel_loop3:\n"
//        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value 
//        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
//        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
//        "dec %[data]\n" // (1) decrease loop iterations by 1
//        "brne .L%=_internel_loop3\n" // (1/2) loop if the counter isn't 0
//
//        // checking here reduces the worst case senario that the input remains undetected for to 8 (from 10)
//        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value 
//        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
//        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
//
//        // decrement the externel loop
//        "dec %[waitloop1]\n" // (1) decrease outer counter by 1
//        "brne .L%=_externel_loop\n" // (1/2) loop if the counter isn't 0
//        
//
//        // loop has timed out since console has not queried for the port yet. 
//        // outer loop loops 255 times and all 3 of the inner loops loop 255 times with 7 instructions for each loop
//        // this totals 85.345ms
//        //"ldi %[receivedBytes],0x30\n"
//        "rjmp .L%=_program_exit\n" // (2) timeout, jump to the end


        // ** Reading the pin now**





        // checks the pin every 7 cycles
        ".L%=_wait_for_low:\n"
        "ld %[inputVal], %a[inPortConsole]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskConsole]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        "dec %[waitloop1]\n" // (1) 
        "brne .L%=_wait_for_low\n" // (1/2) loop if the counter isn't 0
        // timeout if the loop did not find low
        //"ldi %[receivedBytes],0x40\n"
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
        //"ldi %[receivedBytes],0x50\n"
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



        // we should be able to read in time but since i have problems lets add a timeout
        "ldi %[timeout],0x00\n" // (1)

        ".L%=_wait_low_loop:\n"
        "ld %[inputVal], %a[inPortComm]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskStatus]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        //"dec %[timeout]\n"
        "inc %[timeout]\n"
        "rjmp .L%=_wait_low_loop\n" // (2) loop
        //"brne .L%=_wait_low_loop\n" // (1/2) jumps if not 0
        //"rjmp .L%=_program_exit\n"


        // set the timeout to be about 10 cycles (timeout_length)
        ".L%=_found_low:\n"
        //"ldi %[data_temp],0x01\n" // (1)
        //".L%=_wait_high_loop_outer:\n"
        //"ldi %[timeout],0x00\n" // (1)
        ".L%=_wait_high_loop:\n" 
        "ld %[inputVal], %a[inPortComm]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskStatus]\n" // (1) compares the input value to the bitmask
        "brne .L%=_start_reading\n" // (1/2) jumps if the value is 1 from the and (the result was high)
        //"dec %[timeout]\n" // (1) decrements timeout 
        //"brne .L%=_wait_high_loop\n" // (1/2) jumps if not 0
        //"ld %[inputVal], %a[inPortComm]\n" // (2) reads pin then saves to input value 
        //"and %[inputVal], %[bitMaskStatus]\n" // (1) compares the input value to the bitmask
        //"brne .L%=_start_reading\n" // (1/2) jumps if the value is 1 from the and (the result was high)
        //"dec %[timeout]\n" // (1) decrements timeout 
        //"brne .L%=_wait_high_loop_outer\n" // (1/2) jumps if not 0
        //"rjmp .L%=_program_exit\n" // (2) loop 
        "inc %[timeout]\n"
        "rjmp .L%=_wait_high_loop\n" // (2) loop 



        // **Logic to read the data*


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
        // hence are 5 to 12 instructions into the inital bits of the data, this is 16 cycles long
        // we want to check etween the 5th and 12th cycle so we wait 7 cycles
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
        [timeout_length] "M" (0x10) // if we loop 64 times waiting between data then we exit, max should be 32 cycles, this exits when we reach 7x that

        // no clobbers
        ); // end of asm volatile
        
        //set_origin_status();


        return (len - bytecount);
}

void report_convert() {
  if (command[1] == 0x03) {
  } else if (command[1] == 0x01) { 
    contoller_data[4] = contoller_data[4] >> 4 || (contoller_data[5] && 0xF0); // need to check that we dont have horosontal and vertical swapped
    contoller_data[5] = contoller_data[6];
    contoller_data[6] = contoller_data[7];
    contoller_data[7] = 0x00;
  } else if (command[1] == 0x02) { // needs testing
    contoller_data[4] = contoller_data[4] >> 4 || (contoller_data[5] && 0xF0);
    contoller_data[5] = contoller_data[6] >> 4 || (contoller_data[7] && 0xF0);
    contoller_data[6] = 0x00;
    contoller_data[7] = 0x00;
  } else if (command[1] = 0x04) { // want to have a look into this, seans like an odd format to use
    contoller_data[6] = 0x00;
    contoller_data[7] = 0x00;
  } else { // mode 0, 5, 6, 7
    contoller_data[6] = contoller_data[6] >> 4 || (contoller_data[7] && 0xF0);
    contoller_data[7] = 0x00;
  }
  return;
}

//void set_origin_status() {
//  if (origin_called) {
//    button_stats.origin = 1;
//  } else {
//      //button_stats.origin = 0;
//  }
//}


void send_get_new(const uint8_t* buff, uint8_t len) {

    register uint8_t data_temp; // temp for bytes recived
    register uint8_t bytecount;
    //register uint8_t waitloop2;
    register uint8_t inputVal; // temp to store the pinstates when reading. 
    register uint8_t timeout; 
    register uint8_t z_upper_byte;


    // set pin to high
    *outPortConsole |= bitMaskConsole;
    // set pin to output
    *modePortConsole |= bitMaskConsole;


    uintptr_t pointerValue = reinterpret_cast<uintptr_t>(inPortComm);

    register uint8_t low_byte = static_cast<uint8_t>(pointerValue);
    register uint8_t high_byte = static_cast<uint8_t>(pointerValue >> 8);

    //register uint8_t debug;
    asm volatile (
        "mov %[bytecount], %[len]\n"

        // we need to make a record of the mempry location of the z register (buff) so we can load it later
        "mov %[timeout], R30\n" // (1) we dont need timeout until later
        "mov %[z_upper_byte], R31\n"

        
        // This label starts the outer loop, which sends a single byte
        ".L%=_byte_loop:\n"
        "ld %[data_temp], %a[buff]+\n" // (2) load the next byte and increment byte pointer
        "ldi %[inputVal],0x08\n" // (1) set bitcount to 8 bits
        // This label starts the inner loop, which sends a single bit
        ".L%=_bit_loop:\n"
        // we want low for 1us to start sending a bit
        "st %a[port_pointer],%[low]\n" // (2) pull the line low

        // line needs to stay low for 1us
        // 16 cycles, 2 from st above, 1 from lsl below and 2 from brcc, 
        "nop\nnop\nnop\nnop\nnop\n" //(5)
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "nop\n" //(1) (11)

        "lsl %[data_temp]\n" // (1) shift left. MSB goes into carry bit of status reg
        // brcc branches if the carry bit (output from previous line) is a 0
        "brcc .L%=_zero_bit\n" // (1/2) branch if carry is cleared
        "nop\n" // (1)

        "st %a[port_pointer],%[high]\n" // (2) set the line high again
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

        "st %a[port_pointer],%[high]\n" // (2) set the line high

        // logic about ending the bit
        "dec %[inputVal]\n" // (1) subtract 1 from our bit counter
        "breq .L%=_load_next_byte\n" // (1/2) branch if we've sent all the bits of this byte

        // 16 cycles for the high, - 2 for the st, -2 for the logic - 2 for the jump
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\nnop\n" //(5) (10)
        "rjmp .L%=_bit_loop\n" // (2)


        // we are 5 cycles into the high bit
        ".L%=_load_next_byte:\n"
        "dec %[bytecount]\n" // (1) len--
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

        "st %a[port_pointer],%[low]\n" // (2) pull the line low
        // stay low for 1us
        // 16 - 2 (below st) = 14
        "nop\nnop\nnop\nnop\nnop\n" //(5) 
        "nop\nnop\nnop\nnop\n" //(4) (9)

        // lets use this time to setup some stuff for the next bit
        // to start we want to return the buffer to the start
        "mov R30, %[timeout]\n" // (1) we set the buffer back to start
        "mov R31, %[z_upper_byte]\n" // (1) we set the buffer back to the start
        // want to record the adress of the port we overide :)
        // lets use the same registers as a temp storage
        "mov %[timeout], R26\n" // (1) temp storage until after we set the pointer
        "mov %[z_upper_byte], R27\n" // (1) temp storage until we set the pointer

        "mov %[bytecount], %[len]\n" // (1) sets the number of bytes we expect to the len

        "st %a[port_pointer],%[high]\n" // (2) set the line high again
        // just stay high. no need to wait 3us before returning
        

        // ** Time to start the loop for reading data**

        // first lets change the x register to the new one 
        "mov R26, %[low_byte]\n" // (1) low byte of the x register
        "mov R27, %[high_byte]\n" // (1) high byte of the x register 
        "mov %[low_byte], %[timeout]\n" // (1) temp storage for the pointer we used 
        "mov %[high_byte], %[z_upper_byte]\n" // (1) temp storage for the pointer we used 


        // we should be able to read in time but since i have problems lets add a timeout, 
        // according to the how i have written this funciton the line should only be high when other data is being sent, 
        // the high loop will only run for about 48 us at most so if we were to add a timeout then eiher it will be useless 
        // be triggered on edge cases, we set it at 100 cycles, this is more then what it should take so we exit if smth is wrong
        "ldi %[timeout],0x64\n" // (1)
        //"ldi %[debug], 0x00\n"
        ".L%=_wait_low_loop:\n"
        "ld %[inputVal], %a[port_pointer]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskStatus]\n" // (1) compares the input value to the bitmask
        "breq .L%=_found_low\n" // (1/2) jumps if the value is 0 from the and (the result was low)
        //"inc %[debug]\n" // (1)
        "dec %[timeout]\n"
        "brne .L%=_wait_low_loop\n" // (1/2) jumps if not 0
        "rjmp .L%=_program_exit\n"
        //"inc %[debug]\n" // (1)
        //"rjmp .L%=_wait_low_loop\n" // (2)


        // set the timeout to be about 10 cycles (timeout_length)
        ".L%=_found_low:\n"
        "ldi %[timeout],%[timeout_length]\n" // (1)
        ".L%=_wait_high_loop_outer:\n"
        //"ldi %[timeout],0x00\n" // (1)
        ".L%=_wait_high_loop:\n" 
        "ld %[inputVal], %a[port_pointer]\n" // (2) reads pin then saves to input value 
        "and %[inputVal], %[bitMaskStatus]\n" // (1) compares the input value to the bitmask
        "brne .L%=_start_reading\n" // (1/2) jumps if the value is 1 from the and (the result was high)
        //"inc %[debug]\n" // (1)
        "dec %[timeout]\n" // (1) decrements timeout 
        "brne .L%=_wait_high_loop_outer\n" // (1/2) jumps if not 0
        "rjmp .L%=_program_exit\n" // (2)  
        //"rjmp .L%=_wait_high_loop_outer\n" // (2) loop 



        // **Logic to read the data*


        ".L%=_wait_prebits:\n"
        "ldi %[timeout],%[timeout_length]\n" /// (1)
        // logic to wait for line to go low
        ".L%=_wait_high_prebit:\n"
        "ld %[inputVal], %a[port_pointer]\n" // (2) reads pin then saves to input value 
        "andi %[inputVal],  0x01\n" // (1) compares the input value to the bitmask
        "brne .L%=_wait_low_prebit\n" // (1/2) jumps if the value is not 0 from the and (the result was high)
        "dec %[timeout]\n" // (1) decrements timeout 
        "brne .L%=_wait_high_prebit\n" // (1/2) loop if the timeout is not 0
        "rjmp .L%=_program_exit\n" // (2) 
        
        // logic to wait for line to go high 
        ".L%=_wait_low_prebit:\n"
        "ld %[inputVal], %a[port_pointer]\n" // (2) reads pin then saves to input value 
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
        // hence are 5 to 12 instructions into the inital bits of the data, this is 16 cycles long
        // we want to check etween the 5th and 12th cycle so we wait 7 cycles
        "ld %[data_temp], %a[port_pointer]\n" // (2)
        // we are between 12 and 19 instructions into the data bit
        // set a timeout for these to prevent problems

        ".L%=_wait_midbits:\n"
        "ldi %[timeout],%[timeout_length]\n" // (1)
        // need to wait for line to go 0
        ".L%=_wait_high_loop_middle:\n"
        "ld %[inputVal], %a[port_pointer]\n" // (2) reads pin then saves to input value 
        "andi %[inputVal], 0x01\n" // (1) compares the input value to the bitmask
        "brne .L%=_wait_low_loop_middle\n" // (1/2) jumps if the value is not 0 from the and (the result was high)
        "dec %[timeout]\n" // (1) decrements timeout 
        "brne .L%=_wait_high_loop_middle\n" // (1/2) loop if the timeout is not 0
        "rjmp .L%=_program_exit\n" // (2)

        ".L%=_wait_low_loop_middle:\n"
        "ld %[inputVal], %a[port_pointer]\n" // (2) reads pin then saves to input value 
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
        "ld %[inputVal], %a[port_pointer]\n" // (2)
        "and %[inputVal], %[bitMaskInfo]\n" // (1)
        "or %[data_temp], %[inputVal]\n" // (1)
        "st %a[buff]+,%[data_temp]\n" // (2) save %[data] back to memory and increment byte pointer
        "dec %[bytecount]\n" // (1) 
        "breq .L%=_program_exit\n" // (1/2) loop if the counter isn't 0
        "rjmp .L%=_wait_prebits\n" // (2)

        ".L%=_program_exit:\n"

        "mov R26, %[low_byte]\n" // (1) restore the pointer
        "mov R27, %[high_byte]\n" // (1) restore the pointer 

        // outputs:
        : [buff] "+z" (buff), // (read and write) // where to save the bytes to
        [port_pointer] "+x" (outPortConsole), // (read and write)
        [data_temp] "=&r" (data_temp), // (output only) // temp for storing bits recived when constructing bytes, ldi needs the upper registers) (data in read)
        [bytecount] "=&r" (bytecount), // (output only, ldi needs the upper registers) (also the number of bits)
        [inputVal] "=&d" (inputVal), // temp storager for pin value, also used for counting bits
        [timeout] "=&d" (timeout), // used to timeout internel loops if they continue too long
        [z_upper_byte] "=&d" (z_upper_byte) // place to store uoper byte of z register (we use timeout for lower byte)
        //[debug] "=&d" (debug)

        // inputs:
        : [len] "r" (len),
        [high] "r" (*outPortConsole | bitMaskConsole), // precalculate new pin states
        [low] "r" (*outPortConsole & ~bitMaskConsole), // this works because we turn interrupts off
        //[inPortComm] "y" (inPortComm), // where to read from
        [bitMaskStatus] "r" (bitMaskStatus), // for identifying the port to check state
        [bitMaskInfo] "r" (bitMaskInfo), // for identifying the port to check state
        [timeout_length] "M" (0x10), // if we loop 64 times waiting between data then we exit, max should be 32 cycles, this exits when we reach 7x that
        [low_byte] "r" (low_byte), // for the in port pointer, will set at time to read
        [high_byte] "r" (high_byte) // for the in port pointer, will set at time to read
        // no clobbers
        ); // end of asm volatile
  //Serial.println(debug ,DEC);

  return (len - bytecount);
}


