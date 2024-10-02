/** pico-robo-wheels july 2024 -- "The beginning of Davenet" and the robot appocolypse
 * Copyright (C) 2024 {David L. Anderson} - All Rights Reserved
 * 
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */



#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/unique_id.h"  //eeprom unique_id
#include "string.h"
#include "pico-robo-wheels.pio.h"

// LCD 
#include "ssd1306.h"
//#include "font.h"

//#include "ssd1306.h"
#include "image.h"
#include "acme_5_outlines_font.h"
#include "bubblesstandard_font.h"
#include "crackers_font.h"
#include "BMSPA_font.h"

// Core 0 is managed by Freertos.  


// core 0 starts Core 1 slave running before starting  RTOS Kernel.

// Core 1 is used as a slave and IS NOT managed by FreeRTOS
// Core 1 runs the Quadrature encoders and manages position
// Core 1 runs the PWM for the 2 motors L+R
// Data space is shared by Global Variables ( a bad way i know)
// At this point I don't yet understand how 2 cores Share data without corruption
// due to simultainious access
// do for now Core 1 ( non RT slave)  writes to the Variable space and core 0
// Only reads them..  This works as long as all cor1 handles are inputs

// core 1 handles PIO position interrupts from 2 encoders
// Core 1 handles Velocity calcs from encoders
// core 1 incriments core1 counter every 1 sec 


#include "pico/multicore.h"

#include "hardware/pwm.h" // Hardware PWM
#include "hardware/timer.h" // Timer hardware
#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/pio.h"


//Spinlocks
// Core1 is running data collection routines and modifying shared global varibles
// A single spinlock is used for access to all
// core1 -> Core0 direction data  
// Core1 position L/R updates (encoder interupt based)
// Core1 Velocity L/R updates ( encoder interrupt based)

//core0 -> core1 direction data
// Motor PWM ??  assume motor pwm is defined in Rtos core 0 path planner ??



// Two variables to store core number
volatile int corenum_0  ;
volatile int corenum_1  ;

// Global counter for spinlock experimenting
volatile int global_counter = 0 ;

// Core0-Core1 data spinlock number, initiakized by core0 before core1 startup
int spinlock_num_count ;
spin_lock_t *spinlock_count ;



// Motor controller info
//

enum D100AMtrDir {
    FWD = 1,
    REV = 2,
    BRAKE = 0,
    COAST = 3,
};

//typedef MtrDrverinfo {
  //  uint8_t ctrltype;
  //  uint8_t mtrfwd;
  //  uint8_t mtrrev;
//} Mtr;

//MtrDrverinfo D100A = {.ctrltype=0,.mtrfwd=FWD,.mtrrev=2} ;


#include "pico-robo-wheels.pio.h"
// Core 0 PWM 
// define Motor pwm pins to output pwm and direction for each motor
const uint LeftPwmPin =1;  // GPIO #1
 const uint LeftMtrPinA =2;  // GPIO control of Motor driver Hbridge
 const uint LeftMtrPinB =3;

const uint RightPwmPin =8;  //GPIO #8
 const uint RightMtrPinA =9; // GPIO control of Motor driver Hbridge
 const uint RightMtrPinB =10;

const uint PWMTOP = 15624 ;  //top value for PWM counters 
//Core 1
// define Encoder AB input pins
// Each pair must be adjacent  A+1=B
//Left encoder PIO0 block SM0  and Right encoder PIO1 block SM0
// We load same PIO code into both PIO blocks memory
const int LeftEncA =4; // use 2 pio blocks for now... Should be able to move to one PIO Block
const int LeftEncB =5;  // interrupts are not working like I expect to have 2 "copies" of the same 
const int RightEncA =6; // PIO Program Running on two SM's in the same PIO block and having unique 
const int RightEncB =7; // interrupts to call the count up/down IRQ's 
// seperatly Running on 2 pio blocks solves this for now


//core 1
// Position is 64 bits Signed !!
volatile int64_t LeftPosition ;  // 64 bits
volatile int64_t RightPosition ; //64 bits
volatile uint32_t Core1counter=0; 
volatile uint32_t Core1State;  // state of core1


 //core 1
// motor velocity computed using period of interrupts on each wheel
// meausure time from each interrupt and convert to mS. Then average this
// over several interrupts and cacluate velocity in m/s 
// using tire diameter and gear ratio
//Vel (m/s) =  (1/(Tinterupt*cpr)) / gearRatio * Wheeldiam (M)*PI
volatile int RightVelocity=0;
volatile int LeftVelocity=0;
int cpr=100*4; // encoder pulses per revolution * 4 counts per pulse
int GearRatio = 20;  // ratio from encoder revs to wheel revs
float LeftWheelDiam_m = 0.32385;  // wheel diameter in meters
float RightWheelDiam_m = 0.33385;  // wheel diameter in meters
int comp =0;

//core 1
// we measure encoder distance as a small number. Determined by encoder CPR ( counts per rev)
// and Gear ratio from encoder to wheel , and Wheel Diameter in milimeters.
// Keeping these in different units in integer valibles avoids slow floating point lirary 
// calls ( no floating point unit in Pico RP2040)
// since we have multiple 
int DistEncoderRightuMDec;  // distance in uM per encoder pulse *100 
int DistEncoderLeftuMDec;   // distance in uM per encoder pulse *100 

// core 1 keep the last right and left encoder interrupt times for speed calulation
uint64_t LastRightEncIntTimeuS;  
uint64_t LastLeftEncIntTimeuS; 

//Core 0
uint PIOoffset ;

QueueHandle_t charQueue; // charachters are buffered here

QueueHandle_t cmdQueue; // when char queue forms a command this triggers parsing

pico_unique_board_id_t * FlashIdPtr;  // 8 byte flash id buffer, unique per Pico Flash chip, used fpt liscensing

char * PGMVersion = "0.2.5";

// Floating Point used here ONCE on bootup
void ComputeEncoderDistances() {
    LastRightEncIntTimeuS=0; 
    LastLeftEncIntTimeuS=0;  
// determine the distance in uM  that a wheel moves in one encoder count
// we use this to keep integer math in micrometers 64bit and then multiply by number of us
// to get how far we traveled and compute velocity..

    // (one wheel rotation meters)/GearRatio) = one encoder rotation in meters
    // = /CPR = one encoder Pulse in meters
    // = *1e6 = one encoder pulse in u meters
float DistuMPerCountfloat= ((((RightWheelDiam_m*3.14159)/GearRatio)/cpr) *1000e6 ) ;  // 100 * the distance in uM per encoder pulse
DistEncoderRightuMDec= (int) DistuMPerCountfloat ; //Truncate ?? or round ??

 DistuMPerCountfloat= ((((LeftWheelDiam_m*3.14159)/GearRatio)/cpr) *1000e6 ) ;  // 100 * the distance in uM per encoder pulse
DistEncoderLeftuMDec= (int) DistuMPerCountfloat ; //Truncate ?? or round ??


// figure out how long the division takes in the interrupt encoder handler
//  we need it to be pretty fast .. It should be since there is a 64 Bit 
// Hardware Divider in each core..  
uint64_t BegintimeuS=time_us_64(); //read current 64 bit tic ctr
 uint32_t delta_ticsuS;
for (int i=0;i<1000000;i++) {

    // this takes 1.26us From, here ...
    uint64_t currenttimeuS=(uint32_t)(currenttimeuS-LastRightEncIntTimeuS); // compute delta tics since last encoder interrupr
       LastRightEncIntTimeuS=currenttimeuS;  //save now as new time
       RightVelocity = DistEncoderRightuMDec/delta_ticsuS/1000; // divide by 100 to remove the 100X for significant digits  then convert to um/us = m/s
    ///   to here ...
}
uint64_t donetimeus=time_us_64();

comp=donetimeus-BegintimeuS;


}


// core1 running Bare metal
//------------------------------------------------------------------

static void Left_pio_irq_handler (void) {
   // This distance DistEncoderRight is the  amount of motion in one encoder pulse
    // It depends on the encoder CPR and the GearRatio From  encoder to Wheel position
    // 
       uint64_t currenttimeuS=time_us_64(); //read current 64 bit tic ctr

       uint32_t delta_ticsuS=(uint32_t)(currenttimeuS-LastLeftEncIntTimeuS); // compute delta tics since last encoder interrupr
       LastLeftEncIntTimeuS=currenttimeuS;  //save now as new time
       LeftVelocity = DistEncoderLeftuMDec/delta_ticsuS/1000; // divide by 100 to remove the 100X for significant digits  then convert to um/us = m/s
        // Lock spinlock (without disabling interrupts since we are in the int handler and cannot be interrupted again on core1)
        spin_lock_unsafe_blocking(spinlock_count) ; // get spinlock for core 1(write)-> core 0 (read) global access


         // test Which IRQ was raised by state machine on PIO0 sm1
         // Left encoder is serviced by SM0 With SM flags 
        //printf("leftPosition %d \n",pio0_hw->irq);
        if (pio0_hw->irq & 1) // bit sm0
        { //puts("leftPosition down\n");
            LeftPosition -=1;
        }
        // test if irq 1 was raised
        if (pio0_hw->irq & 2) // bit sm1
        {//puts("leftPosition up\n");
            LeftPosition +=1;
        }
        
        spin_unlock_unsafe(spinlock_count) ;

        // clear both interrupts by setting to 1
        pio0_hw->irq = 3;
}

static void Right_pio_irq_handler (void) {
     // to determine motor speed,  velocity = distance/Time = Meters/second
     //we measure how many 1us tics between Encoder interrupt pulses
     // delta_tics. This is our delta time in us 
     // Then we figure distance traveled in one encoder pulse.
     //  CPR = counts Per rotation.  100 counts in our case
     //  1 count= 360deg/100 = 3.6 degs
     //  500 motor rotations = 1 wheel rotation.
     //  1 wheel rotation = 13 " *PI = 13*3.14159 =40.8401" =1.037 Meters
     // 1.037/gearration=1.037/500= meters per encoder rev =.002074 Meters 
     // meters per encoder pulse = .002074/100 = .00002074 meters
    // each onterrupt call represents one encoder pulse that is a certain number of meters of distance 
    // This distance DistEncoderRight is the  amount of motion in one encoder pulse
    // It depends on the encoder CPR and the GearRatio From  encoder to Wheel position
    // 
       uint64_t currenttimeuS=time_us_64(); //read current 64 bit tic ctr

       uint32_t delta_ticsuS=(uint32_t)(currenttimeuS-LastRightEncIntTimeuS); // compute delta tics since last encoder interrupr
       LastRightEncIntTimeuS=currenttimeuS;  //save now as new time
       RightVelocity = DistEncoderRightuMDec/delta_ticsuS/1000; // divide by 100 to remove the 100X for significant digits  then convert to um/us = m/s
       // 1m/s  = 1e6m/

       // Lock spinlock (without disabling interrupts since we are in the int handler and cannot be interrupted again on core1)
        spin_lock_unsafe_blocking(spinlock_count) ; // get spinlock for core 1(write)-> core 0 (read) global access



// test Which IRQ was raised by state machine on PIO sm1
// this determines if we should count up or down
        if (pio1_hw->irq & 1) //bit sm2
        {//puts("RightPosition up\n");
            RightPosition -=1;
        }
        // test if irq 1 was raised
        if (pio1_hw->irq & 2) //bit sm3
        {//puts("RightPosition up\n");
            RightPosition +=1;
        }

        spin_unlock_unsafe(spinlock_count) ;

        // clear both interrupts by setting to 1
        pio1_hw->irq = 0x3;

}


// setup pio0  to do the left motor encoder
void SetupPioEncoderLeft( )  {
    LeftPosition=0;
    LeftVelocity=0;
    LastLeftEncIntTimeuS=time_us_64();
    
   // pio 0 SM0 is used to track A nd B left pulses
        PIO pio = pio0;
        // state machine 0
        uint8_t sm = 0;
        // configure the used pins as input with pull up
        pio_gpio_init(pio, LeftEncA);
        gpio_set_pulls(LeftEncA, true, false);
        pio_gpio_init(pio, LeftEncB);
        gpio_set_pulls(LeftEncB, true, false);
        // load the pio program into the pio memory
         PIOoffset = pio_add_program(pio, &pico_robo_wheels_program);
        // make a sm config
        pio_sm_config c = pico_robo_wheels_program_get_default_config(PIOoffset);
        // set the 'in' pins
        sm_config_set_in_pins(&c, LeftEncA);
        // set shift to left: bits shifted by 'in' enter at the least
        // significant bit (LSB), no autopush
        sm_config_set_in_shift(&c, false, false, 0);
        // set the IRQ handler
        irq_set_exclusive_handler(PIO0_IRQ_0, Left_pio_irq_handler);
        // enable the IRQ
        irq_set_enabled(PIO0_IRQ_0, true);
        // SM0 and SM2 are connected to up/down interrupt for Left Motor
        // on SM0 triggering Core1 IRQ0 with SMbits 0 and 2
        pio0_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS;
        // init the sm.
        // Note: the program starts after the jump table -> initial_pc = 16
        pio_sm_init(pio, sm, 16, &c);
        // enable the sm
        pio_sm_set_enabled(pio, sm, true);
    }


// setup pio0  to do the Right motor encoder
void SetupPioEncoderRight( )  {
    RightPosition=0;
    RightVelocity=0;
    LastRightEncIntTimeuS=time_us_64();

   // pio 0 is used
        PIO pio = pio1;
        // state machine 1
        uint8_t sm = 0;
        // configure the used pins as input with pull up
       
        pio_gpio_init(pio, RightEncA);
        gpio_set_pulls(RightEncA, true, false);
        pio_gpio_init(pio, RightEncB);
        gpio_set_pulls(RightEncB, true, false);
        
        // load the pio program into the pio memory
        uint offset = pio_add_program(pio, &pico_robo_wheels_program);
        // make a sm config
        
        pio_sm_config c = pico_robo_wheels_program_get_default_config(offset);
        // set the 'in' pins
       
        sm_config_set_in_pins(&c, RightEncA);
        // set shift to left: bits shifted by 'in' enter at the least
        // significant bit (LSB), no autopush
        sm_config_set_in_shift(&c, false, false, 0);
          
        // set the IRQ handler
        irq_set_exclusive_handler(PIO1_IRQ_0, Right_pio_irq_handler);
        // enable the IRQ
        irq_set_enabled(PIO1_IRQ_0, true);
        // SM1 and SM3 are connected to up/down interrupt for Right Motor
        // on SM1 triggering Core1 IRQ1 with SMbits 1 and 3
        pio1_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS;
        // init the sm.
        // Note: the program starts after the jump table -> initial_pc = 16
        pio_sm_init(pio, sm, 16, &c);
        // enable the sm
        pio_sm_set_enabled(pio, sm, true);
        
    }


void InitMotorABPins() {
//gpio_set_dir(LeftPwmPin,true);
//gpio_set_drive_strength(LeftPwmPin,GPIO_DRIVE_STRENGTH_12MA);
//gpio_set_drive_strength(LeftMtrPinA,GPIO_DRIVE_STRENGTH_12MA);
//gpio_set_drive_strength(LeftMtrPinB,GPIO_DRIVE_STRENGTH_12MA);

gpio_init(LeftMtrPinA);
gpio_init(LeftMtrPinB);

gpio_init(RightMtrPinA);
gpio_init(RightMtrPinB);

gpio_set_dir(LeftMtrPinA,GPIO_OUT);
gpio_set_dir(LeftMtrPinB,GPIO_OUT);

  gpio_put(LeftMtrPinA,1);
  gpio_put(LeftMtrPinB,1);
//gpio_set_dir(RightPwmPin,true);
gpio_set_dir(RightMtrPinA,GPIO_OUT);
gpio_set_dir(RightMtrPinB,GPIO_OUT);

  gpio_put(RightMtrPinA,1);
  gpio_put(RightMtrPinB,1);
    
}

void InitMotorPWMPins () {

 puts("Setup pwm\n");
     // Tell GPIO s it is allocated to the PWM
    gpio_set_function(RightPwmPin, GPIO_FUNC_PWM);
    gpio_set_function(LeftPwmPin, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to pwm GPIO then configure Pwm
    // Fpwm= Fsys/period  = 125Mhz/ 15624  = 8000.6 hz
    // period =15624 = (Top +1) * (Phase corr+1) *( Divint + DivFrac/16)
    //
    uint Rightslice_num = pwm_gpio_to_slice_num(RightPwmPin);
    pwm_set_clkdiv_int_frac(Rightslice_num,1,0);  // Divint, divfrac  for 8Khz PWM rate
    uint Leftslice_num = pwm_gpio_to_slice_num(LeftPwmPin);
    pwm_set_clkdiv_int_frac(Leftslice_num,1,0);
    // set Top value 8Khz ???
        pwm_set_wrap(Rightslice_num, PWMTOP); // 
          pwm_set_wrap(Leftslice_num, PWMTOP); // 
    // Set channel A B to 50% duty cycle
    pwm_set_chan_level(Rightslice_num, PWM_CHAN_A, PWMTOP/2 ); //50% duty cycle
    pwm_set_chan_level(Leftslice_num, PWM_CHAN_B, PWMTOP/2); //50% duty cycle
    //pwm_set_phase_correct(Rightslice_num,true);
    //pwm_set_phase_correct(Leftslice_num,true);

    // Set the PWM running
    pwm_set_enabled(Rightslice_num, true);
    puts("pwm Right enabled\n");

    pwm_set_enabled(Leftslice_num, true);
    puts("pwm Left enabled\n");
}



// core 1 code here --- not managed by RTOS !!!
void core1_entry(void) {
 
corenum_1=get_core_num();

//toggle GPIO #1 to show core 0 is running
// run core 1 for
// PWM Generators for Motor control
// direction stop start control
// keep position of each wheel
// using Quadrature Encoders for motor position ( PIO inpput buffers and counting code)
// Velocty calulation for each motor
// virtual transmission locking Left and right speeds together
// Acceleration/decelleration control
Core1State =98;

ComputeEncoderDistances();
SetupPioEncoderLeft();
SetupPioEncoderRight();

Core1State =1;

pico_get_unique_board_id(FlashIdPtr); // used as sofware key

// setup PWM Pin I/O
InitMotorABPins();
InitMotorPWMPins();


Core1State =3;
printf("Core 1 Started\n");
while (1) {
// set pwm I/O to defult
// Pwm =0%  stopped dir =FWD

// no interrupts arused for cpu ctr,, just a sleep loop so no need to disable interrupts
// Lock spinlock (without disabling interrupts since we are in the int handler and cannot be interrupted again on core1)
spin_lock_unsafe_blocking(spinlock_count) ; // get spinlock for core 1(write)-> core 0 (read) global access
Core1counter+=1; // foreground action,  interrupts do not touch Core1counter, but core0 reads it
// this should be safe with AB lite bus stalls handling contention with only 1 writer and 1 reader 
spin_unlock_unsafe(spinlock_count) ;

sleep_ms(1000);
//printf("Core 1 Running %d\n",Core1counter);
// Start Encoder PIO and sampling of  position
// set PosL and PosR to 0

// wait for Motor enable 

}
}

// Safe access to core1 counter
uint32_t get_Core1Counter() {
spin_lock_unsafe_blocking(spinlock_count) ; // get spinlock for core 1(write)-> core 0 (read) global access
uint32_t tmp= Core1counter;
// this should be safe with AB lite bus stalls handling contention with only 1 writer and 1 reader 
spin_unlock_unsafe(spinlock_count) ;
 return(tmp);
}

// Safe access to LeftPosition 
uint32_t get_LeftPosition() {
spin_lock_unsafe_blocking(spinlock_count) ; // get spinlock for core 1(write)-> core 0 (read) global access
int64_t tmp= LeftPosition;
// this should be safe with AB lite bus stalls handling contention with only 1 writer and 1 reader 
spin_unlock_unsafe(spinlock_count) ;
 return(tmp);
}

// Safe access to LeftPosition 
uint32_t get_RightPosition() {
spin_lock_unsafe_blocking(spinlock_count) ; // get spinlock for core 1(write)-> core 0 (read) global access
int64_t tmp= RightPosition;
// this should be safe with AB lite bus stalls handling contention with only 1 writer and 1 reader 
spin_unlock_unsafe(spinlock_count) ;
 return(tmp);
}





//Core 0 --- Running under Free RTOS 
//-----------------------------------------------------------
void led_task()
{   int i=0;
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    
    gpio_set_dir(LED_PIN, GPIO_OUT);
  
 
    while (true) {
        gpio_put(LED_PIN, 1);
    
        vTaskDelay(500);
        gpio_put(LED_PIN, 0);
      
        vTaskDelay(500);
        //puts("Robowheels V0.1 \n");
       //printf("count %d\n",i++);
    }
}

// Get usbserial input and put it on the input Queue 
void USBSerial_Inputtask()
{   
   //gets(inputbuffer ); // this blocks until a CR is entered'

// push all received chars from usb input to the charqQueue 
//  when charqueue has a cr then we process command in command parsere which empties the queue
int chint;
char ch;
    while (true) {
        // change this later to interrupt based, for now we poll  ....
        chint= getchar_timeout_us(0);

        if (chint != PICO_ERROR_TIMEOUT) {
            ch=(char)chint;
            //printf("A-CH usbread %d \n",ch);
            int pflag=pdFALSE;
            while(!pflag){
            //printf("A-CH Charque add %d \n",ch);
              pflag=  xQueueSendToBack(charQueue,& ch,0); // if fails retry char 
             //printf("A-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));

                 vTaskDelay(1); //delay 1ms
            }
             pflag=pdFALSE;

            //printf("USBchar val %d Sent\n",(int) ch);
            if (ch==0xd)  {
                // printf("A- add cmdqueue %d \n",ch);
              pflag = xQueueSendToBack(cmdQueue,& ch,0); // if it fails retry.
             // printf("A- pflag %d \n", pflag);
             //  printf("A-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
                vTaskDelay(1); //delay 1ms
             //  printf("A-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
             //  printf("A-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
            }


        } 
       vTaskDelay(10); //delay 1ms 

    }
}

int cmdPosa () {

}

// look in the char que pulling out chars and parsing commands
// a CR in the char que is end of command line.  When this routine finishes
//  the char que hase been read until after the CR
// Then it parses and runs the cmd
void parsecmd() {

// pull chars out and parse the command
// simple parser for now.
// one char commands 
// P?  Postion L,R wheel position
char ch;
char cmdbuff[33];
char * buffp;
int i =0;
//printf("parse cmd\n");
//printf("B-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
ch=0;

while ((ch != 13) && (i < 32)) {
  if(  xQueueReceive( charQueue, &( ch ), ( TickType_t ) 0 ) == pdPASS) {
   // printf ("ParseCmd char %d\n",ch);
    cmdbuff[i++] = ch;
   // printf("C-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
  }
}

cmdbuff[i]=0; //add temintating null
//printf("cmd buff read");
//printf("Length %d chars",i);
//printf("cmdbuff=%s\n",cmdbuff);
//printf("CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));

// Show POSition command
if (cmdbuff[0]=='P' && cmdbuff[1]=='?' ) {
  printf("Position cmd P? received",cmdbuff);
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
//show VELocity
if (cmdbuff[0]=='V' && cmdbuff[1]=='?' ) {
  printf("Show Velocity cmd V? received %s",cmdbuff);
  printf("left velocity %f m/s\n ",(float)(LeftVelocity)/1000E6);
printf("right velocity %F m/s \n",RightVelocity);
}
//Position Reset command  set position to 0
if (cmdbuff[0]=='R' && cmdbuff[1]=='S' ) {
  printf("Position reset RS received",cmdbuff);
  LeftPosition=0;
  RightPosition=0;
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}

//Forward Motors
if (cmdbuff[0]=='F' && cmdbuff[1]=='W' ) {
  printf("Command FW received",cmdbuff);
  gpio_put(LeftMtrPinA,1);
  gpio_put(LeftMtrPinB,0);

  gpio_put(RightMtrPinA,1);
  gpio_put(RightMtrPinB,0);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
// Right turn motors
if (cmdbuff[0]=='R' && cmdbuff[1]=='T' ) {
  printf("Command FW received",cmdbuff);
  gpio_put(LeftMtrPinA,1);
  gpio_put(LeftMtrPinB,0);

  gpio_put(RightMtrPinA,0);
  gpio_put(RightMtrPinB,1);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
// Left turn motors
if (cmdbuff[0]=='L' && cmdbuff[1]=='T' ) {
  printf("Command FW received",cmdbuff);
  gpio_put(LeftMtrPinA,0);
  gpio_put(LeftMtrPinB,1);

  gpio_put(RightMtrPinA,1);
  gpio_put(RightMtrPinB,0);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
// reverse motors
if (cmdbuff[0]=='R' && cmdbuff[1]=='V' ) {
  printf("Command FW received",cmdbuff);
  gpio_put(LeftMtrPinA,0);
  gpio_put(LeftMtrPinB,1);

  gpio_put(RightMtrPinA,0);
  gpio_put(RightMtrPinB,1);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}
// 100A driver .. Green heatsink
//A1,A2=0,0 is the brake;
//A1,A2=1,0 is FWD rotation;
//A1,A2=0,1 is Rev Direction;
//A1,A2=1,1 is not allowed ?? perhaps coast?
//PA is PWM wave input (motor speed regulation);
//G is the common pin with the control board (route B is the same control);

// 12A driver Orange heatsink
//A1,B1 =0,0
//A1,B1 =1,0
//A1,B1 =0,1
//A1,B1 =1,1


// Stop motor using Brake SB
if (cmdbuff[0]=='S' && cmdbuff[1]=='B' ) {
  printf("Command SB received",cmdbuff);
  gpio_put(LeftMtrPinA,0);
  gpio_put(LeftMtrPinB,0);

  gpio_put(RightMtrPinA,0);
  gpio_put(RightMtrPinB,0);
 
  printf("L=%lld,R=%lld\n",LeftPosition,RightPosition);
}

// Set Both motors PWM ( NOT Velocity controlled)
// only use when VC=0 (velocity control =0 means off)
// PW,Value
if (cmdbuff[0]=='P' && cmdbuff[1]=='W' && cmdbuff[2]==',' ) {
  //printf("PWmset command received PW,decimal value received=%d\n",cmdbuff);
  long sval;
  char *end;
  sval=strtol(cmdbuff+3,&end,10);
  printf("val=%ld\n",sval);

printf("Cmd buff length=%d\n",strlen(cmdbuff));
// set pwm block to velocity value
    uint Rightslice_num = pwm_gpio_to_slice_num(RightPwmPin);
    uint Leftslice_num = pwm_gpio_to_slice_num(LeftPwmPin);
    uint PWMchanval=(float) sval*0.01  * PWMTOP;
    printf("PWMChanVal=%d\n",PWMchanval); 
    pwm_set_chan_level(Rightslice_num, PWM_CHAN_A, PWMchanval ); //% duty cycle to chan level
    pwm_set_chan_level(Leftslice_num, PWM_CHAN_B, PWMchanval); //% duty cycle to chan level
  
  //printf("L=%d,R=%d\n",LeftPWMVal,RightPWMVal); // measured velocity
}


//Status
if (cmdbuff[0]=='S'  && cmdbuff[1]=='?' ) {
  printf("Status cmd S? received: %s\n",cmdbuff);
  printf("version= %s\n",PGMVersion);
  printf("Software key= %02X", FlashIdPtr->id[0]);
printf("%02X", FlashIdPtr->id[1]);
printf("%02X", FlashIdPtr->id[2]);
printf("%02X", FlashIdPtr->id[3]);
printf("%02X", FlashIdPtr->id[4]);
printf("%02X", FlashIdPtr->id[5]);
printf("%02X", FlashIdPtr->id[6]);
printf("%02X\n", FlashIdPtr->id[7]);
printf("core1count=%d\n",get_Core1Counter());
printf("left velocity %d\n",LeftVelocity);
printf("right velocity %d\n",RightVelocity);
printf("left Position %d\n",get_LeftPosition());
printf("right Position %d\n",get_RightPosition());

printf("DistEnccountRightuM= %f um/enc_cnt\n",(float) (DistEncoderRightuMDec)/1000.0);
printf("DistEnccountLeftuM= %f um/enc_cnt\n",(float) (DistEncoderLeftuMDec)/1000.0);
printf("comp=%d\n",comp);
}


} 


// wait for a cmd to appear in the cmd task
// then parse the chars unti the first linefeed in the input queue.
void InputCmd_task()
{ int cmdint;
char cmd;
char ch ;
    while (true) {
           //printf("Z- In cmdQueue \n");
        // change this later to interrupt based, for now we poll  ....
        //xQueueCRReceiveFromISR(charQueue,& ch,0); // if this fails char is lost.
          //printf("Z-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
         cmdint=xQueueReceive(cmdQueue,& cmd, ( TickType_t )0); // try to get cmd
              //printf("Z-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
       if (cmdint==pdTRUE){
        if (cmd == 0x0d) 
        { //printf("Z-cmd %d RCVD\n",cmd);
            parsecmd(); //parse and execute cmd
           // printf("Z-cmd parsed\n");
        }
       }
        
          
   // dump rcvd char for now


      // if( charQueue != 0 )
   // {
        // Peek a message on the created queue.  Block for 10 ticks if a
        // message is not immediately available.
        //if( xQueuePeek( charQueue, &( ch ), ( TickType_t ) 1 ) )
        //{
            // ch now points to the last char in the queue
            //  but all items still remains on the queue.
           // if(ch==0x0d)  // last char in queue is a cr , empty and parse the queue

       // }
   // }

    // ... Rest of task code.
    vTaskDelay(10); //delay 1ms
 }

    }


// OLED functions
// core 0 RTOS can only talk to OLED
void OLED_setup_gpios(void) {
    i2c_init(i2c1, 400000);
    gpio_set_function(19, GPIO_FUNC_I2C); //OLED SCL1  
    gpio_set_function(18, GPIO_FUNC_I2C); //OLED SDA1
    gpio_pull_up(19);
    gpio_pull_up(20);
}

void OLED_animation_Task() {
    const char *words[]= {"SSD1306", "DISPLAY", "DRIVER"};
const uint8_t *fonts[4]= {acme_font, bubblesstandard_font, crackers_font, BMSPA_font};
const char *Speedwords[]= { "SPEED m/s","MtrL=32.58", "Mtrl=32.59"};
    const uint8_t num_chars_per_disp[]={7,7,7,5};

#define SLEEPTIME 25


    ssd1306_t disp;
    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c1);
    ssd1306_clear(&disp);

    printf("ANIMATION!\n");

    char buf[8];

    for(;;) {
        for(int y=0; y<31; ++y) {
            ssd1306_draw_line(&disp, 0, y, 127, y);
            ssd1306_show(&disp);
            //sleep_ms(SLEEPTIME);
            vTaskDelay(25); //delay
            ssd1306_clear(&disp);
        }

        for(int y=0, i=1; y>=0; y+=i) {
            ssd1306_draw_line(&disp, 0, 31-y, 127, 31+y);
            ssd1306_draw_line(&disp, 0, 31+y, 127, 31-y);
            ssd1306_show(&disp);
            //sleep_ms(SLEEPTIME);
            vTaskDelay(25); //delay
            ssd1306_clear(&disp);
            if(y==32) i=-1;
        }

        for(int i=0; i<sizeof(words)/sizeof(char *); ++i) {
            ssd1306_draw_string(&disp, 8, 24, 2, words[i]);
            ssd1306_show(&disp);
            //sleep_ms(800);
            vTaskDelay(800); //delay
            ssd1306_clear(&disp);
        }

        for(int y=31; y<63; ++y) {
            ssd1306_draw_line(&disp, 0, y, 127, y);
            ssd1306_show(&disp);
            //sleep_ms(SLEEPTIME);
            vTaskDelay(25); //delay
            ssd1306_clear(&disp);
        }

        // for(size_t font_i=0; font_i<sizeof(fonts)/sizeof(fonts[0]); ++font_i) {
        //     uint8_t c=32;
        //     while(c<=126) {
        //         uint8_t i=0;
        //         for(; i<num_chars_per_disp[font_i]; ++i) {
        //             if(c>126)
        //                 break;
        //             buf[i]=c++;
        //         }
        //         buf[i]=0;

        //         ssd1306_draw_string_with_font(&disp, 8, 24, 2, fonts[font_i], buf);
        //         ssd1306_show(&disp);
        //         //sleep_ms(800);
        //         vTaskDelay(800); //delay
        //         ssd1306_clear(&disp);
        //     }
        // }
        
        ssd1306_clear(&disp);
         ssd1306_bmp_show_image(&disp, image_data, image_size);
         ssd1306_show(&disp);
         //sleep_ms(2000);
         vTaskDelay(2000); //delay

/// my message
        ssd1306_clear(&disp);
        ssd1306_draw_string(&disp, 8, 0, 2, Speedwords[0]);
    for(int i=1; i<sizeof(Speedwords)/sizeof(char *); ++i) {
            ssd1306_draw_string(&disp, 8, 4+(16*i), 2, Speedwords[i]);
    }
            ssd1306_show(&disp);
            //sleep_ms(800);
            vTaskDelay(4800); //delay
            ssd1306_clear(&disp);
        
    }
}

int main()
{
    stdio_init_all();
// create queue to hold 128 chars

OLED_setup_gpios();
//OLED_animation();

corenum_0=get_core_num();

 // Claim and initialize a spinlock for core 0 
    spinlock_num_count = spin_lock_claim_unused(true) ;
    spinlock_count = spin_lock_init(spinlock_num_count) ;



 multicore_launch_core1(core1_entry);


charQueue = xQueueCreate( 128, sizeof( char ) ); //128 char buffer
cmdQueue = xQueueCreate( 100, sizeof( char ) ); //10 cmd avalible  buffer, one CR addred to this per cmd line in input buffer, signals parser
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(USBSerial_Inputtask, "USBSerial_Inputtask", 256, NULL, 3, NULL);
    xTaskCreate(InputCmd_task, "InputCmd_task", 256, NULL, 2, NULL);
    xTaskCreate(OLED_animation_Task, "OLED_Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();
    puts("Start scheduler\n");
    puts("Robowheels V0.1 \n");

    while(1){};
}