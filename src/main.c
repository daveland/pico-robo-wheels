#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include "pico/stdlib.h"


QueueHandle_t charQueue; // charachters are buffered here

QueueHandle_t cmdQueue; // when char queue forms a command this triggers parsing

void led_task()
{   int i=0;
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(50);
        gpio_put(LED_PIN, 0);
        vTaskDelay(50);
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
            printf("A-CH usbread %d \n",ch);
            int pflag=pdFALSE;
            while(!pflag){
            printf("A-CH Charque add %d \n",ch);
              pflag=  xQueueSendToBack(charQueue,& ch,0); // if fails retry char 
              printf("A-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));

                 vTaskDelay(1); //delay 1ms
            }
             pflag=pdFALSE;

            printf("USBchar val %d Sent\n",(int) ch);
            if (ch==0xd)  {
                 printf("A- add cmdqueue %d \n",ch);
              pflag = xQueueSendToBack(cmdQueue,& ch,0); // if it fails retry.
              printf("A- pflag %d \n", pflag);
               printf("A-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
                vTaskDelay(1); //delay 1ms
               printf("A-cmdQueue length %d \n", uxQueueMessagesWaiting(cmdQueue));
               printf("A-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
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
printf("parse cmd\n");
printf("B-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
ch=0;

while ((ch != 13) && (i < 32)) {
  if(  xQueueReceive( charQueue, &( ch ), ( TickType_t ) 0 ) == pdPASS) {
    printf ("ParseCmd char %d\n",ch);
    cmdbuff[i++] = ch;
    printf("C-CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
  }
}

cmdbuff[i]=0; //add temintating null
printf("cmd buff read");
printf("Length %d chars",i);
printf("cmdbuff=%s\n",cmdbuff);
printf("CharQueue length %d \n", uxQueueMessagesWaiting(charQueue));
// POS
if (cmdbuff[0]=='P' && cmdbuff[1]=='?' ) {
  printf("Position cmd P? received",cmdbuff);
}
//VELocity
if (cmdbuff[0]=='V' && cmdbuff[1]=='?' ) {
  printf("Velocity cmd V? received %s",cmdbuff);
}

//Status
if (cmdbuff[0]=='S'  && cmdbuff[1]=='?' ) {
  printf("Status cmd S? received: %s",cmdbuff);
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
        { printf("Z-cmd %d RCVD\n",cmd);
            parsecmd(); //parse and execute cmd
            printf("Z-cmd parsed\n");
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




int main()
{
    stdio_init_all();
// create queue to hold 128 chars

charQueue = xQueueCreate( 128, sizeof( char ) ); //128 char buffer
cmdQueue = xQueueCreate( 100, sizeof( char ) ); //10 cmd avalible  buffer, one CR addred to this per cmd line in input buffer, signals parser
    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(USBSerial_Inputtask, "USBSerial_Inputtask", 256, NULL, 3, NULL);
    xTaskCreate(InputCmd_task, "InputCmd_task", 256, NULL, 2, NULL);
    vTaskStartScheduler();
    puts("Start scheduler\n");
    puts("Robowheels V0.1 \n");

    while(1){};
}