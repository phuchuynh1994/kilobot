#include <kilolib.h>

message_t message;
// Flag to keep track of message transmission.
int message_sent = 0;
int message_lock;
uint8_t colorNum[] = {
  RGB(2,0,1),  //0 - magenta
  RGB(3,0,0),  //1 -**red
  RGB(3,1,0),  //2 - orange
  RGB(2,2,0),  //3 - yellow
  RGB(1,3,0),  //4 - yellowish green
  RGB(0,3,0),  //5 -**green
  RGB(0,2,2),  //6 - cyan
  RGB(1,0,2),  //7 -purple 
  RGB(0,0,3),  //8 -**blue
  RGB(0,1,2),  //9 - sky blue
  RGB(0,0,0)   //10- turn off
};
void setup_message();

void setup()
{
    message_lock = 0;
    setup_message();
}

void blink();

void loop()
{
    // Blink LED magenta whenever a message is sent.
    if (message_sent == 1)
    {
        // Reset flag so LED is only blinked once per message.
        message_sent = 0;
        set_color(colorNum[0]);
        delay(100);
        set_color(RGB(0,0,0));
        
    }
}

message_t *message_tx()
{
    
        return &message;
}

void message_rx(message_t *m, distance_measurement_t *d){

}

void message_tx_success()
{
    message_sent = 1;
}

int main()
{
    kilo_init();
    // Register the message_tx callback function.
    kilo_message_tx = message_tx;
    // Register the message_tx_success callback function.
    kilo_message_tx_success = message_tx_success;
    kilo_start(setup, loop);
    
    return 0;
}



void setup_message(){
    
    message.type = NORMAL;
    message.data[0] = kilo_uid;
    message.crc = message_crc(&message);
    
}




void blink(){
    if(kilo_uid == 0) {
        set_color(RGB(1, 0, 1));
        delay(100);
        set_color(RGB(0, 0, 0));
    }
    else if (kilo_uid == 1)
    {
        set_color(RGB(1, 0, 0));
        delay(100);
        set_color(RGB(0, 0, 0));
    }
    else if (kilo_uid == 2)
    {
        set_color(RGB(0, 1, 0));
        delay(100);
        set_color(RGB(0, 0, 0));
    }
    else if(kilo_uid == 3){
        set_color(RGB(0, 0, 1));
        delay(100);
        set_color(RGB(0, 0, 0));
    }
    else if(kilo_uid == 4){
        set_color(RGB(1, 1, 1));
        delay(100); 
        set_color(RGB(0, 0, 0));
    }
    else if(kilo_uid == 5){
    set_color(RGB(3, 1, 0));
        delay(100);
    set_color(RGB(0, 0, 0));
    }
    else if(kilo_uid == 6){
        set_color(RGB(3,2,2));
        delay(100);
        set_color(RGB(0,0,0));
    }
    else if(kilo_uid == 10){
        set_color(colorNum[9]);
        delay(100);
        set_color(RGB(0,0,0));
    }
    else if(kilo_uid != 0){
        set_color(colorNum[6]);
        delay(100);
        set_color(RGB(0,0,0));
    }
}
