#include "mbed.h"
#include "Servo.h"
#include "Motor.h"

DigitalOut led(p25); // RED LEDs
PwmOut spkr(p24); // Speaker
Servo servo(p26); // Servo
Motor right(p21, p7, p8); // pwm, fwd, rev
Motor left(p22, p9, p10); // pwm, fwd, rev
RawSerial  blue(p28,p27); // bluetooth
DigitalOut fire(p11); // DC motor for kicking

float servo_pos = 0.5;

void sound(){
    spkr = 0.5; // Set speaker volume to maximum
    for (float f = 1.0; f < 510; f +=25) {
        led = !led; // Change LEDs between on and off
        spkr.period(1.0/f); // Changes the frequency of the speaker
        spkr = 0.5; // Solve some issues with changing volume
        wait(0.05); // Gives some time for the sound to play
    }
    led =0; // Turn off LED
    spkr = 0; // Turn off speaker
    spkr.period(0.02); // reset for servo
}

void shoot(){
    sound(); // Play the sound before launching the ball
    fire = 1; // Turn on kicking mechanism
    wait(0.12); // Wait for a complete rotation
    fire = 0; // Turn the kicking mechanism off
}






int main(){
    servo = servo_pos; // Set servo initial position to center
    while(1) {
        if(blue.readable()) { // Check if there was a command
            char c;
            c = blue.getc(); // Check first character
            if (c == '!') {
                c = blue.getc(); // Check seccond character
                if (c == 'B') {
                    c = blue.getc(); // Get which button was pressed/released
                    char press = blue.getc(); // Get if it was a release or press
                    if (press == '1') { // if press
                        switch (c) {
                            case '1':  // button 1
                                shoot();
                                break;
                            
                            case '2': // button 2
                                break;
                                
                            case '3': // button 3
                                //barrel left
                                servo_pos = (servo_pos <= 0.1)? 0.0 : servo_pos-0.1; // make sure position is between 0 and 1
                                servo.write(servo_pos);
                                wait(0.2); // wait for servo to turn
                                break;
                            
                            case '4': // button 4
                                // barrel right
                                servo_pos = (servo_pos >= 0.9)? 1.0 : servo_pos+0.1;
                                servo.write(servo_pos); 
                                wait(0.2);
                                break;
                                
                            case '5': // up
                                left.speed(0.5);
                                right.speed(0.5); 
                                break;
                            
                            case '6': // down
                                left.speed(-0.5);
                                right.speed(-0.5);
                                break;
                                
                            case '7': // left
                                left.speed(0.5);
                                break;
                                
                            case '8': // right
                                right.speed(0.5);
                                break;
                                
                        }
                    } else { // if it was release
                        switch (c) {                                
                            case '5': // up
                                left.speed(0);
                                right.speed(0); 
                                break;
                            
                            case '6': // down
                                left.speed(0);
                                right.speed(0);
                                break;
                                
                            case '7': // left
                                left.speed(0);
                                break;
                                
                            case '8': // right
                                right.speed(0);
                                break;
                        }
                    }
                }
            }
        }
    }   
}