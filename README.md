# Tennis-shooter

import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.DigitalIOButton;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogChannel;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    
    // 8 Motors //
    
    Jaguar driveMotor1;
    Jaguar driveMotor2;
    Jaguar throwing_motor_1;
    Jaguar throwing_motor_2;
    Jaguar arm_motor_right;
    Jaguar arm_motor_left;
    Relay belt_motor_1;
    Relay belt_motor_2;
    
    
    Joystick joy1;
    Timer delayTimer;
//    DigitalInput button;
    DigitalInput armLeftDown_KillSwitch;  //Gathering Arms
    DigitalInput armRightDown_KillSwitch;
    DigitalInput armLeftUp_KillSwitch;
    DigitalInput armRightUp_KillSwitch;
    DigitalInput armThrow_KillSwitch; //Throwing Arm
    Encoder encoder;
    int ticksperrev = 250;
    double pi = 3.1415926535897;
    double wheelDiameter = 4;
    int time ;  
    AnalogChannel sonar;
            
    
    public void robotInit(){
        driveMotor1 = new Jaguar(1);     /* drive motor 1 - left */
        driveMotor2 = new Jaguar(2);     /* drive motor 2 - right */
        throwing_motor_1 = new Jaguar(3); //Ball throwing motors
        throwing_motor_2 = new Jaguar(4);
        arm_motor_right = new Jaguar(5); //Moves gather arms up and down
        arm_motor_left = new Jaguar(6);
        belt_motor_1 = new Relay(1); //Gathers ball into bucket
        belt_motor_2 = new Relay(2);
        joy1 = new Joystick(1);
//        button = new DigitalInput(1);
        armRightDown_KillSwitch = new DigitalInput(1);
        armRightUp_KillSwitch = new DigitalInput(2);
        armLeftDown_KillSwitch = new DigitalInput(3);
        armLeftUp_KillSwitch = new DigitalInput(4);
        armThrow_KillSwitch = new DigitalInput(5);
        //encoder = new Encoder(13, 14);
        encoder = new Encoder(13,14);
        encoder.setDistancePerPulse(wheelDiameter*pi/ticksperrev);
        encoder.setReverseDirection(true);
        encoder.start();
        sonar = new AnalogChannel(1);
        
        
    }
 
    
    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
       
        //Initiating Variables
        boolean rotateBelt = false;
        boolean armState = false; //initializes the state of the Arm, False is Down, True is Up
        double armAngle =  90; // desired arm rotation angle, degrees
        double twist =1;
        // define the variable that will be used and initiate them to zero
            double speed  = 0 ;
            double speed1 = 0 ;
            double speed2 = 0 ;
        
        while(isEnabled()){
            // get x and y from joystick
            double joyY = joy1.getY();  // joyY is the forward/backward speed
            double joyX =joy1.getX();   // JoyX is the turning speed
                // if robot is turning opposite joystick make joyX = -joyX
            
            // set small joystick values to zero
            if (joyY ==0.05){
                joyY=0;
            }          
            if (joyX ==0.05){
                joyX=0;
            }
                                 
            // IF the joystick is displaced in X more that it is dispaced in Y, then the speed of the robot is determined my the X displacement
            if (Math.abs(joyX) > Math.abs(joyY)){ 
                speed = joyX;
            }
            else {
                speed = joyY;
            }  // else speed is determined by the y displacement
            
            // if the robot is not turning
            if (joyX == 0){
                speed1 = speed; // left wheel forwad speed
                speed2 = -speed; // right wheel backward speed
            }
            // turning in place
            else if (joyY == 0){
                speed1 = speed;
                speed2 = speed;
            }
            //turn while moving
            else {
                if (joyX > 0) {   // if turning to the right
                    speed1 = speed; // left motor at speed
                    speed2 = -speed + 1.4*speed*joyY; // at full y right side turns same as left at a slow speed (.4speed)
                                                      // at no y the right side turns oposite the left at full speed
                }
                else {    // if turning to the left
                    speed2 = speed; // right motor at speed
                    speed1 = -speed + 1.4*speed*joyY;  // at full y left side turns same as right at a slow speed (.4speed)
                                                      // at no y the left side turns oposite the right at full speed
                }  
                     
            }
            driveMotor1.set(speed1);
            driveMotor2.set(speed2);
            
        
  
} //end RobotTemplate
