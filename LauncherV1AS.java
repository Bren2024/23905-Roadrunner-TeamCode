/*
Copyright 2021 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *com.qualcomm
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

public class LauncherV1AS {
    
    //this class name
    private String gstrClassName=this.getClass().getSimpleName();
    
    //Declare OpMode members
//    private DcMotor mtrRocketLeft;
    private Servo srvoVector;
    // private Servo srvoPiranhaDogLeft,srvoPiranhaDogRight;
    //private Servo srvoJaw;
    //private Servo srvoThroat;

    //servo positions
    
    
   
    
    //goBildaServer
    //Max PWM Range    500-2500μsec
    //Max PWM Range (Continuous)    900-2100µsec
    //(pulsewidth-500)/2000
    //PiranhaDog servo constants
    //private static double PIRANHADOG_RIGHT_OPEN = 0.55;
    //private static double PIRANHADOG_RIGHT_CLOSE= 0.45d; //(2100-500)/2000
    //private static double PIRANHADOG_LEFT_OPEN=0.45d; //(2100-500)/2000
    //private static double PIRANHADOG_LEFT_CLOSE = 0.55d; //(900-500)/2000
    
    
    //HSRM9382TH  server
    //PWM range 800-2200μsec
    //(pulsewidth-500)/2000
    //                                              
     
    private static Double STICK_DEAD_ZONE=.5;
/*
    private final int CAPSTATE_IDLE=0;
    private final int CAPSTATE_BITING = 1;
    private final int CAPSTATE_RAISING_NECK= 2;
    private final int CAPSTATE_EXTENDING_NECK=3;
    private final int CAP_NECK_EXTEND_POS=2550;
    private long glCapTimeStamp=0;
    private boolean gbCapStarted=false;
    private int gnCapState = CAPSTATE_IDLE;
    */
    
    
    public void initialize(OpMode opMode) {
  
        opMode.telemetry.addData(gstrClassName, "Initializing...");

        //Vector Servo
        srvoVector = opMode.hardwareMap.get(Servo.class, "srvoVector");
        srvoVector.setPosition(0);
        
        //PiranhaDog Servos
        // srvoPiranhaDogLeft = opMode.hardwareMap.get(Servo.class, "srvoPiranhaDogLeft");
        // srvoPiranhaDogLeft.setPosition(PIRANHADOG_LEFT_OPEN);
        // srvoPiranhaDogRight = opMode.hardwareMap.get(Servo.class, "srvoPiranhaDogRight");
        // srvoPiranhaDogRight.setPosition(PIRANHADOG_RIGHT_OPEN);
        
        //srvoJaw = opMode.hardwareMap.get(Servo.class, "srvoJaw");
        //srvoJaw.setPosition(.5);
        
        //srvoThroat = opMode.hardwareMap.get(Servo.class, "srvoThroat");
        //srvoThroat.setPosition(.5);
        
        opMode.telemetry.addData(gstrClassName, "    Initialized");
        

    }

    public void operate(OpMode opMode) {
        opMode.telemetry.addData(gstrClassName,"Launcher Pos:%.2f",
        srvoVector.getPosition());

        if(opMode.gamepad2.guide) {
            srvoVector.setPosition(.4);
        }
    }
    public void shutdown(OpMode opMode) {
        return;
    }
    /*
    public void autonExtendNeckLOp(LinearOpMode linopMode, int nExtendPos) {
        int nNeckCurrPos=mtrGiraffe.getCurrentPosition();
        double dPwr=0d;
        //check if need to extend or contract
        if(nExtendPos>nNeckCurrPos) {
            //need to extend
            dPwr=GIRAFFE_EXTEND_PWR;
            while(nExtendPos>mtrGiraffe.getCurrentPosition()){
                mtrGiraffe.setPower(GIRAFFE_EXTEND_PWR);
            }
            mtrGiraffe.setPower(0d);
        }
        else if (nExtendPos<nNeckCurrPos) {
            dPwr=GIRAFFE_EXTEND_PWR;
            while(nExtendPos<mtrGiraffe.getCurrentPosition()){
                mtrGiraffe.setPower(-GIRAFFE_EXTEND_PWR);
            }
            mtrGiraffe.setPower(0d);
        }
    }
    
    public void autonNeckDownLOp(LinearOpMode linopMode) {
        srvoGiraffeNeck.setPosition(GIRAFFE_NECK_DOWN);
    }
    
    public void autonNeckUpLOp(LinearOpMode linopMode) {
        srvoGiraffeNeck.setPosition(GIRAFFE_NECK_UP);
    }

    public void autonOpenMouthLOp(LinearOpMode linopMode) {
        srvoGiraffeMouth.setPosition(GIRAFFE_MOUTH_OPEN);
    }
    
    public void autonCloseMouthLOp(LinearOpMode linopMode) {
        srvoGiraffeMouth.setPosition(GIRAFFE_MOUTH_CLOSED);
    }
    */
    /*
    private int teleopCap(boolean bButtonPressed) {
        //if got here, the cap button was pressed
        //check if this is the first time pressed
        long lTimeStamp;
        if(bButtonPressed) {
            //starting capping
            glCapTimeStamp=System.currentTimeMillis();
            gnCapState=CAPSTATE_BITING;
            srvoGiraffeMouth.setPosition(GIRAFFE_BITE_ELEMENT);
        }
            
        
        
        switch(gnCapState){
            case CAPSTATE_IDLE:
                return CAPSTATE_IDLE;
            case CAPSTATE_BITING:  //biting
                lTimeStamp=System.currentTimeMillis();
                if((lTimeStamp-glCapTimeStamp)>300) {//300 ms to bite
                   glCapTimeStamp=lTimeStamp;
                   gnCapState=CAPSTATE_RAISING_NECK;
                   return CAPSTATE_RAISING_NECK;
               
                }
                return CAPSTATE_BITING;
            case CAPSTATE_RAISING_NECK: //raising neck
                srvoGiraffeNeck.setPosition(GIRAFFE_NECK_DOWN); //makes neck level to mat
                gnCapState=CAPSTATE_EXTENDING_NECK;
                return CAPSTATE_EXTENDING_NECK;
            case CAPSTATE_EXTENDING_NECK:
                int nNeckCurrPos=mtrGiraffe.getCurrentPosition();
                double dPwr=0d;
                //check if need to extend or contract
                if(CAP_NECK_EXTEND_POS>nNeckCurrPos) {
                    //need to extend
                    dPwr=GIRAFFE_EXTEND_PWR;
                    mtrGiraffe.setPower(GIRAFFE_EXTEND_PWR);
                    return (CAPSTATE_EXTENDING_NECK);
                    
                }
                //done extending
                mtrGiraffe.setPower(0d);
                
                gnCapState=CAPSTATE_IDLE;
                gbCapStarted=false;
                return (gnCapState);
            default:
                return (gnCapState); 
        }
    
        
    }
    */
}
