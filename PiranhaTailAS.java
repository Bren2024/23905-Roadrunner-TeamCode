package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class PiranhaTailAS {

    // todo: write your code here
    //this class name
    private String gstrClassName=this.getClass().getSimpleName();

    //Declare OpMode members
    //private DcMotor mtrTeeth;
    // private Servo srvoPiranhaDogLeft,srvoPiranhaDogRight;
    private Servo srvoTail;
    // private Servo srvoThroatU;
    // private Servo srvoThroatL;
    // private Servo srvoPaw;
    // private Servo srvoTongue;

    //motor constants
    //private static Double TEETH_EAT_PWR=-.85d, TEETH_PUKE_PWR=.5d, TEETH_REST_PWR = 0d;


    //goBildaServer
    //Max PWM Range    500-2500μsec
    //Max PWM Range (Continuous)    900-2100µsec
    //(pulsewidth-500)/2000
    //PiranhaDog servo constants
    //private static double PIRANHADOG_RIGHT_OPEN = 0.55;
    //private static double PIRANHADOG_RIGHT_CLOSE= 0.45d; //(2100-500)/2000
    //private static double PIRANHADOG_LEFT_OPEN=0.45d; //(2100-500)/2000
    //private static double PIRANHADOG_LEFT_CLOSE = 0.55d; //(900-500)/2000
    // private static double TONGUE_INIT = .5d;
    // private static double TONGUE_IN = .6d; //.6
    // private static double TONGUE_OUT = .40d; //.3

    //HSRM9382TH  serv0
    //PWM range 800-2200μsec
    //(pulsewidth-500)/2000
    //
    //srvoJaw is this kind of servo
    //(800-500)/2000 = .15 MIN VALUE
    //(2200-500)/2000 = .85 MAX VALUE
    private static double TAIL_BETWEEN_LEGS = .493d;
    private static double TAIL_REST = .48d;
    private static double TAIL_FLICK = .32d; //.5  - very little,  .3 - too much, .07 - too much, .8 - other way
    public static int TAIL_INIT_AUTON = 0;
    public static int TAIL_INIT_TELEOP = 1;
    // private static double JAW_MEDIUM_BITE = .735d;
    // private static double JAW_SMALL_BITE = .725d;
    // private static double JAW_SPITA = .6d;
    // private static double JAW_SPITB = .4d;
    // private static double JAW_STOP = .8d;



    //private static Double STICK_DEAD_ZONE=.5;
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


    public void initialize(OpMode opMode,int nInitMode) {

        opMode.telemetry.addData(gstrClassName, "Initializing...");
        //opMode.telemetry.addData(gstrClassName, "    Body must be up");
        //opMode.telemetry.addData(gstrClassName, "    If not, Hit Stop, then re-Init");

        //PiranhaDog Motor

        // mtrTeeth = opMode.hardwareMap.get(DcMotor.class, "mtrTeeth");
        // mtrTeeth.setDirection(DcMotorSimple.Direction.REVERSE);
        // mtrTeeth.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // //mtrPiranhaDog.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        // mtrTeeth.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        // mtrTeeth.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // mtrTeeth.setPower(TEETH_REST_PWR);

        //PiranhaDog Servos
        // srvoPiranhaDogLeft = opMode.hardwareMap.get(Servo.class, "srvoPiranhaDogLeft");
        // srvoPiranhaDogLeft.setPosition(PIRANHADOG_LEFT_OPEN);
        // srvoPiranhaDogRight = opMode.hardwareMap.get(Servo.class, "srvoPiranhaDogRight");
        // srvoPiranhaDogRight.setPosition(PIRANHADOG_RIGHT_OPEN);

        srvoTail = opMode.hardwareMap.get(Servo.class, "srvoTail");
        if(nInitMode==TAIL_INIT_AUTON) {
            srvoTail.setPosition(TAIL_REST);
        } else {
            srvoTail.setPosition(TAIL_BETWEEN_LEGS);
        }





        // srvoThroatU = opMode.hardwareMap.get(Servo.class, "srvoThroatU");
        // srvoThroatU.setPosition(.5);

        // srvoThroatL = opMode.hardwareMap.get(Servo.class, "srvoThroatL");
        // srvoThroatL.setPosition(.5);

        // srvoPaw = opMode.hardwareMap.get(Servo.class, "srvoPaw");
        // srvoPaw.setPosition(.50); //.52 is slight squeeze NOTE!! pixel point rests against back plate, not pixel straight


        // srvoTongue = opMode.hardwareMap.get(Servo.class, "srvoTongue");
        // srvoTongue.setPosition(TONGUE_INIT);//.6


        opMode.telemetry.addData(gstrClassName, "    Initialized");


    }

    public void operate(OpMode opMode) {
        opMode.telemetry.addData("PiranhaTail","Tail:%.2f",
                srvoTail.getPosition());

        // move upper jaw
        if (opMode.gamepad2.dpad_up){
            srvoTail.setPosition(TAIL_FLICK);
            // mtrTeeth.setPower(TEETH_PUKE_PWR);
            return;
        }
        // else if (opMode.gamepad2.dpad_down){
        //     srvoTail.setPosition(JAW_SMALL_BITE);
        //   // mtrTeeth.setPower(TEETH_EAT_PWR);
        //     return;
        // }
        // else if (opMode.gamepad2.dpad_left){
        //     srvoJaw.setPosition(JAW_MEDIUM_BITE);
        //     mtrTeeth.setPower(TEETH_EAT_PWR);
        //     return;
        // }
        // else if (opMode.gamepad2.dpad_right){
        //     srvoJaw.setPosition(JAW_LARGE_BITE);
        //     mtrTeeth.setPower(TEETH_EAT_PWR);
        //     return;
        // }

        // srvoTongue.setPosition(TONGUE_IN);
        //intake

        if(opMode.gamepad2.left_bumper) {//intake pixel
            srvoTail.setPosition(TAIL_BETWEEN_LEGS);
            return;
        }
        // else if(opMode.gamepad2.left_trigger >= .5){//eject pixel
        //     mtrTeeth.setPower(TEETH_PUKE_PWR);
        //     return;
        // }
        // else {
        //     srvoJaw.setPosition(TAIL_REST);
        //     mtrTeeth.setPower(TEETH_REST_PWR);
        // }




    }

    public void autonFlickPixel(LinearOpMode linopMode,long lSpitMSec,long lDroolMSec) {


        srvoTail.setPosition(TAIL_FLICK);
        //linopMode.sleep(7000);
        linopMode.sleep(lSpitMSec);
        //mtrTeeth.setPower(TEETH_EAT_PWR + .4);
        // linopMode.sleep(1000);
        srvoTail.setPosition(TAIL_BETWEEN_LEGS);

        // mtrTeeth.setPower(TEETH_PUKE_PWR - .2);
        //linopMode.sleep(5000);
        //mtrTeeth.setPower(0);

        //srvoTongue.setPosition(TONGUE_OUT);
        //linopMode.sleep(500);
        // srvoTongue.setPosition(TONGUE_OUT);
        // srvoThroatU.setPosition(.21);
        // srvoThroatL.setPosition(.21);

        // srvoJaw.setPosition(JAW_SPITA);//eject slowly
        // linopMode.sleep(lSpitMSec);//pixel on matt
        // srvoJaw.setPosition(JAW_STOP);

        // srvoJaw.setPosition(JAW_SPITB);//retract jaw
        //linopMode.sleep(lDroolMSec);//pixel on matt
        // srvoJaw.setPosition(JAW_STOP);
        // srvoTongue.setPosition(TONGUE_IN);

        // srvoThroatU.setPosition(.5);
        // srvoThroatL.setPosition(.5);

    }


    public void shutdown(OpMode opMode) {
        return;
    }

    // public void openPaw(OpMode opMode) {
    //     srvoPaw.setPosition(.42); //.47

    //     return;
    // }
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
