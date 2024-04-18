package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@Autonomous(name="Blue Far V1",group = "AAA")
public class A_BlueFarV1 extends LinearOpMode {

    private SampleSwerveDrive drive;
    private Goggles2V3AS goggles2 = new Goggles2V3AS();
    private PiranhaDogV4AS piranhadog = new PiranhaDogV4AS();
    private PiranhaTailAS piranhatail = new PiranhaTailAS();
    private FreezeRay4BarV1AS freezeray = new FreezeRay4BarV1AS();
    private String gstrClassName=this.getClass().getSimpleName();

    @Override
    public void runOpMode() {
        int nTagToFind=-1;
        int nPropPos=0;
        int chosenTraj;

        drive = new SampleSwerveDrive(hardwareMap);

        goggles2.initialize(this,goggles2.BLUE_CAM);//Red is 'Webcam 1

        piranhadog.initialize(this);
        freezeray.initialize(this);
        piranhatail.initialize(this,piranhatail.TAIL_INIT_AUTON);

        //establishes starting coordinates on the field
        Pose2d startPose = new Pose2d(-36.75, 62.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        ///////////////////LEFT//////////////////
        TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .addTemporalMarker(1.5, () -> { // Can call other parts of the robot
                    piranhatail.autonSetFlickPixel(this, PiranhaTailAS.TAIL_HFLICK);
                })
                .lineToLinearHeading(new Pose2d(-45, 32, Math.toRadians(0))) //-40
                .lineToLinearHeading(new Pose2d(-36.5, 32, Math.toRadians(0))) //38.5, 34
                .build();

        TrajectorySequence leftTraj2 = drive.trajectorySequenceBuilder(leftTraj1.end())
                .addTemporalMarker(() -> { // Can call other parts of the robot
                    piranhatail.autonSetFlickPixel(this, PiranhaTailAS.TAIL_FLICK);
                    sleep(1000);
                    piranhatail.autonSetFlickPixel(this, PiranhaTailAS.TAIL_HFLICK);
                })
                .waitSeconds(1)
                //go back out
                .lineToLinearHeading(new Pose2d(-44, 32, Math.toRadians(0)))
                //store tail while moving away
                .lineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(0)))
                .addTemporalMarker(() -> { // Can call other parts of the robot
                    piranhatail.autonSetFlickPixel(this, PiranhaTailAS.TAIL_BETWEEN_LEGS);
                })
                .waitSeconds(0.5)
                //go past truss
                .splineToLinearHeading(new Pose2d(7, 58, 0), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    // raise 4bar
                    freezeray.autonRaiseWeaponHeight(this,1200);
                })
                //go to backdrop
                .splineToLinearHeading(new Pose2d(51, 43, 0), Math.toRadians(-30))
                .build();

        TrajectorySequence leftTraj3 = drive.trajectorySequenceBuilder(leftTraj2.end())
                //extend bipod
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1200);
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                //release pixel
                .addTemporalMarker(.5, () -> { // Can call other parts of the robot
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1.5)
                //back away
                .lineToLinearHeading(new Pose2d(45, 45, Math.toRadians(0)))
                .build();


        //////////////MID//////////////////////////
        TrajectorySequence midTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .lineToLinearHeading(new Pose2d(-42, 34, Math.toRadians(-90))) //make Y 35??
                .build();

        TrajectorySequence midTraj2 = drive.trajectorySequenceBuilder(midTraj1.end())
                //go to wall
                .lineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(0)))
                //go through truss
                .splineToLinearHeading(new Pose2d(7, 58, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    // raise 4bar
                    freezeray.autonRaiseWeaponHeight(this,1200);
                })
                //go to backdrop
                .splineToLinearHeading(new Pose2d(51, 38, Math.toRadians(0)), Math.toRadians(-30))
                .build();

        TrajectorySequence midTraj3 = drive.trajectorySequenceBuilder(midTraj2.end())
                //extend bipod
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1200);
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                .addTemporalMarker(.5, () -> { // Can call other parts of the robot
                    //release pixel
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1.5)
                //back away
                .back(6)
                .build();



        ///////////////////RIGHT//////////////////
        TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .lineToLinearHeading(new Pose2d(-51, 45, Math.toRadians(-90)))
                .build();

        TrajectorySequence rightTraj2 = drive.trajectorySequenceBuilder(rightTraj1.end())
                //go to wall
                .lineToLinearHeading(new Pose2d(-42, 58, Math.toRadians(0)))
                //go past truss
                .splineToLinearHeading(new Pose2d(7, 58, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    // raise 4bar
                    freezeray.autonRaiseWeaponHeight(this,1200);
                })
                //go to backdrop
                .splineToLinearHeading(new Pose2d(51, 33, Math.toRadians(0)), Math.toRadians(-30))
                .build();

        TrajectorySequence rightTraj3 = drive.trajectorySequenceBuilder(rightTraj2.end())
                //extend bipod
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeaponHeight(this,1200);
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                //release pixel
                .addTemporalMarkerOffset(.5, () -> { // Can call other parts of the robot
                    freezeray.autonShoot(this);
                })
                .waitSeconds(2)
                //back away
                .lineToLinearHeading(new Pose2d(45, 32, Math.toRadians(0)))
                .build();



        telemetry.addData(gstrClassName, "Initialized");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        nPropPos=goggles2.findProp(this,5000);

        telemetry.addData(gstrClassName, "Prop position:%d",nPropPos);
        telemetry.update();

        if (nPropPos == goggles2.PROP_NONE) //pos is -1
            nPropPos = goggles2.PROP_RIGHT;

        if (nPropPos == goggles2.PROP_LEFT) {
            //get to spikemark and drop pixel
            drive.followTrajectorySequence(leftTraj1);
            drive.followTrajectory(buildCorrectionTraj(leftTraj1.end(), 10, 10));
            drive.followTrajectorySequence(leftTraj2);
            drive.followTrajectory(buildCorrectionTraj(leftTraj2.end(), 10, 10));
            drive.followTrajectorySequence(leftTraj3);
            freezeray.autonMakeWeaponSafe(this);
        }
        else if (nPropPos == goggles2.PROP_MID) {
            //go to line
            drive.followTrajectorySequence(midTraj1);
            drive.followTrajectory(buildCorrectionTraj(midTraj1.end(), 10, 10));
            //drop pixel
            piranhatail.autonFlickPixel(this,2200,100);
            //go to backdrop
            drive.followTrajectorySequence(midTraj2);
            drive.followTrajectory(buildCorrectionTraj(midTraj2.end(), 10, 10));
            //push into wall
            drive.followTrajectorySequence(midTraj3);
            freezeray.autonMakeWeaponSafe(this);
        }
        else { //PROP_RIGHT
            //go to line
            drive.followTrajectorySequence(rightTraj1);
            drive.followTrajectory(buildCorrectionTraj(rightTraj1.end(), 10, 10));
            //drop pixel
            piranhatail.autonFlickPixel(this,2200,100);
            //get to wall and go to backdrop
            drive.followTrajectorySequence(rightTraj2);
            drive.followTrajectory(buildCorrectionTraj(rightTraj2.end(), 10, 10));
            //push into wall
            drive.followTrajectorySequence(rightTraj3);
            freezeray.autonMakeWeaponSafe(this);

        }

        Trajectory moveToPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(48, 12))
                .build(); // traj instead of trajSeq for simplicity as this is building during autonomous
        drive.followTrajectory(moveToPark);

        //TODO: COMMENT OUT BELOW WHEN DONE!!
//        TrajectorySequence returnBack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                //go to front of truss
//                .lineToLinearHeading(new Pose2d(10.0, 58.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
//                //go to back of truss
//
//                .lineToLinearHeading(new Pose2d(-42.0, 58.5, Math.toRadians(0))) //x:18-48(two tiles)-8 (other side of prop)
//                //go past truss
//                .lineToLinearHeading(startPose)
//                .build();
//        drive.followTrajectorySequence(returnBack);
    }

    /**
     * Creates a trajectory that strafes from current estimated position to target position
     * @param pose
     * @param maxVel
     * @param maxAccel
     * @return
     */
    private Trajectory buildCorrectionTraj(Pose2d pose, double maxVel, double maxAccel) {
        Trajectory correction = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(pose,
                        SampleSwerveDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(maxAccel))
                .build();
        return correction;
    }

    /**
     * Creates a trajectory that turns to the correct heading, then strafes to the correct position
     * @param pose
     * @param maxVel
     * @param maxAccel
     * @return Built trajectory
     */
    private TrajectorySequence buildCorrectionTraj2(Pose2d pose, double maxVel, double maxAccel) {
        TrajectorySequence correction = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                // Turn to correct
                .turn((pose.getHeading()-drive.getPoseEstimate().getHeading()+3*Math.PI)%(2*Math.PI)-Math.PI) // Clamp between -π & π
                // Strafe to correct
                .lineToLinearHeading(pose,
                        SampleSwerveDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(maxAccel))
                .build();
        return correction;
    }
}
