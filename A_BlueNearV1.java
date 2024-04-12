package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleSwerveDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "drive")
public class A_BlueNearV1 extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(14.75, 62.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //left
        TrajectorySequence leftTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to spike mark
                .lineToLinearHeading(new Pose2d(17,43, Math.toRadians(-60)))
                .addTemporalMarker(() -> { // Can call other parts of the robot
                    piranhatail.autonFlickPixel(this,2200,100);
                })
                .waitSeconds(2.2) //let pixel drop on floor
                //back away from pixel
                .lineToLinearHeading(new Pose2d(17,55, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    freezeray.autonAimWeapon(this,.470d,0.530d); //left .472 right 524
                })
                .splineToLinearHeading(new Pose2d(50,43.5, Math.toRadians(0)), Math.toRadians(-30))
                .build();

        TrajectorySequence leftTraj2 = drive.trajectorySequenceBuilder(leftTraj1.end())
                .addTemporalMarkerOffset(.5, () -> {
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1.5)
                .back(6)
                .build();

        TrajectorySequence midTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to spikemark
                .lineToLinearHeading(new Pose2d(15, 34, Math.toRadians(-90)))
                .build();

        TrajectorySequence midTraj2 = drive.trajectorySequenceBuilder(midTraj1.end())
                //back away
                .strafeTo(new Vector2d(15, 37))
                .addTemporalMarker(() -> {
                    freezeray.autonRaiseWeapon(this);
                })
                //go to backdrop
                .lineToLinearHeading(new Pose2d(53,32, Math.toRadians(0)))
                .build();

        TrajectorySequence midTraj3 = drive.trajectorySequenceBuilder(midTraj2.end())
                .addTemporalMarker(() -> {
                    freezeray.autonAimWeapon(this, .470d, 0.530d); //left .472 right 524)
                })
                .addTemporalMarkerOffset(.5, () -> {
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1.5)
                .back(6)
                .build();

        TrajectorySequence rightTraj1 = drive.trajectorySequenceBuilder(startPose)
                //go to prop
                .strafeTo(new Vector2d(18.75, 58.5))
                .lineToLinearHeading(new Pose2d(11, 40, Math.toRadians(-179)))
                .addTemporalMarker(() -> { // Can call other parts of the robot
                    piranhatail.autonFlickPixel(this,2200,100);
                })
                .waitSeconds(2.2) //wait for pixel drop on floor
                .addTemporalMarkerOffset(.5, () -> {
                    freezeray.autonRaiseWeapon(this);
                })
                //go to backdrop
                .lineToLinearHeading(new Pose2d(54,31, Math.toRadians(0)))
                .build();

        TrajectorySequence rightTraj2 = drive.trajectorySequenceBuilder(rightTraj1.end())
                .addTemporalMarkerOffset(.5, () -> {
                    freezeray.autonShoot(this);
                })
                .waitSeconds(1.5)
                .back(6)
                .build();


        telemetry.addData(gstrClassName, "Initialized");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        nPropPos=goggles2.findProp(this,5000);

        telemetry.addData(gstrClassName, "Prop position:%d",nPropPos);
        telemetry.update();

        if (nPropPos == goggles2.PROP_NONE)
            nPropPos = goggles2.PROP_RIGHT;

        if (nPropPos == goggles2.PROP_LEFT) {
            drive.followTrajectorySequence(leftTraj1);
            drive.followTrajectory(buildCorrectionTraj(leftTraj1.end(), 10, 10));
            drive.followTrajectorySequence(leftTraj2);
            freezeray.autonMakeWeaponSafe(this);
        }
        else if (nPropPos == goggles2.PROP_MID) {
            drive.followTrajectorySequence(midTraj1);
            drive.followTrajectory(buildCorrectionTraj(midTraj1.end(), 10, 10));
            piranhatail.autonFlickPixel(this,2200,100);
            drive.followTrajectorySequence(midTraj2);
            drive.followTrajectory(buildCorrectionTraj(midTraj2.end(), 10, 10));
            drive.followTrajectorySequence(midTraj3);
            freezeray.autonMakeWeaponSafe(this);
        }
        else {
            drive.followTrajectorySequence(rightTraj1);
            drive.followTrajectory(buildCorrectionTraj(rightTraj1.end(), 10, 10));
            drive.followTrajectorySequence(rightTraj2);
            freezeray.autonMakeWeaponSafe(this);
        }

//        Trajectory moveToPark = drive.trajectoryBuilder(chosenTraj.end())
//             .strafeTo(new Vector2d(48, -60))
//                .build(); // traj instead of trajSeq for simplicity as this is building during autonomous
        //freezeray.autonShootPixel2(this,freezeray.RAY_POS_UNHOLSTER,0.472,0.528,0.59,2000,7000);
        //freezeray.autonShootPixel3(this,0.472,0.524,3000,10000);

//        drive.followTrajectory(moveToPark);
        //TODO: COMMENT OUT BELOW WHEN DONE!!
        Trajectory returnBack = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(startPose)
                .build();
        drive.followTrajectory(returnBack);
    }

    private Trajectory buildCorrectionTraj(Pose2d pose) {
        Trajectory correction = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(pose)
                .build();
        return correction;
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
                .lineToLinearHeading(pose,
                        SampleSwerveDrive.getVelocityConstraint(maxVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleSwerveDrive.getAccelerationConstraint(maxAccel))
                .build();
        return correction;
    }
}
