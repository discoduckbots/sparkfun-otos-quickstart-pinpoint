package org.firstinspires.ftc.teamcode.opmode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.os.Environment;
import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.hardware.Grabber;
import org.firstinspires.ftc.teamcode.hardware.HardwareStore;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Gen Autonomous Teleop", group="Linear Opmode")
public class GenerateAutonomousTeleop extends LinearOpMode {

    private PinpointDrive sampleMecanumDrive = null;

    private static double THROTTLE = 0.5;
    private static double STRAFE_THROTTLE = 0.5;
    private static double TURN_THROTTLE = 0.7;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);
        sampleMecanumDrive = new PinpointDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        while (opModeIsActive()) {

            /* Gamepad 1 */

            sampleMecanumDrive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * THROTTLE,
                            -gamepad1.left_stick_x * THROTTLE
                    ),
                    -gamepad1.right_stick_x * TURN_THROTTLE
            ));

            sampleMecanumDrive.updatePoseEstimate(); //?

            Pose2d poseEstimate = sampleMecanumDrive.getPose();
            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading", poseEstimate.heading.real);
            telemetry.update();


            if (gamepad1.left_bumper) {
                if(!leftBumperPressed){
                    leftBumperPressed = true;
                    addAutonomousPoint(sampleMecanumDrive);
                }
            } else{
                leftBumperPressed = false;
            }

            if (gamepad1.right_bumper) {
                if(!rightBumperPressed) {
                    rightBumperPressed = true;
                    completeAutonomousPath(sampleMecanumDrive);
                }
            }
            else{
                rightBumperPressed = false;
            }
        }
    }
    public ArrayList<ACGPose2d> arrayList = new ArrayList<ACGPose2d>();

    private TrajectoryActionBuilder buildSimpleTrajectory (TrajectoryActionBuilder start, double x, double y, double heading) {
        return start.fresh()
                .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(heading));
    }

    private void completeAutonomousPath(PinpointDrive drive) {

        List<TrajectoryActionBuilder> trajectoryList = new ArrayList<TrajectoryActionBuilder>();

        TrajectoryActionBuilder firstTrajectory = drive.actionBuilder(arrayList.get(0).toPose2d())
                .strafeToLinearHeading(arrayList.get(1).toVector2d(), arrayList.get(1).heading); //strafe to score preload
        trajectoryList.add(firstTrajectory);


        for (int i=2; i<arrayList.size(); i++){
            ACGPose2d pose2d = arrayList.get(i);
            TrajectoryActionBuilder previousTrajectory = trajectoryList.get(trajectoryList.size()-1);
            trajectoryList.add(buildSimpleTrajectory(previousTrajectory, pose2d.x, pose2d.y, pose2d.heading));
        }

        for(TrajectoryActionBuilder trajectory: trajectoryList){
            trajectory.build();
        }

        try {
            printStatement(arrayList);


        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void printStatement(ArrayList<ACGPose2d> arrayList) throws IOException {
        File path =Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        File file = new File(path, "GeneratedAutonomous.java");
        FileWriter writer = new FileWriter(file);
        try {
            String AUTONOMOUS = "package org.firstinspires.ftc.teamcode.opmode;\n" +
                    "\n" +
                    "import com.acmerobotics.dashboard.config.Config;\n" +
                    "import com.acmerobotics.roadrunner.ParallelAction;\n" +
                    "import com.acmerobotics.roadrunner.Pose2d;\n" +
                    "import com.acmerobotics.roadrunner.ProfileAccelConstraint;\n" +
                    "import com.acmerobotics.roadrunner.SequentialAction;\n" +
                    "import com.acmerobotics.roadrunner.SleepAction;\n" +
                    "import com.acmerobotics.roadrunner.TrajectoryActionBuilder;\n" +
                    "import com.acmerobotics.roadrunner.TranslationalVelConstraint;\n" +
                    "import com.acmerobotics.roadrunner.Vector2d;\n" +
                    "import com.acmerobotics.roadrunner.ftc.Actions;\n" +
                    "import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n" +
                    "\n" +
                    "import org.firstinspires.ftc.teamcode.MecanumDrive;\n" +
                    "import org.firstinspires.ftc.teamcode.hardware.Arm;\n" +
                    "import org.firstinspires.ftc.teamcode.hardware.Grabber;\n" +
                    "import org.firstinspires.ftc.teamcode.hardware.HardwareStore;\n" +
                    "\n" +
                    "//@Disabled\n" +
                    "@Autonomous(name=\"Generated Autonomous\", group=\"Robot\")\n" +
                    "public class GeneratedAutonomous extends DuckbotAuto{\n" +
                    "\n" +
                    "    private ElapsedTime runtime = new ElapsedTime();\n" +
                    "    \n" +
                    "    private static final double AUTONOMOUS_SPEED = 0.5;\n" +
                    "\n" +
                    "    private MecanumDrive drive = null;\n" +
                    "    private Grabber teleGrabber = null;\n" +
                    "\n" +
                    "    @Override\n" +
                    "    public void runOpMode() {\n" +
                    "       HardwareStore hardwareStore = new HardwareStore(hardwareMap, telemetry, this);\n" +
                    "       DuckbotAuto.AutoArm arm = new DuckbotAuto.AutoArm(hardwareStore);\n" +
                    "       DuckbotAuto.AutoIntake intake = new DuckbotAuto.AutoIntake(hardwareStore);\n" +
                    "       DuckbotAuto.AutoGrabber grabber = new DuckbotAuto.AutoGrabber(hardwareStore);\n" +
                    "\n" +
                    "       drive = hardwareStore.getDrive();\n"+
                    "       teleGrabber = hardwareStore.getGrabber();\n"+
                    "\n" +
                    "       telemetry.addData(\">\", \"Press Play to start op mode\");\n"+
                    "       telemetry.update();\n"+
                    "       teleGrabber.closeGrabber();\n"+
                    "\n";

            String AUTONOMOUS_PART2 = "        waitForStart();\n" +
                    "\n" +
                    "        if (opModeIsActive()) {\n";
            String AUTONOMOUS_PART3 = "       }\n" +
                    "    }\n" +
                    "}\n";
            writer.append(AUTONOMOUS);
            writer.append(AUTONOMOUS_PART2);


            TrajectoryActionBuilder firstTrajectory = sampleMecanumDrive.actionBuilder(arrayList.get(0).toPose2d())
                    .strafeToLinearHeading(arrayList.get(1).toVector2d(), arrayList.get(1).heading); //strafe to score preload

            String TRAJ_CODE =
                    "Pose2d firstPose = new Pose2d(0,0,0);\n"+
                            "Vector2d secondVector = new Vector2d(" + arrayList.get(1).x + ", " + arrayList.get(1).y + ");\n" +

                    "TrajectoryActionBuilder traj1 = drive.actionBuilder(firstPose)\n"+
                    "       .strafeToLinearHeading(secondVector," + arrayList.get(1).heading + ").build();\n";
            TRAJ_CODE+="\n";
            for (int i=2; i<arrayList.size(); i++){
                ACGPose2d pose2d = arrayList.get(i);
                TRAJ_CODE += "TrajectoryActionBuilder traj" + i + " = buildSimpleTrajectory(traj" + (i-1) + "," + pose2d.x + "," + pose2d.y + "," + pose2d.heading + ");\n";
                TRAJ_CODE += "traj" + i + ".build();\n";
            }

            writer.append(AUTONOMOUS_PART3);
        } catch (Exception e) {
            Log.d("AUT", ":exception writing");
        }
        finally {
            try {
                writer.flush();
                writer.close();
            } catch (IOException e) {
                Log.d("AUT", ":exception writing");
                e.printStackTrace();
            }
        }
    }
    private void addAutonomousPoint( PinpointDrive drive  ) {

        if (arrayList.isEmpty()){
            arrayList.add(new ACGPose2d(0,0,0));
        }

        Pose2d poseEstimate = drive.getPose();

        ACGPose2d end = new ACGPose2d(poseEstimate.position.x, poseEstimate.position.y, poseEstimate.heading.real);

        arrayList.add(end);
    }

    private class ACGPose2d {
        public double x;
        public double y;
        public double heading;

        public ACGPose2d(double x, double y, double heading){
            this.x = x;
            this.y=y;
            this.heading=heading;
        }

        public Pose2d toPose2d(){
            return new Pose2d(x, y, heading);
        }

        public Vector2d toVector2d(){
            return new Vector2d(x, y);
        }
    }
}