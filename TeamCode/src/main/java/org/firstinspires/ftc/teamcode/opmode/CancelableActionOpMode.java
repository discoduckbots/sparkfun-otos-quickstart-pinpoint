package org.firstinspires.ftc.teamcode.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class CancelableFollowTrajectoryAction implements Action {
    private final MecanumDrive.FollowTrajectoryAction action;
    private boolean cancelled = false;

    public CancelableFollowTrajectoryAction(TimeTrajectory t) {
        action = new FollowTrajectoryAction(t);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (cancelled) {
            setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return false;
        }

        return action.run(telemetryPacket);
    }

    public void cancelAbruptly() {
        cancelled = true;
    }


}