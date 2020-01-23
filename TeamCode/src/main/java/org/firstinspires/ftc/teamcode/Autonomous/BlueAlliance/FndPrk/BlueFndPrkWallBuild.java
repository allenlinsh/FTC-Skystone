package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.FndPrk;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BlueFndPrkWallBuild", group = "FndPrk")
public class BlueFndPrkWallBuild extends MainAutonomous {
    private String className = getClass().getSimpleName();
    @Override
    public void runOpMode() {
        // Initialize autonomous route
        getPreferences();
        checkPreferences(className);

        // Initialize hardware
        getHardwareMap();
        initCheck();
        print("Status", "Initialized");
        print("Alliance", teamColor);
        print("Foundation", doFoundation);
        print("Skystone", doSkystone);
        print("Parking Position", parking);
        print("Starting Position", starting);
        telemetry.update();

        if ("blue".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        if ("red".equals(teamColor)) pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        led.setPattern(pattern);

        waitForStart();
        print("Status", "Running");
        telemetry.update();

        /* Autonomous
         * Team Alliance:           Blue
         * Skystone:                No
         * Foundation:              Yes
         * Parking:                 Yes
         * Parking Position:        Wall
         * Start Position:          Building Site
         */

        if (opModeIsActive()) {
            runtime.reset();
            runtime.startTime();
            resetAngle();
            while (runtime.milliseconds() < delayTime) {}
            // Foundation
            gripRelease(gripDuration/2);
            playSound("ss_power_up");
            encoderDriveSmoothDist("right", centerPlacement);
            encoderDriveSmooth("back", 1.5 + movementError);
            hookOn();
            encoderDriveSmooth("front", 0.75 + movementError, minPower);
            curve(-90, turnPower);
            encoderDriveSmooth("back", 0.75, minPower);
            hookOff();
            // Parking
            encoderDriveSmoothDist("right", 0.75*inPerBlock-centerPlacement);
            armExtend();
            encoderDriveSmooth("front", 1.625 + 0.25);
            encoderDriveSmooth("right", 0.25, minPower);

            stopAllMotors();
            visionTargets.deactivate();
        }
    }
}
