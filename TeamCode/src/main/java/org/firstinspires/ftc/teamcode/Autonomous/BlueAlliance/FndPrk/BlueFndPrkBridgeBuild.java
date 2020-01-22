package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.FndPrk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BlueFndPrkBridgeBuild", group = "FndPrk")
public class BlueFndPrkBridgeBuild extends MainAutonomous {
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

        waitForStart();
        print("Status", "Running");
        telemetry.update();

        /* Autonomous
         * Team Alliance:           Blue
         * Skystone:                No
         * Foundation:              Yes
         * Parking:                 Yes
         * Parking Position:        Bridge
         * Start Position:          Building Site
         */

        if (opModeIsActive()) {
            runtime.reset();
            runtime.startTime();
            resetAngle();
            while (runtime.milliseconds() < delayTime) {}
            // Foundation
            playSound("ss_power_up");
            encoderDriveSmooth("back", 1.5);
            encoderDriveSmoothDist("right", centerPlacement);
            hookOn();
            encoderDriveSmooth("front", 0.5);
            curve(-90, turnPower);
            encoderDriveSmooth("back", 0.5);
            hookOff();
            // Parking
            armExtend();
            encoderDriveSmoothDist("left", centerPlacement);
            encoderDriveSmooth("front", 1.625);
            encoderDriveSmooth("left", 0.25, minPower);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
