package org.firstinspires.ftc.teamcode.Autonomous.RedAlliance.FndPrk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "RedFndPrkBridgeBuild", group = "FndPrk")
public class RedFndPrkBridgeBuild extends MainAutonomous {
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
            encoderDriveSmooth("front", 1);
            encoderDriveSmoothDist("right", centerPlacement);
            grabFoundation("red");
            armExtend();
            // Parking
            encoderDriveSmooth("left", 1.5, minPower);
            encoderDriveSmooth("right", 1);
            encoderDriveSmooth("front", 1.625, drivePower/2.0);
            timeDrive("right", minPower, 500);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
