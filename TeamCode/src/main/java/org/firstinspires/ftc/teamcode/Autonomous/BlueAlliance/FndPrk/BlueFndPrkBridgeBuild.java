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
            encoderDriveSmooth("front", 1);
            encoderDriveSmoothDist("left", centerPlacement);
            grabFoundation("blue");
            armExtend();
            // Parking
            timeDrive("right", minPower, 1000);
            encoderDriveSmooth("left", 1);
            encoderDriveSmooth("front", 1.625);
            timeDrive("left", minPower, 500);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
