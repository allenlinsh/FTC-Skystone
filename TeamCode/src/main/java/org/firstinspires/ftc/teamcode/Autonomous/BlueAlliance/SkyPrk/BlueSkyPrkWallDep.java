package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.SkyPrk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BlueSkyPrkWallDep", group = "SkyPrk")
public class BlueSkyPrkWallDep extends MainAutonomous {
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
         * Skystone:                Yes
         * Foundation:              No
         * Parking:                 Yes
         * Parking Position:        Wall
         * Start Position:          Building Site
         */

        if (opModeIsActive()) {
            runtime.reset();
            runtime.startTime();
            resetAngle();
            while (runtime.milliseconds() < delayTime) {}
            // Recognize skystone
            playSound("ss_power_up");
            encoderDrive("left", minPower, 0.25);
            encoderDrive("front", minPower, 0.5);
            recognizeTarget("Stone Target");
            // 1st skystone
            encoderDrive("front", minPower, 0.75);
            gyroTurn(90, minTurnPower);
            encoderDrive("front", minPower, 1.25);
            sideGrabSkystone("blue");
            playSound("ss_roger_roger");
            encoderDrive("right", minPower, 1);
            encoderDriveDist("front", minPower, travelX);
            encoderDrive("back", minPower, 3);
            // 2nd skystone
            encoderDrive("front", minPower, 2);
            encoderDrive("left", minPower, 1);
            sideGrabSkystone("blue");
            playSound("ss_roger_roger");
            encoderDrive("right", minPower, 1);
            encoderDriveDist("front", minPower, travelX);
            encoderDrive("back", minPower, 2);
            // Parking
            encoderDrive("right", minPower, 1);
            encoderDrive("front", minPower, 1);
            armExtend();
            encoderDrive("back", minPower, 0.625);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
