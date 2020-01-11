package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.SkyPrk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BlueSkyPrkBridgeDep", group = "SkyPrk")
public class BlueSkyPrkBridgeDep extends MainAutonomous {
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
         * Parking Position:        Bridge
         * Start Position:          Depot
         */

        if (opModeIsActive()) {
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            // Recognize skystone
            playSound("ss_power_up");
            encoderDrive("left", minPower, 0.25);
            encoderDrive("front", minPower, 0.25);
            recognizeTarget("Stone Target");
            playSound("ss_roger_roger");
            // 1st skystone
            encoderDrive("front", minPower, 0.75);
            gyroTurn(90, minTurnPower);
            encoderDrive("front", minPower, 1.25);
            sideGrabSkystone("blue");
            encoderDriveDist("front", minPower, travelX);
            encoderDrive("back", minPower, 3);
            // 2nd skystone
            encoderDrive("front", minPower, 2);
            sideGrabSkystone("blue");
            encoderDriveDist("front", minPower, travelX);
            encoderDrive("back", minPower, 2);
            // Parking
            encoderDrive("front", minPower, 1);
            armExtend();
            encoderDrive("back", minPower, 0.625);
            encoderDrive("left", minPower, 0.25);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
