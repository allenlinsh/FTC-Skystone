package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.FndSkyPrk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BlueFndSkyPrkBridgeDep", group = "FndSkyPrk")
public class BlueFndSkyPrkBridgeDep extends MainAutonomous {
    public String className = getClass().getSimpleName();
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
         * Foundation:              Yes
         * Parking:                 Yes
         * Parking Position:        Bridge
         * Start Position:          Depot
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
            playSound("ss_roger_roger");
            // 1st skystone
            encoderDriveDist("right", minPower, travelX);
            grabSkystone(minPower);
            depotToBuildingSite("blue", minPower, minTurnPower, 3);
            //
            gyroCorrection();
            //
            encoderDriveDist("left", minPower, (travelX + firstPlacement));
            buildSkystone(minPower, 1);
            // 2nd skystone
            encoderDriveDist("right", minPower, (travelX + firstPlacement));
            //
            gyroCorrection();
            //
            buildingSiteToDepot("blue", minPower, minTurnPower, 4);
            grabSkystone(minPower);
            depotToBuildingSite("blue", minPower, minTurnPower, 4);
            encoderDriveDist("left", minPower, (travelX + secondPlacement));
            buildSkystone(minPower, 1);
            // Foundation
            encoderDriveDist("left", minPower, (centerPlacement - secondPlacement));
            grabFoundation("blue");
            encoderDrive("right", minPower, 0.5);
            armExtend();
            // Parking
            encoderDrive("left", minPower, 1);
            encoderDrive("front", minPower, 1.625);
            encoderDrive("left", minPower, 0.25);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
