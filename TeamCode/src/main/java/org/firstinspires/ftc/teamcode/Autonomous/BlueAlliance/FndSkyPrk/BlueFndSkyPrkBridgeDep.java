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
        update();

        waitForStart();
        print("Status", "Running");
        update();

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
            encoderDriveSmooth("left", 0.25);
            encoderDriveSmooth("front", 0.5);
            recognizeTarget("Stone Target");
            playSound("ss_roger_roger");
            // 1st skystone
            encoderDriveSmoothDist("right", travelX);

            armExtend();
            armRaise(armDuration);
            gripRelease(gripDuration);
            encoderDriveSmoothDist("front", travelY, minPower);
            armDrop(armDuration);
            gripHold(gripDuration);
            armRaise(armDuration);
            encoderDriveSmoothDist("back", travelY-0.5*inPerBlock);

            rotate(-90, turnPower);
            encoderDriveSmooth("front", 3);
            rotate(90, turnPower);
            encoderDriveSmoothDist("left", (travelX + firstPlacement));
            buildSkystone(minPower, 1);
            //encoderDriveSmoothDist("right", (travelX + firstPlacement));
            // 2nd skystone
            /*
            buildingSiteToDepot("blue", minPower, minTurnPower, 4);
            grabSkystone(minPower);
            depotToBuildingSite("blue", minPower, minTurnPower, 4);
            encoderDriveSmoothDist("left", minPower, (travelX + secondPlacement));
            buildSkystone(minPower, 1);
            */
            // Foundation
            //encoderDriveSmoothDist("left", (centerPlacement - secondPlacement));
            encoderDriveSmoothDist("left", (centerPlacement - firstPlacement));
            encoderDriveSmoothDist("back", travelY-0.25*inPerBlock);
            //grabFoundation("blue");
            // Parking
            encoderDriveSmooth("left", 0.5);
            armExtend();
            encoderDriveSmooth("front", 1.625);
            timeDrive("left", minPower, 500);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
