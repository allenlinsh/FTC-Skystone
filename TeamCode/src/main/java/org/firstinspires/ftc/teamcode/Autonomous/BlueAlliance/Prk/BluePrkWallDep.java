package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.Prk;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

public class BluePrkWallDep extends MainAutonomous {
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
         * Skystone:                No
         * Foundation:              No
         * Parking:                 Yes
         * Parking Position:        Wall
         * Start Position:          Depot
         */

        if (opModeIsActive()) {
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            // Parking
            encoderDrive("front", minPower, 0.25);
            gyroTurn(90, minTurnPower);
            encoderDrive("right", minPower, 0.25);
            encoderDrive("back", minPower, 1.625);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
