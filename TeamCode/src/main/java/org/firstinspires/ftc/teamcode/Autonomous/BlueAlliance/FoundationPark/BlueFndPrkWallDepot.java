package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.FoundationPark;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

public class BlueFndPrkWallDepot extends MainAutonomous {
    private String className = this.getClass().getSimpleName();
    @Override
    public void runOpMode() {
        // Initialize autonomous route
        getPreferences();
        checkPreferences();
        print("Alliance", teamColor);
        print("Foundation", doFoundation);
        print("Skystone", doSkystone);
        print("Parking Position", parking);
        print("Starting Position", starting);

        // Initialize hardware
        getHardwareMap();
        initCheck();
        print("Status", "Initialized");
        getPreferences();
        telemetry.update();

        waitForStart();
        print("Status", "Running");
        telemetry.update();

        /* Autonomous
         * Team Alliance:           Blue
         * Skystone:                No
         * Foundation:              Yes
         * Parking:                 Yes
         * Parking Position:        Wall
         * Start Position:          Depot
         */
        if (opModeIsActive()) {
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            // Transition
            depotToBuildingsite("blue", minPower, minTurnPower);
            //Grab foundation
            encoderDrive("front", minPower, 1);
            encoderDrive("left", minPower, 0.5);
            grabFoundation("blue");
            // Parking
            encoderDrive("front", minPower, 1.875);
        }

        resetMotor();
        visionTargets.deactivate();
    }
}
