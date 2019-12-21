package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.FoundationPark;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

public class BlueFndPrkBridgeBuild extends MainAutonomous {
    private String className = this.getClass().getSimpleName();
    @Override
    public void runOpMode() {
        // Initialize autonomous route
        getPreferences();
        checkPreferences();

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
            runtime.startTime();
            while (runtime.milliseconds() < delayTime) {}
            // Transition
            encoderDrive("front", minPower, 1);
            encoderDrive("left", minPower, 0.5);
            //Grab foundation
            grabFoundation("blue");
            // Parking
            encoderDrive("left", minPower, 0.5);
            encoderDrive("front", minPower, 1.875);
        }

        resetMotor();
        visionTargets.deactivate();
    }
}
