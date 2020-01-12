package org.firstinspires.ftc.teamcode.Autonomous.BlueAlliance.Prk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.MainAutonomous;

@Autonomous(name = "BluePrkBridgeBuild", group = "Prk")
public class BluePrkBridgeBuild extends MainAutonomous {
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
         * Parking Position:        Bridge
         * Start Position:          Building Site
         */

        if (opModeIsActive()) {
            runtime.reset();
            runtime.startTime();
            resetAngle();
            while (runtime.milliseconds() < delayTime) {}
            // Parking
            encoderDrive("front", minPower, 0.25);
            gyroTurn(90, minTurnPower);
            armExtend();
            encoderDrive("right", minPower, 0.25);
            encoderDrive("left", minPower, 1);
            encoderDrive("front", minPower, 1.625);
            encoderDrive("left", minPower, 0.25);
        }
        stopAllMotors();
        visionTargets.deactivate();
    }
}
