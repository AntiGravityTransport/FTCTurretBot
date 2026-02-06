package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Odometry {
    public GoBildaPinpointDriver pinpoint;

    public Odometry(HardwareMap hardwareMap) {
        // Initialize the hardware using the name in your configuration
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
         * SETTINGS: Adjust these based on your physical robot build.
         * The offsets are the distance from the center of the robot to the pods (in mm).
         */
        pinpoint.setOffsets(-84.0, -168.0);

        // Revised resolution setting to match the standard driver's naming convention
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }
    // Inside Odometry.java
    public String getStatus() {
        // Return the status from your pinpoint driver or tracking system
        return "OK";
    }
    public Pose2D getPose() {
        return pinpoint.getPosition();
    }
}