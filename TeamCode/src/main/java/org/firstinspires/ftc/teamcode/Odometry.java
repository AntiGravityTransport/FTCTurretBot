package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Odometry {
    public GoBildaPinpointDriver pinpoint;

    public Odometry(HardwareMap hardwareMap) {
        // Initialize the hardware using the name in your configuration
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
         * SETTINGS: Adjust these based on your physical robot build.
         * The offsets are the distance from the center of the robot to the pods.
         */
        pinpoint.setOffsets(-84.0, -168.0); // Example offsets in mm
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods. those_4_bar_pods);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }

    public Pose2D getPose() {
        return pinpoint.getPosition();
    }
    
    public String getStatus() {
        return pinpoint.getDeviceStatus().toString();
    }
}