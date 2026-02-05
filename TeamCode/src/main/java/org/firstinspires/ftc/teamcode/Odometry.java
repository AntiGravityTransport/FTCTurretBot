package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

public class Odometry {

    // The actual hardware device
    private I2cDeviceSynch deviceClient;

    // Position Data
    private double xPos = 0;
    private double yPos = 0;
    private double heading = 0;

    public Odometry(HardwareMap hardwareMap, String name) {
        // "odo" should match the name in your Robot Configuration on the Driver Station
        deviceClient = hardwareMap.get(I2cDeviceSynch.class, name);
        deviceClient.setI2cAddress(I2cAddr.create7bit(0x08)); // Default Pinpoint Address
        deviceClient.engage();

        resetHeading();
    }

    public void update() {
        // In a real scenario, we would read the I2C registers here.
        // For the Pinpoint, it updates at 1.5kHz internally.
        // We pull the latest X, Y, and H values.
        // NOTE: This assumes you have configured the Pinpoint via the goBILDA tool
        // or set the offsets in this class.
    }

    public void resetHeading() {
        // Sends command to Pinpoint to recalibrate IMU and zero heading
        deviceClient.write8(0x06, 1);
    }

    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.MM, xPos, yPos, AngleUnit.DEGREES, heading);
    }

    public String getTelemetryData() {
        return String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.2f", xPos, yPos, heading);
    }
}