package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.apriltag.AprilTagDetection;

public class TurretMechanismTutorial {
    private DcMotorEx turret;
    private CRServo right_rotator = null;
    private CRServo left_rotator = null;
    private CRServo right_turret_angler = null;
    private CRServo left_turret_angler = null;

    private enum AnglerState {
        IDLE,
        UP,
        DOWN;
    }

    private TurretMechanismTutorial.AnglerState anglerSystemState = TurretMechanismTutorial.AnglerState.IDLE;


    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double MAX_POWER = 0.6;
    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap){
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setkP(double newKP){
        kP = newKP;
    }

    public double getkP(){
        return kP;
    }
    public void setkD(double newKD){
        kD = newKD;
    }

    public double getkD(){
        return kD;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void update(AprilTagDetection curID){
        double deltaTime = timer.seconds();
        timer.reset();

        if(curID == null){
            turret.setPower(0);
            lastError = 0;
            return;
        }

        //-------- start PD controller --------

        double error = goalX - curID.pose.x;
        double pTerm = error * kP;

        double dTerm = 0;
        if(deltaTime > 0){
            dTerm = ((error - lastError) /deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance){
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        // safety encode check OR magnetic limit switch check, OR other safeties

        turret.setPower(power);
        lastError = error;
    }

}
