package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private final VictorSPX mL = new VictorSPX(Constants.ML_MOTOR);
    private final VictorSPX mL2 = new VictorSPX(Constants.ML2_MOTOR);
    private final VictorSPX mR = new VictorSPX(Constants.MR_MOTOR);
    private final VictorSPX mR2 = new VictorSPX(Constants.MR2_MOTOR);

    private NetworkTableEntry ta, ty, tx;
    private NetworkTable table;

    public Robot() {
        initLime();
        initMotors();
    }

    public void initMotors(){
        double deadbeand = 0.04;

        mL.setInverted(false);
        mL2.setInverted(false);
        mR.setInverted(true);
        mR2.setInverted(true);

        mL.setNeutralMode(NeutralMode.Brake);
        mL2.setNeutralMode(NeutralMode.Brake);
        mR.setNeutralMode(NeutralMode.Brake);
        mR2.setNeutralMode(NeutralMode.Brake);

        mL.configNeutralDeadband(deadbeand);
        mL2.configNeutralDeadband(deadbeand);
        mR.configNeutralDeadband(deadbeand);
        mR2.configNeutralDeadband(deadbeand);

     }
    public void initLime(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    public double getX() {
        return tx.getDouble(0.0);
    }

    public double getY() {
        return ty.getDouble(0.0);
    }

    public double getArea() {
        return ta.getDouble(0.0);
    }

    public void updateLimeDashboard() {
        SmartDashboard.putNumber("Eixo X", getX());
        SmartDashboard.putNumber("Eixo Y", getY());
        SmartDashboard.putNumber("Área", getArea());
    }

    @Override
    public void autonomousInit() {

    }
    @Override
    public void autonomousPeriodic() {
        updateLimeDashboard();

        double txValue = getX();
        double taValue = getArea();

        if (txValue == 0.0 && getY() == 0.0 && taValue == 0.0) {
            stopMotors();
            return;
        }
        if (taValue < Constants.TA_MIN_THRESHOLD) {
            double forwardSpeed = Math.min((Constants.TA_MIN_THRESHOLD - taValue) / Constants.TA_MIN_THRESHOLD, 0.3);
            setMotors(forwardSpeed, forwardSpeed);

            double backwardSpeed = Math.min((taValue - Constants.TA_MAX_THRESHOLD) / (Constants.TA_MAX_THRESHOLD - Constants.TA_MIN_THRESHOLD), 0.3);
            setMotors(-backwardSpeed, -backwardSpeed);
        }
        else if (Math.abs(txValue) > Constants.TX_THRESHOLD) {
            if (txValue > 0) {
                moveRight();
            } else {
                moveLeft();
            }
        } else {
            stopMotors();
        }
    }
    public void setMotors(double lSpd, double rSpd) {
        mL.set(ControlMode.PercentOutput, lSpd);
        mL2.set(ControlMode.PercentOutput, lSpd);
        mR.set(ControlMode.PercentOutput, rSpd);
        mR2.set(ControlMode.PercentOutput, rSpd);
    }

    public void moveLeft() {
        setMotors(-Constants.SPD_ADJUST , Constants.SPD_ADJUST );
    }

    public void moveRight() {
        setMotors(Constants.SPD_ADJUST, -Constants.SPD_ADJUST );
    }

    public void stopMotors() {
        setMotors(0, 0);
    }
}