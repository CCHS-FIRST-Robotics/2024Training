package frc.robot.subsystems.swerveDrive;

import com.ctre.phoenix.led.CANdleStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

public class ModuleIOSparkMax implements ModuleIO {

    CANSparkMax driveMotorController = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax turnMotorController = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    SparkMaxAbsoluteEncoder turnAbsoluteEncoder = turnMotorController.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    RelativeEncoder turnRelativeEncoder = turnMotorController.getEncoder();
    RelativeEncoder driveRelativeEncoder = driveMotorController.getEncoder();

    public ModuleIOSparkMax() {

    }

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveTempCelcius = driveMotorController.getMotorTemperature();
        inputs.turnTempCelcius = turnMotorController.getMotorTemperature();
    }
}
