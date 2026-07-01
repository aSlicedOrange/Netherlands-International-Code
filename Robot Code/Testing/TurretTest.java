// package org.firstinspires.ftc.teamcode.Testing;
// 
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import org.firstinspires.ftc.teamcode.Classes.Flywheel;
// 
// 
// @TeleOp(name = "Oscar kys")
// public class TurretTest extends OpMode {
// 
// 
//     private Flywheel flywheel;
//     private Servo servo;
//     private double servoPosition = 0;
// 
// 
//     @Override
//     public void init(){
//         flywheel = new Flywheel(hardwareMap);
//         servo = hardwareMap.get(Servo.class, "servo");
//     }
// 
//     @Override
//     public void loop(){
//     
//     if (gamepad1.right_trigger > 0.1) {
//         flywheel.setPower(gamepad1.right_trigger);
//         telemetry.addLine("REV");
//     } else {
//         flywheel.setZero();
//     }
//     if (gamepad1.right_bumper) { 
//         servoPosition += 0.05;
//     }
//     if (gamepad1.left_bumper) {
//         servoPosition -= 0.05;
//     }
//     servo.setPosition(servoPosition);
//     telemetry.update();
//     
//     }
// }
// 