// package org.firstinspires.ftc.teamcode.Testing;
// 
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// 
// 
// @TeleOp(name = "Motor Tester")
// public class MotorTester extends OpMode{
// 
// 
//     private DcMotor motor1;
//     private DcMotor motor2;
// 
// 
// 
//     @Override
//     public void init(){
// 
//         motor1 = hardwareMap.get(DcMotor.class, "Motor1");
//         motor2 = hardwareMap.get(DcMotor.class, "Motor2");
//         
//         motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//         motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         
//         motor1.setDirection(DcMotorSimple.Direction.REVERSE);
//         motor2.setDirection(DcMotorSimple.Direction.REVERSE);
//     }
// 
//     @Override
//     public void loop(){
//     
//     if (gamepad1.right_trigger > 0.1) {
//         motor1.setPower(gamepad1.right_trigger);
//     } else {
//         motor1.setPower(0);
//     }
//     if (gamepad1.left_trigger > 0.1) {
//         motor2.setPower(gamepad1.left_trigger);
//     } else {
//         motor2.setPower(0);
//     }
//     
//     
//     
//     }
// }
// 