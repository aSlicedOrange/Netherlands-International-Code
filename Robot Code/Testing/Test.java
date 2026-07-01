// package org.firstinspires.ftc.teamcode.Testing;
// 
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// 
// 
// @TeleOp(name = "Test")
// public class Test extends OpMode{
// 
// 
//     private DcMotor lf;
//     private DcMotor lb;
//     private DcMotor rf;
//     private DcMotor rb;
// 
// 
// 
//     @Override
//     public void init(){
//         
//         lf = hardwareMap.get(DcMotor.class, "frontLeft");
//         lb = hardwareMap.get(DcMotor.class, "backLeft");
//         rf = hardwareMap.get(DcMotor.class, "frontRight");
//         rb = hardwareMap.get(DcMotor.class, "backRight");
//         
//         lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         
//         //lf.setDirection(DcMotorSimple.Direction.REVERSE);
//         lb.setDirection(DcMotorSimple.Direction.REVERSE);
//         rf.setDirection(DcMotorSimple.Direction.REVERSE);
//         //rb.setDirection(DcMotorSimple.Direction.REVERSE);
//     }
// 
//     @Override
//     public void loop(){
//     
//     if (gamepad1.right_trigger > 0.1) {
//         lf.setPower(gamepad1.right_trigger);
//         lb.setPower(gamepad1.right_trigger);
//         rf.setPower(gamepad1.right_trigger);
//         rb.setPower(gamepad1.right_trigger);
//     } else if (gamepad1.left_trigger > 0.1) {
//         lf.setPower(-gamepad1.left_trigger);
//         lb.setPower(-gamepad1.left_trigger);
//         rf.setPower(-gamepad1.left_trigger);
//         rb.setPower(-gamepad1.left_trigger);
//     }
//     
//     }
// }
// 