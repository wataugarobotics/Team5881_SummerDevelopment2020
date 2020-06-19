package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareAndMethods;

@TeleOp(name="2p TeleOP", group="Summer Development 2020")
//@Disabled
public class TwoPersonTeleOP extends OpMode {
    //Create HardwareAndMethods instance called robot
    private HardwareAndMethods robot = new HardwareAndMethods();

    // Declares variables
    private boolean stickPressed = false;
    private boolean platformChanged = false, platformOn = false;
    private boolean parkingChanged = false, parkingOn = false;
    private boolean clawChanged = false, clawOn = false;
    private boolean capstoneChanged = false, capstoneOn = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables
        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start(){}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Speed mod
        if(gamepad1.right_stick_button && !stickPressed){
            stickPressed = true;
            if(robot.speedMod == 1f) {
                robot.speedMod = 2f;
            }else{
                robot.speedMod = 1f;
            }
        }else if(!gamepad1.right_stick_button){
            stickPressed = false;
        }

        //Platform servos
        if(gamepad1.right_bumper && !platformChanged) {
            robot.platformRight.setPosition(platformOn ? 0 : 1);
            robot.platformLeft.setPosition(platformOn ? 1 : 0);
            platformOn = !platformOn;
            platformChanged = true;
        } else if(!gamepad1.right_bumper) platformChanged = false;

        //Parking servo
        if(gamepad1.left_bumper && !parkingChanged) {
            robot.parking.setPosition(parkingOn ? 1 : 0);
            parkingOn = !parkingOn;
            parkingChanged = true;
        } else if(!gamepad1.left_bumper) parkingChanged = false;

        //Claw servo
        if(gamepad2.x && !clawChanged) {
            robot.claw.setPosition(clawOn ? 0 : 1);
            clawOn = !clawOn;
            clawChanged = true;
        } else if(!gamepad2.x) clawChanged = false;

        //Capstone servo
        if(gamepad2.b && !capstoneChanged) {
            robot.capstone.setPosition(capstoneOn ? 0 : 1);
            capstoneOn = !capstoneOn;
            capstoneChanged = true;
        } else if(!gamepad2.b) capstoneChanged = false;


        // Calls mechanum method
        // Mechanum uses the left stick to drive in the x,y directions, and the right stick to turn
        robot.mechanum(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        //Calls lift method
        robot.lift(gamepad2.right_trigger - gamepad2.left_trigger);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

