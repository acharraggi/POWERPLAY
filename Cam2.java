/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Cam2-park using custom signal", group = "Demo")

public class Cam2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private boolean imageFound = false;
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE1  = "/sdcard/FIRST/tflitemodels/model_20220922_142933.tflite";

    private static final String[] LABELS = {
            "Apple",
            "Dolphin",
            "Duck"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZNwnl7/////AAABmURiFco/PkFlhI2OyrSRbyc+gZQs5mhApXMxyPWBYro4Ln0Z67y6iMun9AiI57pcd+3zNDNM5xpU1qUTR3f+KWr/IDCuFMNezcOVsUScfBq0O0ZsjMToSDlOm/3JeEnxd86cVxqRChgbBWQnXKWu2uEkJ4ncz0oYOUuQygZGSntqdrhQ9mfH9GULWLkbkKmIPaE2mpFPec4sHDz/qKx/nrkIxTjHv06NoiFfDsRHSH8k6JGRl4eMaKD7h2U69y4qgIFxb/SZE5XXFK2mU+dZbxtXkYioSkYiu0nlt9QIv0SZ8rdzQ19anVWBF50DKbbDdXCLOdprK04VQQhwS6WctdSYPy/P445lIKsVG/0i3A1O";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0/9.0);
        }

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive() && !imageFound) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                        //Recognition recognition = updatedRecognitions.get(0);
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                        // now drive to the indicated signal zone
                        if (updatedRecognitions.size()>0){
                            if(updatedRecognitions.get(0).getLabel()=="Duck") {
                                imageFound = true;
                                // drive straight ahead
                                telemetry.addData("drive to: ","location 2"); 
                                telemetry.update();
                                leftFrontDrive.setPower(0.5);
                                rightFrontDrive.setPower(0.5);
                                leftBackDrive.setPower(0.5);
                                rightBackDrive.setPower(0.5);
                                runtime.reset();
                                while(runtime.milliseconds()<1300){
                                    telemetry.addData("driving... "," "); 
                                    telemetry.update();
                                }
                                leftFrontDrive.setPower(0.0);
                                rightFrontDrive.setPower(0.0);
                                leftBackDrive.setPower(0.0);
                                rightBackDrive.setPower(0.0);
                            }
                            if(updatedRecognitions.get(0).getLabel()=="Apple") {
                                imageFound = true;
                                // move 45 degrees to the right
                                telemetry.addData("drive to: ","location 3"); 
                                telemetry.update();
                                leftFrontDrive.setPower(0.5);
                                rightFrontDrive.setPower(0.0);
                                leftBackDrive.setPower(0.0);
                                rightBackDrive.setPower(0.5);
                                runtime.reset();
                                while(runtime.milliseconds()<300){
                                    telemetry.addData("driving... "," "); 
                                    telemetry.update();
                                }
                                // move right
                                leftFrontDrive.setPower(0.5);
                                rightFrontDrive.setPower(-0.5);
                                leftBackDrive.setPower(-0.5);
                                rightBackDrive.setPower(0.5);
                                runtime.reset();
                                while(runtime.milliseconds()<1000){
                                    telemetry.addData("driving... "," "); 
                                    telemetry.update();
                                }
                                // drive straight forward
                                leftFrontDrive.setPower(0.5);
                                rightFrontDrive.setPower(0.5);
                                leftBackDrive.setPower(0.5);
                                rightBackDrive.setPower(0.5);
                                runtime.reset();
                                while(runtime.milliseconds()<1100){
                                    telemetry.addData("driving... "," "); 
                                    telemetry.update();
                                }
                                leftFrontDrive.setPower(0.0);
                                rightFrontDrive.setPower(0.0);
                                leftBackDrive.setPower(0.0);
                                rightBackDrive.setPower(0.0);
                            }
                            if(updatedRecognitions.get(0).getLabel()=="Dolphin") {
                                imageFound = true;
                                // move 45 degrees to the left
                                telemetry.addData("drive to: ","location 1"); 
                                telemetry.update();
                                leftFrontDrive.setPower(0.0);
                                rightFrontDrive.setPower(0.5);
                                leftBackDrive.setPower(0.5);
                                rightBackDrive.setPower(0.0);
                                runtime.reset();
                                while(runtime.milliseconds()<300){
                                    telemetry.addData("driving... "," "); 
                                    telemetry.update();
                                }
                                // move left
                                leftFrontDrive.setPower(-0.5);
                                rightFrontDrive.setPower(0.5);
                                leftBackDrive.setPower(0.5);
                                rightBackDrive.setPower(-0.5);
                                runtime.reset();
                                while(runtime.milliseconds()<1000){
                                    telemetry.addData("driving... "," "); 
                                    telemetry.update();
                                }
                                // drive straight forward
                                leftFrontDrive.setPower(0.5);
                                rightFrontDrive.setPower(0.5);
                                leftBackDrive.setPower(0.5);
                                rightBackDrive.setPower(0.5);
                                runtime.reset();
                                while(runtime.milliseconds()<1100){
                                    telemetry.addData("driving... "," "); 
                                    telemetry.update();
                                }
                                //stop    
                                leftFrontDrive.setPower(0.0);
                                rightFrontDrive.setPower(0.0);
                                leftBackDrive.setPower(0.0);
                                rightBackDrive.setPower(0.0);
                            }

                        }
                    }
                    
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 2");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
         tfod.loadModelFromFile(TFOD_MODEL_FILE1, LABELS);
    }
}
