package org.firstinspires.ftc.teamcode.hardware;
import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDetector {
    OpenCvCamera webcam;
    private static final String TAG = "ftc-opencv";

    public SampleDetector(WebcamName webcamName, HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Log.d(TAG, "open webcam " + webcam);

        webcam.setPipeline(new SamplePipeline());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Log.d(TAG, "started streaming");

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.d(TAG, "found an error" + errorCode);

            }
        });
    }

   class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        public Mat processFrame(Mat image) {
            //Log.d(TAG, "viewing frame");

            // Preprocess the image
            Mat gray = new Mat();
            Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);

            // Apply edge detection
            Mat edges = new Mat();
            Imgproc.Canny(gray, edges, 100, 200);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw contours on the image

           // Imgproc.drawContours(image, contours, -1, new Scalar(0, 255, 0), 2); // Draw all contours in green


            // Iterate over each detected block (contour)
            for (MatOfPoint contour : contours) {
                // Approximate the contour
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
                Point[] boxPoints = new Point[4];
                rotatedRect.points(boxPoints);

                double width = rotatedRect.size.width;
                double height = rotatedRect.size.height;
                Log.d("DIMS", "w:" + width + " h:"+height);
                if (width < 20 || height < 20) continue;
                if (width > 650 || height > 650) continue;
                // Draw the bounding box
                for (int j = 0; j < 4; j++) {
                    Imgproc.line(image, boxPoints[j], boxPoints[(j+1)%4], new Scalar(0, 255, 0), 2);
                }

                // Calculate the width and height of the bounding box

                // Draw the width, height, and angle of the bounding box on the image
                String dimensionsText = "Width: " + String.format("%.2f", width) + " | Height: " + String.format("%.2f", height);
                Point textPosition = new Point(boxPoints[0].x + 10, boxPoints[0].y + 10); // Position the text near the top-left corner of the bounding box
                Imgproc.putText(image, dimensionsText, textPosition, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                // Correct the angle calculation
                double angle = rotatedRect.angle;
                if (width < height) {
                    angle += 90;
                }
                // Angle should always be between 0 and 90 degrees
                angle = angle < 0 ? angle + 90 : angle;

                String angleText = "Angle: " + String.format("%.2f", angle);
                Point anglePosition = new Point(boxPoints[0].x + 10, boxPoints[0].y + 30); // Position the angle text near the bounding box
                Imgproc.putText(image, angleText, anglePosition, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);


                // Determine if the block is horizontal or vertical based on aspect ratio
                double aspectRatio = rotatedRect.size.width / rotatedRect.size.height;

            }

            return image;
        }
    }

}
