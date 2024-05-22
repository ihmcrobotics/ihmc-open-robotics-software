package us.ihmc.perception;

import controller_msgs.msg.dds.RigidBodyTransformMessage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point2f;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.ImageMessageDataPacker;
import us.ihmc.robotics.math.filters.AlphaFilteredRigidBodyTransform;

import java.time.Instant;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public class BallDetectionManager
{
   private final Executor detectionExecutor = Executors.newSingleThreadExecutor();

   private final BallDetector ballDetector = new BallDetector();
   private long maskImageSequenceNumber = 0L;
   private double ballDiameter = 0.0;
   private final AlphaFilteredRigidBodyTransform ballPosition = new AlphaFilteredRigidBodyTransform();

   private final ROS2Helper ros2Helper;

   public BallDetectionManager(ROS2Helper ros2Helper)
   {
      this.ros2Helper = ros2Helper;

      ros2Helper.subscribe(PerceptionAPI.BALL_DETECTION_PARAMETERS).addCallback(parametersMessage ->
      {
         double hLow = parametersMessage.getHLow();
         double sLow = parametersMessage.getSLow();
         double vLow = parametersMessage.getVLow();

         double hHigh = parametersMessage.getHHigh();
         double sHigh = parametersMessage.getSHigh();
         double vHigh = parametersMessage.getVHigh();

         ballDiameter = parametersMessage.getBallDiameter();

         ballDetector.setHSVLowerBound(hLow, sLow, vLow);
         ballDetector.setHSVUpperBound(hHigh, sHigh, vHigh);
         ballPosition.setAlpha(parametersMessage.getAlpha());
      });
   }

   // This method does not block
   public void run(RawImage colorImage)
   {
      detectionExecutor.execute(() ->
      {
         // Ensure we have a color image
         if (!colorImage.isAvailable())
            return;
         colorImage.get();

         // get the average focal length used for calculating position of the ball
         double averageFocalLength = 0.5 * (colorImage.getFocalLengthX() + colorImage.getFocalLengthY());

         Point2f ballCenter = new Point2f();
         Mat maskMat = new Mat();

         // Try to detect a ball
         double radius = ballDetector.detect(colorImage, 1.0, ballCenter, maskMat);
         Instant maskAcquisitionTime = Instant.now();

         // If a ball was found
         if (radius > 0.0f && ballCenter.x() > 0.0f && ballCenter.y() > 0.0f)
         {
            // calculate its position (relative to sensor)
            double depth = (ballDiameter * averageFocalLength) / (2.0 * radius);
            double y = -(ballCenter.x() - colorImage.getPrincipalPointX()) / colorImage.getFocalLengthX() * depth;
            double z = -(ballCenter.y() - colorImage.getPrincipalPointY()) / colorImage.getFocalLengthY() * depth;

            // get ball's position with respect to world
            RigidBodyTransform newBallPosition = new RigidBodyTransform(colorImage.getOrientation(), colorImage.getPosition());
            newBallPosition.appendTranslation(depth, y, z);
            ballPosition.update(newBallPosition);

            // send out the ball's position
            RigidBodyTransformMessage message = new RigidBodyTransformMessage();
            MessageTools.toMessage(ballPosition, message);
            ros2Helper.publish(PerceptionAPI.BALL_TRAJECTORY, message);
         }

         // If we got a segmentation image from the detector
         if (maskMat.rows() > 0 && maskMat.cols() > 0)
         {
            // compress the image to PNG
            BytePointer compressedImage = new BytePointer();
            OpenCVTools.compressImagePNG(maskMat, compressedImage);

            // send it out to UI
            ImageMessage imageMessage = new ImageMessage();
            ImageMessageDataPacker dataPacker = new ImageMessageDataPacker(compressedImage.limit());
            dataPacker.pack(imageMessage, compressedImage);
            MessageTools.toMessage(maskAcquisitionTime, imageMessage.getAcquisitionTime());
            imageMessage.setFocalLengthXPixels(colorImage.getFocalLengthX());
            imageMessage.setFocalLengthYPixels(colorImage.getFocalLengthY());
            imageMessage.setPrincipalPointXPixels(colorImage.getPrincipalPointX());
            imageMessage.setPrincipalPointYPixels(colorImage.getPrincipalPointY());
            imageMessage.setImageWidth(maskMat.cols());
            imageMessage.setImageHeight(maskMat.rows());
            imageMessage.getPosition().set(colorImage.getPosition());
            imageMessage.getOrientation().set(colorImage.getOrientation());
            imageMessage.setSequenceNumber(maskImageSequenceNumber++);
            imageMessage.setDepthDiscretization(-1.0f);
            CameraModel.PINHOLE.packMessageFormat(imageMessage);
            ImageMessageFormat.PNG_8UC1.packMessageFormat(imageMessage);
            ros2Helper.publish(PerceptionAPI.BALL_SEGMENTATION_IMAGE, imageMessage);
         }

         maskMat.close();
         ballCenter.close();
         colorImage.release();
      });
   }

   public void destroy()
   {
      ballDetector.destroy();
   }
}
