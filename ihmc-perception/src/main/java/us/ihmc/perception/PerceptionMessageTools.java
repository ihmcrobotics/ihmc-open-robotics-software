package us.ihmc.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

import java.nio.ByteBuffer;
import java.util.Arrays;

public class PerceptionMessageTools
{
   public static Point3D[] unpackDepthImageToPointCloud(ImageMessage imageMessage, double horizontalFieldOfView, double verticalFieldOfView)
   {
      RigidBodyTransform sensorTransform = new RigidBodyTransform(imageMessage.getOrientation(), imageMessage.getPosition());
      Mat compressedDepthImageMat = new Mat(1, 1, opencv_core.CV_8UC1);
      ByteBuffer dataByteBuffer = ByteBuffer.wrap(imageMessage.getData().toArray());
      compressedDepthImageMat.cols(dataByteBuffer.limit());
      compressedDepthImageMat.data(new BytePointer(dataByteBuffer));

      return unpackDepthImageToPointCloud(compressedDepthImageMat,
                                          imageMessage.getImageHeight(),
                                          imageMessage.getImageWidth(),
                                          horizontalFieldOfView,
                                          verticalFieldOfView,
                                          sensorTransform);
   }

   public static Point3D[] unpackDepthImageToPointCloud(Mat compressedDepthImage,
                                                        int imageHeight,
                                                        int imageWidth,
                                                        double horizontalFieldOfView,
                                                        double verticalFieldOfView,
                                                        RigidBodyTransform sensorTransform)
   {
      Mat decompressedDepthImage = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
      opencv_imgcodecs.imdecode(compressedDepthImage, opencv_imgcodecs.IMREAD_UNCHANGED, decompressedDepthImage);

      return convertDepthMatToPointCloud(decompressedDepthImage, horizontalFieldOfView, verticalFieldOfView, sensorTransform);
   }

   private static Point3D[] convertDepthMatToPointCloud(Mat depthMat,
                                                        double horizontalFieldOfView,
                                                        double verticalFieldOfView,
                                                        RigidBodyTransform sensorTransform)
   {
      int depthImageWidth = depthMat.cols();
      int depthImageHeight = depthMat.rows();

      double discreteResolution = 0.001f;

      Point3D[] pointCloudToPopulate = new Point3D[depthImageHeight * depthImageWidth];
      int index = 0;
      for (int y = 0; y < depthImageHeight; y++)
      {

         int yFromCenter = y - (depthImageHeight / 2);
         double angleYFromCenter = yFromCenter / (double) depthImageHeight * verticalFieldOfView;

         RotationMatrix angledRotationMatrixFixedY = new RotationMatrix();
         angledRotationMatrixFixedY.setToPitchOrientation(angleYFromCenter);

         for (int x = 0; x < depthImageWidth; x++)
         {
            double eyeDepthInMeters = depthMat.ptr(x, y).getShort() * discreteResolution;

            int xFromCenter = -x - (depthImageWidth / 2); // flip

            double angleXFromCenter = xFromCenter / (double) depthImageWidth * horizontalFieldOfView;

            // Create additional rotation only transform
            RigidBodyTransform angledRotationTransform = new RigidBodyTransform();
            angledRotationTransform.getRotation().set(angledRotationMatrixFixedY);
            angledRotationTransform.prependYawRotation(angleXFromCenter);

            Point3D beamFramePoint = new Point3D(eyeDepthInMeters, 0.0, 0.0);

            beamFramePoint.applyTransform(angledRotationTransform);

            pointCloudToPopulate[index++] = beamFramePoint;
         }
      }

      Arrays.stream(pointCloudToPopulate).parallel().forEach(sensorTransform::transform);

      return pointCloudToPopulate;
   }
}
