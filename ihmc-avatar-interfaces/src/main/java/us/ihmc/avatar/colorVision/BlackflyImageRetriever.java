package us.ihmc.avatar.colorVision;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.thread.RestartableThrottledThread;

import java.time.Instant;

public class BlackflyImageRetriever
{
   private static final double BLACKFLY_FPS = 30.0;

   private final SpinnakerBlackfly blackfly;
   private final FramePose3D blackflyPoseInOusterFrame = new FramePose3D();
   private final IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics;

   private long sequenceNumber = 0L;
   private int imageWidth = 0;
   private int imageHeight = 0;
   private RawImage distortedImage = null;
   private final RestartableThrottledThread imageGrabThread;

   public BlackflyImageRetriever(SpinnakerBlackfly blackfly, BlackflyLensProperties lensProperties, RobotSide side, RigidBodyTransform ousterToBlackflyTransform)
   {
      this.blackfly = blackfly;
      this.ousterFisheyeColoringIntrinsics = SensorHeadParameters.loadOusterFisheyeColoringIntrinsicsOnRobot(lensProperties);

      blackflyPoseInOusterFrame.set(ousterToBlackflyTransform);

      imageGrabThread = new RestartableThrottledThread(side.getCamelCaseName() + "BlackflyImageGrabber", BLACKFLY_FPS, this::grabImage);
   }

   private void grabImage()
   {
      // grab image
      spinImage spinImage = new spinImage();
      blackfly.getNextImage(spinImage);
      Instant acquisitionTime = Instant.now();

      // Initialize image dimensions from first image
      if (imageWidth == 0 || imageHeight == 0)
      {
         this.imageWidth = blackfly.getWidth(spinImage);
         this.imageHeight = blackfly.getHeight(spinImage);
      }

      // Get image data
      BytePointer spinImageData = new BytePointer((long) imageWidth * imageHeight);
      blackfly.setPointerToSpinImageData(spinImage, spinImageData);
      BytedecoImage sourceByedecoImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC1);
      sourceByedecoImage.changeAddress(spinImageData.address());

      // Upload image to GPU
      GpuMat deviceSourceImage = new GpuMat(imageHeight, imageWidth, opencv_core.CV_8UC1);
      deviceSourceImage.upload(sourceByedecoImage.getBytedecoOpenCVMat());

      // Convert from BayerRG8 to BGR
      GpuMat sourceImageBGR = new GpuMat(imageHeight, imageWidth, opencv_core.CV_8UC3);
      opencv_cudaimgproc.cvtColor(deviceSourceImage, sourceImageBGR, opencv_imgproc.COLOR_BayerBG2BGR);

      distortedImage = new RawImage(sequenceNumber++,
                                    acquisitionTime,
                                    imageWidth,
                                    imageHeight,
                                    0,
                                    null,
                                    sourceImageBGR.clone(),
                                    opencv_core.CV_8UC3,
                                    (float) ousterFisheyeColoringIntrinsics.getFocalLengthX(),
                                    (float) ousterFisheyeColoringIntrinsics.getFocalLengthY(),
                                    (float) ousterFisheyeColoringIntrinsics.getPrinciplePointX(),
                                    (float) ousterFisheyeColoringIntrinsics.getPrinciplePointY(),
                                    blackflyPoseInOusterFrame.getPosition(),
                                    blackflyPoseInOusterFrame.getOrientation());

      // close stuff
      spinImageData.close();
      deviceSourceImage.close();
      sourceImageBGR.close();
      blackfly.releaseImage(spinImage);
   }

   public RawImage getLatestRawImage()
   {
      return distortedImage;
   }

   public void start()
   {
      imageGrabThread.start();
   }

   public void stop()
   {
      imageGrabThread.stop();
   }

   public void destroy()
   {
      stop();
      blackfly.stopAcquiringImages();
   }
}
