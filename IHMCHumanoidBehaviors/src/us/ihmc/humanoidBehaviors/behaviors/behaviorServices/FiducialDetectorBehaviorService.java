package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.awt.image.BufferedImage;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.se.Se3_F64;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.thread.ThreadTools;

public class FiducialDetectorBehaviorService extends ThreadedBehaviorService
{
   private static final double DEFAULT_FIDUCIAL_SIZE = 0.18;
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<VideoPacket>();
   private FiducialDetector<GrayF32> detector;
   private JPEGDecompressor jpegDecompressor;
   private BufferedImage latestUnmodifiedCameraImage;
   private IntrinsicParameters intrinsicParameters = null;
   private Object detectorConch;

   // Yo variables
   private final String prefix = "fiducial";
   private final BooleanYoVariable locationEnabled;
   private final BooleanYoVariable targetIDHasBeenLocated;
   private final LongYoVariable targetIDToLocate;
   private final DoubleYoVariable expectedFiducialSize;
   private final DoubleYoVariable detectorPositionX;
   private final DoubleYoVariable detectorPositionY;
   private final DoubleYoVariable detectorPositionZ;
   private final DoubleYoVariable detectorEulerRotX;
   private final DoubleYoVariable detectorEulerRotY;
   private final DoubleYoVariable detectorEulerRotZ;
   private final YoFramePose cameraPose;
   private final YoFramePose fiducialPoseInWorldFrame;
   private final YoGraphicReferenceFrame locatedFiducialGraphic;
   private final YoGraphicReferenceFrame cameraGraphic;
   private final YoGraphicReferenceFrame detectorGraphic;
   private final ReferenceFrame cameraReferenceFrame;
   private final ReferenceFrame detectorReferenceFrame;
   private final ReferenceFrame locatedFiducialReferenceFrame;

   // Util variables
   private Se3_F64 fiducialToCamera;
   private Matrix3d fiducialRotationMatrix;
   private Quat4d tempFiducialRotationQuat;
   private FramePose tempFiducialDetectorFrame;
   private Vector3d cameraRigidPosition;
   double[] eulerAngles;
   private RigidBodyTransform cameraRigidTransform;

   @SuppressWarnings("serial")
   public FiducialDetectorBehaviorService(AbstractBehavior behaviorInterface, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(FiducialDetectorBehaviorService.class.getSimpleName(), behaviorInterface);

      getBehaviorInterface().attachNetworkListeningQueue(videoPacketQueue, VideoPacket.class);
      jpegDecompressor = new JPEGDecompressor();

      fiducialToCamera = new Se3_F64();
      eulerAngles = new double[3];
      fiducialRotationMatrix = new Matrix3d();
      tempFiducialRotationQuat = new Quat4d();
      cameraRigidPosition = new Vector3d();
      cameraRigidTransform = new RigidBodyTransform();
      tempFiducialDetectorFrame = new FramePose();

      cameraReferenceFrame = new ReferenceFrame(prefix + "CameraReferenceFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(cameraRigidTransform);
         }
      };
      detectorReferenceFrame = new ReferenceFrame(prefix + "DetectorReferenceFrame", cameraReferenceFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setM00(0.0);
            transformToParent.setM01(0.0);
            transformToParent.setM02(1.0);
            transformToParent.setM03(0.0);
            transformToParent.setM10(-1.0);
            transformToParent.setM11(0.0);
            transformToParent.setM12(0.0);
            transformToParent.setM13(0.0);
            transformToParent.setM20(0.0);
            transformToParent.setM21(-1.0);
            transformToParent.setM22(0.0);
            transformToParent.setM23(0.0);
         }
      };

      locationEnabled = new BooleanYoVariable(prefix + "LocationEnabled", getBehaviorInterface().getYoVariableRegistry());
      targetIDHasBeenLocated = new BooleanYoVariable(prefix + "TargetIDHasBeenLocated", getBehaviorInterface().getYoVariableRegistry());
      targetIDToLocate = new LongYoVariable(prefix + "TargetIDToLocate", getBehaviorInterface().getYoVariableRegistry());
      expectedFiducialSize = new DoubleYoVariable(prefix + "ExpectedSize", getBehaviorInterface().getYoVariableRegistry());
      detectorPositionX = new DoubleYoVariable(prefix + "DetectorPositionX", getBehaviorInterface().getYoVariableRegistry());
      detectorPositionY = new DoubleYoVariable(prefix + "DetectorPositionY", getBehaviorInterface().getYoVariableRegistry());
      detectorPositionZ = new DoubleYoVariable(prefix + "DetectorPositionZ", getBehaviorInterface().getYoVariableRegistry());
      detectorEulerRotX = new DoubleYoVariable(prefix + "DetectorEulerRotX", getBehaviorInterface().getYoVariableRegistry());
      detectorEulerRotY = new DoubleYoVariable(prefix + "DetectorEulerRotY", getBehaviorInterface().getYoVariableRegistry());
      detectorEulerRotZ = new DoubleYoVariable(prefix + "DetectorEulerRotZ", getBehaviorInterface().getYoVariableRegistry());
      cameraPose = new YoFramePose(prefix + "CameraPoseWorld", ReferenceFrame.getWorldFrame(), getBehaviorInterface().getYoVariableRegistry());
      fiducialPoseInWorldFrame = new YoFramePose(prefix + "LocatedPoseWorldFrame", ReferenceFrame.getWorldFrame(),
                                                 getBehaviorInterface().getYoVariableRegistry());

      locatedFiducialReferenceFrame = new ReferenceFrame(prefix + "LocatedReferenceFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            fiducialPoseInWorldFrame.getPose(transformToParent);
         }
      };

      cameraGraphic = new YoGraphicReferenceFrame(cameraReferenceFrame, getBehaviorInterface().getYoVariableRegistry(), 0.5);
      detectorGraphic = new YoGraphicReferenceFrame(detectorReferenceFrame, getBehaviorInterface().getYoVariableRegistry(), 1.0);
      locatedFiducialGraphic = new YoGraphicReferenceFrame(locatedFiducialReferenceFrame, getBehaviorInterface().getYoVariableRegistry(), 1.0);

      yoGraphicsListRegistry.registerYoGraphic("Fiducials", cameraGraphic);
      yoGraphicsListRegistry.registerYoGraphic("Fiducials", locatedFiducialGraphic);
      yoGraphicsListRegistry.registerYoGraphic("Fiducials", detectorGraphic);

      detectorConch = new Object();
      expectedFiducialSize.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            synchronized (detectorConch)
            {
               detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(expectedFiducialSize.getDoubleValue()),
                                                       ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);
            }
         }
      });
      expectedFiducialSize.set(DEFAULT_FIDUCIAL_SIZE);
   }

   @Override
   public void doThreadAction()
   {
      synchronized (detectorConch)
      {
         if (videoPacketQueue.isNewPacketAvailable() && locationEnabled.getBooleanValue())
         {
            VideoPacket videoPacket = videoPacketQueue.getLatestPacket();
            latestUnmodifiedCameraImage = jpegDecompressor.decompressJPEGDataToBufferedImage(videoPacket.getData());

            setIntrinsicParameters(latestUnmodifiedCameraImage);

            cameraRigidTransform.setRotation(videoPacket.getOrientation());
            cameraRigidPosition.set(videoPacket.getPosition());
            cameraRigidTransform.setTranslation(cameraRigidPosition);

            cameraReferenceFrame.update();
            cameraGraphic.update();
            detectorReferenceFrame.update();
            detectorGraphic.update();

            cameraPose.setOrientation(videoPacket.getOrientation());
            cameraPose.setPosition(videoPacket.getPosition());

            GrayF32 grayImage = ConvertBufferedImage.convertFrom(latestUnmodifiedCameraImage, true, ImageType.single(GrayF32.class));

            detector.detect(grayImage);

            int matchingFiducial = -1;
            for (int i = 0; i < detector.totalFound(); i++)
            {
               if (detector.getId(i) == targetIDToLocate.getLongValue())
               {
                  matchingFiducial = i;
               }
            }

            if (matchingFiducial > -1)
            {
               detector.getFiducialToCamera(matchingFiducial, fiducialToCamera);

               detectorPositionX.set(fiducialToCamera.getX());
               detectorPositionY.set(fiducialToCamera.getY());
               detectorPositionZ.set(fiducialToCamera.getZ());
               ConvertRotation3D_F64.matrixToEuler(fiducialToCamera.R, EulerType.XYZ, eulerAngles);
               detectorEulerRotX.set(eulerAngles[0]);
               detectorEulerRotY.set(eulerAngles[1]);
               detectorEulerRotZ.set(eulerAngles[2]);

               fiducialRotationMatrix.set(fiducialToCamera.getR().data);
               RotationTools.convertMatrixToQuaternion(fiducialRotationMatrix, tempFiducialRotationQuat);
               
               tempFiducialDetectorFrame.setToZero(detectorReferenceFrame);
               tempFiducialDetectorFrame.setOrientation(tempFiducialRotationQuat);
               tempFiducialDetectorFrame.setPosition(fiducialToCamera.getX(), fiducialToCamera.getY(), fiducialToCamera.getZ());
               tempFiducialDetectorFrame.changeFrame(ReferenceFrame.getWorldFrame());
               
               fiducialPoseInWorldFrame.set(tempFiducialDetectorFrame);

               locatedFiducialReferenceFrame.update();
               locatedFiducialGraphic.update();

               targetIDHasBeenLocated.set(true);
            }
            else
            {
               targetIDHasBeenLocated.set(false);
            }
         }
         else
         {
            ThreadTools.sleep(10);
         }
      }
   }

   private void setIntrinsicParameters(BufferedImage image)
   {
      intrinsicParameters = new IntrinsicParameters();

      // fov values from http://carnegierobotics.com/multisense-s7/
      double fovXinRadian = Math.toRadians(80.0);
      double fovYinRadian = Math.toRadians(45.0);

      int height = image.getHeight();
      int width = image.getWidth();

      double fx = (width / 2.0) / Math.tan(fovXinRadian / 2.0);
      double fy = (height / 2.0) / Math.tan(fovYinRadian / 2.0);
      intrinsicParameters = new IntrinsicParameters();
      intrinsicParameters.width = width;
      intrinsicParameters.height = height;
      intrinsicParameters.cx = width / 2;
      intrinsicParameters.cy = height / 2;
      intrinsicParameters.fx = fx;
      intrinsicParameters.fy = fy;

      detector.setIntrinsic(intrinsicParameters);
   }

   public void setTargetIDToLocate(long targetIDToLocate)
   {
      this.targetIDToLocate.set(targetIDToLocate);
   }

   public void setLocationEnabled(boolean locationEnabled)
   {
      this.locationEnabled.set(locationEnabled);
   }

   public boolean getTargetIDHasBeenLocated()
   {
      return targetIDHasBeenLocated.getBooleanValue();
   }

   public YoFramePose getLocatedFiducialPoseWorldFrame()
   {
      return fiducialPoseInWorldFrame;
   }
}
