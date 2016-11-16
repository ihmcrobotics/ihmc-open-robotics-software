package us.ihmc.ihmcPerception.fiducialDetector;

import java.awt.image.BufferedImage;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
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
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;

public class FiducialDetectorFromCameraImages
{
   private boolean visualize = false;

   private final Se3_F64 fiducialToCamera;
   private final Matrix3d fiducialRotationMatrix;
   private final Quat4d tempFiducialRotationQuat;
   private final FramePose tempFiducialDetectorFrame;
   private final FrameOrientation tempFudictionalDetectorOrientation;
   private final Vector3d cameraRigidPosition;
   private final double[] eulerAngles;
   private final RigidBodyTransform cameraRigidTransform;

   private final ReferenceFrame cameraReferenceFrame, detectorReferenceFrame, locatedFiducialReferenceFrame, reportedFiducialReferenceFrame;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private FiducialDetector<GrayF32> detector;

   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();

   private final YoGraphicReferenceFrame cameraGraphic, detectorGraphic, locatedFiducialGraphic, reportedFiducialGraphic;

   private final String prefix = "fiducial";

   private final DoubleYoVariable expectedFiducialSize = new DoubleYoVariable("expectedFiducialSize", registry);
   private final DoubleYoVariable fieldOfViewXinRadians = new DoubleYoVariable("fovXRadians", registry);
   private final DoubleYoVariable fieldOfViewYinRadians = new DoubleYoVariable("fovYRadians", registry);

   private final DoubleYoVariable detectorPositionX = new DoubleYoVariable(prefix + "DetectorPositionX", registry);
   private final DoubleYoVariable detectorPositionY = new DoubleYoVariable(prefix + "DetectorPositionY", registry);
   private final DoubleYoVariable detectorPositionZ = new DoubleYoVariable(prefix + "DetectorPositionZ", registry);
   private final DoubleYoVariable detectorEulerRotX = new DoubleYoVariable(prefix + "DetectorEulerRotX", registry);
   private final DoubleYoVariable detectorEulerRotY = new DoubleYoVariable(prefix + "DetectorEulerRotY", registry);
   private final DoubleYoVariable detectorEulerRotZ = new DoubleYoVariable(prefix + "DetectorEulerRotZ", registry);

   private final BooleanYoVariable locationEnabled = new BooleanYoVariable(prefix + "LocationEnabled", registry);
   private final BooleanYoVariable targetIDHasBeenLocated = new BooleanYoVariable(prefix + "TargetIDHasBeenLocated", registry);
   private final LongYoVariable targetIDToLocate = new LongYoVariable(prefix + "TargetIDToLocate", registry);

   private final YoFramePose cameraPose = new YoFramePose(prefix + "CameraPoseWorld", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose locatedFiducialPoseInWorldFrame = new YoFramePose(prefix + "LocatedPoseWorldFrame", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose reportedFiducialPoseInWorldFrame = new YoFramePose(prefix + "ReportedPoseWorldFrame", ReferenceFrame.getWorldFrame(), registry);


//   private final YoFrameQuaternion fiducialOrientationQuaternionInWorldFrame = new YoFrameQuaternion(prefix + "LocatedPoseWorldFrame", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameQuaternion fiducialReportedOrientationQuaternionInWorldFrame = new YoFrameQuaternion(prefix + "ReportedPoseWorldFrame", ReferenceFrame.getWorldFrame(), registry);

   public FiducialDetectorFromCameraImages(RigidBodyTransform transformFromReportedToFiducialFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.locationEnabled.set(true);
      this.expectedFiducialSize.set(1.0);

      detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(expectedFiducialSize.getDoubleValue()), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);

      expectedFiducialSize.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(expectedFiducialSize.getDoubleValue()), ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);
         }
      });

      // fov values from http://carnegierobotics.com/multisense-s7/
      //      fieldOfViewXinRadians.set(Math.toRadians(80.0));
      //      fieldOfViewYinRadians.set(Math.toRadians(45.0));

      //      fieldOfViewXinRadians.set(Math.PI/4.0);
      //      fieldOfViewYinRadians.set(Math.PI/4.0);

      fieldOfViewXinRadians.set(1.0);
      fieldOfViewYinRadians.set(1.0);

      fiducialToCamera = new Se3_F64();
      eulerAngles = new double[3];
      fiducialRotationMatrix = new Matrix3d();
      tempFiducialRotationQuat = new Quat4d();
      cameraRigidPosition = new Vector3d();
      cameraRigidTransform = new RigidBodyTransform();
      tempFiducialDetectorFrame = new FramePose();
      tempFudictionalDetectorOrientation = new FrameOrientation();

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

      locatedFiducialReferenceFrame = new ReferenceFrame(prefix + "LocatedReferenceFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            locatedFiducialPoseInWorldFrame.getPose(transformToParent);
         }
      };

      reportedFiducialReferenceFrame = new TransformReferenceFrame(prefix + "ReportedReferenceFrame", locatedFiducialReferenceFrame, transformFromReportedToFiducialFrame);

      if (yoGraphicsListRegistry == null)
      {
         visualize = false;
      }

      if (visualize)
      {
         cameraGraphic = new YoGraphicReferenceFrame(cameraReferenceFrame, registry, 0.5);
         detectorGraphic = new YoGraphicReferenceFrame(detectorReferenceFrame, registry, 1.0);
         locatedFiducialGraphic = new YoGraphicReferenceFrame(locatedFiducialReferenceFrame, registry, 0.1);
         reportedFiducialGraphic = new YoGraphicReferenceFrame(reportedFiducialReferenceFrame, registry, 0.1);

         //         yoGraphicsListRegistry.registerYoGraphic("Fiducials", cameraGraphic);
         //         yoGraphicsListRegistry.registerYoGraphic("Fiducials", detectorGraphic);
         yoGraphicsListRegistry.registerYoGraphic("Fiducials", locatedFiducialGraphic);
         yoGraphicsListRegistry.registerYoGraphic("Fiducials", reportedFiducialGraphic);
      }
      else
      {
         cameraGraphic = detectorGraphic = locatedFiducialGraphic = reportedFiducialGraphic = null;
      }

      parentRegistry.addChild(registry);
   }

   public FiducialDetector<GrayF32> getFiducialDetector()
   {
      return detector;
   }

   public void setNewVideoPacket(VideoPacket videoPacket)
   {
      BufferedImage latestUnmodifiedCameraImage = jpegDecompressor.decompressJPEGDataToBufferedImage(videoPacket.getData());
      setNewVideoPacket(latestUnmodifiedCameraImage, videoPacket.getPosition(), videoPacket.getOrientation());
   }

   private boolean showedImage = false;

   public void setNewVideoPacket(BufferedImage bufferedImage, Point3d cameraPositionInWorld, Quat4d cameraOrientationInWorldXForward)
   {
      if (!locationEnabled.getBooleanValue())
         return;

      //TODO: Don't make a new one every tick. Reuse it.
      IntrinsicParameters intrinsicParameters = setIntrinsicParameters(bufferedImage);

      cameraRigidTransform.setRotation(cameraOrientationInWorldXForward);
      cameraRigidPosition.set(cameraPositionInWorld);
      cameraRigidTransform.setTranslation(cameraRigidPosition);

      cameraReferenceFrame.update();
      detectorReferenceFrame.update();

      cameraPose.setOrientation(cameraOrientationInWorldXForward);
      cameraPose.setPosition(cameraPositionInWorld);

      GrayF32 grayImage = ConvertBufferedImage.convertFrom(bufferedImage, true, ImageType.single(GrayF32.class));

      detector.detect(grayImage);

      int matchingFiducial = -1;
      for (int i = 0; i < detector.totalFound(); i++)
      {
         if (detector.getId(i) == targetIDToLocate.getLongValue())
         {
            matchingFiducial = i;
            //            System.out.println("matchingFiducial = " + matchingFiducial);
         }
      }

      if (matchingFiducial > -1)
      {
         detector.getFiducialToCamera(matchingFiducial, fiducialToCamera);

//         System.out.println("fiducialToCamera = \n" + fiducialToCamera);

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

         locatedFiducialPoseInWorldFrame.set(tempFiducialDetectorFrame);

         locatedFiducialReferenceFrame.update();

         tempFiducialDetectorFrame.setToZero(reportedFiducialReferenceFrame);
         tempFiducialDetectorFrame.changeFrame(ReferenceFrame.getWorldFrame());

         tempFiducialDetectorFrame.getOrientationIncludingFrame(tempFudictionalDetectorOrientation);
         fiducialReportedOrientationQuaternionInWorldFrame.set(tempFudictionalDetectorOrientation);

         reportedFiducialPoseInWorldFrame.set(tempFiducialDetectorFrame);

         targetIDHasBeenLocated.set(true);

         if (visualize)
         {
            cameraGraphic.update();
            detectorGraphic.update();
            locatedFiducialGraphic.update();
            reportedFiducialGraphic.update();
         }

         //         if (!showedImage)
         //         {
         //            System.out.println("Showing image...");
         //            showedImage = true;
         //            Se3_F64 fiducialToCamera = new Se3_F64();
         //            detector.getFiducialToCamera(0, fiducialToCamera);
         //
         //            fiducialToCamera.setTranslation(0.0, 0.0, 0.2);
         //            System.out.println("fiducialToCamera = " + fiducialToCamera);
         //
         //            BufferedImage bufferedImageCopy = new BufferedImage(bufferedImage.getWidth(), bufferedImage.getHeight(), bufferedImage.getType());
         //
         //            Graphics2D graphics = bufferedImageCopy.createGraphics();
         //
         //            VisualizeFiducial.drawCube(fiducialToCamera, intrinsicParameters, detector.getWidth(0), 3, graphics);
         //            VisualizeFiducial.drawLabelCenter(fiducialToCamera, intrinsicParameters, "" + detector.getId(0), graphics);
         ////
         //            ShowImages.showWindow(bufferedImage, "FiducialDetectionImageTest");
         //            System.out.println("Done showing image...");
         //         }
      }
      else
      {
         targetIDHasBeenLocated.set(false);
      }
   }

   private IntrinsicParameters setIntrinsicParameters(BufferedImage image)
   {
      IntrinsicParameters intrinsicParameters = new IntrinsicParameters();

      int height = image.getHeight();
      int width = image.getWidth();

      double fx = (width / 2.0) / Math.tan(fieldOfViewXinRadians.getDoubleValue() / 2.0);
      double fy = (height / 2.0) / Math.tan(fieldOfViewYinRadians.getDoubleValue() / 2.0);
      intrinsicParameters = new IntrinsicParameters();
      intrinsicParameters.width = width;
      intrinsicParameters.height = height;
      intrinsicParameters.cx = width / 2;
      intrinsicParameters.cy = height / 2;
      intrinsicParameters.fx = fx;
      intrinsicParameters.fy = fy;

      detector.setIntrinsic(intrinsicParameters);

      return intrinsicParameters;
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

   public void getLocatedFiducialPoseWorldFrame(FramePose framePoseToPack)
   {
      locatedFiducialPoseInWorldFrame.getFramePoseIncludingFrame(framePoseToPack);
   }

   public void getReportedFiducialPoseWorldFrame(FramePose framePoseToPack)
   {
      reportedFiducialPoseInWorldFrame.getFramePoseIncludingFrame(framePoseToPack);
   }

   public void setFieldOfView(double fieldOfViewXinRadians, double fieldOfViewYinRadians)
   {
      this.fieldOfViewXinRadians.set(fieldOfViewXinRadians);
      this.fieldOfViewYinRadians.set(fieldOfViewYinRadians);
   }

}
