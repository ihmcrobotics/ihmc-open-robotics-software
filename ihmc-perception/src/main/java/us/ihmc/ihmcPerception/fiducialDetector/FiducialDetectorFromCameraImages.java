package us.ihmc.ihmcPerception.fiducialDetector;

import java.awt.FlowLayout;
import java.awt.image.BufferedImage;
import java.awt.image.RescaleOp;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import controller_msgs.msg.dds.VideoPacket;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.se.Se3_F64;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class FiducialDetectorFromCameraImages
{
   private boolean visualize = true;

   private boolean DEBUG = false;

   private final Se3_F64 fiducialToCamera = new Se3_F64();
   private final RotationMatrix fiducialRotationMatrix = new RotationMatrix();
   private final Quaternion tempFiducialRotationQuat = new Quaternion();
   private final FramePose3D tempFiducialDetectorFrame = new FramePose3D();
   private final Vector3D cameraRigidPosition = new Vector3D();
   private final double[] eulerAngles = new double[3];
   private final RigidBodyTransform cameraRigidTransform = new RigidBodyTransform();

   private final ReferenceFrame cameraReferenceFrame, detectorReferenceFrame, locatedFiducialReferenceFrame, reportedFiducialReferenceFrame;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private FiducialDetector<GrayF32> detector;
   private Object expectedFiducialSizeChangedConch = new Object();

   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();

   private final YoGraphicReferenceFrame cameraGraphic, detectorGraphic, locatedFiducialGraphic, reportedFiducialGraphic;

   private String prefix = "fiducial";

   private final YoDouble expectedFiducialSize = new YoDouble("expectedFiducialSize", registry);
   private final YoDouble fieldOfViewXinRadians = new YoDouble("fovXRadians", registry);
   private final YoDouble fieldOfViewYinRadians = new YoDouble("fovYRadians", registry);

   private final YoDouble detectorPositionX = new YoDouble(prefix + "DetectorPositionX", registry);
   private final YoDouble detectorPositionY = new YoDouble(prefix + "DetectorPositionY", registry);
   private final YoDouble detectorPositionZ = new YoDouble(prefix + "DetectorPositionZ", registry);
   private final YoDouble detectorEulerRotX = new YoDouble(prefix + "DetectorEulerRotX", registry);
   private final YoDouble detectorEulerRotY = new YoDouble(prefix + "DetectorEulerRotY", registry);
   private final YoDouble detectorEulerRotZ = new YoDouble(prefix + "DetectorEulerRotZ", registry);

   private final YoBoolean targetIDHasBeenLocated = new YoBoolean(prefix + "TargetIDHasBeenLocated", registry);
   private final GlitchFilteredYoBoolean targetIDHasBeenLocatedFiltered = new GlitchFilteredYoBoolean(prefix + "TargetIDHasBeenLocatedFiltered", registry,
                                                                                                      targetIDHasBeenLocated, 4);
   private final YoLong targetIDToLocate = new YoLong(prefix + "TargetIDToLocate", registry);

   private final YoFramePose3D cameraPose = new YoFramePose3D(prefix + "CameraPoseWorld", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D locatedFiducialPoseInWorldFrame = new YoFramePose3D(prefix + "LocatedPoseWorldFrame", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D reportedFiducialPoseInWorldFrame = new YoFramePose3D(prefix + "ReportedPoseWorldFrame", ReferenceFrame.getWorldFrame(),
                                                                                    registry);

   public FiducialDetectorFromCameraImages(RigidBodyTransform transformFromReportedToFiducialFrame, YoVariableRegistry parentRegistry,
                                           YoGraphicsListRegistry yoGraphicsListRegistry, String prefix)
   {
      this.expectedFiducialSize.set(1.0);
      this.prefix = prefix;
      detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(expectedFiducialSize.getDoubleValue()),
                                              ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);

      expectedFiducialSize.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            synchronized (expectedFiducialSizeChangedConch)
            {
               detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(expectedFiducialSize.getDoubleValue()),
                                                       ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10), GrayF32.class);
            }
         }
      });

      // fov values from http://carnegierobotics.com/multisense-s7/
      fieldOfViewXinRadians.set(Math.toRadians(80.0));
      fieldOfViewYinRadians.set(Math.toRadians(45.0));

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
            transformToParent.set(0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0);
         }
      };

      locatedFiducialReferenceFrame = new ReferenceFrame(prefix + "LocatedReferenceFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            locatedFiducialPoseInWorldFrame.get(transformToParent);
         }
      };

      reportedFiducialReferenceFrame = new TransformReferenceFrame(prefix + "ReportedReferenceFrame", locatedFiducialReferenceFrame,
                                                                   transformFromReportedToFiducialFrame);

      if (yoGraphicsListRegistry == null)
      {
         visualize = false;
      }

      if (visualize)
      {
         cameraGraphic = new YoGraphicReferenceFrame(cameraReferenceFrame, registry, true, 0.5);
         detectorGraphic = new YoGraphicReferenceFrame(detectorReferenceFrame, registry, true, 1.0);
         locatedFiducialGraphic = new YoGraphicReferenceFrame(locatedFiducialReferenceFrame, registry, true, 0.1);
         reportedFiducialGraphic = new YoGraphicReferenceFrame(reportedFiducialReferenceFrame, registry, true, 0.2);

         //         yoGraphicsListRegistry.registerYoGraphic("Fiducials", cameraGraphic);
         //         yoGraphicsListRegistry.registerYoGraphic("Fiducials", detectorGraphic);
         //         yoGraphicsListRegistry.registerYoGraphic("Fiducials", locatedFiducialGraphic);
         yoGraphicsListRegistry.registerYoGraphic("Fiducials", reportedFiducialGraphic);
      }
      else
      {
         cameraGraphic = detectorGraphic = locatedFiducialGraphic = reportedFiducialGraphic = null;
      }

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      this.targetIDHasBeenLocated.set(false);
   }

   public FiducialDetector<GrayF32> getFiducialDetector()
   {
      return detector;
   }

   public void detectFromVideoPacket(VideoPacket videoPacket)
   {
      detect(videoPacket);
   }

   private JFrame frame;
   private ImageIcon image;

   public void detect(VideoPacket videoPacket)
   {
      
      
      BufferedImage bufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(videoPacket.getData().toArray());
    // videoPacket.getPosition(), videoPacket.getOrientation());
      
      
      //increase brightness for sim
      RescaleOp rescaleOp = new RescaleOp(1.9f, 35, null);
      rescaleOp.filter(bufferedImage, bufferedImage);  // Source and destination are the same.
      if (DEBUG)
      {
         if (frame == null)
         {
            frame = new JFrame();

            frame.getContentPane().setLayout(new FlowLayout());

            image = new ImageIcon(bufferedImage);
            frame.getContentPane().add(new JLabel(image));

            frame.pack();
            frame.setVisible(true);
         }
      }
      synchronized (expectedFiducialSizeChangedConch)
      {
         
         
         //setIntrinsicParameters(bufferedImage);

         if(false)
         {
            int height = bufferedImage.getHeight();
            int width = bufferedImage.getWidth();

            double fx = (width / 2.0) / Math.tan(fieldOfViewXinRadians.getDoubleValue() / 2.0);
            double fy = (height / 2.0) / Math.tan(fieldOfViewYinRadians.getDoubleValue() / 2.0);

            intrinsicParameters.width = width;
            intrinsicParameters.height = height;
            intrinsicParameters.cx = width / 2.0;
            intrinsicParameters.cy = height / 2.0;
            intrinsicParameters.fx = fx;
            intrinsicParameters.fy = fy;

            detector.setIntrinsic(intrinsicParameters);
         }
         else
         {
         
         detector.setIntrinsic(HumanoidMessageTools.toIntrinsicParameters(videoPacket.getIntrinsicParameters()));
         } 
         cameraRigidTransform.setRotation(videoPacket.getOrientation());
         cameraRigidPosition.set(videoPacket.getPosition());
         cameraRigidTransform.setTranslation(cameraRigidPosition);

         cameraReferenceFrame.update();
         detectorReferenceFrame.update();

         cameraPose.setOrientation(videoPacket.getOrientation());
         cameraPose.setPosition(videoPacket.getPosition());

         GrayF32 grayImage = ConvertBufferedImage.convertFrom(bufferedImage, true, ImageType.single(GrayF32.class));

         
         if (DEBUG)
         {
            image.setImage(ConvertBufferedImage.convertTo(grayImage, null));
            frame.setVisible(true);
         }
         
         
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
            tempFiducialRotationQuat.set(fiducialRotationMatrix);

            tempFiducialDetectorFrame.setToZero(detectorReferenceFrame);
            tempFiducialDetectorFrame.setOrientation(tempFiducialRotationQuat);
            tempFiducialDetectorFrame.setPosition(fiducialToCamera.getX(), fiducialToCamera.getY(), fiducialToCamera.getZ());
            tempFiducialDetectorFrame.changeFrame(ReferenceFrame.getWorldFrame());

            locatedFiducialPoseInWorldFrame.set(tempFiducialDetectorFrame);

            locatedFiducialReferenceFrame.update();

            tempFiducialDetectorFrame.setToZero(reportedFiducialReferenceFrame);
            tempFiducialDetectorFrame.changeFrame(ReferenceFrame.getWorldFrame());

            reportedFiducialPoseInWorldFrame.set(tempFiducialDetectorFrame);

            targetIDHasBeenLocated.set(true);

            if (visualize)
            {
               cameraGraphic.update();
               detectorGraphic.update();
               locatedFiducialGraphic.update();
               reportedFiducialGraphic.update();
            }
         }
         else
         {
            targetIDHasBeenLocated.set(false);
         }
      }

      targetIDHasBeenLocatedFiltered.update();
   }

   private final IntrinsicParameters intrinsicParameters = new IntrinsicParameters();

  /* private IntrinsicParameters setIntrinsicParameters(BufferedImage image)
   {
      int height = image.getHeight();
      int width = image.getWidth();

      double fx = (width / 2.0) / Math.tan(fieldOfViewXinRadians.getDoubleValue() / 2.0);
      double fy = (height / 2.0) / Math.tan(fieldOfViewYinRadians.getDoubleValue() / 2.0);

      intrinsicParameters.width = width;
      intrinsicParameters.height = height;
      intrinsicParameters.cx = width / 2.0;
      intrinsicParameters.cy = height / 2.0;
      intrinsicParameters.fx = fx;
      intrinsicParameters.fy = fy;

      detector.setIntrinsic(intrinsicParameters);
      this.targetIDHasBeenLocated.set(false);

      return intrinsicParameters;
   }*/

   public void setExpectedFiducialSize(double expectedFiducialSize)
   {
      this.expectedFiducialSize.set(expectedFiducialSize);
      this.targetIDHasBeenLocated.set(false);
   }

   public void setTargetIDToLocate(long targetIDToLocate)
   {
      this.targetIDToLocate.set(targetIDToLocate);
      this.targetIDHasBeenLocated.set(false);
   }

   public boolean getTargetIDHasBeenLocated()
   {
      return targetIDHasBeenLocated.getBooleanValue();
   }

   public void getReportedFiducialPoseWorldFrame(FramePose3D framePoseToPack)
   {
      framePoseToPack.setIncludingFrame(reportedFiducialPoseInWorldFrame);
   }

   public void setFieldOfView(double fieldOfViewXinRadians, double fieldOfViewYinRadians)
   {
      this.fieldOfViewXinRadians.set(fieldOfViewXinRadians);
      this.fieldOfViewYinRadians.set(fieldOfViewYinRadians);
      this.targetIDHasBeenLocated.set(false);
   }

}
