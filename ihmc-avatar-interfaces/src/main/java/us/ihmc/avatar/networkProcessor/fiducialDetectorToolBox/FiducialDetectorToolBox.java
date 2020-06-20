package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

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
import georegression.struct.se.Se3_F64;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;

public class FiducialDetectorToolBox
{
   private boolean DEBUG = false;

   private final Se3_F64 fiducialToCamera = new Se3_F64();
   private final RotationMatrix fiducialRotationMatrix = new RotationMatrix();
   private final Quaternion tempFiducialRotationQuat = new Quaternion();
   private final FramePose3D tempFiducialDetectorFrame = new FramePose3D();
   private final Vector3D cameraRigidPosition = new Vector3D();
   private final double[] eulerAngles = new double[3];
   private final RigidBodyTransform cameraRigidTransform = new RigidBodyTransform();

   private final ReferenceFrame cameraReferenceFrame, detectorReferenceFrame, locatedFiducialReferenceFrame, reportedFiducialReferenceFrame;

   private FiducialDetector<GrayF32> detector;
   private Object expectedFiducialSizeChangedConch = new Object();

   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();

   private String prefix = "fiducial";

   private double expectedFiducialSize;

   private final FramePose3D cameraPose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final FramePose3D locatedFiducialPoseInWorldFrame = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final FramePose3D reportedFiducialPoseInWorldFrame = new FramePose3D(ReferenceFrame.getWorldFrame());
   private RigidBodyTransform transformFromReportedToFiducialFrame = new RigidBodyTransform();

   public FiducialDetectorToolBox()
   {

      detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(expectedFiducialSize),
                                              ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10),
                                              GrayF32.class);

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

      reportedFiducialReferenceFrame = new TransformReferenceFrame(prefix + "ReportedReferenceFrame",
                                                                   locatedFiducialReferenceFrame,
                                                                   transformFromReportedToFiducialFrame);

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
      detect(bufferedImage,
             videoPacket.getPosition(),
             videoPacket.getOrientation(),
             HumanoidMessageTools.toIntrinsicParameters(videoPacket.getIntrinsicParameters()));

   }

   public void detect(BufferedImage bufferedImage, Point3DReadOnly cameraPositionInWorld, QuaternionReadOnly cameraOrientationInWorldXForward,
                      IntrinsicParameters intrinsicParameters)
   {
      detector.setIntrinsic(intrinsicParameters);
      //increase brightness for sim
      RescaleOp rescaleOp = new RescaleOp(2.5f, 35, null);
      rescaleOp.filter(bufferedImage, bufferedImage); // Source and destination are the same.
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

         cameraRigidTransform.getRotation().set(cameraOrientationInWorldXForward);
         cameraRigidPosition.set(cameraPositionInWorld);
         cameraRigidTransform.getTranslation().set(cameraRigidPosition);

         cameraReferenceFrame.update();
         detectorReferenceFrame.update();

         cameraPose.getOrientation().set(cameraOrientationInWorldXForward);
         cameraPose.getPosition().set(cameraPositionInWorld);

         GrayF32 grayImage = ConvertBufferedImage.convertFrom(bufferedImage, true, ImageType.single(GrayF32.class));

         if (DEBUG)
         {
            image.setImage(ConvertBufferedImage.convertTo(grayImage, null));
            frame.setVisible(true);
         }

         detector.detect(grayImage);

         for (int i = 0; i < detector.totalFound(); i++)
         {

            detector.getFiducialToCamera(i, fiducialToCamera);

            fiducialRotationMatrix.set(fiducialToCamera.getR().data);
            tempFiducialRotationQuat.set(fiducialRotationMatrix);

            tempFiducialDetectorFrame.setToZero(detectorReferenceFrame);
            tempFiducialDetectorFrame.getOrientation().set(tempFiducialRotationQuat);
            tempFiducialDetectorFrame.getPosition().set(fiducialToCamera.getX(), fiducialToCamera.getY(), fiducialToCamera.getZ());
            tempFiducialDetectorFrame.changeFrame(ReferenceFrame.getWorldFrame());

            locatedFiducialPoseInWorldFrame.set(tempFiducialDetectorFrame);

            locatedFiducialReferenceFrame.update();

            tempFiducialDetectorFrame.setToZero(reportedFiducialReferenceFrame);
            tempFiducialDetectorFrame.changeFrame(ReferenceFrame.getWorldFrame());

            reportedFiducialPoseInWorldFrame.set(tempFiducialDetectorFrame);

            //TODO send out a detected Fiducial packet containing ID, Pose, And Timestamp

         }

      }

   }

   public void setExpectedFiducialSize(double expectedFiducialSize)
   {
      this.expectedFiducialSize = expectedFiducialSize;
      detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(expectedFiducialSize),
                                              ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 10),
                                              GrayF32.class);

   }

   public void getReportedFiducialPoseWorldFrame(FramePose3D framePoseToPack)
   {
      framePoseToPack.setIncludingFrame(reportedFiducialPoseInWorldFrame);
   }

}
