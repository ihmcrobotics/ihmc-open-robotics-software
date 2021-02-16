package us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox;

import java.awt.FlowLayout;
import java.awt.image.BufferedImage;
import java.awt.image.RescaleOp;
import java.util.concurrent.atomic.AtomicReference;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.alg.distort.pinhole.LensDistortionPinhole;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import controller_msgs.msg.dds.DetectedFiducialPacket;
import controller_msgs.msg.dds.VideoPacket;
import georegression.struct.se.Se3_F64;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.tools.Timer;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FiducialDetectorToolboxController extends ToolboxController
{
   private static final RescaleOp imageRescalingForSim = new RescaleOp(3.5f, 35, null);
   private static final RescaleOp imageRescalingForRealRobot = new RescaleOp(1.5f, 35, null);
   private static RescaleOp imageRescalingOperation;

   private final AtomicReference<VideoPacket> videoPacket = new AtomicReference<VideoPacket>();

   private final Timer statusTimer = new Timer();

   //debugging only
   private boolean DEBUG = false;
   private JFrame frame;
   private ImageIcon image;
   //*************

   private final Se3_F64 fiducialToCamera = new Se3_F64();
   private final RotationMatrix fiducialRotationMatrix = new RotationMatrix();
   private final Vector3D cameraRigidPosition = new Vector3D();
   private final RigidBodyTransform cameraRigidTransform = new RigidBodyTransform();

   private final ReferenceFrame cameraReferenceFrame, detectorReferenceFrame;

   private FiducialDetector<GrayF32> detector;

   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();

   private String prefix = "fiducial";

   //standard size printed on normal paper
   private double expectedFiducialSize = 0.2032;

   private final FramePose3D cameraPose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final FramePose3D reportedFiducialPoseInWorldFrame = new FramePose3D(ReferenceFrame.getWorldFrame());
   private AtomicReference<Boolean> inProcessingThread = new AtomicReference<Boolean>();

   public FiducialDetectorToolboxController(FullHumanoidRobotModel fullRobotModel,
                                            RobotTarget target,
                                            StatusMessageOutputManager statusOutputManager,
                                            YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      imageRescalingOperation = target == RobotTarget.REAL_ROBOT ? imageRescalingForRealRobot : imageRescalingForSim;

      inProcessingThread.set(false);
      detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(expectedFiducialSize), ConfigThreshold.local(ThresholdType.LOCAL_GAUSSIAN, 10),
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
   }

   @Override
   public boolean initialize()
   {
      LogTools.info("Initializing");
      return true;
   }

   public void receivedPacket(VideoPacket packet)
   {
      if (packet != null)
         videoPacket.set(packet);
   }

   @Override
   public void updateInternal()
   {
      if (!inProcessingThread.get())
      {
         VideoPacket latest = videoPacket.getAndSet(null);
         if (latest != null)
         {

            Thread packetPRocessor = new Thread(new Runnable()
            {
               @Override
               public void run()
               {
                  inProcessingThread.set(true);
                  detectFromVideoPacket(latest);
                  inProcessingThread.set(false);
               }
            }, "FiducialDetectorToolboxController image processing");
            packetPRocessor.start();
         }
      }
   }

   private void detectFromVideoPacket(VideoPacket videoPacket)
   {
      BufferedImage bufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(videoPacket.getData().toArray());
      detect(bufferedImage, videoPacket.getPosition(), videoPacket.getOrientation(),
             HumanoidMessageTools.toIntrinsicParameters(videoPacket.getIntrinsicParameters()));

   }

   private void detect(BufferedImage bufferedImage, Point3DReadOnly cameraPositionInWorld, QuaternionReadOnly cameraOrientationInWorldXForward,
                       CameraPinhole intrinsicParameters)
   {
      detector.setLensDistortion(new LensDistortionPinhole(intrinsicParameters), intrinsicParameters.getWidth(), intrinsicParameters.getHeight());

      imageRescalingOperation.filter(bufferedImage, bufferedImage); // Source and destination are the same.

      if (DEBUG)
      {
         if (frame == null)
         {
            System.out.println("FiducialDetectorToolBoxController: Debug True starting jframe");
            frame = new JFrame();

            frame.getContentPane().setLayout(new FlowLayout());

            image = new ImageIcon(bufferedImage);
            frame.getContentPane().add(new JLabel(image));

            frame.pack();
            frame.setVisible(true);
         }
      }

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
         frame.pack();
         frame.repaint();
      }

      detector.detect(grayImage);

      for (int i = 0; i < detector.totalFound(); i++)
      {

         detector.getFiducialToCamera(i, fiducialToCamera);

         fiducialRotationMatrix.set(fiducialToCamera.getR().data);

         reportedFiducialPoseInWorldFrame.setReferenceFrame(detectorReferenceFrame);
         reportedFiducialPoseInWorldFrame.getOrientation().set(fiducialRotationMatrix);
         reportedFiducialPoseInWorldFrame.getPosition().set(fiducialToCamera.getX(), fiducialToCamera.getY(), fiducialToCamera.getZ());
         reportedFiducialPoseInWorldFrame.changeFrame(ReferenceFrame.getWorldFrame());

         DetectedFiducialPacket packet = new DetectedFiducialPacket();
         packet.fiducial_id_ = detector.getId(i);

         Pose3D pose = new Pose3D(reportedFiducialPoseInWorldFrame.getPosition(), reportedFiducialPoseInWorldFrame.getOrientation());

         if (!statusTimer.isRunning(5.0))
         {
            LogTools.info("Found fiducial: id: {} pose: {}", packet.getFiducialId(), pose);
            statusTimer.reset();
         }

         packet.fiducial_transform_to_world_ = pose;

         reportMessage(packet);

      }

   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}
