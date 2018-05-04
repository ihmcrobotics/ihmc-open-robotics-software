package us.ihmc.ihmcPerception.objectDetector;

import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import controller_msgs.msg.dds.BoundingBoxesPacket;
import controller_msgs.msg.dds.HeatMapPacket;
import controller_msgs.msg.dds.ObjectDetectorResultPacket;
import controller_msgs.msg.dds.VideoPacket;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point2D_F64;
import georegression.struct.se.Se3_F64;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;

public class ObjectDetectorFromCameraImages implements PacketConsumer<ObjectDetectorResultPacket>, ConnectionStateListener
{
   private boolean visualize = true;

   private final Se3_F64 fiducialToCamera = new Se3_F64();
   private final RotationMatrix fiducialRotationMatrix = new RotationMatrix();
   private final Quaternion tempFiducialRotationQuat = new Quaternion();
   private final FramePose3D tempFiducialDetectorFrame = new FramePose3D();
   private final Vector3D cameraRigidPosition = new Vector3D();
   private final double[] eulerAngles = new double[3];
   private final RigidBodyTransform cameraRigidTransform = new RigidBodyTransform();

   private final ReferenceFrame cameraReferenceFrame, detectorReferenceFrame, locatedFiducialReferenceFrame, reportedFiducialReferenceFrame;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final Object expectedFiducialSizeChangedConch = new Object();

   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();

   private final YoGraphicReferenceFrame cameraGraphic, detectorGraphic, locatedFiducialGraphic, reportedFiducialGraphic;

   private final String prefix = "object";

   private final YoDouble expectedObjectSize = new YoDouble("expectedObjectSize", registry);
   private final YoDouble fieldOfViewXinRadians = new YoDouble("fovXRadians", registry);
   private final YoDouble fieldOfViewYinRadians = new YoDouble("fovYRadians", registry);

   private final YoDouble detectorPositionX = new YoDouble(prefix + "DetectorPositionX", registry);
   private final YoDouble detectorPositionY = new YoDouble(prefix + "DetectorPositionY", registry);
   private final YoDouble detectorPositionZ = new YoDouble(prefix + "DetectorPositionZ", registry);
   private final YoDouble detectorEulerRotX = new YoDouble(prefix + "DetectorEulerRotX", registry);
   private final YoDouble detectorEulerRotY = new YoDouble(prefix + "DetectorEulerRotY", registry);
   private final YoDouble detectorEulerRotZ = new YoDouble(prefix + "DetectorEulerRotZ", registry);

   private final YoBoolean targetIDHasBeenLocated = new YoBoolean(prefix + "TargetIDHasBeenLocated", registry);

   private final YoFramePose3D cameraPose = new YoFramePose3D(prefix + "CameraPoseWorld", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D locatedFiducialPoseInWorldFrame = new YoFramePose3D(prefix + "LocatedPoseWorldFrame", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D reportedFiducialPoseInWorldFrame = new YoFramePose3D(prefix + "ReportedPoseWorldFrame", ReferenceFrame.getWorldFrame(), registry);

   private final AtomicBoolean detectionRunning = new AtomicBoolean(false);
   private final List<Consumer<DetectionVisualizationPackets>> detectionResultListeners = Collections.synchronizedList(new ArrayList<>());

   private final ConcurrentLinkedQueue<ObjectDetectorResultPacket> results = new ConcurrentLinkedQueue<>();
   private final PacketCommunicator valveDetectorClient = PacketCommunicator.createTCPPacketCommunicatorClient("10.7.4.103", NetworkPorts.VALVE_DETECTOR_SERVER_PORT, new IHMCCommunicationKryoNetClassList());

   public ObjectDetectorFromCameraImages(RigidBodyTransform transformFromReportedToFiducialFrame, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry) throws Exception
   {
      this.expectedObjectSize.set(1.0);
      targetIDHasBeenLocated.set(true);

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
         reportedFiducialGraphic = new YoGraphicReferenceFrame(reportedFiducialReferenceFrame, registry, 0.2);

         //         yoGraphicsListRegistry.registerYoGraphic("Fiducials", cameraGraphic);
         //         yoGraphicsListRegistry.registerYoGraphic("Fiducials", detectorGraphic);
         //         yoGraphicsListRegistry.registerYoGraphic("Fiducials", locatedFiducialGraphic);
//         yoGraphicsListRegistry.registerYoGraphic("Fiducials", reportedFiducialGraphic);
      }
      else
      {
         cameraGraphic = detectorGraphic = locatedFiducialGraphic = reportedFiducialGraphic = null;
      }

      parentRegistry.addChild(registry);

      PrintTools.info(this, "Attempting to connect to valve detector...");
      valveDetectorClient.attachStateListener(this);
      valveDetectorClient.connect();
   }

   public void reset()
   {
      this.targetIDHasBeenLocated.set(false);
   }

   public void addDetectionResultListener(Consumer<DetectionVisualizationPackets> resultListener)
   {
      detectionResultListeners.add(resultListener);
   }

   public void detectFromVideoPacket(VideoPacket videoPacket)
   {

      detectAsync(videoPacket);
   }

   private void detectAsync(VideoPacket videoPacket)
   {
      if (detectionRunning.get())
         return;

      valveDetectorClient.send(videoPacket);

      CompletableFuture.runAsync(() ->
      {
         detectionRunning.set(true);
         try
         {
            while(results.peek() == null)
            {
               ThreadTools.sleep(5);
            }
            BufferedImage latestUnmodifiedCameraImage = jpegDecompressor.decompressJPEGDataToBufferedImage(videoPacket.getData().toArray());
            detect(latestUnmodifiedCameraImage, videoPacket.getPosition(), videoPacket.getOrientation());
         } finally
         {
            detectionRunning.set(false);
         }
      });
   }

   public void detect(BufferedImage bufferedImage, Point3DReadOnly cameraPositionInWorld, QuaternionReadOnly cameraOrientationInWorldXForward)
   {
      synchronized (expectedFiducialSizeChangedConch)
      {
         DenseMatrix64F pixelToNorm = computePixelToNorm(bufferedImage);

         cameraRigidTransform.setRotation(cameraOrientationInWorldXForward);
         cameraRigidPosition.set(cameraPositionInWorld);
         cameraRigidTransform.setTranslation(cameraRigidPosition);

         cameraReferenceFrame.update();
         detectorReferenceFrame.update();

         cameraPose.setOrientation(cameraOrientationInWorldXForward);
         cameraPose.setPosition(cameraPositionInWorld);

         ObjectDetectorResultPacket result = results.poll();

//         Pair<List<Rectangle>, HeatMap> rectanglesAndHeatMaps = detector.detect(bufferedImage);
//
//         rectanglesAndHeatMaps.getLeft().sort((r1, r2) -> -Integer.compare(r1.width * r1.height, r2.width * r2.height));
//
//         HeatMapPacket heatMapPacket = new HeatMapPacket();
//         heatMapPacket.width = rectanglesAndHeatMaps.getRight().w;
//         heatMapPacket.height = rectanglesAndHeatMaps.getRight().h;
//         heatMapPacket.data = rectanglesAndHeatMaps.getRight().data;
//         heatMapPacket.name = "Valve";
//
//         int[] packedBoxes = rectanglesAndHeatMaps.getLeft().stream().flatMapToInt(rect -> IntStream.of(rect.x, rect.y, rect.width, rect.height)).toArray();
//         String[] names = new String[rectanglesAndHeatMaps.getLeft().size()];
//         for (int i = 0; i < names.length; i++)
//         {
//            names[i] = "Valve " + i;
//         }
//         BoundingBoxesPacket boundingBoxesPacket = new BoundingBoxesPacket(packedBoxes, names);

         BoundingBoxesPacket boundingBoxes = result.getBoundingBoxes();
         HeatMapPacket heatMap = result.getHeatMap();
         DetectionVisualizationPackets coactiveVisualizationPackets = new DetectionVisualizationPackets(boundingBoxes, heatMap);
         detectionResultListeners.forEach(consumer -> consumer.accept(coactiveVisualizationPackets));

         if (boundingBoxes.getLabels().size() > 0)
         {
            Rectangle rectangle = new Rectangle(boundingBoxes.getBoundingBoxesXCoordinates().get(0), boundingBoxes.getBoundingBoxesYCoordinates().get(0), boundingBoxes.getBoundingBoxesWidths().get(0), boundingBoxes.getBoundingBoxesHeights().get(0));
            double knownWidth = expectedObjectSize.getDoubleValue();
            Point2D_F64 topLeft = new Point2D_F64(rectangle.x, rectangle.y);
            Point2D_F64 bottomRight = new Point2D_F64(rectangle.x + rectangle.width, rectangle.y + rectangle.height);
            if (Math.abs(topLeft.x - bottomRight.x) < 1e-5)
            {
               targetIDHasBeenLocated.set(false);
               return;
            }
            GeometryMath_F64.mult(pixelToNorm, topLeft, topLeft);
            GeometryMath_F64.mult(pixelToNorm, bottomRight, bottomRight);
            double distance = knownWidth * 0.66f / Math.abs(bottomRight.getX() - topLeft.getX()); // TODO: detectNet output is about 1/3 smaller than the real object, this is a quick hack for now, should re-train the network
            fiducialToCamera.setTranslation(0, 0, distance);
            fiducialToCamera.setRotation(new DenseMatrix64F(3, 3, true, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0));

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
   }

   private DenseMatrix64F computePixelToNorm(BufferedImage image)
   {
      int height = image.getHeight();
      int width = image.getWidth();

      double fx = (width / 2.0) / Math.tan(fieldOfViewXinRadians.getDoubleValue() / 2.0);
      double fy = (height / 2.0) / Math.tan(fieldOfViewYinRadians.getDoubleValue() / 2.0);

      DenseMatrix64F K_inv = new DenseMatrix64F(3, 3);
      K_inv.set(0,0,fx);
      K_inv.set(1,1,fy);
      K_inv.set(0,1,0);
      K_inv.set(0,2,width / 2.0);
      K_inv.set(1,2,height / 2.0);
      K_inv.set(2,2,1);

      CommonOps.invert(K_inv);

      return K_inv;
   }

   public void setExpectedObjectSize(double expectedObjectSize)
   {
      this.expectedObjectSize.set(expectedObjectSize);
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

   private static class DetectionParameters
   {
      final BufferedImage bufferedImage;
      final Point3D cameraPositionInWorld;
      final Quaternion cameraOrientationInWorldXForward;

      private DetectionParameters(BufferedImage bufferedImage, Point3D cameraPositionInWorld, Quaternion cameraOrientationInWorldXForward)
      {
         this.bufferedImage = bufferedImage;
         this.cameraPositionInWorld = new Point3D(cameraPositionInWorld);
         this.cameraOrientationInWorldXForward = new Quaternion(cameraOrientationInWorldXForward);
      }
   }

   public static class DetectionVisualizationPackets
   {
      private final BoundingBoxesPacket boundingBoxesPacket;
      private final HeatMapPacket heatMapPacket;

      public DetectionVisualizationPackets(BoundingBoxesPacket boundingBoxesPacket, HeatMapPacket heatMapPacket)
      {
         this.boundingBoxesPacket = boundingBoxesPacket;
         this.heatMapPacket = heatMapPacket;
      }

      public BoundingBoxesPacket getBoundingBoxesPacket()
      {
         return boundingBoxesPacket;
      }

      public HeatMapPacket getHeatMapPacket()
      {
         return heatMapPacket;
      }
   }

   @Override
   public void connected()
   {
      PrintTools.info("Connected to Valve Detector.");
   }

   @Override
   public void disconnected()
   {
      PrintTools.info("Disconnected from Valve Detector.");
   }

   @Override
   public void receivedPacket(ObjectDetectorResultPacket packet)
   {
      results.add(packet);
   }
}
