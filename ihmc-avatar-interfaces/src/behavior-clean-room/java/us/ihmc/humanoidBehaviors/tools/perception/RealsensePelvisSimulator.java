package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.SimulatedDepthCamera;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class RealsensePelvisSimulator implements Supplier<PlanarRegionsList>
{
   private final TransformReferenceFrame realsenseSensorFrame;
   private volatile PlanarRegionsList map;

   private RemoteSyncedRobotModel syncedRobot;
   private MovingReferenceFrame pelvisFrame;
   private SimulatedDepthCamera simulatedDepthCamera;

   private static final double depthOffsetX = 0.058611;
   private static final double depthOffsetZ = 0.01;
   private static final double depthPitchingAngle = 70.0 / 180.0 * Math.PI;
   private static final double depthThickness = 0.0245;
   private static final double pelvisToMountOrigin = 0.19;

   public static final RigidBodyTransform transform = new RigidBodyTransform();
   static
   {
      transform.appendTranslation(pelvisToMountOrigin, 0.0, 0.0);
      transform.appendTranslation(depthOffsetX, 0.0, depthOffsetZ);
      transform.appendPitchRotation(depthPitchingAngle);
      transform.appendTranslation(depthThickness, 0.0, 0.0);

      // Real robot Realsense D435 sensor has an additional frame change to convert to camera frame
      // which is Z forward instead of X forward. We omit that here because the SimulatedDepthCamera
      // algorithm operates if X forward frame (same as robot)
   }

   private final FramePose3D tempSensorFramePose = new FramePose3D();

   public RealsensePelvisSimulator(PlanarRegionsList map, DRCRobotModel robotModel, ROS2NodeInterface ros2Node)
   {
      this(map, robotModel, ros2Node, 1.5, 30000);
   }

   public RealsensePelvisSimulator(PlanarRegionsList map, DRCRobotModel robotModel, ROS2NodeInterface ros2Node, double range, int sphereScanSize)
   {
      this.map = map;

      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      pelvisFrame = syncedRobot.getReferenceFrames().getPelvisFrame();

      realsenseSensorFrame = new TransformReferenceFrame("Realsense", pelvisFrame, transform);

      double verticalFOV = 80.0; // real fov is 58.0;
      double horizontalFOV = 87.0;
      simulatedDepthCamera = new SimulatedDepthCamera(verticalFOV, horizontalFOV, range, sphereScanSize, realsenseSensorFrame);
   }

   public List<Point3DReadOnly> getPointCloud()
   {
      syncedRobot.update();

      List<Point3DReadOnly> pointCloud;
      if (syncedRobot.hasReceivedFirstMessage())
      {
         pointCloud = simulatedDepthCamera.computePointCloudFrame(map);
      }
      else
      {
         // blank result
         pointCloud = new ArrayList<>();
      }

      return pointCloud;
   }

   public Pose3DReadOnly getSensorPose()
   {
      tempSensorFramePose.setToZero(realsenseSensorFrame);
      tempSensorFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      return tempSensorFramePose;
   }

   public PlanarRegionsList computeRegions()
   {
      syncedRobot.update();

      if (syncedRobot.hasReceivedFirstMessage())
      {
         return simulatedDepthCamera.computeAndPolygonize(map);
      }
      else
      {
         // blank result
         return new PlanarRegionsList();
      }
   }

   @Override
   public PlanarRegionsList get()
   {
      return computeRegions();
   }

   public void setMap(PlanarRegionsList map)
   {
      this.map = map;
   }
}
