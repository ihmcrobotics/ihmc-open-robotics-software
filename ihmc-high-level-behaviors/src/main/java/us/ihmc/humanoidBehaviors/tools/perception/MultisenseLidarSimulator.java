package us.ihmc.humanoidBehaviors.tools.perception;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.ArrayList;
import java.util.function.Consumer;

public class MultisenseLidarSimulator
{
   public static final double THREAD_PERIOD = UnitConversions.hertzToSeconds(10.0);

   private final RemoteSyncedRobotModel syncedRobot;
   private final PlanarRegionsList map;
   private final ArrayList<Consumer<ArrayList<Point3DReadOnly>>> scanListeners = new ArrayList<>();
   private final MovingReferenceFrame neckFrame;
   private final YawPitchRoll sensorRoll = new YawPitchRoll();
   private final PoseReferenceFrame sensorFrame;
   private final FramePose3D sensorPose = new FramePose3D();
   private final Vector3D rangeRay = new Vector3D();

   private final FramePose3D sensorPoseForUser = new FramePose3D();

   private final double fov = Math.toRadians(100.0);
   private final double range = 5.0;
   private final int scanSize;
   private final double angularVelocity = 2.183;

   public MultisenseLidarSimulator(DRCRobotModel robotModel, ROS2Node ros2Node, PlanarRegionsList map, int scanSize)
   {
      this.map = map;
      this.scanSize = scanSize;
      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      neckFrame = syncedRobot.getReferenceFrames().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
      sensorFrame = new PoseReferenceFrame("LidarSensorFrame", neckFrame); // TODO: Add actual Multisense offset

      new PausablePeriodicThread("SpinSensorThread", THREAD_PERIOD, 0, true, this::spinAndUpdate).start();
   }

   private void spinAndUpdate()
   {
      if (syncedRobot.hasReceivedFirstMessage())
      {
         syncedRobot.update();
         sensorRoll.addRoll(angularVelocity * THREAD_PERIOD);
         sensorFrame.setOrientationAndUpdate(sensorRoll);

         ArrayList<Point3DReadOnly> scan = new ArrayList<>();

         // TODO: Calculate sections in parallel

         double maxYaw = fov / 2.0;
         double minYaw = -maxYaw;
         double angleIncrement = fov / scanSize;
         for (double yaw = minYaw; yaw <= maxYaw; yaw += angleIncrement)
         {
            sensorPose.setToZero(sensorFrame);
            sensorPose.getOrientation().setToYawOrientation(yaw);
            sensorPose.changeFrame(ReferenceFrame.getWorldFrame());

            rangeRay.set(Axis3D.X);
            sensorPose.getOrientation().transform(rangeRay);

            ImmutablePair<Point3D, PlanarRegion> planarRegionIntersection = PlanarRegionTools.intersectRegionsWithRay(map,
                                                                                                                      sensorPose.getPosition(),
                                                                                                                      rangeRay);
            if (planarRegionIntersection == null
             || planarRegionIntersection.getLeft().distance(sensorPose.getPosition()) > range)
               continue;

            scan.add(planarRegionIntersection.getLeft());
         }

         for (Consumer<ArrayList<Point3DReadOnly>> scanListener : scanListeners)
         {
            scanListener.accept(scan);
         }
      }
   }

   public void addLidarScanListener(Consumer<ArrayList<Point3DReadOnly>> scanConsumer)
   {
      scanListeners.add(scanConsumer);
   }

   public Pose3DReadOnly getSensorPose()
   {
      sensorPoseForUser.setToZero(sensorFrame);
      sensorPoseForUser.changeFrame(ReferenceFrame.getWorldFrame());
      return sensorPoseForUser;
   }
}
