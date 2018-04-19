package us.ihmc.quadrupedRobotics.planning.bodyPath;

import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.QuadrupedBodyPathPlanMessage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedWaypointBasedBodyPathProvider implements QuadrupedPlanarBodyPathProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final AtomicReference<QuadrupedBodyPathPlanMessage> bodyPathPlanMessage = new AtomicReference<>();
   private final YoDouble timestamp;
   private final MultipleWaypointsPositionTrajectoryGenerator trajectory = new MultipleWaypointsPositionTrajectoryGenerator("bodyPath", worldFrame, registry);

   private final ReferenceFrame supportFrame;
   private final FramePose3D supportPose = new FramePose3D();

   public QuadrupedWaypointBasedBodyPathProvider(QuadrupedReferenceFrames referenceFrames, PacketCommunicator packetCommunicator, YoDouble timestamp, YoVariableRegistry parentRegistry)
   {
      this.timestamp = timestamp;
      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      packetCommunicator.attachListener(QuadrupedBodyPathPlanMessage.class, bodyPathPlanMessage::set);
      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      trajectory.clear();
      appendCurrentPoseAsWaypoint();

      QuadrupedBodyPathPlanMessage bodyPathPlanMessage = this.bodyPathPlanMessage.getAndSet(null);
      List<EuclideanTrajectoryPointMessage> originalBodyPathPoints = bodyPathPlanMessage.getBodyPathPoints();

      double currentTime = timestamp.getDoubleValue();
      boolean isExpressedInAbsoluteTime = bodyPathPlanMessage.getIsExpressedInAbsoluteTime();
      for (int i = 0; i < originalBodyPathPoints.size(); i++)
      {
         EuclideanTrajectoryPointMessage waypoint = originalBodyPathPoints.get(i);

         if(!isExpressedInAbsoluteTime)
         {
            waypoint.setTime(waypoint.getTime() + currentTime);
         }

         if(waypoint.getTime() < currentTime)
         {
            continue;
         }
         else
         {
            trajectory.appendWaypoint(waypoint.getTime(), waypoint.getPosition(), waypoint.getLinearVelocity());
         }
      }

      trajectory.clear();
   }

   private void appendCurrentPoseAsWaypoint()
   {
      supportPose.setToZero(supportFrame);
      supportPose.changeFrame(worldFrame);

      trajectory.appendWaypoint(timestamp.getDoubleValue(), new Point3D(supportPose.getX(), supportPose.getY(), supportPose.getYaw()), new Vector3D());
   }

   private final FramePoint3D trajectoryValue = new FramePoint3D();

   @Override
   public void getPlanarPose(double time, FramePose2D poseToPack)
   {
      trajectory.compute(time);
      trajectory.getPosition(trajectoryValue);
      poseToPack.setIncludingFrame(worldFrame, trajectoryValue.getX(), trajectoryValue.getY(), trajectoryValue.getZ());
   }

   public boolean bodyPathIsAvailable()
   {
      return bodyPathPlanMessage.get() != null;
   }

   public boolean isDone()
   {
      return trajectory.isDone();
   }
}
