package us.ihmc.rdx.ui.teleoperation;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class RDXScriptedTrajectoryStreamer
{
   private final double trajectoryTime;
   private final SideDependentList<MultipleWaypointsPoseTrajectoryGenerator> multiWaypointTrajectories = new SideDependentList<>();
   private boolean isDone = false;
   private boolean initialize = false;
   private final SideDependentList<FramePoint3D> currentHandPositions = new SideDependentList<>(side -> new FramePoint3D());
   private final SideDependentList<FramePose3DReadOnly> currentHandPoses = new SideDependentList<>(side -> new FramePose3D());

   public RDXScriptedTrajectoryStreamer(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      for (RobotSide side : RobotSide.values)
      {
         multiWaypointTrajectories.put(side, new MultipleWaypointsPoseTrajectoryGenerator(side.getLowerCaseName() + "_scriptedTrajectory", 17, registry));
         multiWaypointTrajectories.get(side).clear(ReferenceFrame.getWorldFrame());
      }
   }

   public enum ScriptedTrajectoryType
   {
      HAND_CIRCLES, STRETCH_OUT_ARMS
   }

   public boolean isDone()
   {
      return isDone;
   }

   public void setDone(boolean isDone)
   {
      this.isDone = isDone;
   }

   private SideDependentList<List<FramePose3D>> getPoseWaypoints(ScriptedTrajectoryType trajectoryType)
   {
      SideDependentList<List<FramePose3D>> poseWaypoints = new SideDependentList<>();

      for (RobotSide side : RobotSide.values)
      {
         switch (trajectoryType)
         {
            case STRETCH_OUT_ARMS:
               poseWaypoints.put(side,
                                 List.of(new FramePose3D(new Pose3D(0.3, side.negateIfRightSide(0.2), 0.2, 0.0, -Math.PI / 2.0, 0.0)),
                                         new FramePose3D(new Pose3D(0.0, side.negateIfRightSide(0.6), 0.2, 0.0, -Math.PI / 2.0, side.negateIfRightSide(Math.PI / 2.0))),
                                         new FramePose3D(new Pose3D(0.3, side.negateIfRightSide(0.2), 0.2, 0.0, -Math.PI / 2.0, 0.0))));
               break;
            case HAND_CIRCLES:
               //TODO: this is really choppy with 0 velocity at all the waypoints, consider adjusting or removing
               getCircleWaypoints(poseWaypoints, 17);
               break;
            default:
               throw new RuntimeException("Unhandled trajectory type: " + trajectoryType);
         }
      }
      return poseWaypoints;
   }

   private void createAndInitializeTrajectory(SideDependentList<List<FramePose3D>> poseWaypoints)
   {
      for (RobotSide side : RobotSide.values)
      {
         double numberOfWaypoints = poseWaypoints.get(side).size();

         for (int i =0; i < numberOfWaypoints; i++)
         {
            // This assumes the waypoints are equally spaced in time.
            double timeAtWayPoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
            System.out.println("time at waypoint = " + timeAtWayPoint);
            multiWaypointTrajectories.get(side)
                                     .appendPoseWaypoint(timeAtWayPoint,
                                                         new FramePose3D(ReferenceFrame.getWorldFrame(), poseWaypoints.get(side).get(i)),
                                                         new FrameVector3D(),
                                                         new FrameVector3D());
         }
         multiWaypointTrajectories.get(side).initialize();
      }
   }

   private void computeHandPoseWaypoint(RobotSide robotSide, double time)
   {
      if (time >= trajectoryTime)
      {
         isDone = true;
         return;
      }

      multiWaypointTrajectories.get(robotSide).compute(time);
      currentHandPoses.put(robotSide, multiWaypointTrajectories.get(robotSide).getPose());
   }

   public FramePose3DReadOnly getHandPose(RobotSide robotSide, ScriptedTrajectoryType trajectoryType, double time)
   {
      if (!initialize)
      {
         createAndInitializeTrajectory(getPoseWaypoints(trajectoryType));
         initialize = true;
      }
      computeHandPoseWaypoint(robotSide, time);

      return currentHandPoses.get(robotSide);
   }

   private void getCircleWaypoints(SideDependentList<List<FramePose3D>> poseWaypoints, int numberOfWaypoints)
   {
      double circleRadius = 0.25;
      double circleFrequency = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.3, side.negateIfRightSide(0.225), 0.0));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ?
            new Vector3D(0.0, 0.0, 0.0) :
            new Vector3D());

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<FramePose3D> sidedPoseWaypointList = new ArrayList<>();
         for (double i = 0; i < numberOfWaypoints; i++)
         {
            double time = i * trajectoryTime / (numberOfWaypoints - 1.0);
            FramePoint3D position = circlePositionAt(time, circleFrequency, circleRadius, circleCenters.get(robotSide), circleCenterVelocities.get(robotSide));
            FrameQuaternion orientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0, 1.0);
            sidedPoseWaypointList.add(new FramePose3D(position, orientation));
         }
         poseWaypoints.put(robotSide, sidedPoseWaypointList);
      }
   }

   private static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center)
   {
      return circlePositionAt(time, frequency, radius, center, new Vector3D());
   }

   private static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center, Vector3DReadOnly centerVelocity)
   {
      double angle = 2.0 * Math.PI * frequency * time;
      Vector3D offset = new Vector3D(Axis3D.Z);
      offset.scale(radius);
      RotationMatrixTools.applyRollRotation(angle, offset, offset);
      FramePoint3D position = new FramePoint3D();
      position.add(center, offset);
      position.scaleAdd(time, centerVelocity, position);
      return position;
   }
}
