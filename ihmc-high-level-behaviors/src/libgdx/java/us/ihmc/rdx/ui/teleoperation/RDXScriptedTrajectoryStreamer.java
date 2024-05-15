package us.ihmc.rdx.ui.teleoperation;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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

public class RDXScriptedTrajectoryStreamer
{
   private final double trajectoryTime;
   private final SideDependentList<MultipleWaypointsPoseTrajectoryGenerator> multipleWaypointsTrajectories = new SideDependentList<>();
   private boolean isDone = false;
   private boolean isInitialized = false;
   private final SideDependentList<FramePoint3D> handPositions = new SideDependentList<>(side -> new FramePoint3D());
   private final SideDependentList<FramePose3DReadOnly> handPose = new SideDependentList<>(side -> new FramePose3D());

   public RDXScriptedTrajectoryStreamer(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      for (RobotSide side : RobotSide.values)
      {
         multipleWaypointsTrajectories.put(side, new MultipleWaypointsPoseTrajectoryGenerator(side.getLowerCaseName() + "_scriptedTrajectory", 3, registry));
         multipleWaypointsTrajectories.get(side).clear(ReferenceFrame.getWorldFrame());
      }
   }

   public boolean isDone()
   {
      return isDone;
   }

   public void setDone(boolean isDone)
   {
      this.isDone = isDone;
   }

   public void stretchOutArms(double time)
   {
      if (time >= trajectoryTime)
      {
         isDone = true;
         return;
      }

      // Initialize the trajectory
      if (!isInitialized)
      {

         for (RobotSide side : RobotSide.values)
         {
            Pose3D startPose = new Pose3D(0.3, side.negateIfRightSide(0.2), 0.2, 0.0, -Math.PI / 2.0, 0.0);
            Pose3D midPose = new Pose3D(0.0, side.negateIfRightSide(0.6), 0.2, 0.0, -Math.PI / 2.0, side.negateIfRightSide(Math.PI / 2.0));
            Pose3D endPose = new Pose3D(startPose);

            multipleWaypointsTrajectories.get(side).initialize();
            multipleWaypointsTrajectories.get(side)
                                         .appendPoseWaypoint(0.0,
                                                             new FramePose3D(ReferenceFrame.getWorldFrame(), startPose),
                                                             new FrameVector3D(),
                                                             new FrameVector3D());
            multipleWaypointsTrajectories.get(side)
                                         .appendPoseWaypoint(trajectoryTime / 2.0,
                                                             new FramePose3D(ReferenceFrame.getWorldFrame(), midPose),
                                                             new FrameVector3D(),
                                                             new FrameVector3D());
            multipleWaypointsTrajectories.get(side)
                                         .appendPoseWaypoint(trajectoryTime,
                                                             new FramePose3D(ReferenceFrame.getWorldFrame(), endPose),
                                                             new FrameVector3D(),
                                                             new FrameVector3D());
            multipleWaypointsTrajectories.get(side).initialize();
         }
         isInitialized = true;
      }

      // Compute the point along the trajectory
      for (RobotSide robotSide : RobotSide.values)
      {
         multipleWaypointsTrajectories.get(robotSide).compute(trajectoryTime);
         handPose.put(robotSide, multipleWaypointsTrajectories.get(robotSide).getPose());
      }
   }

   public void handCirclesMotion(double t)
   {
      if (t >= trajectoryTime)
      {
         isDone = true;
         return;
      }

      double circleRadius = 0.25;
      double circleFrequency = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.3, side.negateIfRightSide(0.225), 0.0));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ?
            new Vector3D(0.0, 0.0, 0.0) :
            new Vector3D());

      //      for (double t = 0.0; t < trajectoryTime; t += streamPeriod)

      for (RobotSide robotSide : RobotSide.values)
      {
         handPositions.put(robotSide,
                           circlePositionAt(t,
                                            robotSide.negateIfRightSide(circleFrequency),
                                            circleRadius,
                                            circleCenters.get(robotSide),
                                            circleCenterVelocities.get(robotSide)));
      }
   }

   public FramePoint3D getHandPosition(RobotSide robotSide, double time)
   {
      handCirclesMotion(time);
      return handPositions.get(robotSide);
   }

   public FramePose3DReadOnly getHandPose(RobotSide robotSide, double time)
   {
      stretchOutArms(time);
      return handPose.get(robotSide);
   }

   public static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center)
   {
      return circlePositionAt(time, frequency, radius, center, new Vector3D());
   }

   public static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center, Vector3DReadOnly centerVelocity)
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

   public enum ScriptedTrajectoryType
   {
      CIRCLE
   }
}
