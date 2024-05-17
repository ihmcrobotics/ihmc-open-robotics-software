package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.OneDoFTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class RDXScriptedTrajectoryStreamer
{
   private final double trajectoryTime;
   private final SideDependentList<MultipleWaypointsPoseTrajectoryGenerator> multiWaypointPoseTrajectories = new SideDependentList<>();
   private final SideDependentList<MultipleWaypointsTrajectoryGenerator> multiWaypointJointTrajectories = new SideDependentList<>();
   private boolean isDone = false;
   private boolean initialize = false;
   private final SideDependentList<FramePose3DReadOnly> desiredHandPoses = new SideDependentList<>(side -> new FramePose3D());
   private final SideDependentList<OneDoFJointBasics[]> armJoints;

   public RDXScriptedTrajectoryStreamer(SideDependentList<OneDoFJointBasics[]> armJoints, double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
      this.armJoints = armJoints;

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      for (RobotSide side : RobotSide.values)
      {
         multiWaypointPoseTrajectories.put(side,
                                           new MultipleWaypointsPoseTrajectoryGenerator(side.getLowerCaseName() + "_scriptedPoseTrajectory", 17, registry));
         multiWaypointPoseTrajectories.get(side).clear(ReferenceFrame.getWorldFrame());

         multiWaypointJointTrajectories.put(side, new MultipleWaypointsTrajectoryGenerator(side.getLowerCaseName() + "_scriptedJointTrajectory", 17, registry));
         multiWaypointJointTrajectories.get(side).clear();
      }
   }

   public enum ScriptedTrajectoryType
   {
      HAND_CIRCLES, STRETCH_OUT_ARMS, JOINT_RANGE_OF_MOTION, JOINT_TRAJECTORY_TEST
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
                                 List.of(new FramePose3D(new Pose3D(0.3, side.negateIfRightSide(0.2), 0.2, 0.0, 0.0, 0.0)),
                                         new FramePose3D(new Pose3D(0.0, side.negateIfRightSide(1.0), 0.4, side.negateIfRightSide(Math.PI / 2.0), 0.0, 0.0)),
                                         new FramePose3D(new Pose3D(0.3, side.negateIfRightSide(0.2), 0.2, 0.0, 0.0, 0.0))));
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

   private SideDependentList<List<List<Double>>> getArmJointWaypoints(ScriptedTrajectoryType trajectoryType)
   {
      SideDependentList<List<List<Double>>> armJointWaypoints = new SideDependentList<>();

      // It's easier to use the left arm's limits and then negate the roll and yaw actuators for the right arm.
      double[] upperLimits = new double[armJoints.get(RobotSide.LEFT).length];
      double[] lowerLimits = new double[armJoints.get(RobotSide.LEFT).length];
      double epsilon = 1e-3;
      for (int jointIndex = 0; jointIndex < armJoints.get(RobotSide.LEFT).length; jointIndex++)
      {
         // Epsilon is used to ensure the trajectory generator does not exceed the joint limits.
         upperLimits[jointIndex] = armJoints.get(RobotSide.LEFT)[jointIndex].getJointLimitUpper() - epsilon;
         lowerLimits[jointIndex] = armJoints.get(RobotSide.LEFT)[jointIndex].getJointLimitLower() + epsilon;
      }

      for (RobotSide side : RobotSide.values)
      {
         switch (trajectoryType)
         {
            case JOINT_RANGE_OF_MOTION:
               armJointWaypoints.put(side,
                                     List.of(List.of(0.5, side.negateIfRightSide(0.13), 0.13, -1.0, 0.0, 0.0, 0.0),
                                             List.of(upperLimits[0], side.negateIfRightSide(-0.1), side.negateIfRightSide(-1.5), 0.0, 0.0, 0.0, 0.0),
                                             List.of(lowerLimits[0], side.negateIfRightSide(upperLimits[1]), 0.0, 0.0, 0.0, 0.0, 0.0),
                                             List.of(0.5, side.negateIfRightSide(0.13), 0.13, -1.0, 0.0, 0.0, 0.0)));
               break;
            case JOINT_TRAJECTORY_TEST:
               armJointWaypoints.put(side,
                                     List.of(List.of(0.5, side.negateIfRightSide(0.13), 0.13, -1.0, 0.0, 0.0, 0.0),
                                             List.of(0.0, side.negateIfRightSide(0.0), 0.0, 0.0, 0.0, 0.0, 0.0),
                                             List.of(0.5, side.negateIfRightSide(0.13), 0.13, -1.0, 0.0, 0.0, 0.0)));
               break;
            default:
               throw new RuntimeException("Unhandled trajectory type: " + trajectoryType);
         }
      }
      return armJointWaypoints;
   }

   private void createAndInitializePoseTrajectory(SideDependentList<List<FramePose3D>> poseWaypoints)
   {
      for (RobotSide side : RobotSide.values)
      {
         double numberOfWaypoints = poseWaypoints.get(side).size();

         for (int i = 0; i < numberOfWaypoints; i++)
         {
            // This assumes the waypoints are equally spaced in time.
            double timeAtWayPoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
            System.out.println("time at waypoint = " + timeAtWayPoint);
            multiWaypointPoseTrajectories.get(side)
                                         .appendPoseWaypoint(timeAtWayPoint,
                                                             new FramePose3D(ReferenceFrame.getWorldFrame(), poseWaypoints.get(side).get(i)),
                                                             new FrameVector3D(),
                                                             new FrameVector3D());
         }
         multiWaypointPoseTrajectories.get(side).initialize();
      }
   }

   private void computeHandPoseWaypoint(RobotSide robotSide, double time)
   {
      if (time >= trajectoryTime)
      {
         isDone = true;
         return;
      }

      multiWaypointPoseTrajectories.get(robotSide).compute(time);
      desiredHandPoses.put(robotSide, multiWaypointPoseTrajectories.get(robotSide).getPose());
   }

   public void packHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessageToPack,
                                         RobotSide robotSide,
                                         ScriptedTrajectoryType trajectoryType,
                                         double time)
   {
      handTrajectoryMessageToPack.setSequenceId(0); // Does this matter?
      handTrajectoryMessageToPack.setRobotSide(robotSide.toByte());
      handTrajectoryMessageToPack.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());
      handTrajectoryMessageToPack.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(ReferenceFrame.getWorldFrame().getFrameNameHashCode());

      SideDependentList<List<FramePose3D>> poseWaypoints = getPoseWaypoints(trajectoryType);
      double numberOfWaypoints = poseWaypoints.get(robotSide).size();

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         FramePose3D poseWaypoint = poseWaypoints.get(robotSide).get(i);
         double timeAtWayPoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         handTrajectoryMessageToPack.getSe3Trajectory()
                                    .getTaskspaceTrajectoryPoints()
                                    .add()
                                    .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(timeAtWayPoint,
                                                                                              poseWaypoint.getPosition(),
                                                                                              poseWaypoint.getOrientation(),
                                                                                              new Vector3D(),
                                                                                              new Vector3D()));
      }
   }

   public ArmTrajectoryMessage generateArmTrajectoryMessage(ScriptedTrajectoryType trajectoryType, double trajectoryTime, RobotSide robotSide)
   {
      OneDoFJointBasics[] sidedArmJoints = armJoints.get(robotSide);
      List<List<Double>> jointAngleWaypoints = getArmJointWaypoints(trajectoryType).get(robotSide);
      int numberOfTrajectoryPoints = jointAngleWaypoints.size();
      int numberOfJoints = sidedArmJoints.length;

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
      OneDoFTrajectoryPointCalculator trajectoryPoint1DCalculator = new OneDoFTrajectoryPointCalculator();

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = sidedArmJoints[jointIndex];
         OneDoFJointTrajectoryMessage jointTrajectoryMessage = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add();

         trajectoryPoint1DCalculator.clear();

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double desiredJointPosition = jointAngleWaypoints.get(trajectoryPointIndex).get(jointIndex);
            trajectoryPoint1DCalculator.appendTrajectoryPoint(desiredJointPosition);
         }

         trajectoryPoint1DCalculator.compute(trajectoryTime);
         OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();
         trajectoryData.addTimeOffset(trajectoryTime / (numberOfTrajectoryPoints - 1.0));

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
            jointTrajectoryMessage.getTrajectoryPoints()
                                  .add()
                                  .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint.getTime(),
                                                                                           trajectoryPoint.getPosition(),
                                                                                           trajectoryPoint.getVelocity()));
         }
      }
      return armTrajectoryMessage;
   }

   /** Gets the hand pose at one time instance from a trajectory that is generated on the first call of this method. Good for getting poses for IK streaming. */
   public FramePose3DReadOnly getHandPose(RobotSide robotSide, ScriptedTrajectoryType trajectoryType, double time)
   {
      if (!initialize)
      {
         createAndInitializePoseTrajectory(getPoseWaypoints(trajectoryType));
         initialize = true;
      }
      computeHandPoseWaypoint(robotSide, time);

      return desiredHandPoses.get(robotSide);
   }

   public double getJointAngle(RobotSide side, ScriptedTrajectoryType scriptedTrajectoryType, double scriptedTrajectoryTime, int i)
   {
      //TODO: Implement this method
      return 0;
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
