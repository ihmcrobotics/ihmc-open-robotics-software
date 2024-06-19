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
import us.ihmc.log.LogTools;
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RDXScriptedTrajectoryStreamer
{
   private final double trajectoryTime;
   private final SideDependentList<MultipleWaypointsPoseTrajectoryGenerator> multiWaypointPoseTrajectories = new SideDependentList<>();
   private final SideDependentList<MultipleWaypointsTrajectoryGenerator> multiWaypointJointTrajectories = new SideDependentList<>();
   private boolean isDone = false;
   private boolean initialize = false;
   private final SideDependentList<FramePose3DReadOnly> desiredHandPoses = new SideDependentList<>(side -> new FramePose3D());
   private final SideDependentList<OneDoFJointBasics[]> armJoints;
   private final Map<ScriptedTrajectoryType, SideDependentList<List<List<Double>>>> jointAngleWaypointsMap = new HashMap<>();
   private final Map<ScriptedTrajectoryType, Double> trajectoryTimes = new HashMap<>();
   private final double minimumAllowedDuration = 2.0;

   public RDXScriptedTrajectoryStreamer(SideDependentList<OneDoFJointBasics[]> armJoints, double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
      this.armJoints = armJoints;

      for (ScriptedTrajectoryType trajectoryType : ScriptedTrajectoryType.values())
      {
         trajectoryTimes.put(trajectoryType, trajectoryTime);
      }

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      for (RobotSide side : RobotSide.values)
      {
         multiWaypointPoseTrajectories.put(side,
                                           new MultipleWaypointsPoseTrajectoryGenerator(side.getLowerCaseName() + "_scriptedPoseTrajectory", 17, registry));
         multiWaypointPoseTrajectories.get(side).clear(ReferenceFrame.getWorldFrame());

         multiWaypointJointTrajectories.put(side, new MultipleWaypointsTrajectoryGenerator(side.getLowerCaseName() + "_scriptedJointTrajectory", 17, registry));
         multiWaypointJointTrajectories.get(side).clear();
      }

      for (ScriptedTrajectoryType trajectoryType : ScriptedTrajectoryType.values())
      {
         // TODO: find a better way to differentiate between taskspace and jointspace trajectories
         if (trajectoryType == ScriptedTrajectoryType.HAND_CIRCLES || trajectoryType == ScriptedTrajectoryType.STRETCH_OUT_ARMS
             || trajectoryType == ScriptedTrajectoryType.REACHABILITY_SWEEP)
         {
            continue;
         }
         jointAngleWaypointsMap.put(trajectoryType, getArmJointWaypoints(trajectoryType));
      }
   }

   public enum ScriptedTrajectoryType
   {
      HOME_CONFIGURATION,
      HAND_CIRCLES,
      STRETCH_OUT_ARMS,
      WRIST_RANGE_OF_MOTION,
      BEACH_BALL_FLEX,
      BEACH_BALL_OVERHEAD,
      REACHABILITY_ARMS_BACK,
      REACHABILITY_ARMS_SIDEWAYS,
      REACHABILITY_ARMS_FORWARD,
      REACHABILITY_ARMS_UP,
      REACHABILITY_SWEEP,
      ROM_SHOULDER_PITCH,
      ROM_SHOULDER_ROLL,
      ROM_SHOULDER_YAW,
      ROM_ELBOW,
      ROM_WRIST_YAW,
      ROM_WRIST_ROLL,
      ROM_GRIPPER_YAW,
      DAB_ON_THEM_HATERS,
      BICEP_CURL_SIMPLE,
      BICEP_CURL,
      FRONT_RAISE,
      LATERAL_RAISE,
      SHOULDER_PRESS,
      SHOULDER_PRESS_INITIAL,
      SHOULDER_PRESS_RETURN,
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

   /**
    * Defines waypoints for many different prescripted ArmTrajectoryMessages. These waypoints are used to create the ArmTrajectoryMessage in
    * {@link #generateArmTrajectoryMessage(ScriptedTrajectoryType, RobotSide) generateArmTrajectoryMessage()}. To add a new trajectory, it's easiest to use
    * the model viewer ([Robot]ModelViewer.java) to visualize the desired robot configurations.
    *  */
   private SideDependentList<List<List<Double>>> getArmJointWaypoints(ScriptedTrajectoryType trajectoryType)
   {
      SideDependentList<List<List<Double>>> waypoints = new SideDependentList<>();

      // It's easier to use the left arm's limits and then negate the roll and yaw actuators for the right arm.
      double[] upperLimits = new double[armJoints.get(RobotSide.LEFT).length];
      double[] lowerLimits = new double[armJoints.get(RobotSide.LEFT).length];
      double epsilon = 0.18;
      for (int jointIndex = 0; jointIndex < armJoints.get(RobotSide.LEFT).length; jointIndex++)
      {
         // Epsilon is used to ensure the trajectory generator does not exceed the joint limits.
         upperLimits[jointIndex] = armJoints.get(RobotSide.LEFT)[jointIndex].getJointLimitUpper() - epsilon;
         lowerLimits[jointIndex] = armJoints.get(RobotSide.LEFT)[jointIndex].getJointLimitLower() + epsilon;
      }

      for (RobotSide side : RobotSide.values)
      {
         List<Double> homeConfiguration = List.of(0.5, side.negateIfRightSide(0.13), side.negateIfRightSide(0.13), -1.6, 0.0, 0.0, 0.0);
         switch (trajectoryType)
         {
            case HOME_CONFIGURATION:
               waypoints.put(side, List.of(homeConfiguration));
               break;
            case ROM_SHOULDER_PITCH:
               waypoints.put(side, List.of(homeConfiguration,
                                           // Shoulder Pitch ROM
                                           List.of(upperLimits[0], 0.0, side.negateIfRightSide(-0.3), -2.3, 0.0, 0.0, 0.0),
                                           List.of(lowerLimits[0], side.negateIfRightSide(1.243), side.negateIfRightSide(1.2), 0.0, 0.0, 0.0, 0.0),
                                           List.of(upperLimits[0], 0.0, side.negateIfRightSide(-0.3), -2.3, 0.0, 0.0, 0.0),
                                           homeConfiguration));
               break;
            case ROM_SHOULDER_ROLL:
               waypoints.put(side, List.of(homeConfiguration,
                                           // Shoulder Roll ROM
                                           List.of(0.0, side.negateIfRightSide(upperLimits[1]), 0.0, -2.3, 0.0, 0.0, 0.0),
                                           List.of(0.0, side.negateIfRightSide(lowerLimits[1]), 0.0, -2.3, 0.0, 0.0, 0.0),
                                           List.of(0.0, side.negateIfRightSide(upperLimits[1]), 0.0, -2.3, 0.0, 0.0, 0.0),
                                           homeConfiguration));
               break;
            case ROM_SHOULDER_YAW:
               waypoints.put(side, List.of(homeConfiguration,
                                           // Shoulder Yaw ROM
                                           List.of(-1.0, side.negateIfRightSide(1.4), side.negateIfRightSide(0.13), -1.57, 0.0, 0.0, 0.0),
                                           List.of(-1.0, side.negateIfRightSide(1.4), side.negateIfRightSide(upperLimits[2]), -1.57, 0.0, 0.0, 0.0),
                                           List.of(-1.0, side.negateIfRightSide(1.4), side.negateIfRightSide(lowerLimits[2]), -1.57, 0.0, 0.0, 0.0),
                                           List.of(-1.0, side.negateIfRightSide(1.4), side.negateIfRightSide(upperLimits[2]), -1.57, 0.0, 0.0, 0.0),
                                           List.of(-1.0, side.negateIfRightSide(1.4), side.negateIfRightSide(0.13), -1.57, 0.0, 0.0, 0.0),
                                           homeConfiguration));
               break;
            case ROM_ELBOW:
               waypoints.put(side, List.of(homeConfiguration,
                                           // Elbow ROM
                                           List.of(-3.1, side.negateIfRightSide(2.7), side.negateIfRightSide(-1.6), -0.5, 0.0, 0.0, 0.0),
                                           List.of(-3.1, side.negateIfRightSide(2.7), side.negateIfRightSide(-1.6), lowerLimits[3], 0.0, 0.0, 0.0),
                                           List.of(-3.1, side.negateIfRightSide(2.7), side.negateIfRightSide(-1.6), -0.5, 0.0, 0.0, 0.0),
                                           List.of(-1.1, side.negateIfRightSide(2.7), 0.0, -1.6, 0.0, 0.0, 0.0),
                                           homeConfiguration));
               break;
            case ROM_WRIST_YAW:
               waypoints.put(side,
                             List.of(// Wrist Yaw ROM
                                     List.of(-1.5, side.negateIfRightSide(0.95), 0.0, 0.0, 0.0, side.negateIfRightSide(-1.6), 0.0),
                                     List.of(-1.5, side.negateIfRightSide(0.95), 0.0, 0.0, side.negateIfRightSide(upperLimits[4]), side.negateIfRightSide(-1.6), 0.0),
                                     List.of(-1.5, side.negateIfRightSide(0.95), 0.0, 0.0, side.negateIfRightSide(lowerLimits[4]), side.negateIfRightSide(-1.6), 0.0),
                                     List.of(-1.5, side.negateIfRightSide(0.95), 0.0, 0.0, side.negateIfRightSide(upperLimits[4]), side.negateIfRightSide(-1.6), 0.0),
                                     List.of(-1.5, side.negateIfRightSide(0.95), 0.0, 0.0, 0.0, side.negateIfRightSide(-1.6), 0.0)));
               break;
            case ROM_WRIST_ROLL:
               waypoints.put(side,
                             List.of(// Wrist Roll ROM
                                     List.of(-1.5, side.negateIfRightSide(0.95), side.negateIfRightSide(0.8), -1.6, 0.0, 0.0, 0.0),
                                     List.of(-1.5, side.negateIfRightSide(0.95), side.negateIfRightSide(0.8), -1.6, 0.0, side.negateIfRightSide(upperLimits[5]), 0.0),
                                     List.of(-1.5, side.negateIfRightSide(0.95), side.negateIfRightSide(0.8), -1.6, 0.0, side.negateIfRightSide(lowerLimits[5]), 0.0),
                                     List.of(-1.5, side.negateIfRightSide(0.95), side.negateIfRightSide(0.8), -1.6, 0.0, side.negateIfRightSide(upperLimits[5]), 0.0),
                                     List.of(-1.5, side.negateIfRightSide(0.95), side.negateIfRightSide(0.8), -1.6, 0.0, 0.0, 0.0)));
               break;
            case ROM_GRIPPER_YAW:
               waypoints.put(side,
                             List.of(// Gripper Yaw ROM
                                     List.of(-1.5, side.negateIfRightSide(0.7), 0.0, 0.0, 0.0, 0.0, 0.0),
                                     List.of(-1.5, side.negateIfRightSide(0.7), 0.0, 0.0, 0.0, 0.0, side.negateIfRightSide(upperLimits[6])),
                                     List.of(-1.5, side.negateIfRightSide(0.7), 0.0, 0.0, 0.0, 0.0, side.negateIfRightSide(lowerLimits[6])),
                                     List.of(-1.5, side.negateIfRightSide(0.7), 0.0, 0.0, 0.0, 0.0, side.negateIfRightSide(upperLimits[6])),
                                     List.of(-1.5, side.negateIfRightSide(0.7), 0.0, 0.0, 0.0, 0.0, 0.0)));
               break;
            case REACHABILITY_ARMS_BACK:
               waypoints.put(side, List.of(List.of(1.2, side.negateIfRightSide(0.6), 0.0, 0.0, 0.0, 0.0, 0.0)));
               break;
            case REACHABILITY_ARMS_FORWARD:
               waypoints.put(side,
                             List.of(List.of(-1.5, side.negateIfRightSide(0.6), side.negateIfRightSide(0.0), 0.0, 0.0, 0.0, 0.0)));
               break;
            case REACHABILITY_ARMS_UP:
               waypoints.put(side,
                             List.of(List.of(-3.0, side.negateIfRightSide(1.4), side.negateIfRightSide(0.0), 0.0, 0.0, 0.0, 0.0)));
               break;
            case REACHABILITY_ARMS_SIDEWAYS:
               waypoints.put(side,
                             List.of(
                                     List.of(0.0, side.negateIfRightSide(1.57), 0.0, 0.0, 0.0, 0.0, 0.0)));
               break;
            case WRIST_RANGE_OF_MOTION:
               waypoints.put(side,
                             List.of(
                                   List.of(0.5, side.negateIfRightSide(0.13), side.negateIfRightSide(0.13), -1.6, 0.0, 0.0, 0.0),
                                   List.of(0.5, side.negateIfRightSide(0.13), side.negateIfRightSide(0.13), -1.6, side.negateIfRightSide(upperLimits[4]), side.negateIfRightSide(-0.8), side.negateIfRightSide(upperLimits[6])),
                                   List.of(0.5, side.negateIfRightSide(0.13), side.negateIfRightSide(0.13), -1.6, side.negateIfRightSide(0.0), side.negateIfRightSide(-1.6), side.negateIfRightSide(lowerLimits[6])),
                                   List.of(0.5, side.negateIfRightSide(0.13), side.negateIfRightSide(0.13), -1.6, side.negateIfRightSide(lowerLimits[4]), side.negateIfRightSide(-0.8), side.negateIfRightSide(upperLimits[6])),
                                   List.of(0.5, side.negateIfRightSide(0.13), side.negateIfRightSide(0.13), -1.6, 0.0, 0.0, 0.0)
                             ));
               break;
            case BEACH_BALL_FLEX:
               waypoints.put(side,
                             List.of(homeConfiguration,
                                     List.of(-0.147, side.negateIfRightSide(0.543), side.negateIfRightSide(-1.0), -1.431, side.negateIfRightSide(-1.50), side.negateIfRightSide(0.565), side.negateIfRightSide(0.0)),
                                     List.of(-0.147, side.negateIfRightSide(0.543), side.negateIfRightSide(-1.0), -1.431, side.negateIfRightSide(-1.50), side.negateIfRightSide(0.565), side.negateIfRightSide(0.0)),
                                     List.of(-0.147, side.negateIfRightSide(0.543), side.negateIfRightSide(-1.0), -1.431, side.negateIfRightSide(-1.50), side.negateIfRightSide(0.565), side.negateIfRightSide(0.0)),
                                     List.of(-0.147, side.negateIfRightSide(0.543), side.negateIfRightSide(-1.0), -1.431, side.negateIfRightSide(-1.50), side.negateIfRightSide(0.565), side.negateIfRightSide(0.0)),
                                     homeConfiguration));
               break;
            case BEACH_BALL_OVERHEAD:
               waypoints.put(side,
                             List.of(homeConfiguration,
                                     List.of(-1.645, side.negateIfRightSide(2.15), side.negateIfRightSide(-0.683), -1.3, side.negateIfRightSide(-1.50), side.negateIfRightSide(0.565), 0.0),
                                     List.of(-2.623, side.negateIfRightSide(2.0), side.negateIfRightSide(-1.179), -1.586, side.negateIfRightSide(-1.50), side.negateIfRightSide(0.565), 0.0),
                                     List.of(-2.623, side.negateIfRightSide(2.0), side.negateIfRightSide(-1.179), -1.586, side.negateIfRightSide(-1.50), side.negateIfRightSide(0.565), 0.0),
                                     homeConfiguration));
               break;
            case DAB_ON_THEM_HATERS:
               if (side == RobotSide.LEFT)
               {
                  waypoints.put(side,
                                List.of(homeConfiguration,
                                        List.of(-2.06, side.negateIfRightSide(0.627), side.negateIfRightSide(-0.490), -1.3, 0.0, 0.0, side.negateIfRightSide(-1.57)),
                                        homeConfiguration));
               }
               else
               {
                  waypoints.put(side,
                                List.of(homeConfiguration,
                                        List.of(0.128, side.negateIfRightSide(2.176), 0.0, 0.0, 0.0, 0.0, side.negateIfRightSide(-1.57)),
                                        homeConfiguration));
               }
               break;
            case BICEP_CURL_SIMPLE:
               if (side == RobotSide.LEFT)
               {
                  waypoints.put(side, List.of(List.of(0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0),
                                              List.of(0.0, 0.0, 0.0, -2.2, 0.0, 0.0, 0.0),
                                              List.of(0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0)));
               }
               break;
            case BICEP_CURL:
               if (side == RobotSide.LEFT)
               {
                  waypoints.put(side, List.of(List.of(0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0),
                                              List.of(0.3, 0.0, side.negateIfRightSide(-0.2), -2.2, side.negateIfRightSide(1.57), 0.0, 0.0),
                                              List.of(0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0)));
               }
               break;
            case FRONT_RAISE:
               if (side == RobotSide.LEFT)
               {
                  waypoints.put(side,
                                List.of(List.of(0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0),
                                        List.of(-1.5, side.negateIfRightSide(0.6), 0.0, 0.0, side.negateIfRightSide(-0.94), 0.0, 0.0),
                                        List.of(0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0)));
               }
               break;
            case LATERAL_RAISE:
               if (side == RobotSide.LEFT)
               {
                  waypoints.put(side,
                                List.of(List.of(0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0),
                                        List.of(0.0, side.negateIfRightSide(1.57), 0.0, 0.0, 0.0, 0.0, 0.0),
                                        List.of(0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0)));
               }
               break;
            case SHOULDER_PRESS:
               if (side == RobotSide.LEFT)
               {
                  waypoints.put(side,
                                List.of(List.of(-3.14, side.negateIfRightSide(2.7), side.negateIfRightSide(-1.5), -1.3, 0.0, 0.0, 0.0),
                                        List.of(-3.14, side.negateIfRightSide(1.4), side.negateIfRightSide(-1.5), 0.0, 0.0, 0.0, 0.0),
                                        List.of(-3.14, side.negateIfRightSide(2.7), side.negateIfRightSide(-1.5), -1.3, 0.0, 0.0, 0.0)));
               }
               break;
            case SHOULDER_PRESS_INITIAL:
               if (side == RobotSide.LEFT)
               {
                  waypoints.put(side,
                                List.of(List.of(-1.5, side.negateIfRightSide(0.6), side.negateIfRightSide(0.0), 0.0, 0.0, 0.0, 0.0),
                                        List.of(-3.14, side.negateIfRightSide(2.7), side.negateIfRightSide(-1.5), -1.3, 0.0, 0.0, 0.0)));
               }
               break;
            case SHOULDER_PRESS_RETURN:
               if (side == RobotSide.LEFT)
               {
                  waypoints.put(side,
                                List.of(List.of(-1.5, side.negateIfRightSide(0.6), side.negateIfRightSide(0.0), 0.0, 0.0, 0.0, 0.0),
                                        homeConfiguration));
               }
               break;
            default:
               throw new RuntimeException("Unhandled trajectory type: " + trajectoryType);
         }
      }
      return waypoints;
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

   public ArmTrajectoryMessage generateArmTrajectoryMessage(ScriptedTrajectoryType trajectoryType, RobotSide robotSide)
   {
      double trajectoryDuration = trajectoryTimes.get(trajectoryType);
      OneDoFJointBasics[] sidedArmJoints = armJoints.get(robotSide);
      if (jointAngleWaypointsMap.get(trajectoryType) == null || jointAngleWaypointsMap.get(trajectoryType).get(robotSide) == null)
      {
         LogTools.warn("No waypoints found for ScriptedTrajectoryType." + trajectoryType + " on RobotSide." + robotSide);
         return null;
      }
      List<List<Double>> jointAngleWaypoints = jointAngleWaypointsMap.get(trajectoryType).get(robotSide);
      int numberOfTrajectoryPoints = jointAngleWaypoints.size();
      int numberOfJoints = sidedArmJoints.length;

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);

      // The trajectory waypoint generator doesn't play nice when there's only one waypoint, so we handle that case separately.
      if (numberOfTrajectoryPoints == 1)
      {
         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJointBasics joint = sidedArmJoints[jointIndex];

            double[] desiredJointPositions = jointAngleWaypoints.get(0).stream().mapToDouble(Double::doubleValue).toArray();

            armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryDuration, desiredJointPositions);
         }
      }
      else
      {
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

            trajectoryPoint1DCalculator.compute(trajectoryDuration);
            OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();
            trajectoryData.addTimeOffset(trajectoryDuration / (numberOfTrajectoryPoints - 1.0));

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
            {
               OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
               jointTrajectoryMessage.getTrajectoryPoints()
                                     .add()
                                     .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint.getTime(),
                                                                                              trajectoryPoint.getPosition(),
                                                                                              trajectoryPoint.getVelocity()));
               jointTrajectoryMessage.setWeight(100.0);
            }
         }
      }
      return armTrajectoryMessage;
   }

   public double getTrajectoryDuration(ScriptedTrajectoryType trajectoryType)
   {
      return trajectoryTimes.get(trajectoryType);
   }

   public void setTrajectoryDuration(ScriptedTrajectoryType trajectoryType, double duration)
   {
      duration = Math.max(duration, minimumAllowedDuration);
      if (trajectoryTimes.containsKey(trajectoryType))
      {
         trajectoryTimes.replace(trajectoryType, duration);
      }
      trajectoryTimes.put(trajectoryType, duration);
   }

   /**
    * Gets the hand pose at one time instance from a trajectory that is generated on the first call of this method. Good for getting poses for IK streaming.
    */
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
