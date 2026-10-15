package us.ihmc.gr00t;

import controller_msgs.ArmTrajectoryMessage;
import controller_msgs.NeckTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

/** Converts validated Cartesian policy chunks into fixed-base arm and neck trajectories. */
final class Gr00tHumanoidArmPlanner
{
   static final class Plan
   {
      final Pose3D rootPose;
      final double[][] fullJointPositions;
      final double[] trajectoryTimes;
      final ArmTrajectoryMessage armMessage;
      final NeckTrajectoryMessage neckMessage;
      final double duration;
      final double worstIKQuality;
      final int actionCount;

      private Plan(Pose3D rootPose,
                   double[][] fullJointPositions,
                   double[] trajectoryTimes,
                   ArmTrajectoryMessage armMessage,
                   NeckTrajectoryMessage neckMessage,
                   double duration,
                   double worstIKQuality,
                   int actionCount)
      {
         this.rootPose = rootPose;
         this.fullJointPositions = fullJointPositions;
         this.trajectoryTimes = trajectoryTimes;
         this.armMessage = armMessage;
         this.neckMessage = neckMessage;
         this.duration = duration;
         this.worstIKQuality = worstIKQuality;
         this.actionCount = actionCount;
      }
   }

   static final class BuildResult
   {
      final Plan plan;
      final String rejectionReason;
      final double rejectionQuality;

      private BuildResult(Plan plan, String rejectionReason, double rejectionQuality)
      {
         this.plan = plan;
         this.rejectionReason = rejectionReason;
         this.rejectionQuality = rejectionQuality;
      }

      private static BuildResult accepted(Plan plan)
      {
         return new BuildResult(plan, null, Double.NaN);
      }

      private static BuildResult rejected(String reason)
      {
         return new BuildResult(null, reason, Double.NaN);
      }

      private static BuildResult rejected(String reason, double quality)
      {
         return new BuildResult(null, reason, quality);
      }
   }

   private final ROS2SyncedRobotModel syncedRobot;
   private final Gr00tHumanoidConfiguration configuration;
   private final OneDoFJointBasics[] measuredArmJoints;
   private final OneDoFJointBasics[] armIKPathSeedJoints;
   private final OneDoFJointBasics[] measuredJointsExcludingHands;
   private final int[] armAllJointIndices;
   private final int[] neckAllJointIndices;
   private final String[] neckJointNames;
   private final int neckYawIndex;
   private final int neckPitchIndex;
   private final ArmIKSolver armIKSolver;
   private final PoseReferenceFrame targetFrame = new PoseReferenceFrame("gr00tArmIKTargetFrame", ReferenceFrame.getWorldFrame());

   Gr00tHumanoidArmPlanner(DRCRobotModel robotModel,
                           ROS2SyncedRobotModel syncedRobot,
                           Gr00tHumanoidConfiguration configuration)
   {
      this.syncedRobot = syncedRobot;
      this.configuration = configuration;
      FullHumanoidRobotModel ikRobotModel = robotModel.createFullRobotModel();
      RobotSide armSide = configuration.armSide();
      measuredArmJoints = FullRobotModelUtils.getArmJoints(syncedRobot.getFullRobotModel(),
                                                           armSide,
                                                           robotModel.getJointMap().getArmJointNames(armSide));
      armIKPathSeedJoints = FullRobotModelUtils.getArmJoints(ikRobotModel,
                                                             armSide,
                                                             robotModel.getJointMap().getArmJointNames(armSide));
      measuredJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(syncedRobot.getFullRobotModel());
      armAllJointIndices = findJointIndices(measuredJointsExcludingHands, measuredArmJoints, armSide.getPascalCaseName() + " arm");
      NeckJointName[] semanticNeckJoints = robotModel.getJointMap().getNeckJointNames();
      neckJointNames = new String[semanticNeckJoints.length];
      int yawIndex;
      int pitchIndex;
      for (int index = 0; index < semanticNeckJoints.length; index++)
         neckJointNames[index] = robotModel.getJointMap().getNeckJointName(semanticNeckJoints[index]);
      yawIndex = findNeckJointIndex(semanticNeckJoints, configuration.neckYawJoint());
      pitchIndex = findNeckJointIndex(semanticNeckJoints, configuration.neckPitchJoint());
      if (configuration.neckYawJoint() != null && yawIndex < 0)
         throw new IllegalStateException("Configured neck yaw joint is unavailable: " + configuration.neckYawJoint());
      if (configuration.neckPitchJoint() != null && pitchIndex < 0)
         throw new IllegalStateException("Configured neck pitch joint is unavailable: " + configuration.neckPitchJoint());
      neckYawIndex = yawIndex;
      neckPitchIndex = pitchIndex;
      neckAllJointIndices = findJointIndices(measuredJointsExcludingHands, neckJointNames, "Neck");
      armIKSolver = new ArmIKSolver(armSide, robotModel.getJointMap(), ikRobotModel);
   }

   static boolean isPoorQuality(Plan plan)
   {
      return plan.worstIKQuality > ArmIKSolver.GOOD_QUALITY_MAX;
   }

   private static int[] findJointIndices(OneDoFJointBasics[] allJoints, OneDoFJointBasics[] requestedJoints, String group)
   {
      String[] names = new String[requestedJoints.length];
      for (int index = 0; index < requestedJoints.length; index++)
         names[index] = requestedJoints[index].getName();
      return findJointIndices(allJoints, names, group);
   }

   private static int[] findJointIndices(OneDoFJointBasics[] allJoints, String[] requestedNames, String group)
   {
      int[] indices = new int[requestedNames.length];
      for (int requested = 0; requested < requestedNames.length; requested++)
      {
         indices[requested] = -1;
         for (int joint = 0; joint < allJoints.length; joint++)
         {
            if (requestedNames[requested].equals(allJoints[joint].getName()))
            {
               indices[requested] = joint;
               break;
            }
         }
         if (indices[requested] < 0)
            throw new IllegalStateException(group + " joint missing from preview model: " + requestedNames[requested]);
      }
      return indices;
   }

   /** Builds the whole path under one synchronized-model snapshot; failures never return partial messages. */
   BuildResult build(List<Gr00tHumanoidAction> candidates, boolean initialTransit, boolean allowPoorIK)
   {
      if (candidates == null || candidates.isEmpty())
         return BuildResult.rejected("Action plan rejected: no action candidates");

      synchronized (syncedRobot)
      {
         if (!syncedRobot.getDataReceptionTimerSnapshot().isRunning(0.1))
            return BuildResult.rejected("Action plan rejected: waiting for fresh robot data");

         FullHumanoidRobotModel liveRobotModel = syncedRobot.getFullRobotModel();
         FramePose3D startPose = new FramePose3D(liveRobotModel.getHand(configuration.armSide()).getBodyFixedFrame());
         startPose.changeFrame(ReferenceFrame.getWorldFrame());
         FramePose3D firstPolicyPose = new FramePose3D(ReferenceFrame.getWorldFrame());
         firstPolicyPose.getPosition().set(candidates.get(0).getWristPoseReadOnly(configuration.armSide()).getPosition());
         firstPolicyPose.getOrientation().set(candidates.get(0).getWristPoseReadOnly(configuration.armSide()).getOrientation());
         if (startPose.containsNaN() || firstPolicyPose.containsNaN())
            return BuildResult.rejected("Action plan rejected: non-finite start or target pose");

         int pointCount = candidates.size();
         FramePose3D[] cartesianTargets = createCartesianTargets(candidates, startPose);

         double[] trajectoryTimes = Gr00tHumanoidTrajectoryTools.computeTrajectoryTimes(configuration, cartesianTargets, initialTransit);
         double duration = trajectoryTimes[pointCount];
         if (!Double.isFinite(duration) || duration <= 0.0)
            return BuildResult.rejected("Action plan rejected: invalid required duration");

         seedIKFromMeasuredArm();
         int armJointCount = measuredArmJoints.length;
         double[][] armJointPositions = new double[pointCount + 1][armJointCount];
         double[][] fullJointPositions = createMeasuredPreviewPath(pointCount);
         double[][] neckJointPositions = createNeckPath(candidates, liveRobotModel);
         copyNeckIntoPreview(fullJointPositions, neckJointPositions);
         for (int joint = 0; joint < armJointCount; joint++)
            armJointPositions[0][joint] = measuredArmJoints[joint].getQ();

         double worstQuality = 0.0;
         for (int sample = 1; sample <= pointCount; sample++)
         {
            targetFrame.setPoseAndUpdate(cartesianTargets[sample]);
            armIKSolver.update(syncedRobot.getReferenceFrames().getChestFrame(), targetFrame);
            armIKSolver.solve();
            double quality = armIKSolver.getQuality();
            if (!Double.isFinite(quality))
               return BuildResult.rejected("Action plan rejected: non-finite IK quality at sample " + sample, quality);
            if (quality > ArmIKSolver.GOOD_QUALITY_MAX && !allowPoorIK)
            {
               return BuildResult.rejected("Action plan rejected: IK quality %.4g at sample %d exceeds %.4g"
                                                 .formatted(quality, sample, ArmIKSolver.GOOD_QUALITY_MAX),
                                           quality);
            }
            worstQuality = Math.max(worstQuality, quality);

            OneDoFJointBasics[] solutionJoints = armIKSolver.getSolutionOneDoFJoints();
            if (solutionJoints.length != armJointCount)
               return BuildResult.rejected("Action plan rejected: unexpected IK joint count");
            for (int joint = 0; joint < armJointCount; joint++)
            {
               double q = solutionJoints[joint].getQ();
               if (!Double.isFinite(q))
                  return BuildResult.rejected("Action plan rejected: non-finite IK joint at sample " + sample);
               armJointPositions[sample][joint] = q;
               fullJointPositions[sample][armAllJointIndices[joint]] = q;
               armIKPathSeedJoints[joint].setQ(q);
            }
         }

         ArmTrajectoryMessage armMessage = createArmTrajectoryMessage(configuration.armSide(), trajectoryTimes, armJointPositions);
         NeckTrajectoryMessage neckMessage = neckJointPositions[0].length == 0
                                               ? null
                                               : Gr00tHumanoidTrajectoryTools.createNeckTrajectoryMessage(trajectoryTimes, neckJointPositions);
         Pose3D rootPose = new Pose3D();
         rootPose.set(liveRobotModel.getRootJoint().getJointPose());
         return BuildResult.accepted(new Plan(rootPose,
                                              fullJointPositions,
                                              trajectoryTimes,
                                              armMessage,
                                              neckMessage,
                                              duration,
                                              worstQuality,
                                              candidates.size()));
      }
   }

   private FramePose3D[] createCartesianTargets(List<Gr00tHumanoidAction> candidates, FramePose3D startPose)
   {
      FramePose3D[] targets = new FramePose3D[candidates.size() + 1];
      targets[0] = new FramePose3D(startPose);
      for (int actionIndex = 0; actionIndex < candidates.size(); actionIndex++)
      {
         FramePose3D target = new FramePose3D(ReferenceFrame.getWorldFrame());
         target.getPosition().set(candidates.get(actionIndex).getWristPoseReadOnly(configuration.armSide()).getPosition());
         target.getOrientation().set(candidates.get(actionIndex).getWristPoseReadOnly(configuration.armSide()).getOrientation());
         targets[actionIndex + 1] = target;
      }
      return targets;
   }

   private void seedIKFromMeasuredArm()
   {
      for (int joint = 0; joint < measuredArmJoints.length; joint++)
      {
         armIKPathSeedJoints[joint].setQ(measuredArmJoints[joint].getQ());
         armIKPathSeedJoints[joint].setQd(measuredArmJoints[joint].getQd());
      }
   }

   private double[][] createMeasuredPreviewPath(int pointCount)
   {
      double[][] fullJointPositions = new double[pointCount + 1][measuredJointsExcludingHands.length];
      for (int sample = 0; sample <= pointCount; sample++)
      {
         for (int joint = 0; joint < measuredJointsExcludingHands.length; joint++)
            fullJointPositions[sample][joint] = measuredJointsExcludingHands[joint].getQ();
      }
      return fullJointPositions;
   }

   private double[][] createNeckPath(List<Gr00tHumanoidAction> candidates, FullHumanoidRobotModel liveRobotModel)
   {
      double[][] neckJointPositions = new double[candidates.size() + 1][neckJointNames.length];
      for (int joint = 0; joint < neckJointNames.length; joint++)
         neckJointPositions[0][joint] = liveRobotModel.getOneDoFJointByName(neckJointNames[joint]).getQ();
      for (int actionIndex = 0; actionIndex < candidates.size(); actionIndex++)
      {
         int sample = actionIndex + 1;
         neckJointPositions[sample] = neckJointPositions[0].clone();
         if (neckYawIndex >= 0)
            neckJointPositions[sample][neckYawIndex] = clampToJointLimits(liveRobotModel,
                                                                         neckJointNames[neckYawIndex],
                                                                         candidates.get(actionIndex).getNeckYaw());
         if (neckPitchIndex >= 0)
            neckJointPositions[sample][neckPitchIndex] = clampToJointLimits(liveRobotModel,
                                                                             neckJointNames[neckPitchIndex],
                                                                             candidates.get(actionIndex).getNeckPitch());
      }
      return neckJointPositions;
   }

   private static double clampToJointLimits(FullHumanoidRobotModel robotModel, String jointName, double target)
   {
      OneDoFJointBasics joint = robotModel.getOneDoFJointByName(jointName);
      return clampToLimits(target, joint.getJointLimitLower(), joint.getJointLimitUpper());
   }

   static double clampToLimits(double target, double lowerLimit, double upperLimit)
   {
      return Math.max(lowerLimit, Math.min(upperLimit, target));
   }

   static int findNeckJointIndex(NeckJointName[] availableJoints, NeckJointName configuredJoint)
   {
      if (configuredJoint == null)
         return -1;
      for (int index = 0; index < availableJoints.length; index++)
      {
         if (availableJoints[index] == configuredJoint)
            return index;
      }
      return -1;
   }

   private void copyNeckIntoPreview(double[][] fullJointPositions, double[][] neckJointPositions)
   {
      for (int sample = 0; sample < fullJointPositions.length; sample++)
      {
         for (int joint = 0; joint < neckAllJointIndices.length; joint++)
            fullJointPositions[sample][neckAllJointIndices[joint]] = neckJointPositions[sample][joint];
      }
   }

   private static ArmTrajectoryMessage createArmTrajectoryMessage(RobotSide side,
                                                                  double[] trajectoryTimes,
                                                                  double[][] armJointPositions)
   {
      int pointCount = trajectoryTimes.length - 1;
      ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(side);
      for (int joint = 0; joint < armJointPositions[0].length; joint++)
      {
         var jointTrajectory = message.getJointspaceTrajectory().getJointTrajectoryMessages().add();
         jointTrajectory.setWeight(-1.0);
         for (int sample = 1; sample <= pointCount; sample++)
         {
            var point = jointTrajectory.getTrajectoryPoints().add();
            point.setTime(trajectoryTimes[sample]);
            point.setPosition(armJointPositions[sample][joint]);
            double velocity = sample == pointCount ? 0.0
                                                   : (armJointPositions[sample + 1][joint] - armJointPositions[sample - 1][joint])
                                                     / (trajectoryTimes[sample + 1] - trajectoryTimes[sample - 1]);
            point.setVelocity(velocity);
         }
      }
      return message;
   }
}
