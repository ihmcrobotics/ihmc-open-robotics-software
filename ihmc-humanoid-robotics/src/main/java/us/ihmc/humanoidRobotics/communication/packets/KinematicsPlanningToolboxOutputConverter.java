package us.ihmc.humanoidRobotics.communication.packets;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.math.trajectories.generators.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.generators.SO3TrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class KinematicsPlanningToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage;
   private final KinematicsToolboxOutputConverter converter;

   private final List<KinematicsToolboxOutputStatus> keyFrames = new ArrayList<KinematicsToolboxOutputStatus>();
   private final TDoubleArrayList keyFrameTimes = new TDoubleArrayList();
   private final AtomicReference<KinematicsPlanningToolboxOutputStatus> solution;
   private int numberOfTrajectoryPoints;

   private final SideDependentList<List<String>> armJointNamesFromShoulder = new SideDependentList<List<String>>();

   public KinematicsPlanningToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      converter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);
      solution = new AtomicReference<KinematicsPlanningToolboxOutputStatus>();
      for (RobotSide robotSide : RobotSide.values)
         armJointNamesFromShoulder.put(robotSide, new ArrayList<String>());

      for (RobotSide robotSide : RobotSide.values)
      {
         List<String> armJointNamesFromHand = new ArrayList<String>();
         JointBasics armJoint = converter.getFullRobotModel().getHand(robotSide).getParentJoint();
         while (armJoint.getPredecessor() != converter.getFullRobotModel().getElevator())
         {
            String armJointName = armJoint.getName();
            if (armJointName.contains(robotSide.getLowerCaseName()))
            {
               armJointNamesFromHand.add(armJointName);
            }
            armJoint = armJoint.getPredecessor().getParentJoint();
         }
         for (int i = armJointNamesFromHand.size(); i > 0; i--)
         {
            armJointNamesFromShoulder.get(robotSide).add(armJointNamesFromHand.get(i - 1));
         }
      }
   }

   public void computeWholeBodyTrajectoryMessage(KinematicsPlanningToolboxOutputStatus solution)
   {
      getToolboxSolution(solution);
      computeArmTrajectoryMessages();
      computeChestTrajectoryMessage();
      computePelvisTrajectoryMessage();
   }

   private void computeChestTrajectoryMessage()
   {
      ChestTrajectoryMessage trajectoryMessage = new ChestTrajectoryMessage();

      SO3TrajectoryMessage so3Trajectory = trajectoryMessage.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
      so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      Quaternion[] desiredOrientations = new Quaternion[numberOfTrajectoryPoints];

      SO3TrajectoryPointCalculator orientationCalculator = new SO3TrajectoryPointCalculator();
      orientationCalculator.clear();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         KinematicsToolboxOutputStatus keyFrame = solution.get().getRobotConfigurations().get(i);
         converter.updateFullRobotModel(keyFrame);

         ReferenceFrame controlFrame = converter.getFullRobotModel().getChest().getBodyFixedFrame();
         FramePose3D desiredHandPose = new FramePose3D(controlFrame);
         desiredHandPose.changeFrame(worldFrame);
         Quaternion desiredOrientation = new Quaternion(desiredHandPose.getOrientation());

         desiredOrientations[i] = new Quaternion(desiredOrientation);

         double time = keyFrameTimes.get(i);
         orientationCalculator.appendTrajectoryPoint(time, desiredOrientation);

      }

      orientationCalculator.useSecondOrderInitialGuess();
      orientationCalculator.compute();

      for (int i = 1; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D desiredAngularVelocity = new Vector3D();

         double time = keyFrameTimes.get(i);

         orientationCalculator.getTrajectoryPoint(i).getAngularVelocity(desiredAngularVelocity);

         SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
         trajectoryPoint.setTime(time);
         trajectoryPoint.getOrientation().set(desiredOrientations[i]);
         trajectoryPoint.getAngularVelocity().set(desiredAngularVelocity);
      }

      wholeBodyTrajectoryMessage.getChestTrajectoryMessage().set(trajectoryMessage);
   }

   private void computePelvisTrajectoryMessage()
   {
      PelvisTrajectoryMessage trajectoryMessage = new PelvisTrajectoryMessage();

      trajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
      trajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

      Point3D[] desiredPositions = new Point3D[numberOfTrajectoryPoints];
      Quaternion[] desiredOrientations = new Quaternion[numberOfTrajectoryPoints];

      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
      SO3TrajectoryPointCalculator orientationCalculator = new SO3TrajectoryPointCalculator();
      euclideanTrajectoryPointCalculator.clear();
      orientationCalculator.clear();

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         KinematicsToolboxOutputStatus keyFrame = solution.get().getRobotConfigurations().get(i);
         converter.updateFullRobotModel(keyFrame);

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         ReferenceFrame controlFrame = converter.getFullRobotModel().getPelvis().getBodyFixedFrame();
         FramePose3D desiredHandPose = new FramePose3D(controlFrame);
         desiredHandPose.changeFrame(worldFrame);
         desiredHandPose.get(desiredPosition, desiredOrientation);

         desiredPositions[i] = new Point3D(desiredPosition);
         desiredOrientations[i] = new Quaternion(desiredOrientation);

         double time = keyFrameTimes.get(i);
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(time, new Point3D(desiredPosition));
         orientationCalculator.appendTrajectoryPoint(time, desiredOrientation);
      }

      orientationCalculator.useSecondOrderInitialGuess();
      orientationCalculator.compute();
      euclideanTrajectoryPointCalculator.compute(keyFrameTimes.get(numberOfTrajectoryPoints - 1));
      FrameEuclideanTrajectoryPointList trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      for (int i = 1; i < numberOfTrajectoryPoints; i++)
      {
         Vector3D desiredLinearVelocity = new Vector3D();
         Vector3D desiredAngularVelocity = new Vector3D();

         trajectoryPoints.getTrajectoryPoint(i).get(desiredPositions[i], desiredLinearVelocity);
         double time = trajectoryPoints.getTrajectoryPoint(i).getTime();

         orientationCalculator.getTrajectoryPoint(i).getAngularVelocity(desiredAngularVelocity);

         trajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                          .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, desiredPositions[i], desiredOrientations[i], desiredLinearVelocity,
                                                                                    desiredAngularVelocity));
      }

      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().set(trajectoryMessage);
   }

   private void computeArmTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeArmTrajectoryMessage(robotSide);
   }

   private void computeArmTrajectoryMessage(RobotSide robotSide)
   {
      ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
      JointspaceTrajectoryMessage jointspaceTrajectory = message.getJointspaceTrajectory();

      List<String> armJointNames = armJointNamesFromShoulder.get(robotSide);
      for (String jointName : armJointNames)
      {
         OneDoFTrajectoryPointList trajectoryPoints = new OneDoFTrajectoryPointList();
         for (int i = 1; i < numberOfTrajectoryPoints; i++)
         {
            KinematicsToolboxOutputStatus keyFrame = solution.get().getRobotConfigurations().get(i);

            converter.updateFullRobotModel(keyFrame);
            OneDoFJointBasics oneDoFJoint = converter.getFullRobotModel().getOneDoFJointByName(jointName);
            double desiredPosition = MathTools.clamp(oneDoFJoint.getQ(), oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
            double desiredVelocity = oneDoFJoint.getQd();

            trajectoryPoints.addTrajectoryPoint(keyFrameTimes.get(i), desiredPosition, desiredVelocity);
         }
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = HumanoidMessageTools.createOneDoFJointTrajectoryMessage(trajectoryPoints);

         jointspaceTrajectory.getJointTrajectoryMessages().add().set(oneDoFJointTrajectoryMessage);
      }

      if (robotSide == RobotSide.LEFT)
         wholeBodyTrajectoryMessage.getLeftArmTrajectoryMessage().set(message);
      else
         wholeBodyTrajectoryMessage.getRightArmTrajectoryMessage().set(message);
   }

   private void getToolboxSolution(KinematicsPlanningToolboxOutputStatus toolboxSolution)
   {
      solution.set(toolboxSolution);

      keyFrames.clear();
      keyFrames.addAll(solution.get().getRobotConfigurations());

      keyFrameTimes.clear();
      keyFrameTimes.addAll(solution.get().getKeyFrameTimes());

      numberOfTrajectoryPoints = keyFrames.size();
   }

   public void setMessageToCreate(WholeBodyTrajectoryMessage wholebodyTrajectoryMessage)
   {
      this.wholeBodyTrajectoryMessage = wholebodyTrajectoryMessage;
   }

   public KinematicsToolboxOutputStatus getRobotConfiguration(KinematicsPlanningToolboxOutputStatus solution, double time)
   {
      double minimumGap = Double.MAX_VALUE;
      int nodeIndex = 0;
      for (int i = 0; i < solution.getKeyFrameTimes().size(); i++)
      {
         double gap = Math.abs(time - solution.getKeyFrameTimes().get(i));
         if (gap < minimumGap)
         {
            minimumGap = gap;
            nodeIndex = i;
         }
      }

      return solution.getRobotConfigurations().get(nodeIndex);
   }
}
