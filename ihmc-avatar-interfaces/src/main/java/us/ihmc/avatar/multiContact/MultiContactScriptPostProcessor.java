package us.ihmc.avatar.multiContact;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import controller_msgs.msg.dds.RobotConfigurationData;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryPointOptimizer;

/**
 * This class implement preliminary work for post-processing a pre-generated script defined in
 * jointspace. The objective is to smooth out the execution and potentially stabilize the script.
 */
public class MultiContactScriptPostProcessor
{
   private final RigidBodyBasics rootBody;
   private final FloatingJointBasics rootJoint;
   private final int jointNameHash;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final TDoubleArrayList waypointTimes = new TDoubleArrayList();

   public MultiContactScriptPostProcessor(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      FullHumanoidRobotModel fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      rootBody = fullRobotModel.getElevator();
      rootJoint = fullRobotModel.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = Arrays.hashCode(oneDoFJoints);
   }

   public WholeBodyJointspaceTrajectoryMessage process1(List<KinematicsToolboxSnapshotDescription> rawScript)
   {
      checkJointNameHash(rawScript);

      computeRegularizedWaypointTimes(rawScript);
      List<KinematicsToolboxOutputStatus> desiredRobotConfigurations = rawScript.stream().map(KinematicsToolboxSnapshotDescription::getIkSolution).collect(Collectors.toList());
      RobotConfigurationData startDesiredConfiguration = rawScript.get(0).getControllerConfiguration();
      KinematicsToolboxOutputStatus targetDesiredConfiguration = desiredRobotConfigurations.get(desiredRobotConfigurations.size() - 1);

      double totalTrajectoryTime = 0.0;
      for (int i = 0; i < rawScript.size(); i++)
      {
         totalTrajectoryTime += rawScript.get(i).getExecutionDuration();
      }

      int numberOfJoints = startDesiredConfiguration.getJointAngles().size();
      int numberOfConfigurations = 1 + desiredRobotConfigurations.size();

      TDoubleArrayList startPosition = toTDoubleArrayList(startDesiredConfiguration.getJointAngles());
      TDoubleArrayList startVelocity = zeros(numberOfJoints);
      TDoubleArrayList targetPosition = toTDoubleArrayList(targetDesiredConfiguration.getDesiredJointAngles());
      TDoubleArrayList targetVelocity = zeros(numberOfJoints);
      List<TDoubleArrayList> waypoints = new ArrayList<>();

      for (int i = 0; i < desiredRobotConfigurations.size() - 1; i++)
      {
         waypoints.add(toTDoubleArrayList(desiredRobotConfigurations.get(i).getDesiredJointAngles()));
      }

      TrajectoryPointOptimizer trajectoryPointOptimizer = new TrajectoryPointOptimizer(numberOfJoints);
      trajectoryPointOptimizer.setEndPoints(startPosition, startVelocity, targetPosition, targetVelocity);
      trajectoryPointOptimizer.setWaypoints(waypoints);

      trajectoryPointOptimizer.computeForFixedTime(waypointTimes);

      WholeBodyJointspaceTrajectoryMessage message = new WholeBodyJointspaceTrajectoryMessage();
      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         message.getJointHashCodes().add(oneDoFJoints[jointIndex].hashCode());
         OneDoFJointTrajectoryMessage jointTrajectory = message.getJointTrajectoryMessages().add();

         TrajectoryPoint1DMessage startTrajectoryPoint = jointTrajectory.getTrajectoryPoints().add();
         startTrajectoryPoint.setTime(0.0);
         startTrajectoryPoint.setPosition(startPosition.get(jointIndex));
         startTrajectoryPoint.setVelocity(startVelocity.get(jointIndex));

         for (int waypointIndex = 0; waypointIndex < numberOfConfigurations - 2; waypointIndex++)
         {
            TrajectoryPoint1DMessage trajectoryPoint = jointTrajectory.getTrajectoryPoints().add();
            double time = trajectoryPointOptimizer.getWaypointTime(waypointIndex) * totalTrajectoryTime;
            trajectoryPoint.setTime(time);
            trajectoryPoint.setPosition(waypoints.get(waypointIndex).get(jointIndex));
            trajectoryPoint.setVelocity(trajectoryPointOptimizer.getWaypointTime(waypointIndex) / totalTrajectoryTime);
         }

         TrajectoryPoint1DMessage targetTrajectoryPoint = jointTrajectory.getTrajectoryPoints().add();
         targetTrajectoryPoint.setTime(totalTrajectoryTime);
         targetTrajectoryPoint.setPosition(targetPosition.get(jointIndex));
         targetTrajectoryPoint.setVelocity(targetVelocity.get(jointIndex));
      }

      return message;
   }

   public WholeBodyJointspaceTrajectoryMessage process2(List<KinematicsToolboxSnapshotDescription> rawScript)
   {
      checkJointNameHash(rawScript);

      CenterOfMassCalculator centerOfMassCalculator = new CenterOfMassCalculator(rootBody, ReferenceFrame.getWorldFrame());

      List<TDoubleArrayList> centroidalConfigurations = new ArrayList<>();

      for (KinematicsToolboxSnapshotDescription item : rawScript)
      {
         KinematicsToolboxOutputStatus ikSolution = item.getIkSolution();
         rootJoint.getJointPose().set(ikSolution.getDesiredRootPosition(), ikSolution.getDesiredRootOrientation());
         for (int i = 0; i < ikSolution.getDesiredJointAngles().size(); i++)
         {
            oneDoFJoints[i].setQ(ikSolution.getDesiredJointAngles().get(i));
         }

         rootBody.updateFramesRecursively();
         centerOfMassCalculator.reset();
         centroidalConfigurations.add(toTDoubleArrayList(centerOfMassCalculator.getCenterOfMass().getX(), centerOfMassCalculator.getCenterOfMass().getY()));

      }

      int dimensions = 2;
      TrajectoryPointOptimizer trajectoryPointOptimizer = new TrajectoryPointOptimizer(dimensions);
      trajectoryPointOptimizer.setEndPoints(centroidalConfigurations.get(0),
                                            zeros(dimensions),
                                            centroidalConfigurations.get(centroidalConfigurations.size() - 1),
                                            zeros(dimensions));
      trajectoryPointOptimizer.setWaypoints(centroidalConfigurations.subList(1, centroidalConfigurations.size() - 1));
      trajectoryPointOptimizer.compute();
      TDoubleArrayList waypointTimes = new TDoubleArrayList();
      trajectoryPointOptimizer.getWaypointTimes(waypointTimes);
      System.out.println(waypointTimes);

      List<KinematicsToolboxOutputStatus> desiredRobotConfigurations = rawScript.stream().map(item -> item.getIkSolution()).collect(Collectors.toList());
      KinematicsToolboxOutputStatus startDesiredConfiguration = desiredRobotConfigurations.get(0);
      KinematicsToolboxOutputStatus targetDesiredConfiguration = desiredRobotConfigurations.get(desiredRobotConfigurations.size() - 1);

      int numberOfJoints = startDesiredConfiguration.getDesiredJointAngles().size();
      int numberOfConfigurations = desiredRobotConfigurations.size();

      TDoubleArrayList startPosition = toTDoubleArrayList(startDesiredConfiguration.getDesiredJointAngles());
      TDoubleArrayList startVelocity = zeros(numberOfJoints);
      TDoubleArrayList targetPosition = toTDoubleArrayList(targetDesiredConfiguration.getDesiredJointAngles());
      TDoubleArrayList targetVelocity = zeros(numberOfJoints);
      List<TDoubleArrayList> waypoints = new ArrayList<>();

      for (int i = 1; i < desiredRobotConfigurations.size() - 1; i++)
      {
         waypoints.add(toTDoubleArrayList(desiredRobotConfigurations.get(i).getDesiredJointAngles()));
      }

      trajectoryPointOptimizer = new TrajectoryPointOptimizer(numberOfJoints);
      trajectoryPointOptimizer.setEndPoints(startPosition, startVelocity, targetPosition, targetVelocity);
      trajectoryPointOptimizer.setWaypoints(waypoints);
      trajectoryPointOptimizer.computeForFixedTime(waypointTimes);

      WholeBodyJointspaceTrajectoryMessage message = new WholeBodyJointspaceTrajectoryMessage();
      double trajectoryTimeOffset = 0.0;
      double totalTrajectoryTime = 0.5 * numberOfConfigurations;

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         message.getJointHashCodes().add(oneDoFJoints[jointIndex].hashCode());
         OneDoFJointTrajectoryMessage jointTrajectory = message.getJointTrajectoryMessages().add();

         TrajectoryPoint1DMessage startTrajectoryPoint = jointTrajectory.getTrajectoryPoints().add();
         startTrajectoryPoint.setTime(trajectoryTimeOffset);
         startTrajectoryPoint.setPosition(startPosition.get(jointIndex));
         startTrajectoryPoint.setVelocity(startVelocity.get(jointIndex));

         for (int waypointIndex = 0; waypointIndex < numberOfConfigurations - 2; waypointIndex++)
         {
            TrajectoryPoint1DMessage trajectoryPoint = jointTrajectory.getTrajectoryPoints().add();
            trajectoryPoint.setTime(trajectoryTimeOffset + trajectoryPointOptimizer.getWaypointTime(waypointIndex) * totalTrajectoryTime);
            trajectoryPoint.setPosition(waypoints.get(waypointIndex).get(jointIndex));
            trajectoryPoint.setVelocity(trajectoryPointOptimizer.getWaypointTime(waypointIndex) / totalTrajectoryTime);
         }

         TrajectoryPoint1DMessage targetTrajectoryPoint = jointTrajectory.getTrajectoryPoints().add();
         targetTrajectoryPoint.setTime(trajectoryTimeOffset + totalTrajectoryTime);
         targetTrajectoryPoint.setPosition(targetPosition.get(jointIndex));
         targetTrajectoryPoint.setVelocity(targetVelocity.get(jointIndex));
      }

      return message;
   }

   private void checkJointNameHash(List<KinematicsToolboxSnapshotDescription> rawScript)
   {
      for (KinematicsToolboxSnapshotDescription item : rawScript)
      {
         if (item.getIkSolution().getJointNameHash() != jointNameHash)
            throw new IllegalArgumentException("Incompatible script");
      }
   }

   private void computeRegularizedWaypointTimes(List<KinematicsToolboxSnapshotDescription> script)
   {
      waypointTimes.clear();

      double trajectoryDuration = 0.0;
      for (int i = 0; i < script.size(); i++)
      {
         trajectoryDuration += script.get(i).getExecutionDuration();
      }

      double runningTotal = 0.0;
      for (int i = 0; i < script.size() - 1; i++)
      {
         runningTotal += script.get(i).getExecutionDuration();
         waypointTimes.add(runningTotal / trajectoryDuration);
      }
   }

   private static TDoubleArrayList toTDoubleArrayList(double... values)
   {
      return new TDoubleArrayList(values);
   }

   private static TDoubleArrayList toTDoubleArrayList(TFloatArrayList input)
   {
      TDoubleArrayList output = new TDoubleArrayList(input.size());
      for (int i = 0; i < input.size(); i++)
      {
         output.add(input.get(i));
      }
      return output;
   }

   private static TDoubleArrayList zeros(int size)
   {
      TDoubleArrayList output = new TDoubleArrayList(size);
      for (int i = 0; i < size; i++)
      {
         output.add(0.0);
      }
      return output;
   }
}
