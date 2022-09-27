package us.ihmc.avatar.multiContact;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import ihmc_common_msgs.msg.dds.FrameInformation;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.SO3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SO3TrajectoryPointMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryPointOptimizer;
import us.ihmc.robotics.partNames.RigidBodyName;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This class implement preliminary work for post-processing a pre-generated script defined in
 * jointspace. The objective is to smooth out the execution and potentially stabilize the script.
 */
public class WholeBodyScriptPostProcessor
{
   private final RigidBodyBasics rootBody;
   private final FloatingJointBasics rootJoint;
   private final int jointNameHash;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final TrajectoryPointOptimizer trajectoryPointOptimizer;

   private double durationPerKeyframe = 1.0;
   private double trajectoryInitialDelay = 0.5;

   public WholeBodyScriptPostProcessor(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      rootBody = fullRobotModel.getElevator();
      rootJoint = fullRobotModel.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      jointNameHash = Arrays.hashCode(oneDoFJoints);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      trajectoryPointOptimizer = new TrajectoryPointOptimizer(oneDoFJoints.length);
      
      
      for (int jointIndex = 0; jointIndex < oneDoFJoints.length; jointIndex++)
      {
         System.out.println(oneDoFJoints[jointIndex].getName());
      }
   }

   public WholeBodyTrajectoryMessage generateWholeBodyMessage(List<KinematicsToolboxSnapshotDescription> rawScript, RigidBodyName[] rigidBodiesToControl, double[] trajectoryTiming)
   {
      checkJointNameHash(rawScript);
      List<KinematicsToolboxOutputStatus> desiredRobotConfigurations = rawScript.stream().map(item -> item.getIkSolution()).collect(Collectors.toList());
      KinematicsToolboxOutputStatus startDesiredConfiguration = desiredRobotConfigurations.get(0);
      KinematicsToolboxOutputStatus targetDesiredConfiguration = desiredRobotConfigurations.get(desiredRobotConfigurations.size() - 1);

      int numberOfJoints = startDesiredConfiguration.getDesiredJointAngles().size();

      TDoubleArrayList startPosition = toTDoubleArrayList(startDesiredConfiguration.getDesiredJointAngles());
      TDoubleArrayList startVelocity = zeros(numberOfJoints);
      TDoubleArrayList targetPosition = toTDoubleArrayList(targetDesiredConfiguration.getDesiredJointAngles());
      TDoubleArrayList targetVelocity = zeros(numberOfJoints);
      List<TDoubleArrayList> waypoints = new ArrayList<>();

      for (int i = 0; i < desiredRobotConfigurations.size(); i++)
      {
         waypoints.add(toTDoubleArrayList(desiredRobotConfigurations.get(i).getDesiredJointAngles()));
      }
      
      trajectoryPointOptimizer.setEndPoints(startPosition, startVelocity, targetPosition, targetVelocity);
      trajectoryPointOptimizer.setWaypoints(waypoints);
      trajectoryPointOptimizer.compute(0);

      if(trajectoryTiming == null)
      {
         for (int i = 0; i < 100; i++)
         {
            if (trajectoryPointOptimizer.doFullTimeUpdate())
               break;
            LogTools.info("Iteration: " + i);
         }
      }
         


      double trajectoryDuration = desiredRobotConfigurations.size() * durationPerKeyframe;
      
      WholeBodyTrajectoryMessage message = createWholebodyCommands(fullRobotModel,
                                                                  referenceFrames,
                                                                  rigidBodiesToControl,
                                                                  desiredRobotConfigurations,
                                                                  waypoints,
                                                                  trajectoryTiming,
                                                                  trajectoryDuration,
                                                                  trajectoryInitialDelay);
      return message;
   }

   public WholeBodyTrajectoryMessage createWholebodyCommands(FullHumanoidRobotModel fullRobotModel, 
                                                            HumanoidReferenceFrames humanoidReferenceFrames, 
                                                            RigidBodyName[] rigidBodiesToControl, 
                                                            List<KinematicsToolboxOutputStatus> desiredRobotConfigurations, 
                                                            List<TDoubleArrayList> waypoints, 
                                                            double[] trajectoryTiming, 
                                                            double trajectoryDuration,
                                                            double startOffset)
   {
      //helper variables
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics rightFoot = fullRobotModel.getFoot(RobotSide.RIGHT);
      RigidBodyBasics leftFoot = fullRobotModel.getFoot(RobotSide.LEFT);

      FramePose3D rightFootPose = new FramePose3D();
      FramePose3D pelvisPose = new FramePose3D();
      FrameQuaternion chestOrientation = new FrameQuaternion();
      
      Twist twist = new Twist();
      
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
      FootTrajectoryMessage rightFootTrajectoryMessage = new FootTrajectoryMessage();
      rightFootTrajectoryMessage.setRobotSide(RobotSide.RIGHT.toByte());
      
      //get the trajectories
      SE3TrajectoryMessage pelvisTrajectory = pelvisTrajectoryMessage.getSe3Trajectory();
      SO3TrajectoryMessage chestOrientationTrajectory = chestTrajectoryMessage.getSo3Trajectory();
      SE3TrajectoryMessage rightFootTrajectory = rightFootTrajectoryMessage.getSe3Trajectory();
      
      //reference frames
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      MovingReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      MovingReferenceFrame leftFootFrame = humanoidReferenceFrames.getFootFrame(RobotSide.LEFT);
      
      ReferenceFrame pelvisTrajectoryReferenceFrame = worldFrame;
      ReferenceFrame footTrajectoryReferenceFrame = worldFrame;
      ReferenceFrame chestTrajectoryReferenceFrame = worldFrame;
      

      //set the trajectory frames 
      FrameInformation pelvisFrameInformation = pelvisTrajectory.getFrameInformation();
      pelvisFrameInformation.setDataReferenceFrameId(MessageTools.toFrameId(pelvisTrajectoryReferenceFrame));
      pelvisFrameInformation.setTrajectoryReferenceFrameId(MessageTools.toFrameId(pelvisTrajectoryReferenceFrame));
      
      FrameInformation chestTrajectoryFrameInformation = chestOrientationTrajectory.getFrameInformation();
      chestTrajectoryFrameInformation.setDataReferenceFrameId(MessageTools.toFrameId(chestTrajectoryReferenceFrame));
      chestTrajectoryFrameInformation.setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestTrajectoryReferenceFrame));
      
      FrameInformation rightFootTrajectoryFrameInformation = rightFootTrajectory.getFrameInformation();
      rightFootTrajectoryFrameInformation.setDataReferenceFrameId(MessageTools.toFrameId(footTrajectoryReferenceFrame));
      rightFootTrajectoryFrameInformation.setTrajectoryReferenceFrameId(MessageTools.toFrameId(footTrajectoryReferenceFrame));
      
      
      
      for (int configurationIndex = 0; configurationIndex < desiredRobotConfigurations.size(); configurationIndex++)
      {
         double waypointTime = trajectoryPointOptimizer.getWaypointTime(configurationIndex) * trajectoryDuration + startOffset;
         if(trajectoryTiming != null)
         {
            waypointTime = trajectoryTiming[configurationIndex] + startOffset;
         }
         
         System.out.println("waypointTime: " + waypointTime);
         KinematicsToolboxOutputStatus robotConfiguration = desiredRobotConfigurations.get(configurationIndex);
      
         Point3D desiredRootPosition = robotConfiguration.getDesiredRootPosition();
         Quaternion desiredRootOrientation = robotConfiguration.getDesiredRootOrientation();
         Vector3D desiredRootLinearVelocity = robotConfiguration.getDesiredRootLinearVelocity();
         Vector3D desiredRootAngularVelocity = robotConfiguration.getDesiredRootAngularVelocity();
         
         rootJoint.setJointPosition(desiredRootPosition);
         rootJoint.setJointOrientation(desiredRootOrientation);
         rootJoint.setJointLinearVelocity(desiredRootLinearVelocity);
         rootJoint.setJointAngularVelocity(desiredRootAngularVelocity);
         
         
         TDoubleArrayList jointAngles = waypoints.get(configurationIndex);
         TDoubleArrayList velocities = new TDoubleArrayList(oneDoFJoints.length);
         trajectoryPointOptimizer.getWaypointVelocity(velocities, configurationIndex);

         for (int jointIndex = 0; jointIndex < oneDoFJoints.length; jointIndex++)
         {
            oneDoFJoints[jointIndex].setQ(jointAngles.get(jointIndex));
            oneDoFJoints[jointIndex].setQd(velocities.get(jointIndex));
         }
         fullRobotModel.updateFrames();
         
         
         //right foot message
         rightFootPose.setToZero(rightFoot.getBodyFixedFrame());
         rightFootPose.changeFrame(worldFrame);
         rightFoot.getBodyFixedFrame().getTwistRelativeToOther(worldFrame, twist);
         
         SE3TrajectoryPointMessage footTrajectoryPoint = rightFootTrajectory.getTaskspaceTrajectoryPoints().add();
         footTrajectoryPoint.setTime(waypointTime);
         footTrajectoryPoint.getPosition().set(rightFootPose.getPosition());
         footTrajectoryPoint.getOrientation().set(rightFootPose.getOrientation());
         footTrajectoryPoint.getLinearVelocity().set(twist.getLinearPart());
         footTrajectoryPoint.getAngularVelocity().set(twist.getAngularPart());
         
         //chest message
         chestOrientation.setToZero(chest.getBodyFixedFrame());
         chestOrientation.changeFrame(worldFrame);
         chest.getBodyFixedFrame().getTwistRelativeToOther(worldFrame, twist);
         
         SO3TrajectoryPointMessage chestTrajectoryPoint = chestOrientationTrajectory.getTaskspaceTrajectoryPoints().add();
         chestTrajectoryPoint.setTime(waypointTime);
         chestTrajectoryPoint.getOrientation().set(chestOrientation);
//         chestTrajectoryPoint.getAngularVelocity().set(twist.getAngularPart());
         
         //pelvis message
         pelvisPose.setToZero(pelvis.getBodyFixedFrame());
         pelvisPose.changeFrame(worldFrame);
         pelvis.getBodyFixedFrame().getTwistRelativeToOther(worldFrame, twist);
         
         SE3TrajectoryPointMessage pelvisTrajectoryPoint = pelvisTrajectory.getTaskspaceTrajectoryPoints().add();
         pelvisTrajectoryPoint.setTime(waypointTime);
         pelvisTrajectoryPoint.getPosition().set(pelvisPose.getPosition());
         pelvisTrajectoryPoint.getOrientation().set(pelvisPose.getOrientation());
//         pelvisTrajectoryPoint.getLinearVelocity().set(twist.getLinearPart());
//         pelvisTrajectoryPoint.getAngularVelocity().set(twist.getAngularPart());
      }
      
      //populate messages
      WholeBodyTrajectoryMessage message = new WholeBodyTrajectoryMessage();
      
      for(int i = 0; i < rigidBodiesToControl.length; i++)
      {
         switch (rigidBodiesToControl[i])
         {
            case CHEST:
               message.getChestTrajectoryMessage().set(chestTrajectoryMessage);
               break;
            case PELVIS:
               message.getPelvisTrajectoryMessage().set(pelvisTrajectoryMessage);
               break;
            case RIGHT_FOOT:
               message.getRightFootTrajectoryMessage().set(rightFootTrajectoryMessage);
               break;

            default:
               break;
         }
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

   public void setDurationPerKeyframe(double durationPerKeyframe)
   {
      this.durationPerKeyframe = durationPerKeyframe;
   }

   public void setTrajectoryInitialDelay(double trajectoryInitialDelay)
   {
      this.trajectoryInitialDelay = trajectoryInitialDelay;
   }
}