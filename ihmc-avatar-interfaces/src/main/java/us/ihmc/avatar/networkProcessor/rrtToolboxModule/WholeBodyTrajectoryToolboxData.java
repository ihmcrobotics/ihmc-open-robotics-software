package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialData;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

/**
 * This class is for packing input of the controller as like as a packet
 * {@link WholeBodyTrajectoryToolboxMessage}.
 * <p>
 * - trajectory time, initial configuration.
 * <p>
 * - list of WaypointBasedTrajectoryMessage
 * <p>
 * - list of RigidBodyExplorationConfigurationMessage
 * <p>
 * 
 * @link {WholeBodyTrajectoryToolboxMessage is converted to (this).
 *       <p>
 *       This will be used for {@link HumanoidKinematicsSolver}.
 * @author Inho, Sylvain.
 *
 */
public class WholeBodyTrajectoryToolboxData
{
   private static final boolean VERBOSE = false;

   private final FullHumanoidRobotModel fullRobotModel;
   
   private double trajectoryTime;
   private int dimensionOfExploration = 0;

   private final List<RigidBody> allRigidBodies = new ArrayList<>();
   private final Map<String, RigidBody> nameToRigidBodyMap = new HashMap<>();
   private final Map<RigidBody, ConstrainedRigidBodyTrajectory> rigidBodyDataMap = new HashMap<>();
   
   public WholeBodyTrajectoryToolboxData(FullHumanoidRobotModel fullRobotModel, List<WaypointBasedTrajectoryCommand> endEffectorTrajectories,
                                         List<RigidBodyExplorationConfigurationCommand> explorationConfigurations)
   {
      this.fullRobotModel = fullRobotModel;
      // trajectory time.
      this.trajectoryTime = 0.0;
      for (int i = 0; i < endEffectorTrajectories.size(); i++)
         this.trajectoryTime = Math.max(trajectoryTime, endEffectorTrajectories.get(i).getLastWaypointTime());

      Map<RigidBody, WaypointBasedTrajectoryCommand> trajectoryMap = new HashMap<>();
      for (int i = 0; i < endEffectorTrajectories.size(); i++)
      {
         WaypointBasedTrajectoryCommand traj = endEffectorTrajectories.get(i);
         trajectoryMap.put(traj.getEndEffector(), traj);
      }

      Map<RigidBody, RigidBodyExplorationConfigurationCommand> explorationMap = new HashMap<>();

      for (int i = 0; i < explorationConfigurations.size(); i++)
      {
         RigidBodyExplorationConfigurationCommand exp = explorationConfigurations.get(i);
         explorationMap.put(exp.getRigidBody(), exp);
      }

      Set<RigidBody> rigidBodySet = new HashSet<>(explorationMap.keySet());
      rigidBodySet.addAll(trajectoryMap.keySet());
      allRigidBodies.addAll(rigidBodySet);

      allRigidBodies.forEach(body -> nameToRigidBodyMap.put(body.getName(), body));

      for (RigidBody rigidBody : allRigidBodies)
      {
         WaypointBasedTrajectoryCommand trajectory = trajectoryMap.get(rigidBody);
         RigidBodyExplorationConfigurationCommand exploration = explorationMap.get(rigidBody);
         if (VERBOSE)
         {
            String message = "Received for rigid body: " + rigidBody.getName();
            if (trajectory != null)
               message += " a trajectory request";
            if (exploration != null)
               message += " an exploration request";
            PrintTools.info(message);
         }

         ConstrainedRigidBodyTrajectory constrainedRigidBodyTrajectory = null;
         for (RigidBody candidateRigidBody : ScrewTools.computeSupportAndSubtreeSuccessors(ScrewTools.getRootBody(fullRobotModel.getElevator())))
         {
            if (candidateRigidBody.getName() == rigidBody.getName())
            {
               constrainedRigidBodyTrajectory = new ConstrainedRigidBodyTrajectory(candidateRigidBody, trajectory, exploration);
               break;
            }
         }

         if (constrainedRigidBodyTrajectory == null)
            if (VERBOSE)
               PrintTools.info("fullrobot model has no " + rigidBody);

         rigidBodyDataMap.put(rigidBody, constrainedRigidBodyTrajectory);
         dimensionOfExploration = dimensionOfExploration + exploration.getNumberOfDegreesOfFreedomToExplore();
      }

      if (VERBOSE)
         PrintTools.info("Total exploration dimension is " + dimensionOfExploration);
   }
   
   public SpatialData createRandomInitialSpatialData()
   {
      SpatialData spatialData = new SpatialData();
      
      List<RigidBody> rigidBodiesHasTrajectoryCommand = new ArrayList<>();
      
      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allRigidBodies.get(i);

         if(rigidBodyDataMap.get(rigidBody).hasTrajectoryCommand())
            rigidBodiesHasTrajectoryCommand.add(rigidBody);
      }
      
      for(int i=0;i<rigidBodiesHasTrajectoryCommand.size();i++)
      {         
         RigidBody rigidBody = rigidBodiesHasTrajectoryCommand.get(i);
         
         rigidBodyDataMap.get(rigidBody).appendRandomSpatial(spatialData);
      }

      return spatialData;      
   }

   public SpatialData createRandomSpatialData()
   {
      SpatialData spatialData = new SpatialData();

      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allRigidBodies.get(i);

         rigidBodyDataMap.get(rigidBody).appendRandomSpatial(spatialData);
      }

      return spatialData;
   }

   public List<KinematicsToolboxRigidBodyMessage> createMessages(SpatialNode node)
   {
      List<KinematicsToolboxRigidBodyMessage> messages = new ArrayList<>();
      double timeInTrajectory = node.getTime();
      for (int i = 0; i < node.getSize(); i++)
      {
         RigidBody rigidBody = nameToRigidBodyMap.get(node.getName(i));
         
         Pose3D poseToAppend = node.getSpatial(i);         
         
         messages.add(rigidBodyDataMap.get(rigidBody).createMessage(timeInTrajectory, poseToAppend));
      }
      
      return messages;
   }
   
   public void updateInitialConfiguration(FullHumanoidRobotModel fullRobotModel)
   {
      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allRigidBodies.get(i);
         
         for (RigidBody candidateRigidBody : ScrewTools.computeSupportAndSubtreeSuccessors(ScrewTools.getRootBody(fullRobotModel.getElevator())))
         {
            if (candidateRigidBody.getName() == rigidBody.getName())
            {
               rigidBodyDataMap.get(rigidBody).setInitialPose(candidateRigidBody);
            }
         }
      }
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public int getExplorationDimension()
   {
      return dimensionOfExploration;
   }
}
