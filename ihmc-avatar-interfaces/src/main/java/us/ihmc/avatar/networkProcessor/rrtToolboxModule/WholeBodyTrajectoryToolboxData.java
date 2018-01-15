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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialData;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.SpatialNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;

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
   private static final boolean VERBOSE = true;

   private final FullHumanoidRobotModel fullRobotModel;

   private double trajectoryTime;
   private int dimensionOfExploration = 0;

   private final List<RigidBody> allRigidBodies = new ArrayList<>();
   private final Map<String, RigidBody> nameToRigidBodyMap = new HashMap<>();
   private final Map<RigidBody, ConstrainedRigidBodyTrajectory> rigidBodyDataMap = new HashMap<>();

   private final List<ReachingManifoldCommand> reachingManifolds;

   public WholeBodyTrajectoryToolboxData(FullHumanoidRobotModel fullRobotModel, List<WaypointBasedTrajectoryCommand> endEffectorTrajectories,
                                         List<ReachingManifoldCommand> reachingManifolds,
                                         List<RigidBodyExplorationConfigurationCommand> explorationConfigurations)
   {
      this.fullRobotModel = fullRobotModel;
      this.reachingManifolds = reachingManifolds;

      Map<RigidBody, RigidBodyExplorationConfigurationCommand> explorationMap = new HashMap<>();
      Map<RigidBody, WaypointBasedTrajectoryCommand> trajectoryMap = new HashMap<>();

      for (int i = 0; i < explorationConfigurations.size(); i++)
      {
         RigidBodyExplorationConfigurationCommand exp = explorationConfigurations.get(i);
         explorationMap.put(exp.getRigidBody(), exp);
      }
      Set<RigidBody> rigidBodySet = new HashSet<>(explorationMap.keySet());

      // classify
      if (endEffectorTrajectories != null)
      {
         this.trajectoryTime = 0.0;
         for (int i = 0; i < endEffectorTrajectories.size(); i++)
            this.trajectoryTime = Math.max(trajectoryTime, endEffectorTrajectories.get(i).getLastWaypointTime());

         for (int i = 0; i < endEffectorTrajectories.size(); i++)
         {
            WaypointBasedTrajectoryCommand traj = endEffectorTrajectories.get(i);
            trajectoryMap.put(traj.getEndEffector(), traj);
         }

         rigidBodySet.addAll(trajectoryMap.keySet());
      }

      else if (reachingManifolds != null)
      {
         // TODO .......................... some action is required to deal with.
         this.trajectoryTime = 25.0;
      }
      else
      {
         if (VERBOSE)
            PrintTools.info("no trajectory or manifold");
      }

      allRigidBodies.addAll(rigidBodySet);

      allRigidBodies.forEach(body -> nameToRigidBodyMap.put(body.getName(), body));

      // construct ConstrainedRigidBodyTrajectory
      for (RigidBody rigidBody : allRigidBodies)
      {
         WaypointBasedTrajectoryCommand trajectory;
         if (trajectoryMap.isEmpty())
            trajectory = null;
         else
            trajectory = trajectoryMap.get(rigidBody);

         RigidBodyExplorationConfigurationCommand exploration = explorationMap.get(rigidBody);
         if(exploration == null)
            exploration = new RigidBodyExplorationConfigurationCommand(rigidBody,  new ConfigurationSpaceName[]{});
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

   /**
    * For findInitialGuessSub()
    */
   public SpatialData createRandomInitialSpatialData()
   {
      SpatialData spatialData = new SpatialData();

      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allRigidBodies.get(i);

         if (rigidBodyDataMap.get(rigidBody).hasTrajectoryCommand())
         {
            rigidBodyDataMap.get(rigidBody).appendRandomSpatial(spatialData);
         }
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

         Pose3D poseToAppend = node.getSpatialData(i);

         KinematicsToolboxRigidBodyMessage message = rigidBodyDataMap.get(rigidBody).createMessage(timeInTrajectory, poseToAppend);
         messages.add(message);
      }
            
      return messages;
   }
   
   public double getMaximumDistanceFromManifolds(SpatialNode node)
   {
      double distance = Double.MAX_VALUE;
      for (int j = 0; j < reachingManifolds.size(); j++)
      {
         for (int i = 0; i < node.getSpatialData().getRigidBodySpatials().size(); i++)
         {
            if (node.getSpatialData().getRigidBodyNames().get(i).equals(reachingManifolds.get(j).getRigidBody().getName()))
            {
               ReachingManifoldCommand manifold = reachingManifolds.get(j);

               RigidBody rigidBody = nameToRigidBodyMap.get(node.getName(i));

               Pose3D currentSpatial = rigidBodyDataMap.get(rigidBody).getPoseToWorldFrame(node.getSpatialData(i));

               Pose3D closestPose = manifold.computeClosestPoseOnManifold(currentSpatial);

               distance = currentSpatial.getPositionDistance(closestPose);
            }
         }
      }
      return distance;
   }

   public Pose3D getTestFrame(SpatialNode node)
   {
      for (int j = 0; j < reachingManifolds.size(); j++)
      {
         for (int i = 0; i < node.getSpatialData().getRigidBodySpatials().size(); i++)
         {
            if (node.getSpatialData().getRigidBodyNames().get(i).equals(reachingManifolds.get(j).getRigidBody().getName()))
            {
               ReachingManifoldCommand manifold = reachingManifolds.get(j);

               RigidBody rigidBody = nameToRigidBodyMap.get(node.getName(i));

               Pose3D currentSpatial = rigidBodyDataMap.get(rigidBody).getPoseToWorldFrame(node.getSpatialData(i));

               Pose3D closestPose = manifold.computeClosestPoseOnManifold(currentSpatial);

               return closestPose;
               // TODO get closest pose from manifold
               // and distance.
            }
         }
      }
      return null;
   }

   /**
    * For findInitialGuessSub()
    */
   public void holdConfiguration(FullHumanoidRobotModel fullRobotModel)
   {
      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allRigidBodies.get(i);

         for (RigidBody candidateRigidBody : ScrewTools.computeSupportAndSubtreeSuccessors(ScrewTools.getRootBody(fullRobotModel.getElevator())))
         {
            if (candidateRigidBody.getName() == rigidBody.getName())
            {
               rigidBodyDataMap.get(rigidBody).holdConfiguration(candidateRigidBody);
            }
         }
      }
   }

   /**
    * For findInitialGuessSub()
    */
   public void updateInitialConfiguration()
   {
      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBody rigidBody = allRigidBodies.get(i);
         rigidBodyDataMap.get(rigidBody).updateInitialResult();
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
