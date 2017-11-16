package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxSettings;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * This class is for packing input of the controller as like as a packet {@link WholeBodyTrajectoryToolboxMessage}.
 * <p>
 * - trajectory time, initial configuration.
 * <p>
 * - list of WaypointBasedTrajectoryMessage
 * <p>
 * - list of RigidBodyExplorationConfigurationMessage
 * <p>
 * 
 * @link {WholeBodyTrajectoryToolboxMessage is converted to (this).
 * <p>
 * This will be used for {@link HumanoidKinematicsSolver}.
 * @author Inho, Sylvain.
 *
 */
public class WholeBodyTrajectoryToolboxData
{
   private final FullHumanoidRobotModel fullRobotModel;

   private double trajectoryTime;

   private final KinematicsToolboxOutputStatus initialConfiguration;   
   private final int numberOfInitialGuesses;
   private final int maximumExpansionSize;

   /**
    * Left hand
    * Right hand
    * Chest
    * Pelvis 
    */
   private final List<ConstrainedRigidBodyTrajectory> listOfRigidBodyData = new ArrayList<ConstrainedRigidBodyTrajectory>();
   
   // for title of the node visualizer.
   private final List<String> explorationConfigurationNames = new ArrayList<>();
   // for pick random configuration.
   private final TDoubleArrayList explorationRangeUpperLimits = new TDoubleArrayList();
   private final TDoubleArrayList explorationRangeLowerLimits = new TDoubleArrayList();
   
   // Has map. Rigid body and node data. 0 is time space.
   private Map<Integer, RigidBody> nodeIndexBasedHashMap = new HashMap<>();;  
   
   private CTTaskNode rootNode;
   private CTTaskNodeTree tree;
   
   public WholeBodyTrajectoryToolboxData(FullHumanoidRobotModel fullRobotModel, WholeBodyTrajectoryToolboxConfigurationCommand configuration,
                                         List<WaypointBasedTrajectoryCommand> endEffectorTrajectories,
                                         List<RigidBodyExplorationConfigurationCommand> explorationConfigurations)
   {
      // robot model.      
      this.fullRobotModel = fullRobotModel;
      
      // trajectory time.
      this.trajectoryTime = 0.0;      
      for (int i = 0; i < endEffectorTrajectories.size(); i++)
         this.trajectoryTime = Math.max(trajectoryTime, endEffectorTrajectories.get(i).getLastWaypointTime());

      // initial configuration.
      this.initialConfiguration = configuration.getInitialConfiguration();
      this.numberOfInitialGuesses = configuration.getNumberOfInitialGuesses();
      this.maximumExpansionSize = configuration.getMaximumExpansionSize();

      // create RigidBodyData for all rigid bodies.
      listOfRigidBodyData.clear();
      for(int i=0;i<explorationConfigurations.size();i++)
      {
         RigidBodyExplorationConfigurationCommand rigidBodyExplorationConfigurationCommand = explorationConfigurations.get(i);
         boolean hasWayPointBasedTrajectoryCommand = false;
         for(int j=0;j<endEffectorTrajectories.size();j++)
         {  
            WaypointBasedTrajectoryCommand waypointBasedTrajectoryCommand = endEffectorTrajectories.get(j);
            if(waypointBasedTrajectoryCommand.getEndEffector() == rigidBodyExplorationConfigurationCommand.getRigidBody())
            {
               hasWayPointBasedTrajectoryCommand = true;
               PrintTools.info(""+rigidBodyExplorationConfigurationCommand.getRigidBody().getName() +" has trajectory and exploration");
               listOfRigidBodyData.add(new ConstrainedRigidBodyTrajectory(waypointBasedTrajectoryCommand, rigidBodyExplorationConfigurationCommand));
               continue;
            }
         }
         
         if(!hasWayPointBasedTrajectoryCommand)
         {
            PrintTools.info(""+rigidBodyExplorationConfigurationCommand.getRigidBody().getName() +" has NO trajectory");
            listOfRigidBodyData.add(new ConstrainedRigidBodyTrajectory(this.fullRobotModel, rigidBodyExplorationConfigurationCommand));
         }         
      }
      
      // non defined rigid body
      ArrayList<RigidBody> tempRigidBodyList = new ArrayList<RigidBody>();      
      for(int i=0;i<listOfRigidBodyData.size();i++)
         tempRigidBodyList.add(listOfRigidBodyData.get(i).getRigidBody());
      
      List<RigidBody> listOfRigidBody = WholeBodyTrajectoryToolboxSettings.getListOfRigidBody(fullRobotModel);
      for(int i=0;i<listOfRigidBody.size();i++)
      {
         if(!tempRigidBodyList.contains(listOfRigidBody.get(i)))
         {
            PrintTools.info("Has no " + listOfRigidBody.get(i) +" command");
            ConfigurationSpaceName[] configurationSpaces = WholeBodyTrajectoryToolboxSettings.getDefaultExplorationConfiguratSpaces(fullRobotModel, listOfRigidBody.get(i));            
            RigidBodyExplorationConfigurationCommand dummyCommand = new RigidBodyExplorationConfigurationCommand(listOfRigidBody.get(i), configurationSpaces);
            listOfRigidBodyData.add(new ConstrainedRigidBodyTrajectory(this.fullRobotModel, dummyCommand));
         }
      }
      
      // set exploration node.
      nodeIndexBasedHashMap.clear();      
      for(int i=0;i<listOfRigidBodyData.size();i++)
      {
         ConstrainedRigidBodyTrajectory constrainedRigidBodyTrajectory = listOfRigidBodyData.get(i);
         explorationConfigurationNames.addAll(constrainedRigidBodyTrajectory.getExplorationConfigurationNames());         
         explorationRangeUpperLimits.addAll(constrainedRigidBodyTrajectory.getExplorationRangeUpperLimits());
         explorationRangeLowerLimits.addAll(constrainedRigidBodyTrajectory.getExplorationRangeLowerLimits());
         
         int currentSize = nodeIndexBasedHashMap.size()+1;
         for(int j=0;j<constrainedRigidBodyTrajectory.getExplorationDimension();j++)
            nodeIndexBasedHashMap.put(currentSize+j, constrainedRigidBodyTrajectory.getRigidBody());
      }
      
      // only for pelvis height.
      if(explorationConfigurationNames.contains("pelvis_Z"))
      {
         explorationRangeUpperLimits.replace(explorationConfigurationNames.indexOf("pelvis_Z"), 0.90);
         explorationRangeLowerLimits.replace(explorationConfigurationNames.indexOf("pelvis_Z"), 0.75);
      }
      
      // check exploration configurations.
      for(int j=0;j<explorationConfigurationNames.size();j++)
      {              
         PrintTools.info(""+explorationConfigurationNames.get(j) +" "+ explorationRangeUpperLimits.get(j) +" "+ explorationRangeLowerLimits.get(j));         
      }
      
   }
      
   public int getNumerOfInitialGuesses()
   {
      return numberOfInitialGuesses;
   }
   
   public int getMaximumExpansionSize()
   {
      return maximumExpansionSize;
   }
   
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   public KinematicsToolboxOutputStatus getInitialConfiguration()
   {
      return initialConfiguration;
   }

   public ConstrainedRigidBodyTrajectory getConstrainedRigidBodyTrajectory(RigidBody rigidBody)
   {
      for (int i = 0; i < listOfRigidBodyData.size(); i++)
      {
         if (rigidBody == listOfRigidBodyData.get(i).getRigidBody())
         {
            return listOfRigidBodyData.get(i);
         }
      }

      // warning message.
      return null;
   }
}
