package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
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
            PrintTools.info(""+rigidBodyExplorationConfigurationCommand.getRigidBody().getName() +" has NO trajectory and exploration");
            listOfRigidBodyData.add(new ConstrainedRigidBodyTrajectory(this.fullRobotModel, rigidBodyExplorationConfigurationCommand));
         }         
      }
      PrintTools.info("set done");      
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
