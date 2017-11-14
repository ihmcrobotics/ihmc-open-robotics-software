package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import java.util.List;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;

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
   private double trajectoryTime;
   private KinematicsToolboxOutputStatus  initialConfiguration;
   
   /**
    * Left hand
    * Right hand
    * Chest
    * Pelvis 
    */
   private List<ConstrainedRigidBodyTrajectory> listOfRigidBodyData;
   
   public WholeBodyTrajectoryToolboxData(WholeBodyTrajectoryToolboxCommand wholeBodyTrajectoryToolboxCommand)
   {
      WholeBodyTrajectoryToolboxConfigurationCommand configuration = wholeBodyTrajectoryToolboxCommand.getConfigurationCommand();
      List<WaypointBasedTrajectoryCommand> endEffectorTrajectories = wholeBodyTrajectoryToolboxCommand.getEndEffectorTrajectories();
      List<RigidBodyExplorationConfigurationCommand> explorationConfigurations = wholeBodyTrajectoryToolboxCommand.getRigidBodyExplorationConfigurations();
      
      // trajectory time.
      this.trajectoryTime = 0.0;
      for(int i=0;i<endEffectorTrajectories.size();i++)
         if(endEffectorTrajectories.get(i).getLastWaypointTime() > this.trajectoryTime)
            this.trajectoryTime = endEffectorTrajectories.get(i).getLastWaypointTime();
      
      // initial configuration
      this.initialConfiguration = configuration.getInitialConfiguration();
      
      // create RigidBodyData for all rigid bodies.
      
   }
   
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }
   
   public KinematicsToolboxOutputStatus getInitialConfiguration()
   {
      return initialConfiguration;
   }
   
   public ConstrainedRigidBodyTrajectory getRigidBodyData(long rigidBodyHashCode)
   {
      for(int i=0;i<listOfRigidBodyData.size();i++)
      {
         if(rigidBodyHashCode == listOfRigidBodyData.get(i).getRigidBodyHashCode())
         {
            return listOfRigidBodyData.get(i);
         }
      }
      
      // warning message.
      return null;
   }
}
