package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class KinematicsPlanningToolboxController extends ToolboxController
{
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommandInputManager commandInputManager;

   private final KinematicsPlanningToolboxOutputStatus solution;
   
   private final YoBoolean isDone;

   public KinematicsPlanningToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel, CommandInputManager commandInputManager,
                                              StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.desiredFullRobotModel = fullRobotModel;

      solution = MessageTools.createKinematicsPlanningToolboxOutputStatus();
      solution.setDestination(-1);

      this.commandInputManager = commandInputManager;

      isDone = new YoBoolean("isDone", parentRegistry);
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      
      if (commandInputManager.isNewCommandAvailable(KinematicsPlanningToolboxRigidBodyCommand.class))
      {
         KinematicsPlanningToolboxRigidBodyCommand endEffectorCommand = commandInputManager.pollNewestCommand(KinematicsPlanningToolboxRigidBodyCommand.class);
         PrintTools.info("endEffectorCommand");
      }
      else
      {
         return false;
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsPlanningToolboxCenterOfMassCommand.class))
      {
         KinematicsPlanningToolboxCenterOfMassCommand comCommand = commandInputManager.pollNewestCommand(KinematicsPlanningToolboxCenterOfMassCommand.class);
         PrintTools.info("comCommand");
      }
      else
      {
         return false;
      }
      
      PrintTools.info("Initializing is done");
      return true;
   }

   @Override
   protected void updateInternal() throws Exception
   {

   }

   @Override
   protected boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   public KinematicsPlanningToolboxOutputStatus getSolution()
   {
      return solution;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }
}
