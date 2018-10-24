package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class KinematicsPlanningToolboxController extends ToolboxController
{
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommandInputManager commandInputManager;

   private final KinematicsPlanningToolboxOutputStatus solution;

   private final YoBoolean isDone;

   private final List<KinematicsPlanningToolboxRigidBodyCommand> rigidBodyCommands;
   private final AtomicReference<KinematicsPlanningToolboxCenterOfMassCommand> centerOfMassCommand;
   private final AtomicReference<KinematicsToolboxConfigurationCommand> configurationCommand;

   public KinematicsPlanningToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel, CommandInputManager commandInputManager,
                                              StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.desiredFullRobotModel = fullRobotModel;

      solution = HumanoidMessageTools.createKinematicsPlanningToolboxOutputStatus();
      solution.setDestination(-1);

      this.commandInputManager = commandInputManager;

      rigidBodyCommands = new ArrayList<KinematicsPlanningToolboxRigidBodyCommand>();
      centerOfMassCommand = new AtomicReference<>(null);
      configurationCommand = new AtomicReference<>(null);

      isDone = new YoBoolean("isDone", parentRegistry);
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);

      rigidBodyCommands.clear();
      centerOfMassCommand.set(null);
      configurationCommand.set(null);

      if (commandInputManager.isNewCommandAvailable(KinematicsPlanningToolboxRigidBodyCommand.class))
      {
         List<KinematicsPlanningToolboxRigidBodyCommand> commands = commandInputManager.pollNewCommands(KinematicsPlanningToolboxRigidBodyCommand.class);
         for (int i = 0; i < commands.size(); i++)
            rigidBodyCommands.add(commands.get(i));
         for (int i = 0; i < commands.size(); i++)
            PrintTools.info(""+commands.get(i).getEndEffector());
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsPlanningToolboxCenterOfMassCommand.class))
      {
         KinematicsPlanningToolboxCenterOfMassCommand comCommand = commandInputManager.pollNewestCommand(KinematicsPlanningToolboxCenterOfMassCommand.class);
         centerOfMassCommand.set(comCommand);
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsToolboxConfigurationCommand.class))
      {
         KinematicsToolboxConfigurationCommand command = commandInputManager.pollNewestCommand(KinematicsToolboxConfigurationCommand.class);
         configurationCommand.set(command);
      }

      if (!updateToolboxConfiguration())
         return false;

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

   private boolean updateToolboxConfiguration()
   {
      // TODO : check number of key frames between each rigidbody commands.
      PrintTools.info(""+rigidBodyCommands.size());
      for(int i=0;i<rigidBodyCommands.size();i++)
      {
         PrintTools.info(""+rigidBodyCommands.get(i).getEndEffector());
         int numberOfWayPoints = rigidBodyCommands.get(i).getNumberOfWayPoints();
         for(int j=0;j<numberOfWayPoints;j++)
         {
            PrintTools.info(""+rigidBodyCommands.get(i).getWayPointTime(j));
            PrintTools.info(""+rigidBodyCommands.get(i).getWayPoint(j));
         }
      }
      
      if(centerOfMassCommand.get() != null)
      {
         PrintTools.info(""+centerOfMassCommand.get().getNumberOfWayPoints());   
      }
      else
      {
         // create default com trajectory
      }
      
      if(configurationCommand.get() != null)
      {
         PrintTools.info(""+configurationCommand.get().getPrivilegedJointAngles());   
      }
      else
      {
         // create default configuration
      }
      
      return true;
   }

   public KinematicsPlanningToolboxOutputStatus getSolution()
   {
      return solution;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }
   
   public static void recursivelyModifyGraphics(JointDescription joint, AppearanceDefinition ghostApperance)
   {
      if (joint == null)
         return;
      LinkDescription link = joint.getLink();
      if (link == null)
         return;
      LinkGraphicsDescription linkGraphics = link.getLinkGraphics();

      if (linkGraphics != null)
      {
         ArrayList<Graphics3DPrimitiveInstruction> graphics3dInstructions = linkGraphics.getGraphics3DInstructions();

         if (graphics3dInstructions == null)
            return;

         for (Graphics3DPrimitiveInstruction primitive : graphics3dInstructions)
         {
            if (primitive instanceof Graphics3DInstruction)
            {
               Graphics3DInstruction modelInstruction = (Graphics3DInstruction) primitive;
               modelInstruction.setAppearance(ghostApperance);
            }
         }
      }

      if (joint.getChildrenJoints() == null)
         return;

      for (JointDescription child : joint.getChildrenJoints())
      {
         recursivelyModifyGraphics(child, ghostApperance);
      }
   }
}
