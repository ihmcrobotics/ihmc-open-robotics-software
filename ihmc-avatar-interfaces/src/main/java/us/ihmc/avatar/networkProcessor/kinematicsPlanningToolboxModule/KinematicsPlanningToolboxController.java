package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class KinematicsPlanningToolboxController extends ToolboxController
{
   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);
   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusReference = new AtomicReference<>(null);

   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final CommandInputManager commandInputManager;

   private final KinematicsPlanningToolboxOutputStatus solution;

   private final YoBoolean isDone;

   private final List<KinematicsPlanningToolboxRigidBodyCommand> rigidBodyCommands;
   private final AtomicReference<KinematicsPlanningToolboxCenterOfMassCommand> centerOfMassCommand;
   private final AtomicReference<KinematicsToolboxConfigurationCommand> configurationCommand;

   private final List<RigidBody> ikRigidBodies;
   private final Map<RigidBody, List<KinematicsToolboxRigidBodyMessage>> ikRigidBodyMessageMap;
   private final List<KinematicsToolboxCenterOfMassMessage> ikCenterOfMassMessages;
   private final AtomicReference<KinematicsToolboxConfigurationMessage> ikConfigurationMessage;

   private final static double keyFrameTimeEpsilon = 0.01;
   private final TDoubleArrayList keyFrameTimes;

   public KinematicsPlanningToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullRobotModel, CommandInputManager commandInputManager,
                                              StatusMessageOutputManager statusOutputManager, YoGraphicsListRegistry yoGraphicsListRegistry,
                                              YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.desiredFullRobotModel = fullRobotModel;

      solution = HumanoidMessageTools.createKinematicsPlanningToolboxOutputStatus();
      solution.setDestination(-1);

      this.commandInputManager = commandInputManager;

      rigidBodyCommands = new ArrayList<KinematicsPlanningToolboxRigidBodyCommand>();
      centerOfMassCommand = new AtomicReference<>(null);
      configurationCommand = new AtomicReference<>(null);

      ikRigidBodies = new ArrayList<RigidBody>();
      ikRigidBodyMessageMap = new HashMap<>();
      ikCenterOfMassMessages = new ArrayList<KinematicsToolboxCenterOfMassMessage>();
      ikConfigurationMessage = new AtomicReference<>(null);

      keyFrameTimes = new TDoubleArrayList();

      isDone = new YoBoolean("isDone", parentRegistry);

   }

   @Override
   public boolean initialize()
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

      System.out.println("Initializing is done");
      return true;
   }

   @Override
   public void updateInternal() throws Exception
   {

   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private boolean updateToolboxConfiguration()
   {
      if (rigidBodyCommands.size() == 0)
         new Exception(this.getClass().getSimpleName() + " needs at least one KinematicsPlanningToolboxRigidBodyMessage.");

      // clear
      ikRigidBodies.clear();
      ikRigidBodyMessageMap.clear();
      ikCenterOfMassMessages.clear();
      keyFrameTimes.clear();

      for (int i = 0; i < rigidBodyCommands.size(); i++)
      {
         KinematicsPlanningToolboxRigidBodyCommand command = rigidBodyCommands.get(i);

         RigidBody endEffector = command.getEndEffector();
         ikRigidBodies.add(endEffector);

         List<KinematicsToolboxRigidBodyMessage> rigidBodyMessages = new ArrayList<KinematicsToolboxRigidBodyMessage>();
         for (int j = 0; j < command.getNumberOfWayPoints(); j++)
         {
            Pose3D wayPoint = command.getWayPoint(j);
            KinematicsToolboxRigidBodyMessage rigidBodyMessage = MessageTools.createKinematicsToolboxRigidBodyMessage(endEffector, wayPoint.getPosition(),
                                                                                                                      wayPoint.getOrientation());
            rigidBodyMessage.getControlFramePositionInEndEffector().set(command.getControlFramePose().getPosition());
            rigidBodyMessage.getControlFrameOrientationInEndEffector().set(command.getControlFramePose().getOrientation());
            rigidBodyMessage.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(command.getSelectionMatrix().getLinearPart()));
            rigidBodyMessage.getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(command.getSelectionMatrix().getAngularPart()));
            rigidBodyMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(command.getWeightMatrix().getLinearPart()));
            rigidBodyMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(command.getWeightMatrix().getAngularPart()));
            // TODO : allowable displacement.

            rigidBodyMessages.add(rigidBodyMessage);
         }
         ikRigidBodyMessageMap.put(endEffector, rigidBodyMessages);

         if (i == 0) // save key frame times for the first rigid body message.
            for (int j = 0; j < command.getNumberOfWayPoints(); j++)
               keyFrameTimes.add(command.getWayPointTime(j));
         else if (!checkKeyFrameTimes(command))
            new Exception(command.getClass().getSimpleName() + " must have same key frame times with the first KinematicsPlanningToolboxRigidBodyMessage.");
      }

      if (centerOfMassCommand.get() != null)
      {
         if (!checkKeyFrameTimes(centerOfMassCommand.get()))
            new Exception(centerOfMassCommand.get().getClass().getSimpleName()
                  + " must have same key frame times with KinematicsPlanningToolboxRigidBodyMessage.");

         for (int i = 0; i < centerOfMassCommand.get().getNumberOfWayPoints(); i++)
         {
            KinematicsToolboxCenterOfMassMessage comMessage = MessageTools.createKinematicsToolboxCenterOfMassMessage(centerOfMassCommand.get().getWayPoint(i));
            comMessage.getWeights().set(MessageTools.createWeightMatrix3DMessage(centerOfMassCommand.get().getWeightMatrix()));
            comMessage.getSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(centerOfMassCommand.get().getSelectionMatrix()));

            ikCenterOfMassMessages.add(comMessage);
         }
      }
      else
      {
         for (int i = 0; i < getNumberOfKeyFrames(); i++)
         {
            KinematicsToolboxCenterOfMassMessage comMessage = KinematicsToolboxMessageFactory.holdCenterOfMassCurrentPosition(getDesiredFullRobotModel().getRootBody(),
                                                                                                                              true, true, true);

            ikCenterOfMassMessages.add(comMessage);
         }
      }

      if (configurationCommand.get() != null)
      {

      }
      else
      {
         ikConfigurationMessage.set(KinematicsToolboxMessageFactory.privilegedConfigurationFromFullRobotModel(getDesiredFullRobotModel(), false));
      }

      return true;
   }

   private boolean checkKeyFrameTimes(KinematicsPlanningToolboxRigidBodyCommand command)
   {
      if (command.getNumberOfWayPoints() != keyFrameTimes.size())
         return false;
      for (int i = 0; i < keyFrameTimes.size(); i++)
         if (!EuclidCoreTools.epsilonEquals(command.getWayPointTime(i), keyFrameTimes.size(), keyFrameTimeEpsilon))
            return false;

      return true;
   }

   private boolean checkKeyFrameTimes(KinematicsPlanningToolboxCenterOfMassCommand command)
   {
      if (command.getNumberOfWayPoints() != keyFrameTimes.size())
         return false;
      for (int i = 0; i < keyFrameTimes.size(); i++)
         if (!EuclidCoreTools.epsilonEquals(command.getWayPointTime(i), keyFrameTimes.size(), keyFrameTimeEpsilon))
            return false;

      return true;
   }

   private int getNumberOfKeyFrames()
   {
      return keyFrameTimes.size();
   }

   public KinematicsPlanningToolboxOutputStatus getSolution()
   {
      return solution;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   // TODO : check this method and variable.
   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
   }

   // TODO : check this method and variable.
   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      latestCapturabilityBasedStatusReference.set(newStatus);
   }
}
