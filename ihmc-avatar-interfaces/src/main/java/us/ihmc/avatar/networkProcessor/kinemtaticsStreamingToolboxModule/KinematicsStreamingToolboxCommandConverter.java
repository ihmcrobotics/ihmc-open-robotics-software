package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxInitialConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxPrivilegedConfigurationCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public class KinematicsStreamingToolboxCommandConverter implements CommandConversionInterface
{
   private final RigidBodyHashCodeResolver desiredRigidBodyHashCodeResolver;
   private final ReferenceFrameHashCodeResolver desiredReferenceFrameHashCodeResolver;
   private final JointHashCodeResolver jointHashCodeResolver;

   public KinematicsStreamingToolboxCommandConverter(FullHumanoidRobotModel desiredFullRobotModel, ReferenceFrames desiredReferenceFrames)
   {
      desiredRigidBodyHashCodeResolver = new RigidBodyHashCodeResolver(desiredFullRobotModel);
      desiredReferenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(desiredFullRobotModel, desiredReferenceFrames);
      jointHashCodeResolver = new JointHashCodeResolver(desiredFullRobotModel);
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      if (message instanceof KinematicsStreamingToolboxInputMessage)
         return true;
      if (message instanceof KinematicsToolboxConfigurationMessage)
         return true;
      if (message instanceof KinematicsToolboxPrivilegedConfigurationMessage)
         return true;
      if (message instanceof KinematicsToolboxInitialConfigurationMessage)
         return true;
      return false;
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> void process(C command, M message)
   {
      if (message instanceof KinematicsStreamingToolboxInputMessage inputMessage)
      {
         KinematicsStreamingToolboxInputCommand inputCommand = (KinematicsStreamingToolboxInputCommand) command;
         inputCommand.set(inputMessage, desiredRigidBodyHashCodeResolver, desiredReferenceFrameHashCodeResolver);
      }
      else if (message instanceof KinematicsToolboxConfigurationMessage configurationMessage)
      {
         KinematicsToolboxConfigurationCommand configurationCommand = (KinematicsToolboxConfigurationCommand) command;
         configurationCommand.set(configurationMessage, jointHashCodeResolver);
      }
      else if (message instanceof KinematicsToolboxPrivilegedConfigurationMessage privilegedConfMessage)
      {
         KinematicsToolboxPrivilegedConfigurationCommand privilegedConfCommand = (KinematicsToolboxPrivilegedConfigurationCommand) command;
         privilegedConfCommand.set(privilegedConfMessage, jointHashCodeResolver);
      }
      else if (message instanceof KinematicsToolboxInitialConfigurationMessage initialConfMessage)
      {
         KinematicsToolboxInitialConfigurationCommand initialConfCommand = (KinematicsToolboxInitialConfigurationCommand) command;
         initialConfCommand.set(initialConfMessage, jointHashCodeResolver);
      }
   }
}
