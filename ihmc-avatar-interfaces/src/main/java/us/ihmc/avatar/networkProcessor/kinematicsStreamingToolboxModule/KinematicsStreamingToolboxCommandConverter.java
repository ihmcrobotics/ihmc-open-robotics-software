package us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule;

import toolbox_msgs.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.KinematicsStreamingToolboxContactConfigurationMessage;
import toolbox_msgs.KinematicsStreamingToolboxInitialConfigurationMessage;
import toolbox_msgs.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.KinematicsToolboxConfigurationMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxContactConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInitialConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

/**
 * Converts KST DDS messages into controller commands while resolving rigid-body, joint, and frame hash IDs.
 */
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
      if (message instanceof KinematicsStreamingToolboxInitialConfigurationMessage)
         return true;
      if (message instanceof KinematicsStreamingToolboxContactConfigurationMessage)
         return true;
      if (message instanceof KinematicsStreamingToolboxConfigurationMessage)
         return true;
      if (message instanceof KinematicsToolboxConfigurationMessage)
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
      else if (message instanceof KinematicsStreamingToolboxInitialConfigurationMessage initialConfMessage)
      {
         KinematicsStreamingToolboxInitialConfigurationCommand initialConfCommand = (KinematicsStreamingToolboxInitialConfigurationCommand) command;
         initialConfCommand.set(initialConfMessage, jointHashCodeResolver);
      }
      else if (message instanceof KinematicsStreamingToolboxConfigurationMessage configurationMessage)
      {
         KinematicsStreamingToolboxConfigurationCommand configurationCommand = (KinematicsStreamingToolboxConfigurationCommand) command;
         configurationCommand.setFromMessage(configurationMessage);
      }
      else if (message instanceof KinematicsStreamingToolboxContactConfigurationMessage contactConfMessage)
      {
         KinematicsStreamingToolboxContactConfigurationCommand contactConfCommand = (KinematicsStreamingToolboxContactConfigurationCommand) command;
         contactConfCommand.setFromMessage(contactConfMessage);
      }
      else if (message instanceof KinematicsToolboxConfigurationMessage configurationMessage)
      {
         KinematicsToolboxConfigurationCommand configurationCommand = (KinematicsToolboxConfigurationCommand) command;
         configurationCommand.set(configurationMessage, jointHashCodeResolver);
      }
   }
}
