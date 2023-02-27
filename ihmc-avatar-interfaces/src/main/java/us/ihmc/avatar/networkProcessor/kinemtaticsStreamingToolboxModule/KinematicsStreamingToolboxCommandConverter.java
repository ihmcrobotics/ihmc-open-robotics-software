package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public class KinematicsStreamingToolboxCommandConverter implements CommandConversionInterface
{
   private final ReferenceFrameHashCodeResolver currentReferenceFrameHashCodeResolver;

   private final RigidBodyHashCodeResolver desiredRigidBodyHashCodeResolver;
   private final ReferenceFrameHashCodeResolver desiredReferenceFrameHashCodeResolver;

   public KinematicsStreamingToolboxCommandConverter(FullHumanoidRobotModel currentFullRobotModel,
                                                     ReferenceFrames currentReferenceFrames,
                                                     FullHumanoidRobotModel desiredFullRobotModel,
                                                     ReferenceFrames desiredReferenceFrames)
   {
      currentReferenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(currentFullRobotModel, currentReferenceFrames);

      desiredRigidBodyHashCodeResolver = new RigidBodyHashCodeResolver(desiredFullRobotModel);
      desiredReferenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(desiredFullRobotModel, desiredReferenceFrames);
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      return message instanceof KinematicsStreamingToolboxInputMessage || message instanceof KinematicsStreamingToolboxConfigurationMessage;
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> void process(C command, M message)
   {
      if (message instanceof KinematicsStreamingToolboxInputMessage)
      {
         KinematicsStreamingToolboxInputCommand inputCommand = (KinematicsStreamingToolboxInputCommand) command;
         KinematicsStreamingToolboxInputMessage inputMessage = (KinematicsStreamingToolboxInputMessage) message;
         inputCommand.set(inputMessage, desiredRigidBodyHashCodeResolver, desiredReferenceFrameHashCodeResolver);
      }
      else
      {
         KinematicsStreamingToolboxConfigurationCommand inputCommand = (KinematicsStreamingToolboxConfigurationCommand) command;
         KinematicsStreamingToolboxConfigurationMessage inputMessage = (KinematicsStreamingToolboxConfigurationMessage) message;
         inputCommand.set(inputMessage, currentReferenceFrameHashCodeResolver);
      }
   }
}
