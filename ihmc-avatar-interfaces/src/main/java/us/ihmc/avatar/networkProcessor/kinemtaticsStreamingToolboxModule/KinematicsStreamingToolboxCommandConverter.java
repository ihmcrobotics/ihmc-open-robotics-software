package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

public class KinematicsStreamingToolboxCommandConverter implements CommandConversionInterface
{
   private final RigidBodyHashCodeResolver desiredRigidBodyHashCodeResolver;
   private final ReferenceFrameHashCodeResolver desiredReferenceFrameHashCodeResolver;

   public KinematicsStreamingToolboxCommandConverter(FullHumanoidRobotModel desiredFullRobotModel, ReferenceFrames desiredReferenceFrames)
   {
      desiredRigidBodyHashCodeResolver = new RigidBodyHashCodeResolver(desiredFullRobotModel);
      desiredReferenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(desiredFullRobotModel, desiredReferenceFrames);
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      return message instanceof KinematicsStreamingToolboxInputMessage;
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
   }
}
