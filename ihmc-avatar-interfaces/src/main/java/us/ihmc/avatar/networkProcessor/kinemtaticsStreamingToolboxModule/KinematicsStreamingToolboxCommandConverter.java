package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsStreamingToolboxCommandConverter implements CommandConversionInterface
{
   private final RigidBodyHashCodeResolver rigidBodyHashCodeResolver;
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   public KinematicsStreamingToolboxCommandConverter(FullHumanoidRobotModel fullRobotModel)
   {
      rigidBodyHashCodeResolver = new RigidBodyHashCodeResolver(fullRobotModel);
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, new HumanoidReferenceFrames(fullRobotModel));
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      return message instanceof KinematicsStreamingToolboxInputMessage;
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> void process(C command, M message)
   {
      KinematicsStreamingToolboxInputCommand inputCommand = (KinematicsStreamingToolboxInputCommand) command;
      KinematicsStreamingToolboxInputMessage inputMessage = (KinematicsStreamingToolboxInputMessage) message;
      inputCommand.set(inputMessage, rigidBodyHashCodeResolver, referenceFrameHashCodeResolver);
   }
}
