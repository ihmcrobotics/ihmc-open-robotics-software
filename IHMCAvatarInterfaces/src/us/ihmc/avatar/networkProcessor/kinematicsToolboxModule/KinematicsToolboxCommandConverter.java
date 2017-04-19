package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.kinematicsToolboxAPI.KinematicsToolboxInputCommand;
import us.ihmc.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.communication.packets.KinematicsToolboxInputMessage;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class KinematicsToolboxCommandConverter implements CommandConversionInterface
{
   private final Map<Long, RigidBody> rigidBodyNamedBasedHashMap = new HashMap<>();

   public KinematicsToolboxCommandConverter(RigidBody robotBody)
   {
      RigidBody rootBody = ScrewTools.getRootBody(robotBody);
      RigidBody[] allRigidBodies = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      for (RigidBody rigidBody : allRigidBodies)
         rigidBodyNamedBasedHashMap.put(rigidBody.getNameBasedHashCode(), rigidBody);
   }

   @Override
   public <C extends Command<?, M>, M extends Packet<M>> boolean isConvertible(C command, M message)
   {
      return message instanceof KinematicsToolboxInputMessage || message instanceof KinematicsToolboxRigidBodyMessage;
   }

   @Override
   public <C extends Command<?, M>, M extends Packet<M>> void process(C command, M message)
   {
      if (message instanceof KinematicsToolboxRigidBodyMessage)
      {
         KinematicsToolboxRigidBodyMessage rigiBodyMessage = (KinematicsToolboxRigidBodyMessage) message;
         KinematicsToolboxRigidBodyCommand rigiBodyCommand = (KinematicsToolboxRigidBodyCommand) command;
         rigiBodyCommand.set(rigiBodyMessage, rigidBodyNamedBasedHashMap);
      }
      else if (message instanceof KinematicsToolboxInputMessage)
      {
         KinematicsToolboxInputMessage inputMessage = (KinematicsToolboxInputMessage) message;
         KinematicsToolboxInputCommand inputCommand = (KinematicsToolboxInputCommand) command;
         inputCommand.set(inputMessage, rigidBodyNamedBasedHashMap);
      }
   }
}
