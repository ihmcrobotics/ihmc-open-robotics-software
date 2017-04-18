package us.ihmc.communication.kinematicsToolboxAPI;

import java.util.Map;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxInputMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class KinematicsToolboxInputCommand implements Command<KinematicsToolboxInputCommand, KinematicsToolboxInputMessage>
{
   /**
    * When set to {@code true}, the solver will hold the current x and y coordinates of the center
    * of mass. By 'current', it means that the solver will use the robot configuration data
    * broadcasted by the controller to obtain the center of mass position.
    */
   public boolean holdCurrentCenterOfMassXYPosition = true;
   /**
    * When set to {@code true}, the solver will hold the pose of the active support foot/feet.
    */
   private boolean holdSupporFootPositions = true;

   private boolean hasCenterOfMassTaskBeenSet = false;
   private final KinematicsToolboxCenterOfMassCommand centerOfMassTask = new KinematicsToolboxCenterOfMassCommand();
   private final RecyclingArrayList<KinematicsToolboxRigidBodyCommand> endEffectorTasks = new RecyclingArrayList<>(KinematicsToolboxRigidBodyCommand.class);

   @Override
   public void clear()
   {
   }

   @Override
   public void set(KinematicsToolboxInputCommand other)
   {
   }

   @Override
   public void set(KinematicsToolboxInputMessage message)
   {
      set(message, null);
   }

   public void set(KinematicsToolboxInputMessage message, Map<Long, RigidBody> rigidBodyNamedBasedHashMap)
   {
      holdCurrentCenterOfMassXYPosition = message.holdCurrentCenterOfMassXYPosition();
      holdSupporFootPositions = message.holdSupporFootPositions();

      hasCenterOfMassTaskBeenSet = message.getCenterOfMassTask() != null;
      centerOfMassTask.set(message.getCenterOfMassTask());

      endEffectorTasks.clear();

      for (int i = 0; i < message.getNumberOfEndEffectorTasks(); i++)
         endEffectorTasks.add().set(message.getEndEffectorTask(i), rigidBodyNamedBasedHashMap);
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean holdSupporFootPositions()
   {
      return holdSupporFootPositions;
   }

   public boolean hasCenterOfMassTaskBeenSet()
   {
      return hasCenterOfMassTaskBeenSet;
   }

   public KinematicsToolboxCenterOfMassCommand getCenterOfMassTask()
   {
      return centerOfMassTask;
   }

   public int getNumberOfEndEffectorTasks()
   {
      return endEffectorTasks.size();
   }

   public KinematicsToolboxRigidBodyCommand getEndEffectorTask(int index)
   {
      return endEffectorTasks.get(index);
   }

   @Override
   public Class<KinematicsToolboxInputMessage> getMessageClass()
   {
      return KinematicsToolboxInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }
}
