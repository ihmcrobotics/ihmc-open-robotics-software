package us.ihmc.communication.packets;

import static us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage.*;

import java.util.ArrayList;
import java.util.List;

public class KinematicsToolboxInputMessage extends TrackablePacket<KinematicsToolboxInputMessage>
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
   public boolean holdSupporFootPositions = true;

   public KinematicsToolboxCenterOfMassMessage centerOfMassTask;
   public List<KinematicsToolboxRigidBodyMessage> endEffectorTasks;

   public KinematicsToolboxInputMessage()
   {
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Specifies whether or not the solver should hold the current x and y coordinates of the center
    * of mass.
    * <p>
    * By 'current', it means that the solver will use the robot configuration data broadcasted by
    * the controller to obtain the center of mass position.
    * </p>
    * <p>
    * This is used to force the solver to find a solution that is statically balanced.
    * </p>
    * 
    * @param holdCurrentCenterOfMassXYPosition {@code true} to request the solver to hold the
    *           current center of mass x and y coordinates, {@code false} to let it free.
    */
   public void setHoldCurrentCenterOfMassXYPosition(boolean holdCurrentCenterOfMassXYPosition)
   {
      this.holdCurrentCenterOfMassXYPosition = holdCurrentCenterOfMassXYPosition;
   }

   /**
    * Specifies whether or not the solver should hold the pose of the active support foot/feet.
    * <p>
    * This is used to force the solver to find a solution that is statically balanced.
    * </p>
    * 
    * @param holdSupporFootPositions {@code true} to request the solver to hold the support
    *           foot/feet pose, {@code false} to let them free.
    */
   public void setHoldSupporFootPositions(boolean holdSupporFootPositions)
   {
      this.holdSupporFootPositions = holdSupporFootPositions;
   }

   /**
    * Adds a task for a given end-effector to solve for in the {@code KinematicsToolboxController}.
    * 
    * @param endEffectorTask the desired task that the end-effector should achieve.
    */
   public void addEndEffectorTask(KinematicsToolboxRigidBodyMessage endEffectorTask)
   {
      if (endEffectorTasks == null)
         endEffectorTasks = new ArrayList<>();
      endEffectorTasks.add(endEffectorTask);
   }

   /**
    * Adds a collection of task to solve for in the {@code KinematicsToolboxController}.
    * 
    * @param endEffectorTasks the collection of desired tasks that each end-effector has to achieve.
    */
   public void addEndEffectorTasks(Iterable<KinematicsToolboxRigidBodyMessage> endEffectorTasks)
   {
      endEffectorTasks.forEach(this::addEndEffectorTask);
   }

   /**
    * Adds a collection of task to solve for in the {@code KinematicsToolboxController}.
    * 
    * @param endEffectorTasks the collection of desired tasks that each end-effector has to achieve.
    */
   public void addEndEffectorTasks(KinematicsToolboxRigidBodyMessage... endEffectorTasks)
   {
      for (KinematicsToolboxRigidBodyMessage endEffectorTask : endEffectorTasks)
         addEndEffectorTask(endEffectorTask);
   }

   /**
    * Adds task for the center of mass to solve for in the {@code KinematicsToolboxController}.
    * 
    * @param centerOfMassTask the desired task for the center of mass.
    */
   public void setCenterOfMassTask(KinematicsToolboxCenterOfMassMessage centerOfMassTask)
   {
      this.centerOfMassTask = centerOfMassTask;
   }

   public boolean holdCurrentCenterOfMassXYPosition()
   {
      return holdCurrentCenterOfMassXYPosition;
   }

   public boolean holdSupporFootPositions()
   {
      return holdSupporFootPositions;
   }

   public KinematicsToolboxCenterOfMassMessage getCenterOfMassTask()
   {
      return centerOfMassTask;
   }

   public int getNumberOfEndEffectorTasks()
   {
      return endEffectorTasks.size();
   }

   public KinematicsToolboxRigidBodyMessage getEndEffectorTask(int index)
   {
      return endEffectorTasks.get(index);
   }

   /**
    * Compares each field of this message against the other message and returns {@code true} if they
    * are equal to an {@code epsilon}.
    * <p>
    * Note that this method considers two fields to be equal if they are both {@code null}, and
    * considers two fields to be different if only one is equal to {@code null}.
    * </p>
    * 
    * @return {@code true} if the two messages are equal to an {@code epsilon}, {@code false}
    *         otherwise.
    */
   @Override
   public boolean epsilonEquals(KinematicsToolboxInputMessage other, double epsilon)
   {
      if (holdCurrentCenterOfMassXYPosition != other.holdCurrentCenterOfMassXYPosition)
         return false;
      if (holdSupporFootPositions != other.holdSupporFootPositions)
         return false;
      if (!nullEqualsAndEpsilonEquals(centerOfMassTask, other.centerOfMassTask, epsilon))
         return false;

      if (endEffectorTasks != null)
      {
         if (other.endEffectorTasks == null)
            return false;
         if (endEffectorTasks.size() != other.endEffectorTasks.size())
            return false;
         for (int i = 0; i < endEffectorTasks.size(); i++)
         {
            if (!nullEqualsAndEpsilonEquals(endEffectorTasks.get(i), other.endEffectorTasks.get(i), epsilon))
               return false;
         }
      }
      else if (other.endEffectorTasks != null)
      {
         return false;
      }

      return true;
   }
}
