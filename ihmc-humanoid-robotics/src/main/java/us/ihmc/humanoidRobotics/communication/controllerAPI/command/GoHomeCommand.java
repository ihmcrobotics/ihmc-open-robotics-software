package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.EnumMap;

import org.apache.commons.lang3.mutable.MutableBoolean;

import controller_msgs.msg.dds.GoHomeMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Upon receiving a {@link GoHomeCommand} the controller will bring the given part of the body back
 * to a default configuration called 'home'. It is useful to get back to a safe configuration before
 * walking.
 *
 *
 * @author Sylvain
 *
 */
public class GoHomeCommand implements Command<GoHomeCommand, GoHomeMessage>
{
   private long sequenceId;
   private final SideDependentList<EnumMap<HumanoidBodyPart, MutableBoolean>> sideDependentBodyPartRequestMap = SideDependentList.createListOfEnumMaps(HumanoidBodyPart.class);
   private final EnumMap<HumanoidBodyPart, MutableBoolean> otherBodyPartRequestMap = new EnumMap<>(HumanoidBodyPart.class);
   private double trajectoryTime = 1.0;
   
   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;
   /** the execution time. This number is set if the execution delay is non zero**/
   public double adjustedExecutionTime;

   /**
    *
    */
   public GoHomeCommand()
   {
      for (HumanoidBodyPart bodyPart : HumanoidBodyPart.values)
      {
         if (bodyPart.isRobotSideNeeded())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               sideDependentBodyPartRequestMap.get(robotSide).put(bodyPart, new MutableBoolean(false));
            }
         }
         else
         {
            otherBodyPartRequestMap.put(bodyPart, new MutableBoolean(false));
         }
      }
   }

   /** {@inheritDoc} */
   @Override
   public void clear()
   {
      sequenceId = 0;

      for (HumanoidBodyPart bodyPart : HumanoidBodyPart.values)
      {
         if (bodyPart.isRobotSideNeeded())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               sideDependentBodyPartRequestMap.get(robotSide).get(bodyPart).setValue(false);
            }
         }
         else
         {
            otherBodyPartRequestMap.get(bodyPart).setValue(false);
         }
      }
   }

   /** {@inheritDoc} */
   @Override
   public void setFromMessage(GoHomeMessage message)
   {
      clear();
      sequenceId = message.getSequenceId();
      executionDelayTime = message.getExecutionDelayTime();
      trajectoryTime = message.getTrajectoryTime();

      HumanoidBodyPart bodyPart = HumanoidBodyPart.fromByte(message.getHumanoidBodyPart());
      if (bodyPart.isRobotSideNeeded())
      {
         RobotSide robotSide = RobotSide.fromByte(message.getRobotSide());
         sideDependentBodyPartRequestMap.get(robotSide).get(bodyPart).setValue(true);
      }
      else
      {
         otherBodyPartRequestMap.get(bodyPart).setValue(true);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void set(GoHomeCommand other)
   {
      clear();
      sequenceId = other.sequenceId;
      trajectoryTime = other.trajectoryTime;
      executionDelayTime = other.getExecutionDelayTime();

      for (HumanoidBodyPart bodyPart : HumanoidBodyPart.values)
      {
         if (bodyPart.isRobotSideNeeded())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               if (other.getRequest(robotSide, bodyPart))
                  sideDependentBodyPartRequestMap.get(robotSide).get(bodyPart).setValue(true);
            }
         }
         else if (other.getRequest(bodyPart))
         {
            otherBodyPartRequestMap.get(bodyPart).setValue(true);
         }
      }
   }


   /**
    * @return the duration for going back to the home configuration.
    */
   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   /**
    * Get the request for going home for a given body part. This method is to use only for body
    * parts that are not side dependent, like the pelvis.
    * 
    * @param bodyPart body part to check the request for.
    * @return true if the go home is requested, false otherwise.
    * @throws RuntimeException if the robot side is need for the given body part.
    */
   public boolean getRequest(HumanoidBodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);
      return otherBodyPartRequestMap.get(bodyPart).booleanValue();
   }

   /**
    * Get the request for going home for a given body part. This method is to use for body parts
    * that are side dependent, like a foot.
    * 
    * @param robotSide which side the body part belongs to.
    * @param bodyPart body part to check the request for.
    * @return true if the go home is requested, false otherwise.
    */
   public boolean getRequest(RobotSide robotSide, HumanoidBodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         return sideDependentBodyPartRequestMap.get(robotSide).get(bodyPart).booleanValue();
      else
         return getRequest(bodyPart);
   }

   /** {@inheritDoc} */
   @Override
   public Class<GoHomeMessage> getMessageClass()
   {
      return GoHomeMessage.class;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isCommandValid()
   {
      return true;
   }
   
   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   @Override
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }
   
   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }
   
   /**
    * returns the expected execution time of this command. The execution time will be computed when the controller 
    * receives the command using the controllers time plus the execution delay time.
    * This is used when {@code getExecutionDelayTime} is non-zero
    */
   @Override
   public double getExecutionTime()
   {
      return adjustedExecutionTime;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is received.
    */
   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      this.adjustedExecutionTime = adjustedExecutionTime;
   }
   
   /**
    * tells the controller if this command supports delayed execution
    * (Spoiler alert: It does)
    * @return
    */
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
