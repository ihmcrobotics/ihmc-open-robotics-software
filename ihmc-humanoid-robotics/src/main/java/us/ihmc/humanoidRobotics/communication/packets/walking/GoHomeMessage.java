package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "The message commands the controller to bring the given part of the body back to a default configuration called 'home'."
      + " It is useful to get back to a safe configuration before walking.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/go_home")
public class GoHomeMessage extends Packet<GoHomeMessage>
{
   public enum BodyPart
   {
      @RosEnumValueDocumentation(documentation = "Request the chest to go back to a straight up configuration.")
      ARM,
      @RosEnumValueDocumentation(documentation = "Request the arm to go to a preconfigured home configuration that is elbow lightly flexed, forearm pointing forward, and upper pointing downward.")
      CHEST,
      @RosEnumValueDocumentation(documentation = "Request the pelvis to go back to between the feet, zero pitch and roll, and headed in the same direction as the feet.")
      PELVIS;

      public static final BodyPart[] values = values();

      public boolean isRobotSideNeeded()
      {
         switch (this)
         {
         case ARM:
            return true;
         case CHEST:
         case PELVIS:
            return false;
         default:
            throw new RuntimeException("Should not get there.");
         }
      }
   }

   @RosExportedField(documentation = "Specifies the part of the body the user wants to move back to it home configuration.")
   public BodyPart bodyPart;
   @RosExportedField(documentation = "Needed to identify a side dependent end-effector.")
   public RobotSide robotSide;
   @RosExportedField(documentation = "How long the trajectory will spline from the current desired to the home configuration.")
   public double trajectoryTime;

   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;

   public GoHomeMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public static void checkRobotSide(BodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);
   }

   @Override
   public void set(GoHomeMessage other)
   {
      bodyPart = other.bodyPart;
      robotSide = other.robotSide;
      trajectoryTime = other.trajectoryTime;
      executionDelayTime = other.executionDelayTime;
      setPacketInformation(other);
   }

   public BodyPart getBodyPart()
   {
      return bodyPart;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime;
   }

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * 
    * @return the time to delay this command in seconds
    */
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * 
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }

   @Override
   public boolean epsilonEquals(GoHomeMessage other, double epsilon)
   {
      if (bodyPart != other.bodyPart)
         return false;
      if (robotSide != other.robotSide)
         return false;
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateGoHomeMessage(this);
   }
}
