package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

@RosMessagePacket(documentation = "The message commands the controller to bring the given part of the body back to a default configuration called 'home'."
      + " It is useful to get back to a safe configuration before walking.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/go_home")
public class GoHomeMessage extends Packet<GoHomeMessage>
{
   @RosExportedField(documentation = "Specifies the part of the body the user wants to move back to it home configuration.")
   public byte humanoidBodyPart;
   @RosExportedField(documentation = "Needed to identify a side dependent end-effector.")
   public byte robotSide;
   @RosExportedField(documentation = "How long the trajectory will spline from the current desired to the home configuration.")
   public double trajectoryTime;

   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;

   public GoHomeMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(GoHomeMessage other)
   {
      humanoidBodyPart = other.humanoidBodyPart;
      robotSide = other.robotSide;
      trajectoryTime = other.trajectoryTime;
      executionDelayTime = other.executionDelayTime;
      setPacketInformation(other);
   }

   public byte getBodyPart()
   {
      return humanoidBodyPart;
   }

   public byte getRobotSide()
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
      if (humanoidBodyPart != other.humanoidBodyPart)
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
