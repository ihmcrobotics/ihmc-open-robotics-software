package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

@RosMessagePacket(documentation = "This message commands the controller to start loading a foot that was unloaded to support the robot weight. "
      + " When the robot is performing a 'flamingo stance' (one foot in the air not actually walking) and the user wants the robot to switch back to double support."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/foot_load_bearing")
public class FootLoadBearingMessage extends Packet<FootLoadBearingMessage>
{
   @RosExportedField(documentation = "Needed to identify a side dependent end-effector.")
   public byte robotSide;
   @RosExportedField(documentation = "Wether the end-effector should be loaded or unloaded.")
   public byte request;
   
   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootLoadBearingMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(FootLoadBearingMessage other)
   {
      robotSide = other.robotSide;
      request = other.request;
      setPacketInformation(other);
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   public byte getRequest()
   {
      return request;
   }
   
   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }
   
   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }

   @Override
   public boolean epsilonEquals(FootLoadBearingMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "Foot load bearing:"
            + "\nrobotSide = " + robotSide
            + "\nrequest = " + request;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateFootLoadBearingMessage(this);
   }
}
