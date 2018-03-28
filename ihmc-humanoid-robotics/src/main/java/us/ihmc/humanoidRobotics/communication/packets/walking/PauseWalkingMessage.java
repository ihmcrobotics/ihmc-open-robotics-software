package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "This message pauses the execution of a list of footsteps. If this message is\n"
      + "sent in the middle of executing a footstep, the robot will finish the step and\n" + "pause when back in double support."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/pause_walking")
public class PauseWalkingMessage extends Packet<PauseWalkingMessage>
{
   @RosExportedField(documentation = "True to pause walking, false to unpause and resume an existing plan.")
   public boolean pause;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public PauseWalkingMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(PauseWalkingMessage other)
   {
      pause = other.pause;
      setPacketInformation(other);
   }

   public boolean getPause()
   {
      return pause;
   }

   @Override
   public String toString()
   {
      return ("Paused = " + this.getPause());
   }

   @Override
   public boolean equals(Object obj)
   {
      return ((obj instanceof PauseWalkingMessage) && this.equals((PauseWalkingMessage) obj));
   }

   @Override
   public boolean epsilonEquals(PauseWalkingMessage other, double epsilon)
   {
      return (this.getPause() == other.getPause());
   }
}
