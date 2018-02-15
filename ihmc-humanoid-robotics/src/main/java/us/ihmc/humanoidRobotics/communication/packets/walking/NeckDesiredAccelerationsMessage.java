package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.DesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

@RosMessagePacket(documentation = "This message gives the user the option to bypass IHMC feedback controllers for the neck joints by sending desired neck joint accelerations."
      + " One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/neck_desired_acceleration")
public class NeckDesiredAccelerationsMessage extends Packet<NeckDesiredAccelerationsMessage>
{
   @RosExportedField(documentation = "The desired joint acceleration information.")
   public DesiredAccelerationsMessage desiredAccelerations;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public NeckDesiredAccelerationsMessage()
   {
      desiredAccelerations = new DesiredAccelerationsMessage();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Constructor that sets the desired accelerations in this message to the provided values
    * 
    * @param desiredJointAccelerations
    */
   public NeckDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      desiredAccelerations = new DesiredAccelerationsMessage(desiredJointAccelerations);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(NeckDesiredAccelerationsMessage other)
   {
      desiredAccelerations = new DesiredAccelerationsMessage();
      desiredAccelerations.set(other.desiredAccelerations);
      setPacketInformation(other);
   }

   public void setDesiredAccelerations(DesiredAccelerationsMessage desiredAccelerations)
   {
      this.desiredAccelerations = desiredAccelerations;
   }

   public DesiredAccelerationsMessage getDesiredAccelerations()
   {
      return desiredAccelerations;
   }

   @Override
   public void setUniqueId(long uniqueId)
   {
      super.setUniqueId(uniqueId);
      if (desiredAccelerations != null)
         desiredAccelerations.setUniqueId(uniqueId);
   }

   @Override
   public boolean epsilonEquals(NeckDesiredAccelerationsMessage other, double epsilon)
   {
      return desiredAccelerations.epsilonEquals(other.desiredAccelerations, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateNeckDesiredAccelerationsMessage(this);
   }
}
