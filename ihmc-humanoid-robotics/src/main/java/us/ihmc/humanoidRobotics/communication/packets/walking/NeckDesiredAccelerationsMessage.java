package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.AbstractDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

@RosMessagePacket(documentation = "This message gives the user the option to bypass IHMC feedback controllers for the neck joints by sending desired neck joint accelerations."
      + " One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/neck_desired_acceleration")
public class NeckDesiredAccelerationsMessage extends Packet<NeckDesiredAccelerationsMessage>
{
   @RosExportedField(documentation = "The desired joint acceleration information.")
   public AbstractDesiredAccelerationsMessage desiredAccelerations;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public NeckDesiredAccelerationsMessage()
   {
      desiredAccelerations = new AbstractDesiredAccelerationsMessage();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Random constructor for unit testing this packet
    * 
    * @param random seed
    */
   public NeckDesiredAccelerationsMessage(Random random)
   {
      desiredAccelerations = new AbstractDesiredAccelerationsMessage(random);
   }

   /**
    * Constructor that sets the desired accelerations in this message to the provided values
    * 
    * @param desiredJointAccelerations
    */
   public NeckDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      desiredAccelerations = new AbstractDesiredAccelerationsMessage(desiredJointAccelerations);
   }

   public AbstractDesiredAccelerationsMessage getDesiredAccelerations()
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
