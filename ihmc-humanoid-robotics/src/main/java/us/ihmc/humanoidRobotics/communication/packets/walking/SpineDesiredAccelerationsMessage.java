package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.DesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;

@RosMessagePacket(documentation = "This message gives the user the option to bypass IHMC feedback controllers for the spine joints by sending desired joint accelerations."
      + " One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/spine_desired_joint_accelerations")
public class SpineDesiredAccelerationsMessage extends Packet<SpineDesiredAccelerationsMessage>
{
   @RosExportedField(documentation = "The desired joint acceleration information.")
   public DesiredAccelerationsMessage desiredAccelerations;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public SpineDesiredAccelerationsMessage()
   {
      desiredAccelerations = new DesiredAccelerationsMessage();
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Random constructor for unit testing this packet
    * 
    * @param random seed
    */
   public SpineDesiredAccelerationsMessage(Random random)
   {
      desiredAccelerations = new DesiredAccelerationsMessage(random);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Constructor that sets the desired accelerations in this message to the provided values
    * 
    * @param desiredJointAccelerations
    */
   public SpineDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      desiredAccelerations = new DesiredAccelerationsMessage(desiredJointAccelerations);
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
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
   public boolean epsilonEquals(SpineDesiredAccelerationsMessage other, double epsilon)
   {
      return desiredAccelerations.epsilonEquals(other.desiredAccelerations, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateSpineDesiredAccelerationsMessage(this);
   }
}
