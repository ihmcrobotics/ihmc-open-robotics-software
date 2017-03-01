package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.AbstractDesiredAccelerationsMessage;

@RosMessagePacket(documentation =
      "This message gives the user the option to bypass IHMC feedback controllers for the spine joints by sending desired joint accelerations."
            + " One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations."
            + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/spine_desired_joint_accelerations")
public class SpineDesiredAccelerationsMessage extends AbstractDesiredAccelerationsMessage<SpineDesiredAccelerationsMessage>
{
   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public SpineDesiredAccelerationsMessage()
   {
      super();
   }

   /**
    * Random constructor for unit testing this packet
    * @param random seed
    */
   public SpineDesiredAccelerationsMessage(Random random)
   {
      super(random);
   }

   /**
    * Constructor that sets the desired accelerations in this message to the provided values
    * @param desiredJointAccelerations
    */
   public SpineDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      super(desiredJointAccelerations);
   }

}
