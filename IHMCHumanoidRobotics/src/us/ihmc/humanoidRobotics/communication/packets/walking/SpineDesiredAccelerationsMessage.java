package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.AbstractDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.tools.ArrayTools;

@RosMessagePacket(documentation =
      "This message gives the user the option to bypass IHMC feedback controllers for the spine joints by sending desired joint accelerations."
            + " One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations."
            + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/spine_desired_joint_accelerations")
public class SpineDesiredAccelerationsMessage extends AbstractDesiredAccelerationsMessage<SpineDesiredAccelerationsMessage>
{
   
   public SpineDesiredAccelerationsMessage()
   {
      super();
   }
   
   public SpineDesiredAccelerationsMessage(double[] desiredJointAccelerations)
   {
      super(desiredJointAccelerations);
   }
   
   @Override
   public boolean epsilonEquals(SpineDesiredAccelerationsMessage other, double epsilon)
   {
      if (!ArrayTools.deltaEquals(getDesiredJointAccelerations(), other.getDesiredJointAccelerations(), epsilon))
         return false;
      return true;
   }
   
   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateSpineDesiredAccelerationsMessage(this, true);
   }
}
