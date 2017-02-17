package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.ArrayTools;

@RosMessagePacket(documentation =
      "This message gives the user the option to bypass IHMC feedback controllers for the neck joints by sending desired neck joint accelerations."
      + " One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.",
                  rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
                  topic = "/control/neck_desired_acceleration")
public class NeckDesiredAccelerationsMessage extends Packet<NeckDesiredAccelerationsMessage>
{
   public enum NeckControlMode
   {
      @RosEnumValueDocumentation(documentation = "The IHMC controller controls the arm joints to execute desired inputs given from NeckTrajectoryMessage, HeadTrajectoryMessage."
            + " PD controllers are run for the given inputs and will either compute the desired head spatial acceleration or neck joint desired accelerations."
            + "The desired joint torques to achieve these desired accelerations are computed by the IHMC QP solver & inverse dynamics calculator.")
      IHMC_CONTROL_MODE,
      @RosEnumValueDocumentation(documentation = "The user directly sets what the neck joint desired accelerations have to be."
            + " The IHMC controller will stop tracking positions and the user desired accelerations will be fed to the IHMC QP solver & inverse dynamics to compute the desired joint torques.")
      USER_CONTROL_MODE
   }


   @RosExportedField(documentation = "Specifies the control mode for controlling the neck joints. See NeckControlMode.")
   public NeckControlMode neckControlMode;
   @RosExportedField(documentation = "Specifies the desired joint accelerations. Only necessary when neckControlMode == USER_CONTROL_MODE.")
   public double[] neckDesiredJointAccelerations;

   public NeckDesiredAccelerationsMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public NeckDesiredAccelerationsMessage(Random random)
   {
      neckControlMode = RandomTools.generateRandomEnum(random, NeckControlMode.class);

      int randomNumberOfAccels = random.nextInt(16) + 1;
      neckDesiredJointAccelerations = new double[randomNumberOfAccels];

      for(int i = 0; i < randomNumberOfAccels; i++)
      {
         neckDesiredJointAccelerations[i] = RandomTools.generateRandomDoubleWithEdgeCases(random, 0.01);
      }
   }

   public NeckDesiredAccelerationsMessage(NeckControlMode neckControlMode, double[] neckDesiredJointAccelerations)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.neckControlMode = neckControlMode;
      this.neckDesiredJointAccelerations = neckDesiredJointAccelerations;
   }

   public int getNumberOfJoints()
   {
      if (neckDesiredJointAccelerations == null)
         return 0;
      else
         return neckDesiredJointAccelerations.length;
   }

   public NeckControlMode getNeckControlMode()
   {
      return neckControlMode;
   }

   public double[] getNeckDesiredJointAccelerations()
   {
      return neckDesiredJointAccelerations;
   }

   public double getNeckDesiredJointAcceleration(int jointIndex)
   {
      return neckDesiredJointAccelerations[jointIndex];
   }

   @Override
   public boolean epsilonEquals(NeckDesiredAccelerationsMessage other, double epsilon)
   {
      if (neckControlMode != other.neckControlMode)
         return false;
      if (!ArrayTools.deltaEquals(neckDesiredJointAccelerations, other.neckDesiredJointAccelerations, epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      switch (neckControlMode)
      {
      case IHMC_CONTROL_MODE:
         return NeckControlMode.class.getSimpleName() + " == " + neckControlMode + ".";
      case USER_CONTROL_MODE:
         String ret = NeckControlMode.class.getSimpleName() + " == " + neckControlMode + ", desired accelerations = [";
         NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
         for (int i = 0; i < getNumberOfJoints(); i++)
         {
            double jointDesiredAcceleration = neckDesiredJointAccelerations[i];
            ret += doubleFormat.format(jointDesiredAcceleration);
            if (i < getNumberOfJoints() - 1)
               ret += ", ";
         }
         return ret + "].";
      default:
         return "Invalid " + NeckControlMode.class.getSimpleName() + ": " + neckControlMode;
      }
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validateNeckDesiredAccelerationsMessage(this);
   }
}
