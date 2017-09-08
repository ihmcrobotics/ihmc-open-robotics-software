package us.ihmc.humanoidRobotics.communication.packets.valkyrie;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;

@RosMessagePacket(documentation = "Request a Valkyrie low-level control mode, which is either: stand-prep, calibration, or high-level control.",
      rosPackage = "ihmc_valkyrie_ros",
      topic = "/control/low_level_control_mode")
public class ValkyrieLowLevelControlModeMessage extends Packet<ValkyrieLowLevelControlModeMessage>
{
   public enum ControlMode
   {
      @RosEnumValueDocumentation(documentation = "Simple position controller to keep the robot in a 'ready-to-walk' configuration.")
      STAND_PREP,
      @RosEnumValueDocumentation(documentation = "Automated calibration routine to estimate the joint torque offsets and foot force/torque sensor offsets. The routine takes about 15 seconds.")
      CALIBRATION,
      @RosEnumValueDocumentation(documentation = "Switching to the high level walking controller.")
      HIGH_LEVEL_CONTROL;
   }

   @RosExportedField(documentation = "Specifies the low-level control mode to switch to.")
   public ControlMode requestedControlMode;

   public ValkyrieLowLevelControlModeMessage()
   {
   }

   public void setRequestedControlMode(ControlMode requestedControlMode)
   {
      this.requestedControlMode = requestedControlMode;
   }

   public ControlMode getRequestedControlMode()
   {
      return requestedControlMode;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      String errorMessage = null;
      if (requestedControlMode == null)
         errorMessage = "The field requestedControlMode is null.";
      return errorMessage;
   }

   @Override
   public boolean epsilonEquals(ValkyrieLowLevelControlModeMessage other, double epsilon)
   {
      return requestedControlMode == other.requestedControlMode;
   }

   public ValkyrieLowLevelControlModeMessage(Random random)
   {
      requestedControlMode = RandomNumbers.nextEnum(random, ControlMode.class);
   }
}
