package us.ihmc.quadrupedRobotics;

import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerState;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.EnumYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedPositionTestYoVariables extends QuadrupedTestYoVariables
{
   private final EnumYoVariable<QuadrupedPositionControllerRequestedEvent> userTrigger;
   private final EnumYoVariable<QuadrupedPositionControllerState> positionControllerState;
   private final YoDouble swingDuration;
   private final YoDouble desiredCoMPositionZ;

   @SuppressWarnings("unchecked")
   public QuadrupedPositionTestYoVariables(SimulationConstructionSet scs)
   {
      super(scs);
      
      userTrigger = (EnumYoVariable<QuadrupedPositionControllerRequestedEvent>) scs.getVariable("userTrigger");
      positionControllerState = (EnumYoVariable<QuadrupedPositionControllerState>) scs.getVariable("positionControllerState");
      swingDuration = (YoDouble) scs.getVariable("swingDuration");
      desiredCoMPositionZ = (YoDouble) scs.getVariable("desiredCoMPositionZ");
   }

   public EnumYoVariable<QuadrupedPositionControllerRequestedEvent> getUserTrigger()
   {
      return userTrigger;
   }

   public EnumYoVariable<QuadrupedPositionControllerState> getPositionControllerState()
   {
      return positionControllerState;
   }

   public YoDouble getSwingDuration()
   {
      return swingDuration;
   }

   public YoDouble getDesiredCoMPositionZ()
   {
      return desiredCoMPositionZ;
   }
}
