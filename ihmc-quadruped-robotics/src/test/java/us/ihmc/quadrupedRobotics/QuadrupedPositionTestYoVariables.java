package us.ihmc.quadrupedRobotics;

import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerState;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedPositionTestYoVariables extends QuadrupedTestYoVariables
{
   private final YoEnum<QuadrupedPositionControllerRequestedEvent> userTrigger;
   private final YoEnum<QuadrupedPositionControllerState> positionControllerState;
   private final YoDouble swingDuration;
   private final YoDouble desiredCoMPositionZ;

   @SuppressWarnings("unchecked")
   public QuadrupedPositionTestYoVariables(SimulationConstructionSet scs)
   {
      super(scs);
      
      userTrigger = (YoEnum<QuadrupedPositionControllerRequestedEvent>) scs.getVariable("userTrigger");
      positionControllerState = (YoEnum<QuadrupedPositionControllerState>) scs.getVariable("positionControllerState");
      swingDuration = (YoDouble) scs.getVariable("swingDuration");
      desiredCoMPositionZ = (YoDouble) scs.getVariable("desiredCoMPositionZ");
   }

   public YoEnum<QuadrupedPositionControllerRequestedEvent> getUserTrigger()
   {
      return userTrigger;
   }

   public YoEnum<QuadrupedPositionControllerState> getPositionControllerState()
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
