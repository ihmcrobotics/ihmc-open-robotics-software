package us.ihmc.quadrupedRobotics;

import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionControllerState;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedPositionTestYoVariables extends QuadrupedTestYoVariables
{
   private final EnumYoVariable<QuadrupedPositionControllerRequestedEvent> userTrigger;
   private final EnumYoVariable<QuadrupedPositionControllerState> positionControllerState;
   private final DoubleYoVariable swingDuration;
   private final DoubleYoVariable desiredCoMPositionZ;

   @SuppressWarnings("unchecked")
   public QuadrupedPositionTestYoVariables(SimulationConstructionSet scs)
   {
      super(scs);
      
      userTrigger = (EnumYoVariable<QuadrupedPositionControllerRequestedEvent>) scs.getVariable("userTrigger");
      positionControllerState = (EnumYoVariable<QuadrupedPositionControllerState>) scs.getVariable("positionControllerState");
      swingDuration = (DoubleYoVariable) scs.getVariable("swingDuration");
      desiredCoMPositionZ = (DoubleYoVariable) scs.getVariable("desiredCoMPositionZ");
   }

   public EnumYoVariable<QuadrupedPositionControllerRequestedEvent> getUserTrigger()
   {
      return userTrigger;
   }

   public EnumYoVariable<QuadrupedPositionControllerState> getPositionControllerState()
   {
      return positionControllerState;
   }

   public DoubleYoVariable getSwingDuration()
   {
      return swingDuration;
   }

   public DoubleYoVariable getDesiredCoMPositionZ()
   {
      return desiredCoMPositionZ;
   }
}
