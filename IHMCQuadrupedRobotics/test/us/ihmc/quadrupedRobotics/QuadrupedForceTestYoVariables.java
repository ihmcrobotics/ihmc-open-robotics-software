package us.ihmc.quadrupedRobotics;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedForceTestYoVariables extends QuadrupedTestYoVariables
{
   private final EnumYoVariable<QuadrupedForceControllerRequestedEvent> userTrigger;
   private final EnumYoVariable<QuadrupedForceControllerState> forceControllerState;

   @SuppressWarnings("unchecked")
   public QuadrupedForceTestYoVariables(SimulationConstructionSet scs)
   {
      super(scs);
      
      userTrigger = (EnumYoVariable<QuadrupedForceControllerRequestedEvent>) scs.getVariable("usertrigger");
      forceControllerState = (EnumYoVariable<QuadrupedForceControllerState>) scs.getVariable("forceControllerState");
   }

   public EnumYoVariable<QuadrupedForceControllerRequestedEvent> getUserTrigger()
   {
      return userTrigger;
   }

   public EnumYoVariable<QuadrupedForceControllerState> getForceControllerState()
   {
      return forceControllerState;
   }
}
