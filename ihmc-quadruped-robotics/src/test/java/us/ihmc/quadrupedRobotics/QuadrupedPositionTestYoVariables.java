package us.ihmc.quadrupedRobotics;

import us.ihmc.quadrupedRobotics.controller.QuadrupedPositionControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedPositionControllerState;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedPositionTestYoVariables extends QuadrupedTestYoVariables
{
   private final YoEnum<QuadrupedPositionControllerRequestedEvent> userTrigger;
   private final YoEnum<QuadrupedPositionControllerState> positionControllerState;
   private final YoDouble swingDuration;
   private final YoDouble desiredCoMPositionZ;

   private final YoDouble yoPlanarVelocityInputX;
   private final YoDouble yoPlanarVelocityInputY;
   private final YoDouble yoPlanarVelocityInputZ;

   private final YoDouble yoComPositionInputZ;

   @SuppressWarnings("unchecked")
   public QuadrupedPositionTestYoVariables(SimulationConstructionSet scs)
   {
      super(scs);
      
      userTrigger = (YoEnum<QuadrupedPositionControllerRequestedEvent>) scs.getVariable("userTrigger");
      positionControllerState = (YoEnum<QuadrupedPositionControllerState>) scs.getVariable("positionControllerState");
      swingDuration = (YoDouble) scs.getVariable("swingDuration");
      desiredCoMPositionZ = (YoDouble) scs.getVariable("desiredCoMPositionZ");

      yoPlanarVelocityInputX = (YoDouble) scs.getVariable("planarVelocityInputX");
      yoPlanarVelocityInputY = (YoDouble) scs.getVariable("planarVelocityInputY");
      yoPlanarVelocityInputZ = (YoDouble) scs.getVariable("planarVelocityInputZ");

      yoComPositionInputZ = (YoDouble) scs.getVariable("comPositionInputZ");
   }

   public YoDouble getYoComPositionInputZ()
   {
      return yoComPositionInputZ;
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

   public YoDouble getYoPlanarVelocityInputX()
   {
      return yoPlanarVelocityInputX;
   }

   public YoDouble getYoPlanarVelocityInputY()
   {
      return yoPlanarVelocityInputY;
   }

   public YoDouble getYoPlanarVelocityInputZ()
   {
      return yoPlanarVelocityInputZ;
   }
}
