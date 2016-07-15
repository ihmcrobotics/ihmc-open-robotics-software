package us.ihmc.quadrupedRobotics;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedForceTestYoVariables extends QuadrupedTestYoVariables
{
   private final EnumYoVariable<QuadrupedForceControllerRequestedEvent> userTrigger;
   private final EnumYoVariable<QuadrupedForceControllerState> forceControllerState;
   
   private final DoubleYoVariable stanceHeight;
   private final DoubleYoVariable groundPlanePointZ;
   
   private final DoubleYoVariable xGaitEndPhaseShiftInput;
   private final DoubleYoVariable xGaitEndDoubleSupportDurationInput;
   private final DoubleYoVariable xGaitStanceWidthInput;
   private final DoubleYoVariable xGaitStanceLengthInput;
   private final DoubleYoVariable xGaitStepGroundClearanceInput;
   private final DoubleYoVariable xGaitStepDurationInput;

   @SuppressWarnings("unchecked")
   public QuadrupedForceTestYoVariables(SimulationConstructionSet scs)
   {
      super(scs);
      
      userTrigger = (EnumYoVariable<QuadrupedForceControllerRequestedEvent>) scs.getVariable("usertrigger");
      forceControllerState = (EnumYoVariable<QuadrupedForceControllerState>) scs.getVariable("forceControllerState");
      
      stanceHeight = (DoubleYoVariable) scs.getVariable("param__stanceHeight");
      groundPlanePointZ = (DoubleYoVariable) scs.getVariable("groundPlanePointZ");
      
      xGaitEndPhaseShiftInput = (DoubleYoVariable) scs.getVariable("endPhaseShiftInput");
      xGaitEndDoubleSupportDurationInput = (DoubleYoVariable) scs.getVariable("endDoubleSupportDurationInput");
      xGaitStanceWidthInput = (DoubleYoVariable) scs.getVariable("stanceWidthInput");
      xGaitStanceLengthInput = (DoubleYoVariable) scs.getVariable("stanceLengthInput");
      xGaitStepGroundClearanceInput = (DoubleYoVariable) scs.getVariable("stepGroundClearanceInput");
      xGaitStepDurationInput = (DoubleYoVariable) scs.getVariable("stepDurationInput");
   }

   public EnumYoVariable<QuadrupedForceControllerRequestedEvent> getUserTrigger()
   {
      return userTrigger;
   }

   public EnumYoVariable<QuadrupedForceControllerState> getForceControllerState()
   {
      return forceControllerState;
   }

   public DoubleYoVariable getStanceHeight()
   {
      return stanceHeight;
   }

   public DoubleYoVariable getGroundPlanePointZ()
   {
      return groundPlanePointZ;
   }

   public DoubleYoVariable getXGaitEndDoubleSupportDurationInput()
   {
      return xGaitEndDoubleSupportDurationInput;
   }

   public DoubleYoVariable getXGaitEndPhaseShiftInput()
   {
      return xGaitEndPhaseShiftInput;
   }

   public DoubleYoVariable getXGaitStanceWidthInput()
   {
      return xGaitStanceWidthInput;
   }

   public DoubleYoVariable getXGaitStanceLengthInput()
   {
      return xGaitStanceLengthInput;
   }

   public DoubleYoVariable getXGaitStepGroundClearanceInput()
   {
      return xGaitStepGroundClearanceInput;
   }

   public DoubleYoVariable getXGaitStepDurationInput()
   {
      return xGaitStepDurationInput;
   }
}
