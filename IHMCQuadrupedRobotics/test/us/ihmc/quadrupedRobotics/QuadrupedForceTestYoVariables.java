package us.ihmc.quadrupedRobotics;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerState;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class QuadrupedForceTestYoVariables extends QuadrupedTestYoVariables
{
   private final EnumYoVariable<QuadrupedForceControllerRequestedEvent> userTrigger;
   private final EnumYoVariable<QuadrupedForceControllerState> forceControllerState;
   
   private final DoubleYoVariable stanceHeight;
   private final DoubleYoVariable groundPlanePointZ;
   
   // XGait
   private final DoubleYoVariable xGaitEndPhaseShiftInput;
   private final DoubleYoVariable xGaitEndDoubleSupportDurationInput;
   private final DoubleYoVariable xGaitStanceWidthInput;
   private final DoubleYoVariable xGaitStanceLengthInput;
   private final DoubleYoVariable xGaitStepGroundClearanceInput;
   private final DoubleYoVariable xGaitStepDurationInput;
   
   // Step
   private final EnumYoVariable<RobotQuadrant> timedStepQuadrant;
   private final DoubleYoVariable timedStepDuration;
   private final DoubleYoVariable timedStepGroundClearance;
   private final DoubleYoVariable timedStepGoalPositionX;
   private final DoubleYoVariable timedStepGoalPositionY;
   private final DoubleYoVariable timedStepGoalPositionZ;

   private final DoubleYoVariable comPositionEstimateX;
   private final DoubleYoVariable comPositionEstimateY;
   private final DoubleYoVariable comPositionEstimateZ;

   @SuppressWarnings("unchecked")
   public QuadrupedForceTestYoVariables(SimulationConstructionSet scs)
   {
      super(scs);
      
      userTrigger = (EnumYoVariable<QuadrupedForceControllerRequestedEvent>) scs.getVariable("userTrigger");
      forceControllerState = (EnumYoVariable<QuadrupedForceControllerState>) scs.getVariable("forceControllerState");
      
      stanceHeight = (DoubleYoVariable) scs.getVariable("param__stanceHeight");
      groundPlanePointZ = (DoubleYoVariable) scs.getVariable("groundPlanePointZ");
      
      xGaitEndPhaseShiftInput = (DoubleYoVariable) scs.getVariable("endPhaseShiftInput");
      xGaitEndDoubleSupportDurationInput = (DoubleYoVariable) scs.getVariable("endDoubleSupportDurationInput");
      xGaitStanceWidthInput = (DoubleYoVariable) scs.getVariable("stanceWidthInput");
      xGaitStanceLengthInput = (DoubleYoVariable) scs.getVariable("stanceLengthInput");
      xGaitStepGroundClearanceInput = (DoubleYoVariable) scs.getVariable("stepGroundClearanceInput");
      xGaitStepDurationInput = (DoubleYoVariable) scs.getVariable("stepDurationInput");
      
      comPositionEstimateX = (DoubleYoVariable) scs.getVariable("comPositionEstimateX");
      comPositionEstimateY = (DoubleYoVariable) scs.getVariable("comPositionEstimateY");
      comPositionEstimateZ = (DoubleYoVariable) scs.getVariable("comPositionEstimateZ");
      
      timedStepQuadrant = (EnumYoVariable<RobotQuadrant>) scs.getVariable("timedStepQuadrant");
      timedStepDuration = (DoubleYoVariable) scs.getVariable("timedStepDuration");
      timedStepGroundClearance = (DoubleYoVariable) scs.getVariable("timedStepGroundClearance");
      timedStepGoalPositionX = (DoubleYoVariable) scs.getVariable("timedStepGoalPositionX");
      timedStepGoalPositionY = (DoubleYoVariable) scs.getVariable("timedStepGoalPositionY");
      timedStepGoalPositionZ = (DoubleYoVariable) scs.getVariable("timedStepGoalPositionZ");
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

   public DoubleYoVariable getComPositionEstimateX()
   {
      return comPositionEstimateX;
   }

   public DoubleYoVariable getComPositionEstimateY()
   {
      return comPositionEstimateY;
   }

   public DoubleYoVariable getComPositionEstimateZ()
   {
      return comPositionEstimateZ;
   }
   
   public EnumYoVariable<RobotQuadrant> getTimedStepQuadrant()
   {
      return timedStepQuadrant;
   }

   public DoubleYoVariable getTimedStepDuration()
   {
      return timedStepDuration;
   }

   public DoubleYoVariable getTimedStepGroundClearance()
   {
      return timedStepGroundClearance;
   }

   public DoubleYoVariable getTimedStepGoalPositionX()
   {
      return timedStepGoalPositionX;
   }

   public DoubleYoVariable getTimedStepGoalPositionY()
   {
      return timedStepGoalPositionY;
   }

   public DoubleYoVariable getTimedStepGoalPositionZ()
   {
      return timedStepGoalPositionZ;
   }
}
