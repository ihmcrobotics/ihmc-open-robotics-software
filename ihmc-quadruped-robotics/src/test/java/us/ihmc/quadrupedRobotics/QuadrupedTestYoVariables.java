package us.ihmc.quadrupedRobotics;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSteppingStateEnum;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedTestYoVariables
{
   private final YoEnum<QuadrupedControllerRequestedEvent> userTrigger;
   private final YoEnum<QuadrupedSteppingRequestedEvent> stepTrigger;
   private final YoEnum<QuadrupedControllerEnum> controllerState;
   private final YoEnum<QuadrupedSteppingStateEnum> steppingState;

   private final YoDouble stanceHeight;
   private final YoDouble groundPlanePointZ;

   // Step
   private final YoEnum<RobotQuadrant> timedStepQuadrant;
   private final YoDouble timedStepDuration;
   private final YoDouble timedStepGroundClearance;
   private final YoDouble timedStepGoalPositionX;
   private final YoDouble timedStepGoalPositionY;
   private final YoDouble timedStepGoalPositionZ;

   // CoM
   private final YoDouble comPositionEstimateX;
   private final YoDouble comPositionEstimateY;
   private final YoDouble currentHeightInWorld;

   private final YoDouble bodyCurrentOrientationQx;
   private final YoDouble bodyCurrentOrientationQy;
   private final YoDouble bodyCurrentOrientationQz;
   private final YoDouble bodyCurrentOrientationQs;

   private final YoDouble comPositionSetpointX;
   private final YoDouble comPositionSetpointY;
   private final YoDouble desiredHeightInWorld;


   private final YoDouble yoTime;

   private final YoDouble robotBodyX;
   private final YoDouble robotBodyY;
   private final YoDouble robotBodyZ;
   private final YoDouble robotBodyYaw;

   private final QuadrantDependentList<YoBoolean> controllerFootSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoBoolean> footSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionXs = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionYs = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionZs = new QuadrantDependentList<>();
   
   private final YoBoolean limitJointTorques;

   private final Quaternion bodyOrientation = new Quaternion();


   public QuadrupedTestYoVariables(SimulationConstructionSet scs)
   {
      yoTime = (YoDouble) scs.getVariable("t");

      robotBodyX = (YoDouble) scs.getVariable("q_x");
      robotBodyY = (YoDouble) scs.getVariable("q_y");
      robotBodyZ = (YoDouble) scs.getVariable("q_z");
      robotBodyYaw = (YoDouble) scs.getVariable("q_yaw");
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerFootSwitches.set(robotQuadrant, (YoBoolean) scs.getVariable(robotQuadrant.getCamelCaseName() + "QuadrupedTouchdownFootSwitch_controllerThinksHasTouchedDown"));
         footSwitches.set(robotQuadrant, (YoBoolean) scs.getVariable(robotQuadrant.getCamelCaseName() + "TouchdownDetected"));
         solePositionXs.set(robotQuadrant, (YoDouble) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionX"));
         solePositionYs.set(robotQuadrant, (YoDouble) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionY"));
         solePositionZs.set(robotQuadrant, (YoDouble) scs.getVariable(robotQuadrant.getCamelCaseName() + "SolePositionZ"));
      }
      
      limitJointTorques = (YoBoolean) scs.getVariable("limitJointTorques");

      userTrigger = (YoEnum<QuadrupedControllerRequestedEvent>) scs.getVariable("requestedControllerState");
      stepTrigger = (YoEnum<QuadrupedSteppingRequestedEvent>) scs.getVariable("stepTrigger");
      controllerState = (YoEnum<QuadrupedControllerEnum>) scs.getVariable("controllerCurrentState");
      steppingState = (YoEnum<QuadrupedSteppingStateEnum>) scs.getVariable("steppingCurrentState");

      stanceHeight = (YoDouble) scs.getVariable("stanceHeight");
      groundPlanePointZ = (YoDouble) scs.getVariable("groundPlanePointZ");

      comPositionEstimateX = (YoDouble) scs.getVariable("comPositionEstimateX");
      comPositionEstimateY = (YoDouble) scs.getVariable("comPositionEstimateY");
      currentHeightInWorld = (YoDouble) scs.getVariable("currentHeightInWorld");

      timedStepQuadrant = (YoEnum<RobotQuadrant>) scs.getVariable("timedStepQuadrant");
      timedStepDuration = (YoDouble) scs.getVariable("timedStepDuration");
      timedStepGroundClearance = (YoDouble) scs.getVariable("timedStepGroundClearance");
      timedStepGoalPositionX = (YoDouble) scs.getVariable("timedStepGoalPositionX");
      timedStepGoalPositionY = (YoDouble) scs.getVariable("timedStepGoalPositionY");
      timedStepGoalPositionZ = (YoDouble) scs.getVariable("timedStepGoalPositionZ");

      bodyCurrentOrientationQx = (YoDouble) scs.getVariable("bodyCurrentOrientationQx");
      bodyCurrentOrientationQy = (YoDouble) scs.getVariable("bodyCurrentOrientationQy");
      bodyCurrentOrientationQz = (YoDouble) scs.getVariable("bodyCurrentOrientationQz");
      bodyCurrentOrientationQs = (YoDouble) scs.getVariable("bodyCurrentOrientationQs");

      comPositionSetpointX = (YoDouble) scs.getVariable("comPositionSetpointX");
      comPositionSetpointY = (YoDouble) scs.getVariable("comPositionSetpointY");
      desiredHeightInWorld = (YoDouble) scs.getVariable("desiredHeightInWorld");
   }

   public YoDouble getRobotBodyX()
   {
      return robotBodyX;
   }

   public YoDouble getRobotBodyY()
   {
      return robotBodyY;
   }

   public YoDouble getRobotBodyZ()
   {
      return robotBodyZ;
   }

   public YoDouble getRobotBodyYaw()
   {
      return robotBodyYaw;
   }

   public YoDouble getYoTime()
   {
      return yoTime;
   }

   public QuadrantDependentList<YoBoolean> getControllerFootSwitches()
   {
      return controllerFootSwitches;
   }

   public QuadrantDependentList<YoBoolean> getFootSwitches()
   {
      return footSwitches;
   }

   public QuadrantDependentList<YoDouble> getSolePositionXs()
   {
      return solePositionXs;
   }

   public QuadrantDependentList<YoDouble> getSolePositionYs()
   {
      return solePositionYs;
   }

   public QuadrantDependentList<YoDouble> getSolePositionZs()
   {
      return solePositionZs;
   }

   public YoBoolean getLimitJointTorques()
   {
      return limitJointTorques;
   }

   public YoEnum<QuadrupedControllerRequestedEvent> getUserTrigger()
   {
      return userTrigger;
   }

   public YoEnum<QuadrupedSteppingRequestedEvent> getStepTrigger()
   {
      return stepTrigger;
   }

   public YoEnum<QuadrupedControllerEnum> getControllerState()
   {
      return controllerState;
   }

   public YoEnum<QuadrupedSteppingStateEnum> getSteppingState()
   {
      return steppingState;
   }

   public YoDouble getStanceHeight()
   {
      return stanceHeight;
   }

   public YoDouble getGroundPlanePointZ()
   {
      return groundPlanePointZ;
   }

   public YoDouble getComPositionEstimateX()
   {
      return comPositionEstimateX;
   }

   public YoDouble getComPositionEstimateY()
   {
      return comPositionEstimateY;
   }

   public YoDouble getCurrentHeightInWorld()
   {
      return currentHeightInWorld;
   }

   public YoEnum<RobotQuadrant> getTimedStepQuadrant()
   {
      return timedStepQuadrant;
   }

   public YoDouble getTimedStepDuration()
   {
      return timedStepDuration;
   }

   public YoDouble getTimedStepGroundClearance()
   {
      return timedStepGroundClearance;
   }

   public YoDouble getTimedStepGoalPositionX()
   {
      return timedStepGoalPositionX;
   }

   public YoDouble getTimedStepGoalPositionY()
   {
      return timedStepGoalPositionY;
   }

   public YoDouble getTimedStepGoalPositionZ()
   {
      return timedStepGoalPositionZ;
   }

   public double getBodyEstimateYaw()
   {
      bodyOrientation.set(bodyCurrentOrientationQx.getDoubleValue(), bodyCurrentOrientationQy.getDoubleValue(), bodyCurrentOrientationQz.getDoubleValue(),
                          bodyCurrentOrientationQs.getDoubleValue());
      return bodyOrientation.getYaw();
   }

   public double getBodyEstimatePitch()
   {
       bodyOrientation.set(bodyCurrentOrientationQx.getDoubleValue(), bodyCurrentOrientationQy.getDoubleValue(), bodyCurrentOrientationQz.getDoubleValue(),
                                 bodyCurrentOrientationQs.getDoubleValue());
      return bodyOrientation.getPitch();
   }

   public double getBodyEstimateRoll()
   {
      bodyOrientation.set(bodyCurrentOrientationQx.getDoubleValue(), bodyCurrentOrientationQy.getDoubleValue(), bodyCurrentOrientationQz.getDoubleValue(),
                          bodyCurrentOrientationQs.getDoubleValue());
      return bodyOrientation.getRoll();
   }

   public YoDouble getComPositionSetpointX()
   {
      return comPositionSetpointX;
   }

   public YoDouble getComPositionSetpointY()
   {
      return comPositionSetpointY;
   }

   public YoDouble getHeightInWorldSetpoint()
   {
      return desiredHeightInWorld;
   }
}
