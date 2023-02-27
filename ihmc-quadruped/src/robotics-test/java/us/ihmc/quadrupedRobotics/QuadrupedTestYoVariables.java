package us.ihmc.quadrupedRobotics;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedFallDetector.FallDetectionType;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedTestYoVariables
{
   private final YoDouble yoTime;

   private final YoDouble robotBodyX;
   private final YoDouble robotBodyY;
   private final YoDouble robotBodyZ;
   private final YoDouble robotBodyYaw;

   private final YoBoolean isFallDetected;
   private final YoEnum<FallDetectionType> fallDetectionType;

   private final QuadrantDependentList<YoBoolean> controllerFootSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoBoolean> footSwitches = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionXs = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionYs = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> solePositionZs = new QuadrantDependentList<>();

   private final YoBoolean limitJointTorques;

   private final YoEnum<HighLevelControllerName> userTrigger;
   private final YoEnum<QuadrupedSteppingRequestedEvent> stepTrigger;
   private final YoEnum<HighLevelControllerName> controllerState;
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

   private final YoDouble timeToPrepareForStanding;
   private final YoDouble minimumTimeDoneWithStandPrep;

   private final YoDouble toWalkingTransitionDuration;
   private final YoDouble timeToMoveSittingDown;

   private final Quaternion bodyOrientation = new Quaternion();

   @SuppressWarnings("unchecked")
   public QuadrupedTestYoVariables(SimulationConstructionSet2 scs)
   {
      yoTime = (YoDouble) scs.getTime();

      String rootJointName = scs.getRobots().get(0).getFloatingRootJoint().getName();
      robotBodyX = (YoDouble) scs.findVariable("q_" + rootJointName + "_x");
      robotBodyY = (YoDouble) scs.findVariable("q_" + rootJointName + "_y");
      robotBodyZ = (YoDouble) scs.findVariable("q_" + rootJointName + "_z");
      robotBodyYaw = (YoDouble) scs.findVariable("q_" + rootJointName + "_yaw");

      isFallDetected = (YoBoolean) scs.findVariable("isFallDetected");
      fallDetectionType = (YoEnum<FallDetectionType>) scs.findVariable("fallDetectionReason");

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         controllerFootSwitches.set(robotQuadrant,
                                    (YoBoolean) scs.findVariable(robotQuadrant.getCamelCaseName()
                                          + "QuadrupedTouchdownFootSwitch_controllerThinksHasTouchedDown"));
         footSwitches.set(robotQuadrant, (YoBoolean) scs.findVariable(robotQuadrant.getCamelCaseName() + "TouchdownDetected"));
         solePositionXs.set(robotQuadrant, (YoDouble) scs.findVariable(robotQuadrant.getCamelCaseName() + "SolePositionX"));
         solePositionYs.set(robotQuadrant, (YoDouble) scs.findVariable(robotQuadrant.getCamelCaseName() + "SolePositionY"));
         solePositionZs.set(robotQuadrant, (YoDouble) scs.findVariable(robotQuadrant.getCamelCaseName() + "SolePositionZ"));
      }

      limitJointTorques = (YoBoolean) scs.findVariable("limitJointTorques");

      userTrigger = (YoEnum<HighLevelControllerName>) scs.findVariable("requestedControllerState");
      stepTrigger = (YoEnum<QuadrupedSteppingRequestedEvent>) scs.findVariable("stepTrigger");
      controllerState = (YoEnum<HighLevelControllerName>) scs.findVariable("controllerCurrentState");
      steppingState = (YoEnum<QuadrupedSteppingStateEnum>) scs.findVariable("steppingCurrentState");

      stanceHeight = (YoDouble) scs.findVariable("stanceHeight");
      groundPlanePointZ = (YoDouble) scs.findVariable("groundPlanePointZ");

      comPositionEstimateX = (YoDouble) scs.findVariable("comPositionEstimateX");
      comPositionEstimateY = (YoDouble) scs.findVariable("comPositionEstimateY");
      currentHeightInWorld = (YoDouble) scs.findVariable("currentHeightInWorld");

      timedStepQuadrant = (YoEnum<RobotQuadrant>) scs.findVariable("timedStepQuadrant");
      timedStepDuration = (YoDouble) scs.findVariable("timedStepDuration");
      timedStepGroundClearance = (YoDouble) scs.findVariable("timedStepGroundClearance");
      timedStepGoalPositionX = (YoDouble) scs.findVariable("timedStepGoalPositionX");
      timedStepGoalPositionY = (YoDouble) scs.findVariable("timedStepGoalPositionY");
      timedStepGoalPositionZ = (YoDouble) scs.findVariable("timedStepGoalPositionZ");

      bodyCurrentOrientationQx = (YoDouble) scs.findVariable("bodyCurrentOrientationQx");
      bodyCurrentOrientationQy = (YoDouble) scs.findVariable("bodyCurrentOrientationQy");
      bodyCurrentOrientationQz = (YoDouble) scs.findVariable("bodyCurrentOrientationQz");
      bodyCurrentOrientationQs = (YoDouble) scs.findVariable("bodyCurrentOrientationQs");

      comPositionSetpointX = (YoDouble) scs.findVariable("comPositionSetpointX");
      comPositionSetpointY = (YoDouble) scs.findVariable("comPositionSetpointY");
      desiredHeightInWorld = (YoDouble) scs.findVariable("desiredHeightInWorld");

      timeToPrepareForStanding = (YoDouble) scs.findVariable("timeToPrepareForStanding");
      minimumTimeDoneWithStandPrep = (YoDouble) scs.findVariable("minimumTimeDoneWithStandPrep");
      toWalkingTransitionDuration = (YoDouble) scs.findVariable("toWalkingTransitionDuration");
      timeToMoveSittingDown = (YoDouble) scs.findVariable("timeToMoveSittingDown");
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

   public YoBoolean getIsFallDetected()
   {
      return isFallDetected;
   }

   public YoEnum<FallDetectionType> getFallDetectionType()
   {
      return fallDetectionType;
   }

   public YoEnum<HighLevelControllerName> getUserTrigger()
   {
      return userTrigger;
   }

   public YoEnum<QuadrupedSteppingRequestedEvent> getStepTrigger()
   {
      return stepTrigger;
   }

   public YoEnum<HighLevelControllerName> getControllerState()
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
      bodyOrientation.set(bodyCurrentOrientationQx.getDoubleValue(),
                          bodyCurrentOrientationQy.getDoubleValue(),
                          bodyCurrentOrientationQz.getDoubleValue(),
                          bodyCurrentOrientationQs.getDoubleValue());
      return bodyOrientation.getYaw();
   }

   public double getBodyEstimatePitch()
   {
      bodyOrientation.set(bodyCurrentOrientationQx.getDoubleValue(),
                          bodyCurrentOrientationQy.getDoubleValue(),
                          bodyCurrentOrientationQz.getDoubleValue(),
                          bodyCurrentOrientationQs.getDoubleValue());
      return bodyOrientation.getPitch();
   }

   public double getBodyEstimateRoll()
   {
      bodyOrientation.set(bodyCurrentOrientationQx.getDoubleValue(),
                          bodyCurrentOrientationQy.getDoubleValue(),
                          bodyCurrentOrientationQz.getDoubleValue(),
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

   public YoDouble getTimeToPrepareForStanding()
   {
      return timeToPrepareForStanding;
   }

   public YoDouble getMinimumTimeDoneWithStandPrep()
   {
      return minimumTimeDoneWithStandPrep;
   }

   public double getTimeInStandPrep()
   {
      return timeToPrepareForStanding.getDoubleValue() + minimumTimeDoneWithStandPrep.getDoubleValue();
   }

   public double getToWalkingTransitionDuration()
   {
      return toWalkingTransitionDuration.getDoubleValue();
   }

   public double getTimeToMoveSittingDown()
   {
      return timeToMoveSittingDown.getDoubleValue();
   }
}
