package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedFootStateMachine.FootEvent;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineState;

public class WDIPeterPlanarWalkerController implements RobotController
{
   private final double MAX_HIP_ANGLE = 0.8;
   private double HIP_DEFUALT_P_GAIN = 100.0;
   private double HIP_DEFUALT_D_GAIN = 10.0;

   private double KNEE_DEFUALT_P_GAIN = 10000.0;
   private double KNEE_DEFUALT_D_GAIN = 1000.0;

   private double deltaT;

   private PeterPlanarWalkerRobot robot;
   private YoVariableRegistry registry = new YoVariableRegistry("Controller");
   private SideDependentList<PIDController> kneeControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> hipControllers = new SideDependentList<PIDController>();

   private DoubleYoVariable desiredKneeExtension = new DoubleYoVariable("desiredKneeExtension", registry);
   private DoubleYoVariable desiredPitch = new DoubleYoVariable("desiredPitch", registry);
   private DoubleYoVariable desiredHeight = new DoubleYoVariable("desiredHeight", registry);
   private DoubleYoVariable swingTime = new DoubleYoVariable("swingTime", registry);
   private DoubleYoVariable desiredSwingLegHipAngle = new DoubleYoVariable("desiredSwingLegHipAngle", registry);
   private DoubleYoVariable scaleForVelToAngle = new DoubleYoVariable("scaleForVelToAngle", registry);
   private DoubleYoVariable desiredKneeStance = new DoubleYoVariable("desiredKneeStance", registry);
   private DoubleYoVariable angleForCapture = new DoubleYoVariable("angleForCapture", registry);
   private DoubleYoVariable feedForwardAngle = new DoubleYoVariable("feedForwardAngle", registry);
   private DoubleYoVariable velocityErrorAngle = new DoubleYoVariable("velocityErrorAngle", registry);
   private DoubleYoVariable feedForwardGain = new DoubleYoVariable("feedForwardGain", registry);
   private DoubleYoVariable lastStepHipAngle = new DoubleYoVariable("lastStepHipAngle", registry);
   private DoubleYoVariable stepToStepHipAngleDelta = new DoubleYoVariable("stepToStepHipAngleDelta", registry);

   private DoubleYoVariable swingTimeForThisStep = new DoubleYoVariable("swingTimeForThisStep", registry);
   private BooleanYoVariable initalizedKneeExtension = new BooleanYoVariable("initalizedKneeExtension", registry);
   private BooleanYoVariable initalizedKneeDoubleExtension = new BooleanYoVariable("initalizedKneeDoubleExtension", registry);

   private DoubleYoVariable kneeMoveStartTime = new DoubleYoVariable("kneeMoveStartTime", registry);
   private DoubleYoVariable startingHipAngle = new DoubleYoVariable("startingHipAngle", registry);
   private DoubleYoVariable maxVelocityErrorAngle = new DoubleYoVariable("maxVelocityErrorAngle", registry);

   private DoubleYoVariable desiredBodyVelocity = new DoubleYoVariable("desiredBodyVelocity", registry);
   private DoubleYoVariable alphaFilterVariable = new DoubleYoVariable("alphaFilterVariable", registry);
   private AlphaFilteredYoVariable filteredDesiredVelocity = new AlphaFilteredYoVariable("filteredDesiredVelocity", registry, alphaFilterVariable,
                                                                                         desiredBodyVelocity);

   private YoMinimumJerkTrajectory trajectorySwingHip;
   private YoMinimumJerkTrajectory trajectorySwingKnee;

   private EnumYoVariable<RobotSide> swingLeg = new EnumYoVariable<RobotSide>("swingLeg", registry, RobotSide.class);

   private FiniteStateMachineBuilder<ControllerState, ControllerEvent> stateMachineBuilder;
   private FiniteStateMachine<ControllerState, ControllerEvent> stateMachine;
   private final YoVariable<?> timestamp;
   private final YoPlanarWalkerTimedStep stepCommand;

   public WDIPeterPlanarWalkerController(PeterPlanarWalkerRobot robot, double deltaT, RobotSide side, YoVariable<?> timestamp)
   {
      String prefix = side.getLowerCaseName();
      this.robot = robot;
      this.deltaT = deltaT;
      this.timestamp = timestamp;
      this.stepCommand = new YoPlanarWalkerTimedStep(prefix + "StepCommand", registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         PIDController pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_Knee", registry);
         pidController.setProportionalGain(KNEE_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(KNEE_DEFUALT_D_GAIN);
         kneeControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_Hip", registry);
         pidController.setProportionalGain(HIP_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(HIP_DEFUALT_D_GAIN);
         hipControllers.put(robotSide, pidController);
      }

      trajectorySwingHip = new YoMinimumJerkTrajectory("trajectorySwingHip", registry);
      trajectorySwingKnee = new YoMinimumJerkTrajectory("trajectorySwingKnee", registry);
      desiredHeight.set(robot.nominalHeight);
      desiredKneeStance.set(robot.lowerLinkLength / 2.0);
      swingTime.set(0.3);
      scaleForVelToAngle.set(0.9);
      feedForwardGain.set(0.05);
      stepToStepHipAngleDelta.set(0.3);
      maxVelocityErrorAngle.set(0.3);
      alphaFilterVariable.set(0.9999);

      initializeStateMachine(side);
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub
   }

   private void initializeStateMachine(RobotSide supportLegSide)
   {
      stateMachineBuilder = new FiniteStateMachineBuilder(ControllerState.class, ControllerEvent.class, "stateMachineBuilder", registry);

      stateMachineBuilder.addState(ControllerState.SUPPORT, new SupportState(supportLegSide));
      stateMachineBuilder.addState(ControllerState.SWING, new SwingState(supportLegSide));
      stateMachineBuilder.addTransition(ControllerEvent.TIMEOUT, ControllerState.SUPPORT, ControllerState.SWING);
      stateMachineBuilder.addTransition(ControllerEvent.TIMEOUT, ControllerState.SWING, ControllerState.SUPPORT);

      stateMachine = stateMachineBuilder.build(ControllerState.SUPPORT);
   }

   private class SupportState implements FiniteStateMachineState<ControllerEvent>
   {
      private RobotSide supportLeg;

      public SupportState(RobotSide robotSide)
      {
         supportLeg = robotSide;
      }

      @Override
      public void onEntry()
      {
      }

      @Override
      public ControllerEvent process()
      {
         controlHipToMaintainPitch(supportLeg);

         //add swing leg torque to stand leg
         addOppositeLegHipTorque(supportLeg);

         //controlKneeToMaintainBodyHeight(supportLeg);
         controlKneeToPosition(supportLeg, desiredKneeStance.getDoubleValue(), 0.0);

//         if (initalizedKneeExtension.getBooleanValue() && robot.isFootOnGround(swingLeg.getEnumValue()))
//         {
            return ControllerEvent.TIMEOUT;
//         }
//         else
//         {
//            return null;
//         }
      }

      @Override
      public void onExit()
      {
      }
   }

   private class SwingState implements FiniteStateMachineState<ControllerEvent>
   {
      private RobotSide swingLeg;

      public SwingState(RobotSide robotSide)
      {
         swingLeg = robotSide;
      }

      @Override
      public void onEntry()
      {
      }

      @Override
      public ControllerEvent process()
      {
         double currentTime = timestamp.getValueAsDouble();
         double liftOffTime = stepCommand.getTimeInterval().getStartTime();
         double touchDownTime = stepCommand.getTimeInterval().getEndTime();
         double timeInCurrentState = currentTime - liftOffTime;

         if ((timeInCurrentState > swingTimeForThisStep.getDoubleValue() / 2.0) && !initalizedKneeExtension.getBooleanValue())
         {
            double currentKneePosition = robot.getKneePosition(swingLeg);
            trajectorySwingKnee.setParams(currentKneePosition, 0.0, 0.0, desiredKneeStance.getDoubleValue(), 0.0, 0.0, 0.0,
                                          swingTimeForThisStep.getDoubleValue() / 2.0);
            initalizedKneeExtension.set(true);
            kneeMoveStartTime.set(timeInCurrentState);
         }
         else if ((timeInCurrentState > swingTimeForThisStep.getDoubleValue() && !initalizedKneeDoubleExtension.getBooleanValue()))
         {
            double currentKneePosition = robot.getKneePosition(swingLeg);
            trajectorySwingKnee.setParams(currentKneePosition, 0.0, 0.0, desiredKneeStance.getDoubleValue() + 0.5, 0.0, 0.0, 0.0, 0.125);
            initalizedKneeDoubleExtension.set(true);
            kneeMoveStartTime.set(timeInCurrentState);
         }

         trajectorySwingKnee.computeTrajectory(timeInCurrentState - kneeMoveStartTime.getDoubleValue());
         double desiredKneePositon = trajectorySwingKnee.getPosition();
         double desiredKneeVelocity = trajectorySwingKnee.getVelocity();
         controlKneeToPosition(swingLeg, desiredKneePositon, desiredKneeVelocity);

         desiredSwingLegHipAngle.set(getDesireHipAngle());
         trajectorySwingHip.setParams(startingHipAngle.getDoubleValue(), 0.0, 0.0, desiredSwingLegHipAngle.getDoubleValue(), 0.0, 0.0, 0.0,
                                      swingTimeForThisStep.getDoubleValue());

         trajectorySwingHip.computeTrajectory(timeInCurrentState);
         double desiredHipAngle = trajectorySwingHip.getPosition();
         double currentHipAngle = robot.getHipPosition(swingLeg);
         double currentHipAngleRate = robot.getHipVelocity(swingLeg);

         PIDController pidController = hipControllers.get(swingLeg);
         double controlEffort = pidController.compute(currentHipAngle, desiredHipAngle, currentHipAngleRate, 0.0, deltaT);
         robot.setHipTorque(swingLeg, controlEffort);

         if (currentTime >= touchDownTime)
         {
            return ControllerEvent.TIMEOUT;
         }
         else
            return null;
      }

      @Override
      public void onExit()
      {
      }
   }

   private double getDesireHipAngle()
   {
      double legLength = robot.upperLinkLength + desiredKneeStance.getDoubleValue();
      angleForCapture.set(HipAngleCapturePointCalculator.getHipAngle(robot.getBodyVelocity(), legLength));
      angleForCapture.set(-angleForCapture.getDoubleValue() * Math.signum(robot.getBodyVelocity()));

      //only use a fraction of it
      //angleForCapture.set(0.8 *angleForCapture.getDoubleValue());

      //limit this angle
      angleForCapture.set(MathTools.clipToMinMax(angleForCapture.getDoubleValue(), MAX_HIP_ANGLE));

      //angle is opposite sign of desired velocity
      double velocityError = (filteredDesiredVelocity.getDoubleValue() - robot.getBodyVelocity());
      velocityErrorAngle.set(velocityError * scaleForVelToAngle.getDoubleValue());
      velocityErrorAngle.set(MathTools.clipToMinMax(velocityErrorAngle.getDoubleValue(), maxVelocityErrorAngle.getDoubleValue()));

      feedForwardAngle.set(filteredDesiredVelocity.getDoubleValue() * feedForwardGain.getDoubleValue());
      double angle = angleForCapture.getDoubleValue() + feedForwardAngle.getDoubleValue() + velocityErrorAngle.getDoubleValue();

      angle = MathTools.clipToMinMax(angle, lastStepHipAngle.getDoubleValue() - stepToStepHipAngleDelta.getDoubleValue(),
                                     lastStepHipAngle.getDoubleValue() + stepToStepHipAngleDelta.getDoubleValue());
      return angle;
   }

   private void controlHipToMaintainPitch(RobotSide robotSide)
   {
      double currentPitch = robot.getBodyPitch();
      double currentPitchRate = robot.getBodyPitchVelocity();

      double controlEffort = -hipControllers.get(robotSide).compute(currentPitch, desiredPitch.getDoubleValue(), currentPitchRate, 0.0, deltaT);
      robot.setHipTorque(robotSide, controlEffort);
   }

   private void addOppositeLegHipTorque(RobotSide legToAddTorque)
   {
      double oppositeLegTorque = robot.getHipTorque(legToAddTorque.getOppositeSide());
      double currentTorque = robot.getHipTorque(legToAddTorque);
      robot.setHipTorque(legToAddTorque, currentTorque - oppositeLegTorque);
   }

   private void controlKneeToMaintainBodyHeight(RobotSide robotSide)
   {
      double currentHeight = robot.getBodyHeight();
      double currentHeightRate = robot.getBodyHeightVelocity();

      double controlEffort = kneeControllers.get(robotSide).compute(currentHeight, desiredHeight.getDoubleValue(), currentHeightRate, 0.0, deltaT);
      robot.setKneeTorque(robotSide, controlEffort);
   }

   private void controlKneeToPosition(RobotSide robotSide, double desiredPosition, double desiredVelocity)
   {
      double kneePosition = robot.getKneePosition(robotSide);
      double kneePositionRate = robot.getKneeVelocity(robotSide);

      double controlEffort = kneeControllers.get(robotSide).compute(kneePosition, desiredPosition, kneePositionRate, desiredVelocity, deltaT);
      robot.setKneeTorque(robotSide, controlEffort);
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   @Override
   public void doControl()
   {
      //      for (RobotSide robotSide : RobotSide.values)
      //      {
      //         double currentPosition = robot.getKneePosition(robotSide);
      //         double currentVelocity = robot.getKneeVelocity(robotSide);
      //
      //         double effort = kneeControllers.get(robotSide).compute(currentPosition, desiredKneeExtension.getDoubleValue(), currentVelocity, 0.0, deltaT);
      //         robot.setKneeTorque(robotSide, effort);
      //      }

      filteredDesiredVelocity.update();
      stateMachine.process();
   }

   public enum ControllerState
   {
      SUPPORT, SWING;
   }

   public enum ControllerEvent
   {
      TIMEOUT;
   }

}
