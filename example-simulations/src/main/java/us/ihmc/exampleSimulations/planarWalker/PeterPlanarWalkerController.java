package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;

public class PeterPlanarWalkerController implements RobotController
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

   private YoDouble desiredKneeExtension = new YoDouble("desiredKneeExtension", registry);
   private YoDouble desiredPitch = new YoDouble("desiredPitch", registry);
   private YoDouble desiredHeight = new YoDouble("desiredHeight", registry);
   private YoDouble swingTime = new YoDouble("swingTime", registry);
   private YoDouble desiredSwingLegHipAngle = new YoDouble("desiredSwingLegHipAngle", registry);
   private YoDouble scaleForVelToAngle = new YoDouble("scaleForVelToAngle", registry);
   private YoDouble desiredKneeStance = new YoDouble("desiredKneeStance", registry);
   private YoDouble angleForCapture = new YoDouble("angleForCapture", registry);
   private YoDouble feedForwardAngle = new YoDouble("feedForwardAngle", registry);
   private YoDouble velocityErrorAngle = new YoDouble("velocityErrorAngle", registry);
   private YoDouble feedForwardGain = new YoDouble("feedForwardGain", registry);
   private YoDouble lastStepHipAngle = new YoDouble("lastStepHipAngle", registry);
   private YoDouble stepToStepHipAngleDelta = new YoDouble("stepToStepHipAngleDelta", registry);

   private YoDouble swingTimeForThisStep = new YoDouble("swingTimeForThisStep", registry);
   private YoBoolean initalizedKneeExtension = new YoBoolean("initalizedKneeExtension", registry);
   private YoBoolean initalizedKneeDoubleExtension = new YoBoolean("initalizedKneeDoubleExtension", registry);

   private YoDouble kneeMoveStartTime = new YoDouble("kneeMoveStartTime", registry);
   private YoDouble startingHipAngle = new YoDouble("startingHipAngle", registry);
   private YoDouble timeInState = new YoDouble("timeInState", registry);
   private YoDouble maxVelocityErrorAngle = new YoDouble("maxVelocityErrorAngle", registry);

   private YoDouble desiredBodyVelocity = new YoDouble("desiredBodyVelocity", registry);
   private YoDouble alphaFilterVariable = new YoDouble("alphaFilterVariable", registry);
   private AlphaFilteredYoVariable filteredDesiredVelocity = new AlphaFilteredYoVariable("filteredDesiredVelocity", registry, alphaFilterVariable, desiredBodyVelocity);
   
   
   private YoMinimumJerkTrajectory trajectorySwingHip;
   private YoMinimumJerkTrajectory trajectorySwingKnee;
    
   private YoEnum<RobotSide> swingLeg = new YoEnum<RobotSide>("swingLeg", registry, RobotSide.class);

   private StateMachine<ControllerState> stateMachine;

   public PeterPlanarWalkerController(PeterPlanarWalkerRobot robot, double deltaT)
   {
      this.robot = robot;
      this.deltaT = deltaT;

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
      desiredKneeStance.set(robot.lowerLinkLength/2.0);
      swingTime.set(0.3);
      scaleForVelToAngle.set(0.9);
      feedForwardGain.set(0.05);
      stepToStepHipAngleDelta.set(0.3);
      maxVelocityErrorAngle.set(0.3);
      alphaFilterVariable.set(0.9999);
      
      initializeStateMachine();
   }

   @Override
   public void initialize()
   {

   }

   private void initializeStateMachine()
   {

      stateMachine = new StateMachine<PeterPlanarWalkerController.ControllerState>("controllerState", "switchTime", ControllerState.class, robot.getYoTime(),
            registry);

      SingleSupportState leftSupportState = new SingleSupportState(ControllerState.LEFT_SUPPORT);
      SingleSupportState rightSupportState = new SingleSupportState(ControllerState.RIGHT_SUPPORT);
      StartState startState = new StartState(ControllerState.START);

      startState.setDefaultNextState(rightSupportState.getStateEnum());
      leftSupportState.setDefaultNextState(rightSupportState.getStateEnum());
      rightSupportState.setDefaultNextState(leftSupportState.getStateEnum());

      stateMachine.addState(startState);
      stateMachine.addState(leftSupportState);
      stateMachine.addState(rightSupportState);
      stateMachine.setCurrentState(startState.getStateEnum());
   }

   private class StartState extends State<ControllerState>
   {

      public StartState(ControllerState stateEnum)
      {
         super(stateEnum);
         // TODO Auto-generated constructor stub
      }

      @Override
      public void doAction()
      {
         this.transitionToDefaultNextState();
      }

      @Override
      public void doTransitionIntoAction()
      {
         
      }

      @Override
      public void doTransitionOutOfAction()
      {
         
      }
      
   }
   
   private class SingleSupportState extends State<ControllerState>
   {
      private RobotSide supportLeg;
     
      public SingleSupportState(ControllerState controllerState)
      {
         super(controllerState);

         if (controllerState.equals(ControllerState.LEFT_SUPPORT))
            supportLeg = RobotSide.LEFT;
         else
            supportLeg = RobotSide.RIGHT;
      }

      @Override
      public void doAction()
      {
         timeInState.set(this.getTimeInCurrentState());
         
       

         //Swing Leg
         if ((this.getTimeInCurrentState() > swingTimeForThisStep.getDoubleValue()/2.0) && !initalizedKneeExtension.getBooleanValue())
         {
            double currentKneePosition = robot.getKneePosition(swingLeg.getEnumValue());
            trajectorySwingKnee.setParams(currentKneePosition, 0.0, 0.0, desiredKneeStance.getDoubleValue(), 0.0, 0.0, 0.0, swingTimeForThisStep.getDoubleValue()/2.0);
            initalizedKneeExtension.set(true);
            kneeMoveStartTime.set(this.getTimeInCurrentState());
         }
         else if((this.getTimeInCurrentState() > swingTimeForThisStep.getDoubleValue() && !initalizedKneeDoubleExtension.getBooleanValue()))
         {
            double currentKneePosition = robot.getKneePosition(swingLeg.getEnumValue());
            trajectorySwingKnee.setParams(currentKneePosition, 0.0, 0.0, desiredKneeStance.getDoubleValue() + 0.5, 0.0, 0.0, 0.0, 0.125);
            initalizedKneeDoubleExtension.set(true);
            kneeMoveStartTime.set(this.getTimeInCurrentState());
         }
         
         trajectorySwingKnee.computeTrajectory(this.getTimeInCurrentState() - kneeMoveStartTime.getDoubleValue());
         double desiredKneePositon = trajectorySwingKnee.getPosition();
         double desiredKneeVelocity = trajectorySwingKnee.getVelocity();
         controlKneeToPosition(swingLeg.getEnumValue(), desiredKneePositon, desiredKneeVelocity);

         desiredSwingLegHipAngle.set(getDesireHipAngle());
         trajectorySwingHip.setParams(startingHipAngle.getDoubleValue(), 0.0, 0.0, desiredSwingLegHipAngle.getDoubleValue(), 0.0, 0.0, 0.0, swingTimeForThisStep.getDoubleValue());
         
         trajectorySwingHip.computeTrajectory(this.getTimeInCurrentState());
         double desiredHipAngle = trajectorySwingHip.getPosition();
         double currentHipAngle = robot.getHipPosition(swingLeg.getEnumValue());
         double currentHipAngleRate = robot.getHipVelocity(swingLeg.getEnumValue());
         
         PIDController pidController = hipControllers.get(swingLeg.getEnumValue());
         double controlEffort = pidController.compute(currentHipAngle, desiredHipAngle, currentHipAngleRate, 0.0, deltaT);
         robot.setHipTorque(swingLeg.getEnumValue(), controlEffort);
         
         
         //Stance leg
         controlHipToMaintainPitch(supportLeg);
         
         //add swing leg torque to stand leg
         addOppositeLegHipTorque(supportLeg);
         
         //controlKneeToMaintainBodyHeight(supportLeg);
         controlKneeToPosition(supportLeg, desiredKneeStance.getDoubleValue(), 0.0);
         
         if (initalizedKneeExtension.getBooleanValue() && robot.isFootOnGround(swingLeg.getEnumValue()))
            this.transitionToDefaultNextState();
      }

      @Override
      public void doTransitionIntoAction()
      {
         swingLeg.set(supportLeg.getOppositeSide());
         swingTimeForThisStep.set(swingTime.getDoubleValue());
         initalizedKneeExtension.set(false);
         initalizedKneeDoubleExtension.set(false);
         kneeMoveStartTime.set(0.0);
         
         startingHipAngle.set(robot.getHipPosition(swingLeg.getEnumValue()));
         
         double currentKneePosition = robot.getKneePosition(swingLeg.getEnumValue());
         double desiredRetractedPosition = 0.1;
         trajectorySwingKnee.setParams(currentKneePosition, 0.0, 0.0, desiredRetractedPosition, 0.0, 0.0, 0.0, swingTimeForThisStep.getDoubleValue()/2.0);
         
         
         //retract knee
         robot.setKneeTorque(swingLeg.getEnumValue(), -10.0);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         lastStepHipAngle.set(desiredSwingLegHipAngle.getDoubleValue());
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
      angleForCapture.set(MathTools.clamp(angleForCapture.getDoubleValue(), MAX_HIP_ANGLE));
      
      //angle is opposite sign of desired velocity
      double velocityError = (filteredDesiredVelocity.getDoubleValue() - robot.getBodyVelocity());
      velocityErrorAngle.set(velocityError * scaleForVelToAngle.getDoubleValue());
      velocityErrorAngle.set(MathTools.clamp(velocityErrorAngle.getDoubleValue(), maxVelocityErrorAngle.getDoubleValue()));

      
      
      feedForwardAngle.set(filteredDesiredVelocity.getDoubleValue() * feedForwardGain.getDoubleValue());
      double angle = angleForCapture.getDoubleValue() + feedForwardAngle.getDoubleValue() + velocityErrorAngle.getDoubleValue();
    
      angle = MathTools.clamp(angle, lastStepHipAngle.getDoubleValue() - stepToStepHipAngleDelta.getDoubleValue(), lastStepHipAngle.getDoubleValue() + stepToStepHipAngleDelta.getDoubleValue());
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
      stateMachine.doAction();

      stateMachine.checkTransitionConditions();

   }

   public enum ControllerState
   {
      START, LEFT_SUPPORT, RIGHT_SUPPORT;
   }

}
