package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class PeterPlanarWalkerController implements RobotController
{
   private double DEFUALT_P_GAIN = 10000.0;
   private double DEFUALT_D_GAIN = 100.0;

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
   private DoubleYoVariable desiredBodyVelocity = new DoubleYoVariable("desiredBodyVelocity", registry);
   private DoubleYoVariable scaleForVelToAngle = new DoubleYoVariable("scaleForVelToAngle", registry);

   private MinimumJerkTrajectory trajectorySwingHip;

   private EnumYoVariable<RobotSide> swingLeg = new EnumYoVariable<RobotSide>("swingLeg", registry, RobotSide.class);

   private StateMachine<ControllerState> stateMachine;

   public PeterPlanarWalkerController(PeterPlanarWalkerRobot robot, double deltaT)
   {
      this.robot = robot;
      this.deltaT = deltaT;

      for (RobotSide robotSide : RobotSide.values)
      {
         PIDController pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_Knee", registry);
         pidController.setProportionalGain(DEFUALT_P_GAIN);
         pidController.setDerivativeGain(DEFUALT_D_GAIN);
         kneeControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_Hip", registry);
         pidController.setProportionalGain(DEFUALT_P_GAIN);
         pidController.setDerivativeGain(DEFUALT_D_GAIN);
         hipControllers.put(robotSide, pidController);
      }

      trajectorySwingHip = new MinimumJerkTrajectory();
      desiredHeight.set(robot.nominalHeight);

      swingTime.set(0.3);
      scaleForVelToAngle.set(0.5);

      initializeStateMachine();
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   private void initializeStateMachine()
   {

      stateMachine = new StateMachine<PeterPlanarWalkerController.ControllerState>("controllerState", "switchTime", ControllerState.class, robot.getYoTime(),
            registry);

      SingleSupportState leftSupportState = new SingleSupportState(ControllerState.LEFT_SUPPORT);
      SingleSupportState rightSupportState = new SingleSupportState(ControllerState.RIGHT_SUPPORT);

      leftSupportState.setDefaultNextState(rightSupportState.getStateEnum());
      rightSupportState.setDefaultNextState(leftSupportState.getStateEnum());

      stateMachine.addState(leftSupportState);
      stateMachine.addState(rightSupportState);
      stateMachine.setCurrentState(leftSupportState.getStateEnum());
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
         controlHipToMaintainPitch(supportLeg);
         contorlKneeToMaintainBodyHeight(supportLeg);
         
         trajectorySwingHip.computeTrajectory(this.getTimeInCurrentState());
         double desiredHipAngle = trajectorySwingHip.getPosition();
         double currentHipAngle = robot.getHipPosition(swingLeg.getEnumValue());
         double currentHipAngleRate = robot.getHipVelocity(swingLeg.getEnumValue());
         
         PIDController pidController = hipControllers.get(swingLeg.getEnumValue());
         double controlEffort = pidController.compute(currentHipAngle, desiredHipAngle, currentHipAngleRate, 0.0, deltaT);
         robot.setHipTorque(swingLeg.getEnumValue(), controlEffort);
      }

      @Override
      public void doTransitionIntoAction()
      {
         swingLeg.set(supportLeg.getOppositeSide());

         //angle is opposite sign of desired velocity
         double velocityError = -(robot.getBodyVelocity() - desiredBodyVelocity.getDoubleValue());
         double angle = -desiredBodyVelocity.getDoubleValue() + velocityError * scaleForVelToAngle.getDoubleValue();
         desiredSwingLegHipAngle.set(angle);

         double currentAngle = robot.getHipPosition(swingLeg.getEnumValue());

         trajectorySwingHip.setMoveParameters(currentAngle, 0.0, 0.0, desiredSwingLegHipAngle.getDoubleValue(), 0.0, 0.0, swingTime.getDoubleValue());

         //retract knee
         robot.setKneeTorque(swingLeg.getEnumValue(), -10.0);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         
         
      }
   }
   
  

   private void controlHipToMaintainPitch(RobotSide robotSide)
   {
      double currentPitch = robot.getBodyPitch();
      double currentPitchRate = robot.getBodyPitchVelocity();

      double controlEffort = -hipControllers.get(robotSide).compute(currentPitch, desiredPitch.getDoubleValue(), currentPitchRate, 0.0, deltaT);
      robot.setHipTorque(robotSide, controlEffort);
   }

   private void contorlKneeToMaintainBodyHeight(RobotSide robotSide)
   {
      double currentHeight = robot.getBodyHeight();
      double currentHeightRate = robot.getBodyHeightVelocity();

      double controlEffort = kneeControllers.get(robotSide).compute(currentHeight, desiredHeight.getDoubleValue(), currentHeightRate, 0.0, deltaT);
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
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public String getDescription()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void doControl()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         double currentPosition = robot.getKneePosition(robotSide);
         double currentVelocity = robot.getKneeVelocity(robotSide);

         double effort = kneeControllers.get(robotSide).compute(currentPosition, desiredKneeExtension.getDoubleValue(), currentVelocity, 0.0, deltaT);
         robot.setKneeTorque(robotSide, effort);
      }

      stateMachine.doAction();

      stateMachine.checkTransitionConditions();

   }

   public enum ControllerState
   {
      LEFT_SUPPORT, RIGHT_SUPPORT;
   }

}
