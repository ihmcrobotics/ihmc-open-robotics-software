package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class LIPMWalkerController implements RobotController
{
   private final LIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   
   
   private final YoDouble kpBodyPitch = new YoDouble("kpBodyPitch", registry);
   private final YoDouble kdBodyPitch = new YoDouble("kdBodyPitch", registry);
   private final YoDouble desiredBodyPitch = new YoDouble("desiredBodyPitch", registry);
   
   private final YoDouble kpKnee = new YoDouble("kpKnee", registry);
   private final YoDouble kdKnee = new YoDouble("kdKnee", registry);
   private final YoDouble kpHip = new YoDouble("kpHip", registry);
   private final YoDouble kdHip = new YoDouble("kdHip", registry);

   private final YoDouble comXVelocity = new YoDouble("comXVelocity", registry);
   private final YoDouble comXPositionFromFoot = new YoDouble("comXPositionFromFoot", registry);
   
   private final YoDouble q_d_leftKnee = new YoDouble("q_d_leftKnee", registry);
   private final YoDouble q_d_rightKnee = new YoDouble("q_d_rightKnee", registry);
   private final YoDouble q_d_leftHip = new YoDouble("q_d_leftHip", registry);
   private final YoDouble q_d_rightHip = new YoDouble("q_d_rightHip", registry);

   private final YoDouble q_rightHipWorld = new YoDouble("q_rightHipWorld", registry);
   private final YoDouble q_leftHipWorld = new YoDouble("q_leftHipWorld", registry);

   private final YoDouble comHeight = new YoDouble("comHeight", registry);

   private final SideDependentList<YoDouble> desiredKneeLengths = new SideDependentList<YoDouble>(q_d_leftKnee, q_d_rightKnee);
   private final SideDependentList<YoDouble> desiredHipAngles = new SideDependentList<YoDouble>(q_d_leftHip, q_d_rightHip);
   private final SideDependentList<YoDouble> worldHipAngles = new SideDependentList<YoDouble>(q_leftHipWorld, q_rightHipWorld);

   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);

   private final YoDouble orbitalEnergy = new YoDouble("orbitalEnergy", registry);

   private final YoDouble hipDiffAngle = new YoDouble("hipDiffAngle", registry);


   private enum States
   {
      SUPPORT, SWING;
   }

   private final SideDependentList<StateMachine<States, State>> stateMachines;

   private final double g  = 9.81;

   public LIPMWalkerController(LIPMWalkerRobot robot)
   {
      this.robot = robot;

      initialize();

      stateMachines = setupStateMachines();
   }

   @Override
   public void initialize()
   {
      kpBodyPitch.set(1000.0);
      kdBodyPitch.set(100.0);
      desiredBodyPitch.set(0.0);
      
      kpKnee.set(30000.0);
      kdKnee.set(1000.0);

      kpHip.set(971.25);
      kdHip.set(80.0);

      q_d_leftKnee.set(1.0);
      q_d_rightKnee.set(1.0);

      q_d_leftHip.set(0.0);
      q_d_rightHip.set(0.0);

      desiredHeight.set(1.0);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      for(RobotSide robotSide : RobotSide.values())
      {
         stateMachines.get(robotSide).doAction();
         stateMachines.get(robotSide).doTransitions();
      }

      LogTools.info("StateMachine: Left:{} Right:{}", stateMachines.get(RobotSide.LEFT).getCurrentStateKey(), stateMachines.get(RobotSide.RIGHT).getCurrentStateKey());
   }

   private void controlSupportLeg(RobotSide side)
   {
      double kneeLength = robot.getKneeLength(side);
      double kneeVelocity = robot.getKneeVelocity(side);

      //         double desiredKneeLength = desiredKneeLengths.get(side).getValue();

      Point3DReadOnly centerOfMassPosition = robot.getCenterOfMassPosition();
      Vector3DReadOnly centerOfMassVelocity = robot.getCenterOfMassVelocity();
      double mass = robot.getMass();

      /* Compute and set orbital energy YoDouble. Based on virtual spring-mass system. */
      comXVelocity.set(robot.getCenterOfMassVelocity().getX());
      comXPositionFromFoot.set(robot.getCenterOfMassXDistanceFromSupportFoot());

      double orbitalEnergyValue = 0.5 * comXVelocity.getValue() * comXVelocity.getValue() - 0.5 * g / desiredHeight.getDoubleValue() * comXPositionFromFoot.getValue() * comXPositionFromFoot.getValue();
      orbitalEnergy.set(orbitalEnergyValue);

      comHeight.set(centerOfMassPosition.getZ());

      double feedForwardSupportKneeForce = g * mass * kneeLength / centerOfMassPosition.getZ();
      double feedBackKneeForce =
            kpKnee.getValue() * (desiredHeight.getValue() - comHeight.getValue()) + kdKnee.getValue() * (0.0 - centerOfMassVelocity.getZ());
      double totalSupportKneeForce = feedForwardSupportKneeForce + feedBackKneeForce;
      if (totalSupportKneeForce < 10.0)
         totalSupportKneeForce = 10.0;
      
      robot.setKneeForce(side, totalSupportKneeForce);
      robot.setHipTorque(side, -kpBodyPitch.getValue() * (desiredBodyPitch.getValue() - robot.getBodyPitchAngle()) + kdBodyPitch.getValue() * robot.getBodyPitchAngularVelocity());

      worldHipAngles.get(side).set(robot.getHipAngle(side) + robot.getBodyPitchAngle());
   }

   private void controlSwingLeg(RobotSide side, Vector3DReadOnly footLocation)
   {
      double kneeLength;
      double kneeVelocity;
      double feedBackKneeForce;
      double desiredHipAngle;

      double hipAngle = robot.getHipAngle(side) + robot.getBodyPitchAngle();
      worldHipAngles.get(side).set(hipAngle);
      double supportHipAngle = robot.getHipAngle(side.getOppositeSide()) + robot.getBodyPitchAngle();
      double hipVelocity = robot.getHipVelocity(side);

      double desiredKneeLength = EuclidCoreTools.squareRoot(footLocation.lengthSquared() / 4 + desiredHeight.getValue() * desiredHeight.getValue());

//      desiredHipAngle = - EuclidCoreTools.atan2(footLocation.length() / 2, desiredHeight.getValue());
      desiredHipAngle = -1.0;

      /* Compute knee force. */
      kneeLength = robot.getKneeLength(side);
      kneeVelocity = robot.getKneeVelocity(side);

      if(hipAngle > -0.05)
         desiredKneeLength = 0.70;
      else if(hipAngle < -0.1)
         desiredKneeLength = 1.0425;


      feedBackKneeForce = 10 * (desiredKneeLength - kneeLength) + 5 * (0.0 - kneeVelocity);
      robot.setKneeForce(side, feedBackKneeForce);

      /* Compute hip torque. */
      desiredHipAngles.get(side).set(desiredHipAngle);
      double feedBackHipTorque = kpHip.getValue() * (desiredHipAngle - hipAngle) + kdHip.getValue() * (0.0 - hipVelocity);
      
      robot.setHipTorque(side, feedBackHipTorque);
   }

   Vector3DReadOnly calculateStepLocation()
   {
      Vector3DReadOnly footLocation = new Vector3D(1.0, 0.0, 0.0);
      return footLocation;
   }

   private SideDependentList<StateMachine<States, State>> setupStateMachines()
   {
      // States and Actions:
      StateMachineFactory<States, State> leftFactory = new StateMachineFactory<>(States.class);
      StateMachineFactory<States, State> rightFactory = new StateMachineFactory<>(States.class);

      leftFactory.setNamePrefix("leftState");
      rightFactory.setNamePrefix("rightState");
      
      leftFactory.setRegistry(registry);
      rightFactory.setRegistry(registry);
      
      leftFactory.buildClock(robot.getRobot().getYoTime());
      rightFactory.buildClock(robot.getRobot().getYoTime());
 
      // Left State Transitions:
      leftFactory.addTransition(States.SUPPORT, States.SWING, new HeelOffGroundCondition(RobotSide.LEFT));
      leftFactory.addTransition(States.SWING, States.SUPPORT, new HeelOnGroundCondition(RobotSide.LEFT));

      // Right State Transitions:
      rightFactory.addTransition(States.SUPPORT, States.SWING, new HeelOffGroundCondition(RobotSide.RIGHT));
      rightFactory.addTransition(States.SWING, States.SUPPORT, new HeelOnGroundCondition(RobotSide.RIGHT));

      // Assemble the Left State Machine:
      leftFactory.addState(States.SUPPORT, new SupportState(RobotSide.LEFT));
      leftFactory.addState(States.SWING, new SwingState(RobotSide.LEFT));

      // Assemble the Right State Machine:
      rightFactory.addState(States.SUPPORT, new SupportState(RobotSide.RIGHT));
      rightFactory.addState(States.SWING, new SwingState(RobotSide.RIGHT));

      return new SideDependentList<>(leftFactory.build(States.SUPPORT), rightFactory.build(States.SWING));
   }

   private class SupportState implements State
   {
      private final RobotSide robotSide;

      public SupportState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public void onEntry()
      {

      }

      @Override
      public void doAction(double timeInState)
      {
         // Support Side:
         controlSupportLeg(robotSide);
      }

      @Override
      public void onExit(double timeInState)
      {

      }
   }

   private class SwingState implements State
   {
      private final RobotSide robotSide;

      public SwingState(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public void onEntry()
      {

      }

      @Override
      public void doAction(double timeInState)
      {
         Vector3DReadOnly footLocation = calculateStepLocation();
         controlSwingLeg(robotSide, footLocation);
      }

      @Override
      public void onExit(double timeInState)
      {

      }
   }

   private class HeelOffGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOffGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         return robot.getKneeForce(robotSide) < 10.0 && worldHipAngles.get(robotSide).getValue() > 0.1;
      }
   }

   private class HeelOnGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOnGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         LogTools.info("Side: {} getKneeForce(): {}", robotSide, robot.getKneeForce(robotSide));
         return robot.getKneeForce(robotSide) > 0.0 && worldHipAngles.get(robotSide).getValue() < -0.1;
      }
   }

}
