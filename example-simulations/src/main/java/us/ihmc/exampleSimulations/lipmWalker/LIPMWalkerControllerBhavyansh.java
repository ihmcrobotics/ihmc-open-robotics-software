package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

public class LIPMWalkerControllerBhavyansh implements RobotController
{
   private final LIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private YoDouble t;

   private final YoDouble kpKnee = new YoDouble("kpKnee", registry);
   private final YoDouble kdKnee = new YoDouble("kdKnee", registry);
   private final YoDouble kpHip = new YoDouble("kpHip", registry);
   private final YoDouble kdHip = new YoDouble("kdHip", registry);

   private final YoDouble q_d_leftKnee = new YoDouble("q_d_leftKnee", registry);
   private final YoDouble q_d_rightKnee = new YoDouble("q_d_rightKnee", registry);
   private final YoDouble q_d_leftHip = new YoDouble("q_d_leftHip", registry);
   private final YoDouble q_d_rightHip = new YoDouble("q_d_rightHip", registry);

   private final YoDouble q_rightHipWorld = new YoDouble("q_rightHipWorld", registry);
   private final YoDouble q_leftHipWorld = new YoDouble("q_leftHipWorld", registry);

   private final YoDouble footXTarget = new YoDouble("footXTarget", registry);

   private final YoDouble comHeight = new YoDouble("comHeight", registry);

   private final YoDouble comXVelocity = new YoDouble("comXVelocity", registry);
   private final YoDouble comXPositionFromFoot = new YoDouble("comXPositionFromFoot", registry);

   private final SideDependentList<YoDouble> desiredKneeLengths = new SideDependentList<YoDouble>(q_d_leftKnee, q_d_rightKnee);
   private final SideDependentList<YoDouble> desiredHipAngles = new SideDependentList<YoDouble>(q_d_leftHip, q_d_rightHip);
   private final SideDependentList<YoDouble> worldHipAngles = new SideDependentList<YoDouble>(q_leftHipWorld, q_rightHipWorld);
   private final SideDependentList<StateMachine<States, State>> stateMachines;

   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);
   private final YoDouble orbitalEnergy = new YoDouble("orbitalEnergy", registry);
   private final YoDouble strideLength = new YoDouble("strideLength", registry);
   private final YoDouble hipDiffAngle = new YoDouble("hipDiffAngle", registry);
   private final YoDouble swingTime = new YoDouble("swingTime", registry);
   private final YoDouble lastStepLength = new YoDouble("lastStepLength", registry);

   private final YoDouble leftFootZForce = new YoDouble("leftFootZForce", registry);
   private final YoDouble rightFootZForce = new YoDouble("rightFootZForce", registry);

   private ArrayList<Double> groundContactPositions = new ArrayList<Double>();
   private final SideDependentList<YoDouble> footZForces = new SideDependentList<YoDouble>(leftFootZForce, rightFootZForce);

   private PolynomialBasics trajectorySwingHipPitch;
   private PolynomialBasics trajectorySwingKneeLength;
   private Vector3DReadOnly nextStepCoMLocation;

   private enum States
   {
      SUPPORT, SWING;
   }

   private YoDouble timeOfLastFootSwitch = new YoDouble("timeOfLastFootSwitch", registry);
   private YoBoolean swingTrajectoriesCalculated = new YoBoolean("footLocationCalculated", registry);
   private YoBoolean swingFootGroundContact = new YoBoolean("swingFootGroundContact", registry);
   private YoDouble desiredTopVelocity = new YoDouble("desiredTopVelocity", registry);
   private YoDouble desiredEnergy = new YoDouble("desiredEnergy", registry);

   private final double g = 9.81;

   public LIPMWalkerControllerBhavyansh(LIPMWalkerRobot robot)
   {
      this.robot = robot;

      t = (YoDouble) robot.getRobot().findVariable("t");
      trajectorySwingHipPitch = new YoPolynomial("trajectorySwingHipAngle", 6, registry);
      trajectorySwingKneeLength = new YoPolynomial("trajectorySwingKneeLength", 6, registry);

      initialize();

      stateMachines = setupStateMachines();


   }

   @Override
   public void initialize()
   {

      kpKnee.set(1000.0);
      kdKnee.set(100.0);

      kpHip.set(2000.0);
      kdHip.set(60.0);

      q_d_leftKnee.set(0.8);
      q_d_rightKnee.set(0.7);

      q_d_leftHip.set(0.0);
      q_d_rightHip.set(0.0);

      desiredHeight.set(0.8);

      strideLength.set(0.3);
      swingTime.set(0.4);

      timeOfLastFootSwitch.set(0.0);
      swingTrajectoriesCalculated.set(false);
      swingFootGroundContact.set(false);
      desiredTopVelocity.set(0.7);
      desiredEnergy.set(0.5 * desiredTopVelocity.getValue() * desiredTopVelocity.getValue());

      orbitalEnergy.set(0.5 * 0.7 * 0.7);
      nextStepCoMLocation = calculateStepCoMPosition();
      footXTarget.set(nextStepCoMLocation.getX());

      calculateSwingTrajectories(RobotSide.RIGHT, nextStepCoMLocation, 0.0, true);

      groundContactPositions.add(0.0);

      leftFootZForce.set(robot.getFootZForce(RobotSide.LEFT));
      rightFootZForce.set(robot.getFootZForce(RobotSide.RIGHT));
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      filterFootForces();

      for (RobotSide robotSide : RobotSide.values())
      {
         stateMachines.get(robotSide).doAction();
         stateMachines.get(robotSide).doTransitions();
      }

      LogTools.info("State -> Left: {}, Right: {}",
                    stateMachines.get(RobotSide.LEFT).getCurrentStateKey(),
                    stateMachines.get(RobotSide.RIGHT).getCurrentStateKey());
   }

   public void filterFootForces()
   {
      leftFootZForce.set(leftFootZForce.getValue() * 0.9 + robot.getFootZForce(RobotSide.LEFT) * 0.1);
      rightFootZForce.set(rightFootZForce.getValue() * 0.9 + robot.getFootZForce(RobotSide.RIGHT) * 0.1);
   }

   private void computeOrbitalEnergy(RobotSide supportSide)
   {

      /* Compute and set orbital energy YoDouble. Based on virtual spring-mass system. */
      comXVelocity.set(robot.getCenterOfMassVelocity().getX());
      comXPositionFromFoot.set(robot.getCenterOfMassXDistanceFromSupportFoot());
      double orbitalEnergyValue = 0.5 * comXVelocity.getValue() * comXVelocity.getValue()
                                  - 0.5 * g / desiredHeight.getDoubleValue() * comXPositionFromFoot.getValue() * comXPositionFromFoot.getValue();
      orbitalEnergy.set(orbitalEnergyValue);
   }

   Vector3DReadOnly calculateStepCoMPosition()
   {
      double energy = orbitalEnergy.getValue();
      double x_final = (strideLength.getValue() / 2) + (desiredHeight.getValue() / (g * strideLength.getValue()) * (desiredEnergy.getValue() - energy));

      LogTools.info("Xf: {} \tEnergy: {} \tDesiredEnergy: {}", x_final, energy, desiredEnergy);

      Vector3DReadOnly stepCoMPosition = new Vector3D(x_final, 0.0, 0.0);
      return stepCoMPosition;
   }

   private void calculateSwingTrajectories(RobotSide swingSide, Vector3DReadOnly stepCoMPosition, double comPositionFromSupportFoot, boolean initial)
   {
      double distanceToNextStep = strideLength.getValue() - stepCoMPosition.getX();
      double desiredHipAngle = -EuclidCoreTools.atan2(distanceToNextStep, desiredHeight.getValue());
      double desiredKneeLength = EuclidCoreTools.squareRoot(
            distanceToNextStep * distanceToNextStep + desiredHeight.getValue() * desiredHeight.getValue());

      LogTools.info("DistanceToNextStep: {} \tCoMX: {} \tDesiredKneeLength: {}", distanceToNextStep, comPositionFromSupportFoot, desiredKneeLength);

      double hipAngle = worldHipAngles.get(swingSide).getValue();
      double kneeLength = robot.getKneeLength(swingSide);
      trajectorySwingHipPitch.setQuintic(comPositionFromSupportFoot, nextStepCoMLocation.getX(), hipAngle, 0.0, 0.0, desiredHipAngle, 0.0, 0.0);
      trajectorySwingKneeLength.setQuintic(comPositionFromSupportFoot, nextStepCoMLocation.getX(), kneeLength, 0.0, 0.0, desiredKneeLength, 0.0, 0.0);
   }

   private void controlSupportLeg(RobotSide side)
   {
      double kneeLength = robot.getKneeLength(side);
      double kneeVelocity = robot.getKneeVelocity(side);

      //         double desiredKneeLength = desiredKneeLengths.get(side).getValue();

      Point3DReadOnly centerOfMassPosition = robot.getCenterOfMassPosition();
      Vector3DReadOnly centerOfMassVelocity = robot.getCenterOfMassVelocity();
      double mass = robot.getMass();

      computeOrbitalEnergy(side);
      comHeight.set(centerOfMassPosition.getZ());

      double feedForwardSupportKneeForce = g * mass * kneeLength / centerOfMassPosition.getZ();
      double feedBackKneeForce =
            kpKnee.getValue() * (desiredHeight.getValue() - comHeight.getValue()) + kdKnee.getValue() * (0.0 - centerOfMassVelocity.getZ());
      robot.setKneeForce(side, feedForwardSupportKneeForce + feedBackKneeForce);
      robot.setHipTorque(side, 0.0);

      worldHipAngles.get(side).set(robot.getHipAngle(side) + robot.getBodyPitchAngle());
   }

   private void controlSwingLeg(RobotSide side, double timeInState)
   {
      double feedBackKneeForce;

      double hipAngle = robot.getHipAngle(side) + robot.getBodyPitchAngle();
      worldHipAngles.get(side).set(hipAngle);

      double supportHipAngle = robot.getHipAngle(side.getOppositeSide()) + robot.getBodyPitchAngle();
      double hipVelocity = robot.getHipVelocity(side);
      double kneeLength = robot.getKneeLength(side);
      double kneeVelocity = robot.getKneeVelocity(side);

      if (StrictMath.abs(comXPositionFromFoot.getValue()) < 0.01 && swingTrajectoriesCalculated.getValue() == false)
      {
         nextStepCoMLocation = calculateStepCoMPosition();
         footXTarget.set(nextStepCoMLocation.getX());
         calculateSwingTrajectories(side, nextStepCoMLocation, robot.getCenterOfMassXDistanceFromSupportFoot(), false);
         swingTrajectoriesCalculated.set(true);
      }

      double desiredKneeLength = 0.6;
      double desiredKneeVelocity = 0.0;
      double desiredHipAngle = 0.0;
      double desiredHipVelocity = 0.0;

      if (comXPositionFromFoot.getValue() > 0.0)
      {
         trajectorySwingKneeLength.compute(comXPositionFromFoot.getValue());
         trajectorySwingHipPitch.compute(comXPositionFromFoot.getValue());
         desiredKneeLength = trajectorySwingKneeLength.getValue();
         desiredKneeVelocity = trajectorySwingKneeLength.getVelocity();
         desiredHipAngle = trajectorySwingHipPitch.getValue();
         desiredHipVelocity = trajectorySwingHipPitch.getVelocity();
      }

      /* ----------------------------------- Compute and set knee force. ----------------------------------------------*/
      desiredKneeLengths.get(side).set(desiredKneeLength);
      feedBackKneeForce = 1000 * (desiredKneeLengths.get(side).getValue() - kneeLength) + 100 * (desiredKneeVelocity - kneeVelocity);
      robot.setKneeForce(side, feedBackKneeForce);

      /* ----------------------------------- Compute and set hip torque. --------------------------------------------*/
      double feedBackHipTorque = kpHip.getValue() * (desiredHipAngle - hipAngle) + kdHip.getValue() * (desiredHipVelocity - hipVelocity);
      desiredHipAngles.get(side).set(desiredHipAngle);
      robot.setHipTorque(side, feedBackHipTorque);

      //      LogTools.info("CoMX: {}, KneeLength: {}, HipAngle: {}", comXPositionFromFoot.getValue(), desiredKneeLength, desiredHipAngle);
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
         controlSwingLeg(robotSide, timeInState);
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
         if (swingFootGroundContact.getValue())
         {
            swingFootGroundContact.set(false);
            return true;
         }
         else
            return false;
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
         double timeDiff = t.getValue() - timeOfLastFootSwitch.getValue();
         boolean fs = footZForces.get(robotSide).getValue() > 10.0 && timeDiff > 0.1; // Eliminates switch bouncing.
         LogTools.info("Switch: {} {} {}", fs, footZForces.get(robotSide).getValue(), timeDiff);
         if (fs)
         {
            LogTools.info("N -> ComX: {}", robot.getCenterOfMassXDistanceFromSupportFoot());

            groundContactPositions.add(robot.getFootPosition(robotSide).getX());

            if (groundContactPositions.size() >= 2)
               lastStepLength.set(
                     groundContactPositions.get(groundContactPositions.size() - 1) - groundContactPositions.get(groundContactPositions.size() - 2));

            timeOfLastFootSwitch.set(t.getValue());
            swingFootGroundContact.set(true);

            swingTrajectoriesCalculated.set(false);
         }
         return fs;
      }
   }
}
