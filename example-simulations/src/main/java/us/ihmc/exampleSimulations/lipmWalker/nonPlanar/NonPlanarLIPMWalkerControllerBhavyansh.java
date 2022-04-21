package us.ihmc.exampleSimulations.lipmWalker.nonPlanar;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;

public class NonPlanarLIPMWalkerControllerBhavyansh implements RobotController
{
   private final NonPlanarLIPMWalkerRobot robot;
   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private YoDouble t;

   private final YoDouble kpBody = new YoDouble("kpBody", registry);
   private final YoDouble kdBody = new YoDouble("kdBody", registry);

   private final YoDouble kpKnee = new YoDouble("kpKnee", registry);
   private final YoDouble kdKnee = new YoDouble("kdKnee", registry);
   private final YoDouble kpHip = new YoDouble("kpHip", registry);
   private final YoDouble kdHip = new YoDouble("kdHip", registry);
   private final YoDouble kpHipRoll = new YoDouble("kpHipRoll", registry);
   private final YoDouble kdHipRoll = new YoDouble("kdHipRoll", registry);

   private final YoDouble q_d_leftKnee = new YoDouble("q_d_leftKnee", registry);
   private final YoDouble q_d_rightKnee = new YoDouble("q_d_rightKnee", registry);
   private final YoDouble q_d_leftHipPitch = new YoDouble("q_d_leftHipPitch", registry);
   private final YoDouble q_d_rightHipPitch = new YoDouble("q_d_rightHipPitch", registry);
   private final YoDouble q_d_leftHipRoll = new YoDouble("q_d_leftHipRoll", registry);
   private final YoDouble q_d_rightHipRoll = new YoDouble("q_d_rightHipRoll", registry);

   private final YoDouble q_rightHipPitchWorld = new YoDouble("q_rightHipPitchWorld", registry);
   private final YoDouble q_leftHipPitchWorld = new YoDouble("q_leftHipPitchWorld", registry);
   private final YoDouble q_rightHipRollWorld = new YoDouble("q_rightHipRollWorld", registry);
   private final YoDouble q_leftHipRollWorld = new YoDouble("q_leftHipRollWorld", registry);

   private final YoDouble footXTarget = new YoDouble("footXTarget", registry);
   private final YoDouble footYTarget = new YoDouble("footYTarget", registry);

   private final YoDouble comHeight = new YoDouble("comHeight", registry);

   private final YoDouble comXVelocity = new YoDouble("comXVelocity", registry);
   private final YoDouble comXPositionFromFoot = new YoDouble("comXPositionFromFoot", registry);
   private final YoDouble comYVelocity = new YoDouble("comYVelocity", registry);
   private final YoDouble comYPositionFromFoot = new YoDouble("comYPositionFromFoot", registry);

   private final SideDependentList<YoDouble> desiredKneeLengths = new SideDependentList<YoDouble>(q_d_leftKnee, q_d_rightKnee);
   private final SideDependentList<YoDouble> desiredHipPitchAngles = new SideDependentList<YoDouble>(q_d_leftHipPitch, q_d_rightHipPitch);

   private final SideDependentList<YoDouble> desiredHipRollAngles = new SideDependentList<YoDouble>(q_d_leftHipRoll, q_d_rightHipRoll);
   private final SideDependentList<YoDouble> worldHipPitchAngles = new SideDependentList<YoDouble>(q_leftHipPitchWorld, q_rightHipPitchWorld);
   private final SideDependentList<YoDouble> worldHipRollAngles = new SideDependentList<YoDouble>(q_leftHipRollWorld, q_rightHipRollWorld);
   private final SideDependentList<StateMachine<States, State>> stateMachines;

   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);
   private final YoDouble orbitalEnergyX = new YoDouble("orbitalEnergyX", registry);
   private final YoDouble orbitalEnergyY = new YoDouble("orbitalEnergyY", registry);
   private final YoDouble strideLengthX = new YoDouble("strideLengthX", registry);
   private final YoDouble strideLengthY = new YoDouble("strideLengthY", registry);
   private final YoDouble hipDiffAngle = new YoDouble("hipDiffAngle", registry);
   private final YoDouble swingTime = new YoDouble("swingTime", registry);
   private final YoDouble lastStepLength = new YoDouble("lastStepLength", registry);

   private final YoDouble leftFootZForce = new YoDouble("leftFootZForce", registry);
   private final YoDouble rightFootZForce = new YoDouble("rightFootZForce", registry);

   private ArrayList<Double> groundContactPositions = new ArrayList<Double>();
   private final SideDependentList<YoDouble> footZForces = new SideDependentList<YoDouble>(leftFootZForce, rightFootZForce);

   private PolynomialBasics trajectorySwingHipPitch;

   private PolynomialBasics trajectorySwingHipRoll;
   private PolynomialBasics trajectorySwingKneeLength;
   private final YoFrameVector3D finalCoMPosition;

   private final YoInteger numberOfStepsTaken = new YoInteger("numberOfStepsTaken", registry);

   private enum States
   {
      SUPPORT, SWING;
   }

   private YoDouble timeOfLastFootSwitch = new YoDouble("timeOfLastFootSwitch", registry);
   private YoBoolean swingTrajectoriesCalculated = new YoBoolean("footLocationCalculated", registry);
   private YoBoolean swingFootGroundContact = new YoBoolean("swingFootGroundContact", registry);
   private YoDouble desiredTopVelocity = new YoDouble("desiredTopVelocity", registry);
   private YoDouble desiredEnergyX = new YoDouble("desiredEnergyX", registry);
   private YoDouble desiredEnergyY = new YoDouble("desiredEnergyY", registry);

   private final double g = 9.81;

   public NonPlanarLIPMWalkerControllerBhavyansh(NonPlanarLIPMWalkerRobot robot)
   {
      this.robot = robot;

      t = (YoDouble) robot.getRobot().findVariable("t");
      trajectorySwingKneeLength = new YoPolynomial("trajectorySwingKneeLength", 6, registry);
      trajectorySwingHipPitch = new YoPolynomial("trajectorySwingHipPitchAngle", 6, registry);
      trajectorySwingHipRoll = new YoPolynomial("trajectorySwingHipRollAngle", 6, registry);

      finalCoMPosition = new YoFrameVector3D("nextStepCoMLocation", ReferenceFrame.getWorldFrame(), registry);

      initialize();

      stateMachines = setupStateMachines();


   }

   @Override
   public void initialize()
   {
      numberOfStepsTaken.set(0);

      kpBody.set(200.0);
      kdBody.set(30.0);

      kpKnee.set(1000.0);
      kdKnee.set(100.0);

      kpHip.set(1000.0);
      kdHip.set(50.0);

      kpHipRoll.set(2000.0);
      kdHipRoll.set(200.0);

      q_d_leftKnee.set(0.8);
      q_d_rightKnee.set(0.7);

      q_d_leftHipPitch.set(0.0);
      q_d_rightHipPitch.set(0.0);

      desiredHeight.set(0.8);

      strideLengthX.set(0.5);
      strideLengthY.set(0.2);
      swingTime.set(0.4);

      timeOfLastFootSwitch.set(0.0);
      swingTrajectoriesCalculated.set(false);
      swingFootGroundContact.set(false);
      desiredTopVelocity.set(0.7);
      desiredEnergyX.set(0.5 * desiredTopVelocity.getValue() * desiredTopVelocity.getValue());
      desiredEnergyY.set(0.0);

      orbitalEnergyX.set(0.5 * 0.7 * 0.7);
      orbitalEnergyX.set(0.0);
      calculateStepCoMPosition(RobotSide.RIGHT);
      footXTarget.set(finalCoMPosition.getX());
      footYTarget.set(finalCoMPosition.getY());

      calculateSwingTrajectories(RobotSide.RIGHT, finalCoMPosition, 0.0, 0.0, true);

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

//      LogTools.info("State -> Left: {}, Right: {}",
//                    stateMachines.get(RobotSide.LEFT).getCurrentStateKey(),
//                    stateMachines.get(RobotSide.RIGHT).getCurrentStateKey());
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
      comXPositionFromFoot.set(robot.getCenterOfMassPositionFromSupportFoot().getX());

      comYVelocity.set(robot.getCenterOfMassVelocity().getY());
      comYPositionFromFoot.set(robot.getCenterOfMassPositionFromSupportFoot().getY());

      double orbitalEnergyValueX = 0.5 * comXVelocity.getValue() * comXVelocity.getValue()
                                  - 0.5 * g / desiredHeight.getDoubleValue() * comXPositionFromFoot.getValue() * comXPositionFromFoot.getValue();
      orbitalEnergyX.set(orbitalEnergyValueX);

      double orbitalEnergyValueY = 0.5 * comYVelocity.getValue() * comYVelocity.getValue()
                                  - 0.5 * g / desiredHeight.getDoubleValue() * comYPositionFromFoot.getValue() * comYPositionFromFoot.getValue();
      orbitalEnergyY.set(orbitalEnergyValueY);
   }

   private void calculateStepCoMPosition(RobotSide swingSide)
   {
      double energyX = orbitalEnergyX.getValue();
      double x_final = (strideLengthX.getValue() / 2) + (desiredHeight.getValue() / (g * strideLengthX.getValue()) * (desiredEnergyX.getValue() - energyX));

      double energyY = orbitalEnergyY.getValue();
      double y_final = (strideLengthY.getValue() / 2) + (desiredHeight.getValue() / (g * strideLengthY.getValue()) * (desiredEnergyY.getValue() - energyY));

      LogTools.info("Xf: {} \tEnergyX: {} \tDesiredEnergyX: {}", x_final, energyX, desiredEnergyX);
      LogTools.info("Yf: {} \tEnergyY: {} \tDesiredEnergyY: {}", y_final, energyY, desiredEnergyY);

      Vector3DReadOnly stepCoMPosition = new Vector3D(x_final, y_final, 0.0);
      finalCoMPosition.set(stepCoMPosition);
   }

   private void calculateSwingTrajectories(RobotSide swingSide, Vector3DReadOnly stepCoMPosition, double comPositionFromSupportFootX, double comPositionFromSupportFootY, boolean initial)
   {
      double distanceToNextStepX = strideLengthX.getValue() - stepCoMPosition.getX();
      double distanceToNextStepY = strideLengthY.getValue() - stepCoMPosition.getY();

      double desiredHipAnglePitch = -EuclidCoreTools.atan2(distanceToNextStepX, desiredHeight.getValue());
      double desiredHipAngleRoll = -EuclidCoreTools.atan2(distanceToNextStepY, desiredHeight.getValue());


      if(swingSide.equals(RobotSide.LEFT))
      {
         desiredHipAngleRoll *= -1;
      }
      LogTools.info("Desired Hip Angle Roll: SwingSide: {} {} {}", swingSide, distanceToNextStepY, desiredHipAngleRoll);

      double totalDistanceToNextStep = EuclidCoreTools.norm(distanceToNextStepX, distanceToNextStepY);
      double desiredKneeLength = EuclidCoreTools.squareRoot(
            totalDistanceToNextStep * totalDistanceToNextStep + desiredHeight.getValue() * desiredHeight.getValue());

      LogTools.info("DistanceToNextStepX: {} \tCoMX: {} \tDesiredKneeLength: {}", distanceToNextStepX, comPositionFromSupportFootX, desiredKneeLength);
      LogTools.info("DistanceToNextStepY: {} \tCoMY: {} \tDesiredKneeLength: {}", distanceToNextStepY, comPositionFromSupportFootY, desiredKneeLength);

      double pitchAngle = worldHipPitchAngles.get(swingSide).getValue();
      double rollAngle = worldHipRollAngles.get(swingSide).getValue();
      double kneeLength = robot.getKneeLength(swingSide);
      trajectorySwingKneeLength.setQuintic(comPositionFromSupportFootX, finalCoMPosition.getX(), kneeLength, 0.0, 0.0, desiredKneeLength, 0.0, 0.0);
      trajectorySwingHipPitch.setQuintic(comPositionFromSupportFootX, finalCoMPosition.getX(), pitchAngle, 0.0, 0.0, desiredHipAnglePitch, 0.0, 0.0);
//      trajectorySwingHipRoll.setQuintic(comPositionFromSupportFootY, finalCoMPosition.getY(), rollAngle, 0.0, 0.0, desiredHipAngleRoll, 0.0, 0.0);
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
      robot.setHipTorquePitch(side, -kpBody.getValue() * (0.0 - robot.getBodyPitchAngle()) + kdBody.getValue() * robot.getBodyPitchAngularVelocity());
      robot.setHipTorqueRoll(side, -kpHipRoll.getValue() * (0.0 - robot.getBodyPitchAngle()) + kdHipRoll.getValue() * robot.getBodyRollAngularVelocity());

      worldHipPitchAngles.get(side).set(robot.getHipAnglePitch(side) + robot.getBodyPitchAngle());
      worldHipRollAngles.get(side).set(robot.getHipAngleRoll(side) + robot.getBodyRollAngle());
   }

   private void controlSwingLeg(RobotSide side, double timeInState)
   {
      double feedBackKneeForce;

      /* Get joint positions. */
      double hipPitchAngle = robot.getHipAnglePitch(side) + robot.getBodyPitchAngle();
      double hipRollAngle = robot.getHipAngleRoll(side) + robot.getBodyRollAngle();
      double kneeLength = robot.getKneeLength(side);

      /* Set world rotational hip joint positions. */
      worldHipPitchAngles.get(side).set(hipPitchAngle);
      worldHipRollAngles.get(side).set(hipRollAngle);

      /* Get joint velocities. */
      double hipPitchVelocity = robot.getHipVelocityPitch(side);
      double hipRollVelocity = robot.getHipVelocityRoll(side);
      double kneeVelocity = robot.getKneeVelocity(side);

      double desiredKneeLength = 0.6;
      double desiredKneeVelocity = 0.0;
      double desiredHipPitchAngle = 0.0;
      double desiredHipPitchVelocity = 0.0;
      double desiredHipRollAngle = 0.0;
      double desiredHipRollVelocity = 0.0;

      /* When the stance is almost vertical, calculate foot placement and swing trajectories. */
      if (StrictMath.abs(comXPositionFromFoot.getValue()) < 0.01 && swingTrajectoriesCalculated.getValue() == false)
      {
         calculateStepCoMPosition(side);
         footXTarget.set(finalCoMPosition.getX());
         footYTarget.set(finalCoMPosition.getY());
         calculateSwingTrajectories(side, finalCoMPosition, robot.getCenterOfMassPositionFromSupportFoot().getX(), robot.getCenterOfMassPositionFromSupportFoot().getY(), false);

         swingTrajectoriesCalculated.set(true);
      }


      if (comXPositionFromFoot.getValue() > 0.0)
      {
         trajectorySwingKneeLength.compute(comXPositionFromFoot.getValue());
         trajectorySwingHipPitch.compute(comXPositionFromFoot.getValue());
         //         trajectorySwingHipRoll.compute(comYPositionFromFoot.getValue());

         desiredKneeLength = trajectorySwingKneeLength.getValue();
         desiredKneeVelocity = trajectorySwingKneeLength.getVelocity();
         desiredHipPitchAngle = trajectorySwingHipPitch.getValue();
         desiredHipPitchVelocity = trajectorySwingHipPitch.getVelocity();
         //         desiredHipRollAngle = trajectorySwingHipRoll.getValue();
         //         desiredHipRollVelocity = trajectorySwingHipRoll.getVelocity();

      }

      /* ----------------------------------- Compute and set knee force based on desired values. ----------------------------------------------*/
      desiredKneeLengths.get(side).set(desiredKneeLength);
      feedBackKneeForce = 1300 * (desiredKneeLengths.get(side).getValue() - kneeLength) + 70 * (desiredKneeVelocity - kneeVelocity);
      robot.setKneeForce(side, feedBackKneeForce);

      /* ----------------------------------- Compute and set hip torques based on desired values. --------------------------------------------*/
      double feedBackHipTorquePitch = kpHip.getValue() * (desiredHipPitchAngle - hipPitchAngle) + kdHip.getValue() * (desiredHipPitchVelocity - hipPitchVelocity);
      desiredHipPitchAngles.get(side).set(desiredHipPitchAngle);
      robot.setHipTorquePitch(side, feedBackHipTorquePitch);

      /*----------------------------------- Compute and set hip roll torque based on desired values. --------------------------------------------*/
      double feedBackHipTorqueRoll = 600 * (desiredHipRollAngle - hipRollAngle) + 5 * (0.0 - hipRollVelocity);
      desiredHipRollAngles.get(side).set(desiredHipRollAngle);
      robot.setHipTorqueRoll(side, feedBackHipTorqueRoll);

      //      LogTools.info("CoMX: {}, KneeLength: {}, HipAngle: {}", comXPositionFromFoot.getValue(), desiredKneeLength, desiredHipPitchAngle);
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
            numberOfStepsTaken.set(numberOfStepsTaken.getValue() + 1);
            LogTools.info("Step: {} -> ComX: {}", robot.getCenterOfMassPositionFromSupportFoot(), numberOfStepsTaken.getValue());

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
