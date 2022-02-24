package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.tools.EuclidCoreTools;
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
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class LIPMWalkerControllerJames implements RobotController
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

   private final SideDependentList<YoDouble> desiredKneeLengths = new SideDependentList<YoDouble>(q_d_leftKnee, q_d_rightKnee);
   private final SideDependentList<YoDouble> desiredHipAngles = new SideDependentList<YoDouble>(q_d_leftHip, q_d_rightHip);
   private final SideDependentList<YoDouble> worldHipAngles = new SideDependentList<YoDouble>(q_leftHipWorld, q_rightHipWorld);

   private final YoDouble comHeight = new YoDouble("comHeight", registry);
   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);

   private final YoDouble desiredTopOfStrideVelocity = new YoDouble("desiredTopOfStrideVelocity", registry);
   private final YoDouble topOfStrideVelocity = new YoDouble("topOfStrideVelocity", registry);
   private final YoBoolean isTopOfStrideVelocityRecorded = new YoBoolean("isTopOfStrideVelocityRecorded", registry);
   private final YoDouble desiredStepLength = new YoDouble("desiredStepLength", registry);
   private final YoDouble kpStepLength = new YoDouble("kpStepDouble", registry);

   private final YoDouble f_leftFoot = new YoDouble("f_leftFoot", registry);
   private final YoDouble f_rightFoot = new YoDouble("f_rightFoot", registry);
   private final SideDependentList<YoDouble> footForces = new SideDependentList<YoDouble>(f_leftFoot, f_rightFoot);

   private RobotSide supportSide;

   private final YoDouble orbitalEnergy = new YoDouble("orbitalEnergy", registry);

   private enum States
   {
      SUPPORT, SWING;
   }

//   private final SideDependentList<StateMachine<States, State>> stateMachines;

   private final double g  = 9.81;

   public LIPMWalkerControllerJames(LIPMWalkerRobot robot)
   {
      this.robot = robot;
      initialize();
//      stateMachines = setupStateMachines();
   }

   @Override
   public void initialize()
   {
      kpBodyPitch.set(1000.0);
      kdBodyPitch.set(100.0);
      desiredBodyPitch.set(0.0);

      kpKnee.set(750.0);
      kdKnee.set(50.0);

      kpHip.set(1000.0);
      kdHip.set(100.0);

      q_d_leftKnee.set(1.0);
      q_d_rightKnee.set(1.0);

      q_d_leftHip.set(0.0);
      q_d_rightHip.set(0.0);

      desiredHeight.set(1.0);
      desiredStepLength.set(0.5);

      desiredTopOfStrideVelocity.set(1.0);
      isTopOfStrideVelocityRecorded.set(false);

      kpStepLength.set(0.1);

      supportSide = RobotSide.LEFT;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      RobotSide swingSide = supportSide.getOppositeSide();
      Point3DReadOnly centerOfMassPosition = robot.getCenterOfMassPosition();
      Vector3DReadOnly centerOfMassVelocity = robot.getCenterOfMassVelocity();
      double mass = robot.getMass();

      comXVelocity.set(centerOfMassVelocity.getX());
      comHeight.set(centerOfMassPosition.getZ());
      footForces.get(supportSide).set(robot.getFootZForce(supportSide));
      footForces.get(swingSide).set(robot.getFootZForce(swingSide));

      // If in the first half of step, the desired angles / lengths are computed to satisfy an isoceles triangle with base given by step length and height
      // given by desired CoM height.
      if (!isTopOfStrideVelocityRecorded.getValue())
      {
         double desiredKneeLength = Math.sqrt((desiredHeight.getValue() * desiredHeight.getValue()) + (0.5 * desiredStepLength.getValue() * 0.5 * desiredStepLength.getValue()));
         desiredKneeLengths.get(supportSide).set(desiredKneeLength);
         desiredKneeLengths.get(swingSide).set(desiredKneeLength);
         double desiredHipAngle = Math.atan2(0.5 * desiredStepLength.getValue(), desiredHeight.getValue());
         desiredHipAngles.get(supportSide).set(desiredHipAngle);
         desiredHipAngles.get(swingSide).set(-desiredHipAngle);
      }

      // Support Leg
      double kneeLength = robot.getKneeLength(supportSide);
      double feedForwardKneeForce = g * mass * kneeLength / centerOfMassPosition.getZ();
      double feedBackKneeForce = kpKnee.getValue() * (desiredHeight.getValue() - comHeight.getValue()) + kdKnee.getValue() * (0.0 - centerOfMassVelocity.getZ());
      // NOTE: not happy about this fudge factor on the feedforward -- I can't find a good enough set of knee gains.
      robot.setKneeForce(supportSide, 0.94 * feedForwardKneeForce + feedBackKneeForce);
      // TODO: if totalForce < 0, cap below, or make below cap 10% of body force

      double hipAngle = robot.getHipAngle(supportSide) + robot.getBodyPitchAngle();
      worldHipAngles.get(supportSide).set(hipAngle);
      double hipVelocity = robot.getHipVelocity(supportSide);
      double feedBackHipTorque = kpHip.getValue() * (desiredHipAngles.get(supportSide).getValue() - hipAngle) + kdKnee.getValue() * (0.0 - hipVelocity);
      robot.setHipTorque(supportSide, feedBackHipTorque);

      // Swing Leg
      hipAngle = robot.getHipAngle(swingSide) + robot.getBodyPitchAngle();
      worldHipAngles.get(swingSide).set(hipAngle);
      hipVelocity = robot.getHipVelocity(swingSide);
      feedBackHipTorque = kpHip.getValue() * (desiredHipAngles.get(swingSide).getValue() - hipAngle) + kdKnee.getValue() * (0.0 - hipVelocity);
      robot.setHipTorque(swingSide, feedBackHipTorque);

      kneeLength = robot.getKneeLength(swingSide);
      double kneeVelocity = robot.getKneeVelocity(swingSide);
      // If the swing leg is behind the support leg, retract it to half length to prevent scuffing.
      // NOTE: The > and dependence on angles means this only works for forward walking.
      if (worldHipAngles.get(swingSide).getValue() > worldHipAngles.get(supportSide).getValue())
         feedBackKneeForce = kpKnee.getValue() * (0.5 - kneeLength) + kdKnee.getValue() * (0.0 - kneeVelocity);
      else
         feedBackKneeForce = kpKnee.getValue() * (desiredKneeLengths.get(swingSide).getValue() - kneeLength) + kdKnee.getValue() * (0.0 - kneeVelocity);
      robot.setKneeForce(swingSide, feedBackKneeForce);

      // When swing leg crosses in front of support leg, record top of stride velocity, and set a new target hip angle and knee length for the swing leg
      // depending on whether we're going slower or faster than we want to.
      if (robot.getFootPosition(supportSide).getX() - centerOfMassPosition.getX() < 0.0)
      {
         if (!isTopOfStrideVelocityRecorded.getValue())
         {
            topOfStrideVelocity.set(comXVelocity.getValue());
            double newSwingHipAngle;
            if (topOfStrideVelocity.getValue() < desiredTopOfStrideVelocity.getValue())
               newSwingHipAngle = desiredHipAngles.get(swingSide).getValue() - 0.062 * (desiredTopOfStrideVelocity.getValue() - topOfStrideVelocity.getValue());
            else
               newSwingHipAngle = desiredHipAngles.get(swingSide).getValue() + 0.062 * (desiredTopOfStrideVelocity.getValue() - topOfStrideVelocity.getValue());
            desiredHipAngles.get(swingSide).set(newSwingHipAngle);
            double newSwingKneeLength = desiredHeight.getValue() / Math.cos(newSwingHipAngle);
            desiredKneeLengths.get(swingSide).set(newSwingKneeLength);
            isTopOfStrideVelocityRecorded.set(true);
         }
      }

      // Support Leg Switching
      boolean isSwingFootOnGround = robot.getFootPosition(swingSide).getZ() < 0.001 && worldHipAngles.get(swingSide).getValue() < worldHipAngles.get(supportSide).getValue();
      boolean isSwingAnklePastVertical = isTopOfStrideVelocityRecorded.getValue() && worldHipAngles.get(swingSide).getValue() > 0.0;
      if ( isSwingFootOnGround || isSwingAnklePastVertical )
      {
         // We change swing leg hip angle halfway through the step, for the next step let's assume that the new hip angle we chose was perfect and set BOTH
         // hip angles to be that by updating the step length as below. We'll end up selecting a new hip angle halfway through the next step anyway.
         // NOTE: The minus in front of the tan is because hip angles are negative going forward. As tan is an odd function, we need to prefix a minus to
         // get the desired results.
         desiredStepLength.set(2.0 * desiredHeight.getValue() * -Math.tan(desiredHipAngles.get(swingSide).getValue()));
         supportSide = supportSide.getOppositeSide();
         isTopOfStrideVelocityRecorded.set(false);
      }
   }
}