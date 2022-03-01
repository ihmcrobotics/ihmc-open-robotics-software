package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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
   private final YoDouble comZVelocity = new YoDouble("comZVelocity", registry);
   private final YoDouble comXPositionFromFoot = new YoDouble("comXPositionFromFoot", registry);

   private final YoDouble q_d_leftKnee = new YoDouble("q_d_leftKnee", registry);
   private final YoDouble q_d_rightKnee = new YoDouble("q_d_rightKnee", registry);
   private final YoDouble q_d_leftHip = new YoDouble("q_d_leftHip", registry);
   private final YoDouble q_d_rightHip = new YoDouble("q_d_rightHip", registry);

   private final YoDouble q_rightHipWorld = new YoDouble("q_rightHipWorld", registry);
   private final YoDouble q_leftHipWorld = new YoDouble("q_leftHipWorld", registry);

   private final YoDouble finalKneeLength = new YoDouble("finalKneeLength", registry);

   private final SideDependentList<YoDouble> desiredKneeLengths = new SideDependentList<YoDouble>(q_d_leftKnee, q_d_rightKnee);
   private final SideDependentList<YoDouble> desiredHipAngles = new SideDependentList<YoDouble>(q_d_leftHip, q_d_rightHip);
   private final SideDependentList<YoDouble> worldHipAngles = new SideDependentList<YoDouble>(q_leftHipWorld, q_rightHipWorld);

   private final YoDouble footIntoGroundLength = new YoDouble("footIntoGroundLength", registry);

   private final YoDouble comHeight = new YoDouble("comHeight", registry);
   private final YoDouble desiredHeight = new YoDouble("desiredHeight", registry);

   private final YoDouble desiredTopOfStrideVelocity = new YoDouble("desiredTopOfStrideVelocity", registry);
   private final YoDouble topOfStrideVelocity = new YoDouble("topOfStrideVelocity", registry);
   private final YoBoolean isTopOfStrideVelocityRecorded = new YoBoolean("isTopOfStrideVelocityRecorded", registry);
   private final YoDouble desiredStepLength = new YoDouble("desiredStepLength", registry);
   private final YoDouble kpStepLength = new YoDouble("kpStepLength", registry);

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
      kpBodyPitch.set(200.0);
      kdBodyPitch.set(30.0);
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

      kpStepLength.set(0.5);
      
      footIntoGroundLength.set(0.01);

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
      // Simulation data.
      RobotSide swingSide = supportSide.getOppositeSide();
      Point3DReadOnly centerOfMassPosition = robot.getCenterOfMassPosition();
      Vector3DReadOnly centerOfMassVelocity = robot.getCenterOfMassVelocity();
      comXVelocity.set(centerOfMassVelocity.getX());
      comZVelocity.set(centerOfMassVelocity.getZ());
      comHeight.set(centerOfMassPosition.getZ());
      footForces.get(supportSide).set(robot.getFootZForce(supportSide));
      footForces.get(swingSide).set(robot.getFootZForce(swingSide));

      // When swing leg crosses in front of support leg, record top of stride velocity, and set a new target hip angle and knee length for the swing leg
      // depending on whether we're going slower or faster than we want to.
      if (robot.getFootPosition(supportSide).getX() - centerOfMassPosition.getX() > 0.0)
      {
         // Pre-half step.
         calculatePreHalfStepSwingLegKinematics(swingSide);
      }
      else
      {
         // Post-half step.
         if (!isTopOfStrideVelocityRecorded.getValue())
         {
            topOfStrideVelocity.set(comXVelocity.getValue());
            calculatePostHalfStepSwingLegKinematics(swingSide);
            isTopOfStrideVelocityRecorded.set(true);  // locks out this if statement to keep post half step kinematics fixed
         }
      }

      // Control.
      controlSupportLeg(supportSide);
      controlSwingLeg(swingSide);

      // Update desired swing knee length to further drive swing leg into the ground as stance lengthens.
      updateSwingLegGroundDrivingHeuristic(swingSide);

      // Support Leg Switching.
      boolean isSwingFootOnGround = robot.getFootPosition(swingSide).getZ() < 0.001 &&
                                    worldHipAngles.get(swingSide).getValue() < worldHipAngles.get(supportSide).getValue();
      if ( isSwingFootOnGround )
      {
         supportSide = supportSide.getOppositeSide();
         isTopOfStrideVelocityRecorded.set(false);
      }
   }

   public void controlSwingLeg(RobotSide side)
   {
      // Hip -- PD control on desired hip angle.
      double hipAngle = robot.getHipAngle(side) + robot.getBodyPitchAngle();
      worldHipAngles.get(side).set(hipAngle);
      double hipVelocity = robot.getHipVelocity(side);
      double feedBackHipTorque = kpHip.getValue() * (desiredHipAngles.get(side).getValue() - hipAngle) + kdKnee.getValue() * (0.0 - hipVelocity);
      robot.setHipTorque(side, feedBackHipTorque);

      // Knee -- PD control on desired knee length.
      double kneeLength = robot.getKneeLength(side);
      double kneeVelocity = robot.getKneeVelocity(side);
      // If the swing leg is behind the support leg, retract it to half length to prevent scuffing.
      // NOTE: The > and dependence on angles means this only works for forward walking.
      double feedBackKneeForce;
      if (worldHipAngles.get(side).getValue() > worldHipAngles.get(supportSide).getValue())
         feedBackKneeForce = kpKnee.getValue() * (0.5 - kneeLength) + kdKnee.getValue() * (0.0 - kneeVelocity);
      else
         feedBackKneeForce = kpKnee.getValue() * (desiredKneeLengths.get(side).getValue() - kneeLength) + kdKnee.getValue() * (0.0 - kneeVelocity);
      robot.setKneeForce(side, feedBackKneeForce);
   }

   public void controlSupportLeg(RobotSide side)
   {
      // Knee -- PD control around desired CoM height.
      double mass = robot.getMass();
      double kneeLength = robot.getKneeLength(side);
      double feedForwardKneeForce = mass * g * kneeLength / comHeight.getValue();
      double feedBackKneeForce = kpKnee.getValue() * (desiredHeight.getValue() - comHeight.getValue()) + kdKnee.getValue() * (0.0 - comZVelocity.getValue());
      robot.setKneeForce(supportSide, feedForwardKneeForce + feedBackKneeForce);

      // Hip -- PD control around desired body pitch (keep upright).
      double hipAngle = robot.getHipAngle(supportSide) + robot.getBodyPitchAngle();
      worldHipAngles.get(supportSide).set(hipAngle);
      double feedBackHipTorque = -kpBodyPitch.getValue() * (desiredBodyPitch.getValue() - robot.getBodyPitchAngle()) - kdBodyPitch.getValue() * (0.0 - robot.getBodyPitchAngularVelocity());
      robot.setHipTorque(supportSide, feedBackHipTorque);
   }

   public void calculatePreHalfStepSwingLegKinematics(RobotSide side)
   {
      // Hip -- trigonometry based on desired step length and desired CoM height.
      double desiredHipAngle = Math.atan2(0.5 * desiredStepLength.getValue(), desiredHeight.getValue());
      desiredHipAngles.get(side).set(-desiredHipAngle);

      // Knee -- pythagoras based on desired step length and desired CoM height.
      double desiredKneeLength = Math.sqrt((desiredHeight.getValue() * desiredHeight.getValue()) + (0.5 * desiredStepLength.getValue() * 0.5 * desiredStepLength.getValue()));
      desiredKneeLengths.get(side).set(desiredKneeLength + footIntoGroundLength.getValue());
   }

   public void calculatePostHalfStepSwingLegKinematics(RobotSide side)
   {
      // Hip -- Raibert style proportional heuristic, change swing leg hip angle depending on whether we're going faster
      // or slower than we want to, as measured at top of stride.
      double newSwingHipAngle;
      newSwingHipAngle = desiredHipAngles.get(side).getValue() + kpStepLength.getValue() * (desiredTopOfStrideVelocity.getValue() - topOfStrideVelocity.getValue());
      if (newSwingHipAngle > -0.05)
         newSwingHipAngle = -0.05;
      if (newSwingHipAngle > 1.0)
         newSwingHipAngle = 1.0;
      desiredHipAngles.get(side).set(newSwingHipAngle);

      // Knee -- desired length updated based on trig with the new hip angle, plus an extra heuristic on knee length to ensure ground contact.
      double newSwingKneeLength = desiredHeight.getValue() / Math.cos(newSwingHipAngle) + footIntoGroundLength.getValue();
      finalKneeLength.set(newSwingKneeLength);
   }

   public void updateSwingLegGroundDrivingHeuristic(RobotSide side)
   {
      double percentOver = 0.5 + 0.5 * robot.getHipAngle(supportSide) / (0.2);
      if (percentOver < 0.5) percentOver = 0.5;
      if (percentOver > 1.5) percentOver = 1.5;

      desiredKneeLengths.get(side).set(finalKneeLength.getValue() * percentOver);
   }
}