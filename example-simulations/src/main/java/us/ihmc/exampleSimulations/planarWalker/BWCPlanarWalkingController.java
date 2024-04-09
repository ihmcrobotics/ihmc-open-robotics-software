package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.scs2.definition.visual.ColorDefinitions.*;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.*;

public class BWCPlanarWalkingController implements Controller, SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BWCPlanarWalkingRobot controllerRobot;

   private final YoDouble desiredBodyHeight = new YoDouble("desiredBodyHeight", registry);
   private final YoDouble desiredSwingHeight = new YoDouble("desiredSwingHeight", registry);
   private final YoDouble desiredSwingDuration = new YoDouble("desiredSwingDuration", registry);
   private final YoFrameVector2D desiredCoMVelocity;

   private final YoDouble supportLegLengthKp = new YoDouble("supportLegLengthKp", registry);
   private final YoDouble supportLegLengthKd = new YoDouble("supportLegLengthKd", registry);
   private final YoDouble torsoPitchKp = new YoDouble("supportPitchKp", registry);
   private final YoDouble torsoPitchKd = new YoDouble("supportPitchKd", registry);
   private final YoDouble torsoRollKp = new YoDouble("supportRollKp", registry);
   private final YoDouble torsoRollKd = new YoDouble("supportRollKd", registry);

   //Debug
   private final YoDouble comVelocityFromFoot = new YoDouble("comVelocityFromFoot", registry);

   private final YoDouble comVelocityAdjustmentGain = new YoDouble("comVelocityAdjustmentGain", registry);
   private final YoDouble swingFootStepAdjustmentGain = new YoDouble("swingFootStepAdjustmentGain", registry);
   private final YoDouble swingFootHeightKp = new YoDouble("swingFootHeightKp", registry);
   private final YoDouble swingFootHeightKd = new YoDouble("swingFootHeightKd", registry);
   private final YoDouble swingHipPitchKp = new YoDouble("swingHipPitchKp", registry);
   private final YoDouble swingHipPitchKd = new YoDouble("swingHipPitchKd", registry);
   private final YoDouble swingHipRollKp = new YoDouble("swingHipRollKp", registry);
   private final YoDouble swingHipRollKd = new YoDouble("swingHipRollKd", registry);

   private final YoFrameVector3D desiredLIPMForce;
   private final YoFrameVector3D desiredLIPMAcceleration;

   private final SideDependentList<YoFramePoint3D> desiredTouchdownPositions = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> desiredFootPositions = new SideDependentList<>();
   private final SideDependentList<YoFrameVector3D> desiredLegForceVectors = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> currentFootPositions = new SideDependentList<>();

   private enum LegStateName
   {Swing, Support}

   private final SideDependentList<StateMachine<LegStateName, State>> legStateMachines = new SideDependentList<>();
   private final SideDependentList<YoBoolean> hasFootHitGround = new SideDependentList<>();

   public BWCPlanarWalkingController(BWCPlanarWalkingRobot controllerRobot, RobotSide initialSwingSide)
   {
      this.controllerRobot = controllerRobot;
      desiredLIPMForce = new YoFrameVector3D("desiredLIPMForce", controllerRobot.getWorldFrame(), registry);
      desiredLIPMAcceleration = new YoFrameVector3D("desiredLIPMAcceleration", controllerRobot.getWorldFrame(), registry);
      desiredCoMVelocity = new YoFrameVector2D("desiredCoMVelocity", controllerRobot.getWorldFrame(), registry);

      // set up our leg length gains for the leg length controller
      supportLegLengthKp.set(1000.0);
      supportLegLengthKd.set(750.0);

      swingFootHeightKp.set(1000.0);
      swingFootHeightKd.set(150.0);

      swingHipPitchKp.set(200.0);
      swingHipPitchKd.set(1.0);

      swingHipRollKp.set(200.0);
      swingHipRollKd.set(1.0);

      torsoPitchKp.set(250.0);
      torsoPitchKd.set(100.0);

      torsoRollKp.set(250.0);
      torsoRollKd.set(100.0);

      // These two account for mismatch between our robot and the LIPM
      comVelocityAdjustmentGain.set(0.85); // also accounts for converting velocity at the top of the arc to average velocity
      swingFootStepAdjustmentGain.set(0.85);

      desiredBodyHeight.set(0.75);
      desiredSwingHeight.set(0.1);
      desiredSwingDuration.set(0.5);

      for (RobotSide robotSide : RobotSide.values)
      {
         hasFootHitGround.put(robotSide, new YoBoolean(robotSide.getLowerCaseName() + "HasFootHitGround", registry));

         StateMachineFactory<LegStateName, State> stateMachineFactory = new StateMachineFactory<>(LegStateName.class);
         stateMachineFactory.setNamePrefix(robotSide.getLowerCaseName() + "LegState").setRegistry(registry).buildYoClock(controllerRobot.getTime());

         stateMachineFactory.addState(LegStateName.Support, new SupportFootState(robotSide));
         stateMachineFactory.addState(LegStateName.Swing, new SwingFootState(robotSide));

         stateMachineFactory.addDoneTransition(LegStateName.Swing, LegStateName.Support);
         stateMachineFactory.addTransition(LegStateName.Support, LegStateName.Swing, new StartSwingCondition(robotSide));

         LegStateName initialState = robotSide == initialSwingSide ? LegStateName.Swing : LegStateName.Support;
         legStateMachines.put(robotSide, stateMachineFactory.build(initialState));

         desiredTouchdownPositions.put(robotSide,
                                       new YoFramePoint3D(robotSide.getLowerCaseName() + "DesiredTouchdownPosition",
                                                          controllerRobot.getWorldFrame(),
                                                          registry));
         desiredFootPositions.put(robotSide,
                                  new YoFramePoint3D(robotSide.getLowerCaseName() + "DesiredFootPositions", controllerRobot.getWorldFrame(), registry));
         currentFootPositions.put(robotSide,
                                  new YoFramePoint3D(robotSide.getLowerCaseName() + "CurrentFootPositions", controllerRobot.getWorldFrame(), registry));
         desiredLegForceVectors.put(robotSide,
                                    new YoFrameVector3D(robotSide.getLowerCaseName() + "DesiredLegForceVector", controllerRobot.getWorldFrame(), registry));
      }

      registry.addChild(controllerRobot.getYoRegistry());
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
      controllerRobot.update();

      for (RobotSide robotSide : RobotSide.values)
      {
         legStateMachines.get(robotSide).doActionAndTransition();
      }
   }

   private class StartSwingCondition implements StateTransitionCondition
   {
      private final RobotSide candidateSwingState;

      public StartSwingCondition(RobotSide robotSide)
      {
         candidateSwingState = robotSide;
      }

      @Override
      public boolean testCondition(double timeInCurrentState)
      {
         if (hasFootHitGround.get(candidateSwingState.getOppositeSide()).getValue())
            return true;

         return false;
      }
   }

   private class SwingFootState implements State
   {
      private final RobotSide swingSide;
      private final YoPolynomial swingFootHeightTrajectory;
      private final YoPolynomial swingFootXTrajectory;
      private final YoPolynomial swingFootYTrajectory;
      private final PDController swingFootHeightController;
      private final PDController swingHipPitchController;
      private final PDController swingHipRollController;
      private final YoDouble swingLegFeedbackForce;
      private final YoDouble swingLegFeedforwardForce;
      private final YoDouble swingLegForce;
      private final YoDouble swingHipPitchTorque;
      private final YoDouble swingHipRollTorque;
      private final YoFramePoint3D footPositionAtStart;
      private final YoFramePoint2D footTouchdownPosition;
      private final YoDouble footDesiredPosition;

      private final YoFrameVector3D currentFootVelocity;

      public SwingFootState(RobotSide swingSide)
      {
         this.swingSide = swingSide;

         swingFootHeightTrajectory = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootHeight", 5, registry);
         swingFootXTrajectory = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootX", 4, registry);
         swingFootYTrajectory = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootY", 4, registry);
         swingFootHeightController = new PDController(swingFootHeightKp,
                                                      swingFootHeightKd,
                                                      swingSide.getLowerCaseName() + "SwingFootHeightController",
                                                      registry);
         swingHipPitchController = new PDController(swingHipPitchKp, swingHipPitchKd, swingSide.getLowerCaseName() + "SwingHipPitchController", registry);
         swingHipRollController = new PDController(swingHipRollKp, swingHipRollKd, swingSide.getLowerCaseName() + "SwingHipRollController", registry);
         swingLegFeedbackForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredSwingLegFeedbackForce", registry);
         swingLegFeedforwardForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredSwingLegFeedforwardForce", registry);
         swingLegForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredSwingLegForce", registry);
         swingHipPitchTorque = new YoDouble(swingSide.getLowerCaseName() + "DesiredSwingHipPitchTorque", registry);
         swingHipRollTorque = new YoDouble(swingSide.getLowerCaseName() + "DesiredSwingHipRollTorque", registry);
         footPositionAtStart = new YoFramePoint3D(swingSide.getLowerCaseName() + "FootPositionAtStart", controllerRobot.getCenterOfMassFrame(), registry);
         footTouchdownPosition = new YoFramePoint2D(swingSide.getLowerCaseName() + "FootTouchdownPosition", controllerRobot.getCenterOfMassFrame(), registry);
         footDesiredPosition = new YoDouble(swingSide.getLowerCaseName() + "FootDesiredPosition", registry);

         currentFootVelocity = new YoFrameVector3D(swingSide.getLowerCaseName() + "CurrentFootVelocity", controllerRobot.getWorldFrame(), registry);
      }

      @Override
      public void onEntry()
      {
         FramePoint3D footPosition = new FramePoint3D(controllerRobot.getFootFrame(swingSide));
         footPosition.changeFrame(controllerRobot.getWorldFrame());
         footPositionAtStart.setMatchingFrame(footPosition);

         hasFootHitGround.get(swingSide).set(false);

         double initialTime = 0.0;
         double swingDuration = desiredSwingDuration.getDoubleValue();
         double initialHeight = footPosition.getZ();
         double initialVelocity = 0.0;
         double finalHeight = 0.0;
         double finalVelocity = 0.0;
         swingFootHeightTrajectory.setQuarticUsingWayPoint(initialTime,
                                                           0.5 * swingDuration,
                                                           swingDuration,
                                                           initialHeight,
                                                           initialVelocity,
                                                           desiredSwingHeight.getDoubleValue(),
                                                           finalHeight,
                                                           finalVelocity);

         computeDesiredTouchdownPositionVector(0.00001, footTouchdownPosition);
         swingFootXTrajectory.setCubic(0.0, swingDuration, footPositionAtStart.getX(), footTouchdownPosition.getX());
         swingFootYTrajectory.setCubic(0.0, swingDuration, footPositionAtStart.getY(), footTouchdownPosition.getY());
      }

      @Override
      public void doAction(double timeInState)
      {
         swingFootHeightTrajectory.compute(timeInState);

         FramePoint3D footPosition = new FramePoint3D(controllerRobot.getFootFrame(swingSide));
         footPosition.changeFrame(controllerRobot.getWorldFrame());

         double currentHeight = footPosition.getZ();
         double desiredHeight = swingFootHeightTrajectory.getValue();
         // This is approximately the right velocity. Good enough for stability, but not great overall.
         Twist footTwist = new Twist();
         //         controllerRobot.getFootFrame(swingSide).getTwistRelativeToOther(controllerRobot.getWorldFrame(), footTwist);
         controllerRobot.getFootFrame(swingSide).getTwistRelativeToOther(controllerRobot.getCenterOfMassFrame(), footTwist);
         currentFootVelocity.setMatchingFrame(footTwist.getLinearPart());
         double currentVelocity = currentFootVelocity.getZ();
         double desiredVelocity = swingFootHeightTrajectory.getVelocity();

         swingLegFeedbackForce.set(swingFootHeightController.compute(currentHeight, desiredHeight, currentVelocity, desiredVelocity));
         swingLegFeedforwardForce.set(swingFootHeightTrajectory.getAcceleration() * controllerRobot.getFootMass());

         swingLegForce.set(swingLegFeedbackForce.getDoubleValue() + swingLegFeedforwardForce.getValue());

         // set the desired torque to the knee joint to achieve the desired swing foot height
         controllerRobot.getKneeJoint(swingSide).setTau(swingLegForce.getDoubleValue());

         computeDesiredTouchdownPositionVector(timeInState, footTouchdownPosition);
         swingFootXTrajectory.setCubic(0.0, desiredSwingDuration.getDoubleValue(), footPositionAtStart.getX(), footTouchdownPosition.getX());
         swingFootYTrajectory.setCubic(0.0, desiredSwingDuration.getDoubleValue(), footPositionAtStart.getY(), footTouchdownPosition.getY());
         swingFootXTrajectory.compute(timeInState);
         swingFootYTrajectory.compute(timeInState);

         // Calculate the hip pitch torque
         double desiredFootPositionX = swingFootXTrajectory.getValue();
         footDesiredPosition.set(desiredFootPositionX);
         double desiredFootVelocityX = swingFootXTrajectory.getVelocity();

         footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
         double currentFootPositionX = footPosition.getX();
         double currentFootVelocityX = controllerRobot.getVelocityOfFootRelativeToCoM(swingSide).getX();
         double hipPitchTorque = swingHipPitchController.compute(currentFootPositionX, desiredFootPositionX, currentFootVelocityX, desiredFootVelocityX);
         swingHipPitchTorque.set(-hipPitchTorque);

         controllerRobot.getHipPitchJoint(swingSide).setTau(swingHipPitchTorque.getDoubleValue());

         // Calculate the hip roll torque //TODO: verify these signs
         double desiredFootPositionY = swingFootYTrajectory.getValue();
         footDesiredPosition.set(desiredFootPositionY);
         double desiredFootVelocityY = swingFootYTrajectory.getVelocity();

//         footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
         double currentFootPositionY = footPosition.getY();
         double currentFootVelocityY = controllerRobot.getVelocityOfFootRelativeToCoM(swingSide).getY();
         double hipRollTorque = swingHipRollController.compute(currentFootPositionY, desiredFootPositionY, currentFootVelocityY, desiredFootVelocityY);
         swingHipRollTorque.set(hipRollTorque); //TODO check this sign

         //TODO: finish adding hip roll to the robot definition and robot model
         controllerRobot.getHipRollJoint(swingSide).setTau(swingHipRollTorque.getDoubleValue());

         // Set the desired foot position in world
         footPosition.setX(desiredFootPositionX);
         footPosition.setY(desiredFootPositionY);
         footPosition.changeFrame(controllerRobot.getWorldFrame());
         footPosition.setZ(desiredHeight);
         desiredFootPositions.get(swingSide).setMatchingFrame(footPosition);

         // Set the desired foot position in com frame
         footPosition.setZ(0.0);
         footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
         footPosition.setX(footTouchdownPosition.getX());
         footPosition.setY(footTouchdownPosition.getY());
         desiredTouchdownPositions.get(swingSide).setMatchingFrame(footPosition);
      }

      // Compute the caputure point
      private double computeDesiredTouchdownPosition()
      {
         double currentCoMVelocity = -controllerRobot.getVelocityOfFootRelativeToCoM(swingSide.getOppositeSide()).getX();
         comVelocityFromFoot.set(currentCoMVelocity);
         double omega = Math.sqrt(9.81 / desiredBodyHeight.getDoubleValue());
         double adjustedVelocity = comVelocityAdjustmentGain.getDoubleValue() * desiredCoMVelocity.getX();
         double velocitySquareDifference = currentCoMVelocity * currentCoMVelocity - adjustedVelocity * adjustedVelocity;
         double sign = Math.signum(currentCoMVelocity);

         if (desiredCoMVelocity.getX() != 0)
            sign = Math.signum(velocitySquareDifference) * Math.signum(desiredCoMVelocity.getX());

         return sign * Math.sqrt(Math.abs(velocitySquareDifference)) / omega;

         // return swingFootStepAdjustmentGain.getDoubleValue() / omega * currentCoMVelocity;
      }

      private void computeDesiredTouchdownPositionVector(double timeInState, YoFramePoint2D touchdownPositionToPack)
      {
         //FIXME remove this garbage
         double hipOffset = swingSide.negateIfRightSide(0.025); //TODO pass this from the robot definition
         FrameVector2D currentCoMVelocity = new FrameVector2D(controllerRobot.getVelocityOfFootRelativeToCoM(swingSide.getOppositeSide()));
         currentCoMVelocity.scale(-1.0); // We need to negate the velocity to get CoM relative to foot
         double omega = Math.sqrt(9.81 / desiredBodyHeight.getDoubleValue());
         FrameVector2D adjustedVelocity = new FrameVector2D(desiredCoMVelocity);
         adjustedVelocity.scale(comVelocityAdjustmentGain.getDoubleValue());

         //The adjustment to the footstep for a desired COM velocity is undefined at t=0, so we fudge it a little.
         if (timeInState == 0.0)
         {
            timeInState = 0.00001;
         }

         // Deviation to step from the capture point given a desired velocity. Should be 0 with no desired velocity.
         double capturePointX = 1 / omega * currentCoMVelocity.getX();
         double deltaX = -desiredCoMVelocity.getX() * timeInState / (Math.exp(omega * timeInState) - 1);
         double touchdownX = capturePointX + deltaX;

         //TODO: replace 0.1 here with the previous step width?
         //TODO: consider constructing a heading vector and controlling part of the step locations using a PD controller
         double offsetFromSidewaysRocking = swingSide.negateIfRightSide(0.1 / (1 + Math.exp(omega * timeInState)));
//         double capturePointY = 1 / omega * currentCoMVelocity.getY() + hipOffset;
         double capturePointY = 1 / omega * currentCoMVelocity.getY() + offsetFromSidewaysRocking + hipOffset;
         double deltaY = -desiredCoMVelocity.getY() * timeInState / (Math.exp(omega * timeInState) - 1);
         double touchdownY = capturePointY + deltaY;

         touchdownPositionToPack.set(touchdownX, touchdownY);
      }

      @Override
      public void onExit(double timeInState)
      {
         desiredTouchdownPositions.get(swingSide).setToNaN();
         desiredFootPositions.get(swingSide).setToNaN();
      }

      @Override
      public boolean isDone(double timeInState)
      {
         // terminate if we make contact
         if (timeInState > 0.1 * desiredSwingDuration.getDoubleValue() && controllerRobot.getGroundContactPoint(swingSide).getInContact().getValue())
            return true;

         return timeInState > desiredSwingDuration.getDoubleValue();
      }
   }

   private class SupportFootState implements State
   {
      private final RobotSide supportSide;

      private final PDController supportLegLengthController;
      private final PDController supportPitchPostureController;
      private final PDController supportRollPostureController;
      private final YoDouble supportLegDesiredKneeForce;
      private final YoDouble supportLegFeedbackKneeForce;

      public SupportFootState(RobotSide supportSide)
      {
         this.supportSide = supportSide;

         supportLegDesiredKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SupportLegDesiredKneeForce", registry);
         supportLegFeedbackKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SupportLegFeedbackKneeForce", registry);
         supportLegLengthController = new PDController(supportLegLengthKp,
                                                       supportLegLengthKd,
                                                       supportSide.getLowerCaseName() + "SupportLegLengthController",
                                                       registry);
         supportPitchPostureController = new PDController(torsoPitchKp, torsoPitchKd, supportSide.getLowerCaseName() + "SupportPostureController", registry);
         supportRollPostureController = new PDController(torsoRollKp, torsoRollKd, supportSide.getLowerCaseName() + "SupportRollPostureController", registry);
      }

      @Override
      public void onEntry()
      {
         hasFootHitGround.get(supportSide).set(true);
      }

      @Override
      public void doAction(double timeInState)
      {
         FramePoint3D footPosition = new FramePoint3D(controllerRobot.getFootFrame(supportSide));
         footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());

         currentFootPositions.get(supportSide).setMatchingFrame(footPosition);

         FrameVector3D pendulum = new FrameVector3D(footPosition);

         double omega = Math.sqrt(9.81 / Math.abs(pendulum.getZ()));
         desiredLIPMAcceleration.setMatchingFrame(pendulum);
         desiredLIPMAcceleration.setZ(0.0);
         desiredLIPMAcceleration.scale(omega * omega);

         // the foot is resting on the ground, so we don't need to include its weight here
         double weight = 9.81 * (controllerRobot.getMass() - controllerRobot.getFootMass());
         desiredLIPMForce.set(desiredLIPMAcceleration);
         desiredLIPMForce.addZ(weight);

         desiredLegForceVectors.get(supportSide).set(desiredLIPMForce);

         double bodyHeight = controllerRobot.getCenterOfMassPosition().getZ();
         double bodyHeightVelocity = controllerRobot.getCenterOfMassVelocity().getZ();

         double desiredHeight = desiredBodyHeight.getDoubleValue();
         double desiredHeightVelocity = 0.0;
         // compute and record the desired torque to hold the leg at the desired length
         double effort = supportLegLengthController.compute(bodyHeight, desiredHeight, bodyHeightVelocity, desiredHeightVelocity);
         supportLegFeedbackKneeForce.set(effort);
         supportLegDesiredKneeForce.set(effort + desiredLIPMForce.norm());

         // set the desired torque to the knee joint to hold the leg at the desired length
         controllerRobot.getKneeJoint(supportSide).setTau(-supportLegDesiredKneeForce.getDoubleValue());

         FrameQuaternion baseOrientation = new FrameQuaternion(controllerRobot.getFloatingJoint().getFrameAfterJoint());
         baseOrientation.changeFrame(controllerRobot.getWorldFrame());
         effort = supportPitchPostureController.compute(baseOrientation.getPitch(),
                                                        0.0,
                                                        controllerRobot.getFloatingJoint().getFrameAfterJoint().getTwistOfFrame().getAngularPartY(),
                                                        0.0);
         controllerRobot.getHipPitchJoint(supportSide).setTau(-effort);

         effort = supportRollPostureController.compute(baseOrientation.getRoll(),
                                                       0.0,
                                                       controllerRobot.getFloatingJoint().getFrameAfterJoint().getTwistOfFrame().getAngularPartX(),
                                                       0.0);

         controllerRobot.getHipRollJoint(supportSide).setTau(-effort); //TODO: verify sign
      }

      @Override
      public void onExit(double timeInState)
      {
         desiredLegForceVectors.get(supportSide).setToNaN();
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return false;
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (RobotSide robotSide : RobotSide.values)
      {
         group.addChild(newYoGraphicPoint3D(robotSide.getLowerCaseName() + "TouchdownPosition", desiredTouchdownPositions.get(robotSide), 0.015, Red()));
         group.addChild(newYoGraphicPoint3D(robotSide.getLowerCaseName() + "CurrentFootPosition", desiredFootPositions.get(robotSide), 0.015, Blue()));
         group.addChild(newYoGraphicArrow3D(robotSide.getLowerCaseName() + "GroundReactionForce",
                                            currentFootPositions.get(robotSide),
                                            desiredLegForceVectors.get(robotSide),
                                            0.01,
                                            Red()));
      }
      group.setVisible(true);
      return group;
   }
}
