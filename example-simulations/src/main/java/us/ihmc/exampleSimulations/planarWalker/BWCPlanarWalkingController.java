package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class BWCPlanarWalkingController implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BWCPlanarWalkingRobot controllerRobot;

   private final YoDouble desiredBodyHeight = new YoDouble("desiredBodyHeight", registry);
   private final YoDouble desiredSwingHeight = new YoDouble("desiredSwingHeight", registry);
   private final YoDouble desiredSwingDuration = new YoDouble("desiredSwingDuration", registry);

   private final YoDouble supportLegLengthKp = new YoDouble("supportLegLengthKp", registry);
   private final YoDouble supportLegLengthKd = new YoDouble("supportLegLengthKd", registry);

   private final YoDouble swingFootStepAdjustmentGain = new YoDouble("swingFootStepAdjustmentGain", registry);
   private final YoDouble swingFootHeightKp = new YoDouble("swingFootHeightKp", registry);
   private final YoDouble swingFootHeightKd = new YoDouble("swingFootHeightKd", registry);
   private final YoDouble swingHipPitchKp = new YoDouble("swingHipPitchKp", registry);
   private final YoDouble swingHipPitchKd = new YoDouble("swingHipPitchKd", registry);

   private final YoFrameVector3D desiredLIPMForce;

   private enum LegStateName {Swing, Support}
   private final SideDependentList<StateMachine<LegStateName, State>> legStateMachines = new SideDependentList<>();
   private final SideDependentList<YoBoolean> hasFootHitGround = new SideDependentList<>();

   public BWCPlanarWalkingController(BWCPlanarWalkingRobot controllerRobot, RobotSide initialSwingSide)
   {
      this.controllerRobot = controllerRobot;
      desiredLIPMForce = new YoFrameVector3D("desiredLIPMForce", controllerRobot.getWorldFrame(), registry);

      // set up our leg length gains for the leg length controller
      supportLegLengthKp.set(500.0);
      supportLegLengthKd.set(150.0);

      swingFootHeightKp.set(500.0);
      swingFootHeightKd.set(150.0);

      swingHipPitchKp.set(200.0);
      swingHipPitchKd.set(1.0);

      swingFootStepAdjustmentGain.set(1.5);

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
      private final PDController swingFootHeightController;
      private final PDController swingHipPitchController;
      private final YoDouble swingLegForce;
      private final YoDouble swingHipForce;
      private final YoFramePoint3D footPositionAtStart;
      private final YoDouble footTouchdownPosition;
      private final YoDouble footDesiredPosition;

      public SwingFootState(RobotSide swingSide)
      {
         this.swingSide = swingSide;

         swingFootHeightTrajectory = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootHeight", 5, registry);
         swingFootXTrajectory = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootX", 4, registry);
         swingFootHeightController = new PDController(swingFootHeightKp, swingFootHeightKd, swingSide.getLowerCaseName() + "SwingFootHeightController", registry);
         swingHipPitchController = new PDController(swingHipPitchKp, swingHipPitchKd, swingSide.getLowerCaseName() + "SwingHipPitchController", registry);
         swingLegForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredLegForce", registry);
         swingHipForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredHipForce", registry);
         footPositionAtStart = new YoFramePoint3D(swingSide.getLowerCaseName() + "FootPositionAtStart", controllerRobot.getCenterOfMassFrame(), registry);
         footTouchdownPosition = new YoDouble(swingSide.getLowerCaseName() + "FootTouchdownPosition", registry);
         footDesiredPosition = new YoDouble(swingSide.getLowerCaseName() + "FootDesiredPosition", registry);
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

         swingFootXTrajectory.setCubic(0.0, swingDuration, footPositionAtStart.getX(), computeDesiredTouchdownPosition());
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
         double currentVelocity = controllerRobot.getKneeJoint(swingSide).getQd();
         double desiredVelocity = swingFootHeightTrajectory.getVelocity();

         double legForce = swingFootHeightController.compute(currentHeight, desiredHeight, currentVelocity, desiredVelocity);
         swingLegForce.set(legForce);

         // set the desired torque to the knee joint to achieve the desired swing foot height
         controllerRobot.getKneeJoint(swingSide).setTau(swingLegForce.getDoubleValue());

         footTouchdownPosition.set(computeDesiredTouchdownPosition());
         swingFootXTrajectory.setCubic(0.0, desiredSwingDuration.getDoubleValue(), footPositionAtStart.getX(), footTouchdownPosition.getDoubleValue());
         swingFootXTrajectory.compute(timeInState);

         double desiredFootPositionX = swingFootXTrajectory.getValue();
         footDesiredPosition.set(desiredFootPositionX);
         double desiredFootVelocityX = swingFootXTrajectory.getVelocity();

         footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
         double currentFootPositionX = footPosition.getX();
         double currentFootVelocityX = -controllerRobot.getHipJoint(swingSide).getQd() * controllerRobot.getLegLength(swingSide); // FIXME make this have some actual value
         double hipForce = swingHipPitchController.compute(currentFootPositionX, desiredFootPositionX, currentFootVelocityX, desiredFootVelocityX);
         swingHipForce.set(-hipForce);

         controllerRobot.getHipJoint(swingSide).setTau(swingHipForce.getDoubleValue());
      }

      private double computeDesiredTouchdownPosition()
      {
         double currentCoMVelocity = -controllerRobot.getVelocityOfFootRelativeToCoM(swingSide.getOppositeSide()).getX();
         double omega = Math.sqrt(9.81 / desiredBodyHeight.getDoubleValue());
         return swingFootStepAdjustmentGain.getDoubleValue() / omega * currentCoMVelocity;
      }

      @Override
      public void onExit(double timeInState)
      {

      }

      @Override
      public boolean isDone(double timeInState)
      {
         if (timeInState > 0.1 * desiredSwingDuration.getDoubleValue() && controllerRobot.getGroundContactPoint(swingSide).getInContact().getValue())
            return true;

         return timeInState > desiredSwingDuration.getDoubleValue();
      }
   }

   private class SupportFootState implements State
   {
      private final RobotSide supportSide;

      private final PDController supportLegLengthController;
      private final YoDouble supportLegDesiredKneeForce;
      private final YoDouble supportLegFeedbackKneeForce;

      public SupportFootState(RobotSide supportSide)
      {
         this.supportSide = supportSide;

         supportLegDesiredKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SupportLegDesiredKneeForce", registry);
         supportLegFeedbackKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SupportLegFeedbackKneeForce", registry);
         supportLegLengthController = new PDController(supportLegLengthKp, supportLegLengthKd, supportSide.getLowerCaseName() + "SupportLegLengthController", registry);
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

         FrameVector3D force = new FrameVector3D(footPosition);
         force.setY(0.0);
         force.normalize();
         double weight = 9.81 * controllerRobot.getMass();
         force.scale(0.75 * weight / force.getZ()); // TODO fake scal
         desiredLIPMForce.setMatchingFrame(force);

         double bodyHeight = controllerRobot.getCenterOfMassPosition().getZ();
         double bodyHeightVelocity = controllerRobot.getCenterOfMassVelocity().getZ();

         double desiredHeight = desiredBodyHeight.getDoubleValue();
         double desiredHeightVelocity = 0.0;
         // compute and record the desired torque to hold the leg at the desired length
         double torque = supportLegLengthController.compute(bodyHeight, desiredHeight, bodyHeightVelocity, desiredHeightVelocity);
         supportLegFeedbackKneeForce.set(torque);
         supportLegDesiredKneeForce.set(torque + force.norm());

         // set the desired torque to the knee joint to hold the leg at the desired length
         controllerRobot.getKneeJoint(supportSide).setTau(-supportLegDesiredKneeForce.getDoubleValue());
         controllerRobot.getHipJoint(supportSide).setTau(0.0);
      }

      @Override
      public void onExit(double timeInState)
      {

      }

      @Override
      public boolean isDone(double timeInState)
      {
         return false;
      }
   }
}
