package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
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
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
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

   private final YoDouble supportLegLengthKp = new YoDouble("supportLegLengthKp", registry);
   private final YoDouble supportLegLengthKd = new YoDouble("supportLegLengthKd", registry);
   private final YoDouble supportPitchKp = new YoDouble("supportPitchKp", registry);
   private final YoDouble supportPitchKd = new YoDouble("supportPitchKd", registry);

   private final YoDouble swingFootStepAdjustmentGain = new YoDouble("swingFootStepAdjustmentGain", registry);
   private final YoDouble swingFootHeightKp = new YoDouble("swingFootHeightKp", registry);
   private final YoDouble swingFootHeightKd = new YoDouble("swingFootHeightKd", registry);
   private final YoDouble swingHipPitchKp = new YoDouble("swingHipPitchKp", registry);
   private final YoDouble swingHipPitchKd = new YoDouble("swingHipPitchKd", registry);

   private final YoFrameVector3D desiredLIPMForce;
   private final YoFrameVector3D desiredLIPMAcceleration;

   private final SideDependentList<YoFramePoint3D> desiredTouchdownPositions = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> desiredFootPositions = new SideDependentList<>();
   private final SideDependentList<YoFrameVector3D> desiredLegForceVectors = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> currentFootPositions = new SideDependentList<>();

   private enum LegStateName {Swing, Support}
   private final SideDependentList<StateMachine<LegStateName, State>> legStateMachines = new SideDependentList<>();
   private final SideDependentList<YoBoolean> hasFootHitGround = new SideDependentList<>();
   // This will hold the adjustable target x position
   private final YoDouble rxDistance;
   private final YoDouble desiredWalkingVelocity = new YoDouble("desiredWalkingVelocity", registry);

   private PDController armPitchController;
   private final YoDouble armPitchKp = new YoDouble("armPitchKp", registry);
   private final YoDouble armPitchKd = new YoDouble("armPitchKd", registry);


   public BWCPlanarWalkingController(BWCPlanarWalkingRobot controllerRobot, RobotSide initialSwingSide)
   {
      this.controllerRobot = controllerRobot;
      desiredLIPMForce = new YoFrameVector3D("desiredLIPMForce", controllerRobot.getWorldFrame(), registry);
      desiredLIPMAcceleration = new YoFrameVector3D("desiredLIPMAcceleration", controllerRobot.getWorldFrame(), registry);

      // set up our leg length gains for the leg length controller
      supportLegLengthKp.set(1000.0);
      supportLegLengthKd.set(750.0);

      swingFootHeightKp.set(1000.0);
      swingFootHeightKd.set(150.0);

      swingHipPitchKp.set(200.0);
      swingHipPitchKd.set(1.0);

      supportPitchKp.set(250.0);
      supportPitchKd.set(100.0);

      swingFootStepAdjustmentGain.set(0.85);

      desiredBodyHeight.set(0.75);
      desiredSwingHeight.set(0.1);
      desiredSwingDuration.set(0.5);

      // Initialize rxDistance with a default value
      this.rxDistance = new YoDouble("rxDistance", registry);
      this.rxDistance.set(0.0);

      // Existing initialization code
      desiredWalkingVelocity.set(0.0); // Default walking velocity in m/s (example value)

      // Value for Arms
      armPitchKp.set(100.0); // Example gain values
      armPitchKd.set(10.0);
      armPitchController = new PDController(armPitchKp, armPitchKd, "armPitchController", registry);


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

         desiredTouchdownPositions.put(robotSide, new YoFramePoint3D(robotSide.getLowerCaseName() + "DesiredTouchdownPosition", controllerRobot.getWorldFrame(), registry));
         desiredFootPositions.put(robotSide, new YoFramePoint3D(robotSide.getLowerCaseName() + "DesiredFootPositions", controllerRobot.getWorldFrame(), registry));
         currentFootPositions.put(robotSide, new YoFramePoint3D(robotSide.getLowerCaseName() + "CurrentFootPositions", controllerRobot.getWorldFrame(), registry));
         desiredLegForceVectors.put(robotSide, new YoFrameVector3D(robotSide.getLowerCaseName() + "DesiredLegForceVector", controllerRobot.getWorldFrame(), registry));
      }

      registry.addChild(controllerRobot.getYoRegistry());
   }

   private double computeDesiredArmPosition(RobotSide side) {

      return 0.0; // Arms to aim for a neutral position of 0 radians
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

      for (RobotSide robotSide : RobotSide.values) {
         legStateMachines.get(robotSide).doActionAndTransition();

         // Control code for the arms, using the armPitchController
         double currentPitch = controllerRobot.getArmJoint(robotSide).getQ();
         double desiredPitch = computeDesiredArmPosition(robotSide);
         double currentVelocity = controllerRobot.getArmJoint(robotSide).getQd();
         double desiredVelocity = 0;

         double controlEffort = armPitchController.compute(currentPitch, desiredPitch, currentVelocity, desiredVelocity);
         controllerRobot.getArmJoint(robotSide).setTau(controlEffort);
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
      private final YoDouble swingLegFeedbackForce;
      private final YoDouble swingLegFeedforwardForce;
      private final YoDouble swingLegForce;
      private final YoDouble swingHipForce;
      private final YoFramePoint3D footPositionAtStart;
      private final YoDouble footTouchdownPosition;
      private final YoDouble footDesiredPosition;

      private final YoFrameVector3D currentFootVelocity;

      public SwingFootState(RobotSide swingSide)
      {
         this.swingSide = swingSide;

         swingFootHeightTrajectory = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootHeight", 5, registry);
         swingFootXTrajectory = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootX", 4, registry);
         swingFootHeightController = new PDController(swingFootHeightKp, swingFootHeightKd, swingSide.getLowerCaseName() + "SwingFootHeightController", registry);
         swingHipPitchController = new PDController(swingHipPitchKp, swingHipPitchKd, swingSide.getLowerCaseName() + "SwingHipPitchController", registry);
         swingLegFeedbackForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredLegFeedbackForce", registry);
         swingLegFeedforwardForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredLegFeedforwardForce", registry);
         swingLegForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredLegForce", registry);
         swingHipForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredHipForce", registry);
         footPositionAtStart = new YoFramePoint3D(swingSide.getLowerCaseName() + "FootPositionAtStart", controllerRobot.getCenterOfMassFrame(), registry);
         footTouchdownPosition = new YoDouble(swingSide.getLowerCaseName() + "FootTouchdownPosition", registry);
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

//         swingFootXTrajectory.setCubic(0.0, swingDuration, footPositionAtStart.getX(), computeDesiredTouchdownPosition());
         swingFootXTrajectory.setCubic(0.0, swingDuration, footPositionAtStart.getX(), computeCapturePointBasedTouchdownPosition());
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
         controllerRobot.getFootFrame(swingSide).getTwistRelativeToOther(controllerRobot.getWorldFrame(), footTwist); currentFootVelocity.setMatchingFrame(footTwist.getLinearPart());
         double currentVelocity = currentFootVelocity.getZ(); double desiredVelocity = swingFootHeightTrajectory.getVelocity();

         swingLegFeedbackForce.set(swingFootHeightController.compute(currentHeight, desiredHeight, currentVelocity, desiredVelocity));
         swingLegFeedforwardForce.set(swingFootHeightTrajectory.getAcceleration() * controllerRobot.getFootMass());

         swingLegForce.set(swingLegFeedbackForce.getDoubleValue() + swingLegFeedforwardForce.getValue());

         // set the desired torque to the knee joint to achieve the desired swing foot height
         controllerRobot.getKneeJoint(swingSide).setTau(swingLegForce.getDoubleValue());

//         footTouchdownPosition.set(computeDesiredTouchdownPosition());
         footTouchdownPosition.set(computeCapturePointBasedTouchdownPosition());
         swingFootXTrajectory.setCubic(0.0, desiredSwingDuration.getDoubleValue(), footPositionAtStart.getX(), footTouchdownPosition.getDoubleValue()); swingFootXTrajectory.compute(timeInState);

         double desiredFootPositionX = swingFootXTrajectory.getValue();
         footDesiredPosition.set(desiredFootPositionX);
         double desiredFootVelocityX = swingFootXTrajectory.getVelocity();

         footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
         double currentFootPositionX = footPosition.getX();
         double currentFootVelocityX = controllerRobot.getVelocityOfFootRelativeToCoM(swingSide).getX();
         double hipForce = swingHipPitchController.compute(currentFootPositionX, desiredFootPositionX, currentFootVelocityX, desiredFootVelocityX); swingHipForce.set(-hipForce);

         controllerRobot.getHipJoint(swingSide).setTau(swingHipForce.getDoubleValue());

         footPosition.setX(desiredFootPositionX);
         footPosition.changeFrame(controllerRobot.getWorldFrame());
         footPosition.setZ(desiredHeight);
         desiredFootPositions.get(swingSide).setMatchingFrame(footPosition);

         footPosition.setZ(0.0);
         footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
         footPosition.setX(footTouchdownPosition.getDoubleValue());
         desiredTouchdownPositions.get(swingSide).setMatchingFrame(footPosition);
      }

//      private double computeDesiredTouchdownPosition()
//      {
//         //this is the capture points
//         double currentCoMVelocity = -controllerRobot.getVelocityOfFootRelativeToCoM(swingSide.getOppositeSide()).getX();
//         double omega = Math.sqrt(9.81 / desiredBodyHeight.getDoubleValue());
//         return swingFootStepAdjustmentGain.getDoubleValue() / omega * currentCoMVelocity;
//      }

      private double computeCapturePointBasedTouchdownPosition()
      {
         double g = 9.81; // Acceleration due to gravity in m/s^2
         double height = desiredBodyHeight.getDoubleValue();
         double omega = Math.sqrt(g / height);
         double comVelocityX = controllerRobot.getCenterOfMassVelocity().getX();
         // Use the desired walking velocity directly to influence the capture point calculation
//         double adjustedVelocity = desiredWalkingVelocity.getDoubleValue();
//         double directionFactor = -1.0; // Positive for forward, negative for backward

         // Adjust the capture point by a certain gain if necessary and apply direction factor
//         return (comVelocityX / omega + rxDistance.getDoubleValue()) * directionFactor;
//         return adjustedVelocity / omega + rxDistance.getDoubleValue();
         return (comVelocityX / omega - rxDistance.getDoubleValue());
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
      private final PDController supportPostureController;
      private final YoDouble supportLegDesiredKneeForce;
      private final YoDouble supportLegFeedbackKneeForce;

      public SupportFootState(RobotSide supportSide)
      {
         this.supportSide = supportSide;

         supportLegDesiredKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SupportLegDesiredKneeForce", registry);
         supportLegFeedbackKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SupportLegFeedbackKneeForce", registry);
         supportLegLengthController = new PDController(supportLegLengthKp, supportLegLengthKd, supportSide.getLowerCaseName() + "SupportLegLengthController", registry);
         supportPostureController = new PDController(supportPitchKp, supportPitchKd, supportSide.getLowerCaseName() + "SupportPostureController", registry);

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
         double torque = supportLegLengthController.compute(bodyHeight, desiredHeight, bodyHeightVelocity, desiredHeightVelocity);
         supportLegFeedbackKneeForce.set(torque);
         supportLegDesiredKneeForce.set(torque + desiredLIPMForce.norm());

         // set the desired torque to the knee joint to hold the leg at the desired length
         controllerRobot.getKneeJoint(supportSide).setTau(-supportLegDesiredKneeForce.getDoubleValue());

         FrameQuaternion baseOrientation = new FrameQuaternion(controllerRobot.getFloatingJoint().getFrameAfterJoint());
         baseOrientation.changeFrame(controllerRobot.getWorldFrame());
         torque = supportPostureController.compute(baseOrientation.getPitch(), 0.0, controllerRobot.getFloatingJoint().getFrameAfterJoint().getTwistOfFrame().getAngularPartY(), 0.0);
         controllerRobot.getHipJoint(supportSide).setTau(-torque);
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
         group.addChild(newYoGraphicArrow3D(robotSide.getLowerCaseName() + "GroundReactionForce", currentFootPositions.get(robotSide), desiredLegForceVectors.get(robotSide), 0.01, Red()));
      }
      group.setVisible(true);
      return group;
   }
}