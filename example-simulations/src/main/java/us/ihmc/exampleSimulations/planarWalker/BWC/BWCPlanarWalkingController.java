package us.ihmc.exampleSimulations.planarWalker.BWC;

import us.ihmc.commonWalkingControlModules.controlModules.foot.SwingState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
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
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class BWCPlanarWalkingController implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final BWCPlanarWalkingRobot controllerRobot;
   private final SideDependentList<PDController> legLengthControllers = new SideDependentList<>();

   private final SideDependentList<YoDouble> desiredKneeForces = new SideDependentList<>();

   private final SideDependentList<YoDouble> desiredSupportLegLength = new SideDependentList<>();
   private final SideDependentList<YoDouble> desiredSwingLegLength = new SideDependentList<>();

   private final YoDouble desiredSwingDuration = new YoDouble("desiredSwingDuration", registry);
   private final YoDouble desiredSwingHeight = new YoDouble("desiredSwingHeight", registry);

   private final YoDouble legLengthKp = new YoDouble("legLengthKp", registry);
   private final YoDouble legLengthKd = new YoDouble("legLengthKd", registry);

   private final YoDouble supportLegLengthKp = new YoDouble("supportLegLengthKp", registry);
   private final YoDouble supportLegLengthKd = new YoDouble("supportLegLengthKd", registry);

   private final YoDouble swingFootHeightKp = new YoDouble("swingFootHeightKp", registry);
   private final YoDouble swingFootHeightKd = new YoDouble("swingFootHeightKd", registry);

   private final YoDouble swingHipKp = new YoDouble("swingFootHeightKp", registry);
   private final YoDouble swingHipKd = new YoDouble("swingFootHeightKd", registry);

   private enum LegStateName {SWING, SUPPORT};

   private final SideDependentList<StateMachine<LegStateName, State>> legStateMachines = new SideDependentList();

   private final SideDependentList<YoBoolean> hasFootHitGround = new SideDependentList<>();

   public BWCPlanarWalkingController(BWCPlanarWalkingRobot controllerRobot)
   {
      this.controllerRobot = controllerRobot;

      legLengthKp.set(1500.0);
      legLengthKd.set(50.0);

      supportLegLengthKp.set(1500.0);
      supportLegLengthKd.set(50.0);

      swingFootHeightKp.set(200.0);
      swingFootHeightKd.set(1.0);

      PDController leftLegLengthController = new PDController(legLengthKp, legLengthKd, "leftLegLengthController", registry);
      PDController rightLegLengthController = new PDController(legLengthKp, legLengthKd, "rightLegLengthController", registry);
      legLengthControllers.put(RobotSide.LEFT, leftLegLengthController);
      legLengthControllers.put(RobotSide.RIGHT, rightLegLengthController);

      desiredSwingDuration.set(0.5);
      desiredSwingHeight.set(0.1);

      for (RobotSide robotSide : RobotSide.values())
      {
         desiredKneeForces.put(robotSide, new YoDouble(robotSide.getLowerCaseName()  + "LegDesiredKneeForce", registry));
         desiredSupportLegLength.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "DesiredSupportLegLength", registry));
         desiredSupportLegLength.get(robotSide).set((BWCPlanarWalkingRobotDefinition.shinLength + BWCPlanarWalkingRobotDefinition.thighLength) / 2);
         hasFootHitGround.put(robotSide, new YoBoolean(robotSide.getLowerCaseName() + "HasFootHitGround", registry));

         StateMachineFactory<LegStateName, State> stateStateMachineFactory = new StateMachineFactory<>(LegStateName.class);
         stateStateMachineFactory.setNamePrefix(robotSide.getLowerCaseName() + "LegState").setRegistry(registry).buildYoClock(controllerRobot.getTime());

         stateStateMachineFactory.addState(LegStateName.SUPPORT, new SupportFootState(robotSide));
         stateStateMachineFactory.addState(LegStateName.SWING, new SwingFootState(robotSide));

         stateStateMachineFactory.addDoneTransition(LegStateName.SWING, LegStateName.SUPPORT);
         stateStateMachineFactory.addTransition(LegStateName.SUPPORT, LegStateName.SWING, new StartSwingCondition(robotSide));

         legStateMachines.put(robotSide, stateStateMachineFactory.build(LegStateName.SUPPORT));
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

      for (RobotSide robotSide : RobotSide.values())
      {
         legStateMachines.get(robotSide).doActionAndTransition();
      }
   }

   private class SupportFootState implements State
   {
      private final RobotSide supportSide;

      private final PDController supportLegLengthController;

      private final YoDouble supportLegDesiredKneeForce;

      SupportFootState(RobotSide supportSide)
      {
         this.supportSide = supportSide;
         supportLegLengthController = new PDController(supportLegLengthKp, supportLegLengthKd, supportSide.getLowerCaseName() + "SupportLegLengthController", registry);

         supportLegDesiredKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SupportLegDesiredKneeForce", registry);
      }

      @Override
      public void onEntry()
      {
         hasFootHitGround.get(supportSide).set(true);
      }

      @Override
      public void doAction(double timeInState)
      {
         double kneePosition = controllerRobot.getLegLength(supportSide);
         double kneeVelocity = -controllerRobot.getKneeJoint(supportSide).getQd();

         double desiredKneePosition = desiredSupportLegLength.get(supportSide).getDoubleValue();
         double desiredKneeVelocity = 0.0;

         double torque = -supportLegLengthController.compute(kneePosition, desiredKneePosition, kneeVelocity, desiredKneeVelocity);

         supportLegDesiredKneeForce.set(torque);
         controllerRobot.getKneeJoint(supportSide).setTau(torque);
      }

      @Override
      public void onExit(double timeInState)
      {

      }
   }

   private class SwingFootState implements State
   {
      private final RobotSide swingSide;

      private final YoPolynomial swingFootHeightTrajectory;

      private final PDController swingFootHeightController;

      private final PDController swingHipPitchController;

      private final YoDouble swingLegDesiredKneeForce;

      SwingFootState(RobotSide swingSide)
      {
         this.swingSide = swingSide;

         swingFootHeightTrajectory = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootHeightTrajectory", 5, registry);

         swingFootHeightController = new PDController(swingFootHeightKp, swingFootHeightKd, swingSide.getLowerCaseName() + "SwingFootHeightController", registry);

         swingHipPitchController = new PDController(swingHipKp, swingHipKd, swingSide.getLowerCaseName() + "SwingHipPitchController", registry);

         swingLegDesiredKneeForce = new YoDouble(swingSide.getLowerCaseName() + "SwingLegDesiredKneeForce", registry);
      }

      @Override
      public void onEntry()
      {
         hasFootHitGround.get(swingSide).set(false);

         FramePoint3D footPosition = new FramePoint3D(controllerRobot.getFootFrame(swingSide));
         footPosition.changeFrame(controllerRobot.getWorldFrame());

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
      }

      @Override
      public void doAction(double timeInState)
      {
         // Knee Control
         swingFootHeightTrajectory.compute(timeInState);

         FramePoint3D footPosition = new FramePoint3D(controllerRobot.getFootFrame(swingSide));
         footPosition.changeFrame(controllerRobot.getWorldFrame());

         double currentHeight = footPosition.getZ();
         double desiredHeight = swingFootHeightTrajectory.getValue();
         double currentVelocity = controllerRobot.getKneeJoint(swingSide).getQd(); //This is approx correct, good enough but not perfect
         double desiredVelocity = swingFootHeightTrajectory.getVelocity();

         double legForce = swingFootHeightController.compute(currentHeight, desiredHeight, currentVelocity, desiredVelocity);

         swingLegDesiredKneeForce.set(legForce);
         controllerRobot.getKneeJoint(swingSide).setTau(legForce);

         // Hip Control
         double desiredFootPositionX = 0.0;
         double desiredFootVelocityX = 0.0;

         footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
         double currentFootPositionX = footPosition.getX();
         double currentFootVelocityX = -controllerRobot.getHipJoint(swingSide).getQd();

         double swingHipForce = -swingHipPitchController.compute(currentFootPositionX, desiredFootPositionX, currentFootVelocityX, desiredFootVelocityX);
         controllerRobot.getHipJoint(swingSide).setTau(swingHipForce);
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return hasFootHitGround.get(swingSide).getBooleanValue();
      }

      @Override
      public void onExit(double timeInState)
      {

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
}
