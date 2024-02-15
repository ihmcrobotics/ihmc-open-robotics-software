package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BWCPlanarWalkingController implements Controller
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BWCPlanarWalkingRobot controllerRobot;

   private final YoDouble desirdSupportLegLength = new YoDouble("desiredLegLength", registry);
   private final YoDouble supportLegLengthKp = new YoDouble("supportLegLengthKp", registry);
   private final YoDouble supportLegLengthKd = new YoDouble("supportLegLengthKd", registry);

   private enum LegStateName {Swing, Support}
   private final SideDependentList<StateMachine<LegStateName, State>> legStateMachines = new SideDependentList<>();

   public BWCPlanarWalkingController(BWCPlanarWalkingRobot controllerRobot)
   {
      this.controllerRobot = controllerRobot;

      // set up our leg length gains for the leg length controller
      supportLegLengthKp.set(1500.0);
      supportLegLengthKd.set(50.0);

      desirdSupportLegLength.set((BWCPlanarWalkingRobotDefinition.shinLength + BWCPlanarWalkingRobotDefinition.thighLength) / 2.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         StateMachineFactory<LegStateName, State> stateMachineFactory = new StateMachineFactory<>(LegStateName.class);
         stateMachineFactory.setNamePrefix(robotSide.getLowerCaseName() + "LegState").setRegistry(registry).buildYoClock(controllerRobot.getTime());
         stateMachineFactory.addState(LegStateName.Support, new SupportFootState(robotSide));
         legStateMachines.put(robotSide, stateMachineFactory.build(LegStateName.Support));
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

   private class SwingFootState implements State
   {
      @Override
      public void onEntry()
      {

      }

      @Override
      public void doAction(double timeInState)
      {

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

   private class SupportFootState implements State
   {
      private final RobotSide supportSide;

      private final PDController supportLegLengthController;
      private final YoDouble supportLegDesiredKneeForce;

      public SupportFootState(RobotSide supportSide)
      {
         this.supportSide = supportSide;

         supportLegDesiredKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SUpportLegDesiredKneeForce", registry);
         supportLegLengthController = new PDController(supportLegLengthKp, supportLegLengthKd, supportSide.getLowerCaseName() + "SupportLegLengthController", registry);
      }

      @Override
      public void onEntry()
      {

      }

      @Override
      public void doAction(double timeInState)
      {
         double legLength = controllerRobot.getLegLength(supportSide);
         double legLengthVelocity = -controllerRobot.getKneeJoint(supportSide).getQd();

         double desiredLegLength = desirdSupportLegLength.getDoubleValue();
         double desiredKneeVelocity = 0.0;
         // compute and record the desired torque to hold the leg at the desired length
         double torque = supportLegLengthController.compute(legLength, desiredLegLength, legLengthVelocity, desiredKneeVelocity);
         supportLegDesiredKneeForce.set(torque);

         // set the desired torque to the knee joint to hold the leg at the desired length
         controllerRobot.getKneeJoint(supportSide).setTau(-torque);
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
