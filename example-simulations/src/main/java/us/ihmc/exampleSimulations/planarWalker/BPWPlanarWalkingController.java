package us.ihmc.exampleSimulations.planarWalker;

import org.lwjgl.system.linux.Stat;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BPWPlanarWalkingController implements Controller
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final BPWPLanarWalkingRobot controllerRobot;

    private final YoDouble desiredSupportLegLength = new YoDouble("desiredLegLength", registry);
    private final YoDouble supportingLegLengthKp = new YoDouble("supportingLegLengthKp", registry);
    private final YoDouble supportingLegLengthKd = new YoDouble("supportingLegLengthKd", registry);

    private enum LegStateName {Swing, Support};
    private final SideDependentList<StateMachine<LegStateName, State>> legStateMachines = new SideDependentList<>();


    public BPWPlanarWalkingController( BPWPLanarWalkingRobot controllerRobot)
    {
        this.controllerRobot = controllerRobot;

        // Set up the leg length controller gains
        supportingLegLengthKp.set(1500.0);
        supportingLegLengthKd.set(50.0);

        desiredSupportLegLength.set((BPWPlanarWalkingRobotDefinition.shinLength + BPWPlanarWalkingRobotDefinition.thighLength) / 2.0);

        // Setting up the leg length controllers
        for( RobotSide robotSide : RobotSide.values)
        {
            StateMachineFactory<LegStateName, State> stateMachine = new StateMachineFactory<>(LegStateName.class);
            stateMachine.setNamePrefix(robotSide.getLowerCaseName() + "LegState").setRegistry(registry);
            stateMachine.addState(LegStateName.Support, new SupportFootState(robotSide));
            legStateMachines.put(robotSide, stateMachine.build(LegStateName.Support));
        }

        registry.addChild(controllerRobot.getYoRegistry());



    }

    @Override
    public void doControl()
    {
        controllerRobot.update();

        for( RobotSide robotSide : RobotSide.values)
        {
           legStateMachines.get(robotSide).doActionAndTransition();
        }

    }

    private class SwingFootState implements State
    {

        @Override
        public void onEntry() {

        }

        @Override
        public void doAction(double timeInState) {

        }

        @Override
        public void onExit(double timeInState) {

        }

        @Override
        public boolean isDone(double timeInState) {
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

            supportLegDesiredKneeForce = new YoDouble(supportSide.getShortLowerCaseName() + "SupportLegDesiredKneeForce", registry);
            supportLegLengthController = new PDController(supportingLegLengthKp, supportingLegLengthKd, supportSide.getLowerCaseName() + "SupportLegLengthController", registry);
        }

        @Override
        public void onEntry() {

        }

        @Override
        public void doAction(double timeInState)
        {
            double legLength = controllerRobot.getLegLength(supportSide);
            double legLengthVel = -controllerRobot.getKneeJoint(supportSide).getQd();

            double desiredLegLength = desiredSupportLegLength.getDoubleValue();
            double desiredKneeVelocity = 0.0;

            // Compute the required torque to stay at the desired leg length
            double torque = supportLegLengthController.compute(legLength, desiredLegLength, legLengthVel, desiredKneeVelocity);

            controllerRobot.getKneeJoint(supportSide).setTau(-torque);
        }

        @Override
        public void onExit(double timeInState) {

        }

        @Override
        public boolean isDone(double timeInState) {
            return false;
        }
    }


    @Override
    public YoRegistry getYoRegistry() {
        return registry;
    }

}
