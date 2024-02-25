package us.ihmc.exampleSimulations.planarWalker;

import org.bytedeco.javacv.Frame;
import org.lwjgl.system.linux.Stat;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
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

public class BPWPlanarWalkingController implements Controller
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final BPWPLanarWalkingRobot controllerRobot;

    private final YoDouble desiredSupportLegLength = new YoDouble("desiredLegLength", registry);
    private final YoDouble desiredSwingHeight = new YoDouble("desiredSwingHeight", registry);
    private final YoDouble desiredSwingDuration = new YoDouble("desiredSwingDuration", registry);

    private final YoDouble supportingLegLengthKp = new YoDouble("supportingLegLengthKp", registry);
    private final YoDouble supportingLegLengthKd = new YoDouble("supportingLegLengthKd", registry);

    private final YoDouble swingFootHeightKp = new YoDouble("swingFootHeightKp", registry);
    private final YoDouble swingFootHeightKd = new YoDouble("swingFootHeightKd", registry);

    private final YoDouble swingHipPitchKp = new YoDouble("swingHipPitchKp", registry);
    private final YoDouble swingHipPitchKd = new YoDouble("swingHipPitchKd", registry);

    private enum LegStateName {Swing, Support};
    private final SideDependentList<StateMachine<LegStateName, State>> legStateMachines = new SideDependentList<>();
    private final SideDependentList<YoBoolean> hasFootHitGround = new SideDependentList<>();



    public BPWPlanarWalkingController( BPWPLanarWalkingRobot controllerRobot, RobotSide initialSwingSide)
    {
        this.controllerRobot = controllerRobot;

        // Set up the leg length controller gains
        supportingLegLengthKp.set(1500.0);
        supportingLegLengthKd.set(50.0);

        // Setting gains for swing
        swingFootHeightKp.set(200.0);
        swingFootHeightKd.set(1.0);

        desiredSupportLegLength.set((BPWPlanarWalkingRobotDefinition.shinLength + BPWPlanarWalkingRobotDefinition.thighLength) / 2.0);
        desiredSwingHeight.set(0.1);
        desiredSwingDuration.set(0.5);

        // Setting up the leg length controllers
        for( RobotSide robotSide : RobotSide.values)
        {
            hasFootHitGround.put(robotSide, new YoBoolean(robotSide.getLowerCaseName() + "HasFootHitGround", registry));

            StateMachineFactory<LegStateName, State> stateMachine = new StateMachineFactory<>(LegStateName.class);
            stateMachine.setNamePrefix(robotSide.getLowerCaseName() + "LegState").setRegistry(registry);

            stateMachine.buildYoClock(controllerRobot.getTime());

            stateMachine.addState(LegStateName.Support, new SupportFootState(robotSide));
            stateMachine.addState(LegStateName.Swing, new SwingFootState(robotSide));

            stateMachine.addDoneTransition(LegStateName.Swing, LegStateName.Support);
            stateMachine.addTransition(LegStateName.Support, LegStateName.Swing, new StartSwingCondition(robotSide));

            LegStateName initialState = robotSide == initialSwingSide ? LegStateName.Swing : LegStateName.Support;
            legStateMachines.put(robotSide, stateMachine.build(initialState));
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

    private class StartSwingCondition implements StateTransitionCondition
    {
        private final RobotSide candidateSwingState;
        public StartSwingCondition(RobotSide robotSide)
        {
            candidateSwingState = robotSide;
        }

        @Override
        public boolean testCondition(double timeInCurrentState) {
            if (hasFootHitGround.get(candidateSwingState.getOppositeSide()).getValue())
                return true;

            return false;
        }
    }
    private class SwingFootState implements State
    {
        private final RobotSide swingSide;
        private final YoPolynomial swingFootHeightTraj;
        private final PDController swingFootHeightController;
        private final PDController swingHipPitchController;
        private final YoDouble swingLegForce;
        private final YoDouble swingHipForce;


        public SwingFootState(RobotSide swingSide)
        {
            this.swingSide = swingSide;

            swingFootHeightTraj = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootHeight", 5, registry);
            swingFootHeightController = new PDController(swingFootHeightKp, swingFootHeightKd, swingSide.getLowerCaseName() + "SwingFootHeightController", registry);
            swingHipPitchController = new PDController(swingHipPitchKp, swingHipPitchKd, swingSide.getLowerCaseName() + "swingHipPitchController", registry);
            swingLegForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredLegForce", registry);
            swingHipForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredHipForce", registry);
        }


        @Override
        public void onEntry() {
            FramePoint3D footPosition = new FramePoint3D(controllerRobot.getFootFrame(swingSide));
            footPosition.changeFrame(controllerRobot.getWorldFrame());

            hasFootHitGround.get(swingSide).set(false);

            double initialTime = 0.0;
            double swingDuration = desiredSwingDuration.getDoubleValue();
            double initialHeight = footPosition.getZ();
            double initialVelocity = 0.0;
            double finalHeight = 0.0;
            double finalVelocity = 0.0;
            swingFootHeightTraj.setQuarticUsingWayPoint(initialTime,
                    0.5 * swingDuration,
                    swingDuration,
                    initialHeight,
                    initialVelocity,
                    desiredSwingHeight.getDoubleValue(),
                    finalHeight,
                    finalVelocity);

        }

        @Override
        public void doAction(double timeInState) {

            // Calculate Center of Mass Position
            FramePoint3D centerOfMassPoint = new FramePoint3D(controllerRobot.getCenterOfMassFrame());
            centerOfMassPoint.changeFrame(controllerRobot.getWorldFrame());
            double centerOfMassHeight = centerOfMassPoint.getZ();
            double gravity = 9.81;

            swingFootHeightTraj.compute(timeInState);

            FramePoint3D footPosition = new FramePoint3D(controllerRobot.getFootFrame(swingSide));
            footPosition.changeFrame(controllerRobot.getWorldFrame());

            double currentHeight = footPosition.getZ();
            double desiredHeight = swingFootHeightTraj.getValue();
            // This is approximately the right velocity. Good enough for stability, but not great overall.
            double currentVelocity = controllerRobot.getKneeJoint(swingSide).getQd();
            double desiredVelocity = swingFootHeightTraj.getVelocity();

            double legForce = swingFootHeightController.compute(currentHeight, desiredHeight, currentVelocity, desiredVelocity);
            swingLegForce.set(legForce);

            // set the desired torque to the knee joint to achieve the desired swing foot height
            controllerRobot.getKneeJoint(swingSide).setTau(legForce);


            double desiredFootPositionX = 0.0;
//                    Math.sqrt(centerOfMassHeight/gravity) * controllerRobot.getCenterOfMassVelocity().getX();
            double desiredFootVelocityX = 0.0;

            footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
            double currentFootPositionX = footPosition.getX();
            double currentFootVelocityX = -controllerRobot.getHipJoint(swingSide).getQd(); // FIXME make this have some actual value
            double hipForce = -swingHipPitchController.compute(currentFootPositionX, desiredFootPositionX, currentFootVelocityX, desiredFootVelocityX);
            swingLegForce.set(legForce);

            controllerRobot.getHipJoint(swingSide).setTau(hipForce);

        }

        @Override
        public void onExit(double timeInState) {

        }

        @Override
        public boolean isDone(double timeInState) {
            return timeInState > desiredSwingDuration.getDoubleValue();
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
            hasFootHitGround.get(supportSide).set(true);

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
