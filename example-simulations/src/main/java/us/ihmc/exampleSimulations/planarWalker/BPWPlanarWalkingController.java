package us.ihmc.exampleSimulations.planarWalker;

import org.bytedeco.javacv.Frame;
import org.lwjgl.system.linux.Stat;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
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

public class BPWPlanarWalkingController implements Controller
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final BPWPLanarWalkingRobot controllerRobot;

    private final YoDouble desiredSupportLegLength = new YoDouble("desiredLegLength", registry);
    private final YoDouble desiredBodyHeight = new YoDouble("desiredBodyHeight", registry);
    private final YoDouble desiredSwingHeight = new YoDouble("desiredSwingHeight", registry);
    private final YoDouble desiredSwingDuration = new YoDouble("desiredSwingDuration", registry);

    private final YoDouble supportingLegLengthKp = new YoDouble("supportingLegLengthKp", registry);
    private final YoDouble supportingLegLengthKd = new YoDouble("supportingLegLengthKd", registry);

    private final YoDouble swingFootHeightKp = new YoDouble("swingFootHeightKp", registry);
    private final YoDouble swingFootHeightKd = new YoDouble("swingFootHeightKd", registry);

    private final YoDouble swingHipPitchKp = new YoDouble("swingHipPitchKp", registry);
    private final YoDouble swingHipPitchKd = new YoDouble("swingHipPitchKd", registry);

    private final YoDouble supportPitchKp = new YoDouble("supportPitchKp", registry);
    private final YoDouble supportPitchKd = new YoDouble("supportPitchKd", registry);

    private enum LegStateName {Swing, Support};
    private final SideDependentList<StateMachine<LegStateName, State>> legStateMachines = new SideDependentList<>();
    private final SideDependentList<YoBoolean> hasFootHitGround = new SideDependentList<>();

    private YoDouble desiredWalkingSpeed = new YoDouble("desiredWalkingSpeed", registry);

    private YoDouble desiredFootPositionXFromHip = new YoDouble("desiredFootPositionXFromHip", registry);

    private static SideDependentList<YoDouble> desiredHipPosition = new SideDependentList<>();

    private static SideDependentList<YoFrameVector3D> supportStateLegForce = new SideDependentList<>();
    private final YoFrameVector3D desiredLIPMForce;

    private final YoDouble swingFootStepAdjustmentGain = new YoDouble("swingFootStepAdjustmentGain", registry);

    private final YoDouble velocityDebugR = new YoDouble("velocitydebugR", registry);
    private final YoDouble velocityDebugC = new YoDouble("velocitydebugC", registry);


    public BPWPlanarWalkingController( BPWPLanarWalkingRobot controllerRobot, RobotSide initialSwingSide)
    {
        this.controllerRobot = controllerRobot;
        desiredLIPMForce = new YoFrameVector3D("desiredLIPMForce", controllerRobot.getWorldFrame(), registry);

        // Set up the leg length controller gains
        supportingLegLengthKp.set(1000.0);
        supportingLegLengthKd.set(500.0);

        // Setting gains for swing
        swingFootHeightKp.set(200.0);
        swingFootHeightKd.set(100.0);

        desiredSupportLegLength.set((BPWPlanarWalkingRobotDefinition.shinLength + BPWPlanarWalkingRobotDefinition.thighLength) / 2.0);
        desiredSwingHeight.set(0.1);
        desiredSwingDuration.set(0.5);
        desiredBodyHeight.set(0.75);

        swingHipPitchKp.set(200.0);
        swingHipPitchKd.set(1.0);

        supportPitchKp.set(250.0);
        supportPitchKp.set(100.0);

        swingFootStepAdjustmentGain.set(1.5);
        // Setting desired Walking Speed to zero for now
        desiredWalkingSpeed.set(0.0);

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

            YoDouble desiredPosition = new YoDouble(robotSide.getLowerCaseName() + "desiredHipPosition", registry);
            desiredHipPosition.put(robotSide, desiredPosition);


            YoFrameVector3D supportLegForce = new YoFrameVector3D(robotSide.getLowerCaseName() + "supportLegVector", controllerRobot.getCenterOfMassFrame(), registry);
            supportStateLegForce.put(robotSide, supportLegForce);

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
        private final YoPolynomial swingFootXTraj;
        private final YoFramePoint3D footPositionAtStart;
        private final YoDouble footTouchdownPosition;
        private final YoDouble footDesiredPosition;


        public SwingFootState(RobotSide swingSide)
        {
            this.swingSide = swingSide;

            swingFootHeightTraj = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootHeight", 5, registry);
            swingFootHeightController = new PDController(swingFootHeightKp, swingFootHeightKd, swingSide.getLowerCaseName() + "SwingFootHeightController", registry);
            swingHipPitchController = new PDController(swingHipPitchKp, swingHipPitchKd, swingSide.getLowerCaseName() + "swingHipPitchController", registry);
            swingLegForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredLegForce", registry);
            swingHipForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredHipForce", registry);

            swingFootXTraj = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootX", 4, registry);
            footPositionAtStart = new YoFramePoint3D(swingSide.getLowerCaseName() + "FootPositionAtStart", controllerRobot.getCenterOfMassFrame(), registry);
            footTouchdownPosition = new YoDouble(swingSide.getLowerCaseName() + "FootTouchdownPosition", registry);
            footDesiredPosition = new YoDouble(swingSide.getLowerCaseName() + "FootDesiredPosition", registry);


        }


        @Override
        public void onEntry() {
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
            swingFootHeightTraj.setQuarticUsingWayPoint(initialTime,
                    0.5 * swingDuration,
                    swingDuration,
                    initialHeight,
                    initialVelocity,
                    desiredSwingHeight.getDoubleValue(),
                    finalHeight,
                    finalVelocity);

            swingFootXTraj.setCubic(0.0, swingDuration, footPositionAtStart.getX(), computeDesiredTouchdownPosition());

        }

        @Override
        public void doAction(double timeInState) {

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
            controllerRobot.getKneeJoint(swingSide).setTau(swingLegForce.getDoubleValue());

            footTouchdownPosition.set(computeDesiredTouchdownPosition());
            swingFootXTraj.setCubic(0.0, desiredSwingDuration.getDoubleValue(), footPositionAtStart.getX(), footTouchdownPosition.getDoubleValue());
            swingFootXTraj.compute(timeInState);


            double desiredFootPositionX = swingFootXTraj.getValue();
            footDesiredPosition.set(desiredFootPositionX);
            double desiredFootVelocityX = swingFootXTraj.getVelocity();

            footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
            double currentFootPositionX = footPosition.getX();

            double currentFootVelocityX = -controllerRobot.getHipJoint(swingSide).getQd() * controllerRobot.getLegLength(swingSide); // FIXME make this have some actual value
            // why this calculation
            double trial = controllerRobot.getFootVelocityRelativeToCOM(swingSide).getX();

//            velocityDebugR.set(currentFootVelocityX);
//            velocityDebugC.set(trial);
            double hipForce = swingHipPitchController.compute(currentFootPositionX, desiredFootPositionX, currentFootVelocityX, desiredFootVelocityX);
            swingHipForce.set(-hipForce);

            controllerRobot.getHipJoint(swingSide).setTau(swingHipForce.getDoubleValue());



        }
        private double computeDesiredTouchdownPosition()
        {
            double currentCoMVelocity = -controllerRobot.getFootVelocityRelativeToCOM(swingSide.getOppositeSide()).getX();

            // why not do this
            double trial = controllerRobot.getCenterOfMassVelocity().getX();

//            velocityDebugR.set(currentCoMVelocity);
//            velocityDebugC.set(trial);
            double omega = Math.sqrt(9.81 / desiredBodyHeight.getDoubleValue());
            return swingFootStepAdjustmentGain.getDoubleValue() / omega * currentCoMVelocity;
//            return 1 / omega * currentCoMVelocity;
        }

        @Override
        public void onExit(double timeInState) {

        }

        @Override
        public boolean isDone(double timeInState) {
            if (timeInState > 0.1 * desiredSwingDuration.getDoubleValue() && controllerRobot.getGroundContactPoint(swingSide).getInContact().getValue())
                return true;

            return timeInState > desiredSwingDuration.getDoubleValue();
        }
    }

    private class SupportFootState implements State
    {
        private final RobotSide supportSide;
        private final PDController supportLegLengthController;
        private final PDController supportPitchController;
        private final YoDouble supportLegDesiredKneeForce;
        private final YoDouble supportLegFeedbackKneeForce;
        public SupportFootState(RobotSide supportSide)
        {
            this.supportSide = supportSide;

            supportLegDesiredKneeForce = new YoDouble(supportSide.getShortLowerCaseName() + "SupportLegDesiredKneeForce", registry);
            supportLegFeedbackKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SupportLegFeedbackKneeForce", registry);
            supportLegLengthController = new PDController(supportingLegLengthKp, supportingLegLengthKd, supportSide.getLowerCaseName() + "SupportLegLengthController", registry);
            supportPitchController = new PDController(supportPitchKp, supportPitchKd, supportSide.getLowerCaseName() + "supportPitchController", registry);
        }

        @Override
        public void onEntry() {
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

            double bodyHeight = controllerRobot.getCenterOfMassPoint().getZ();
            double bodyHeightVelocity = controllerRobot.getCenterOfMassVelocity().getZ();

            double desiredHeight = desiredBodyHeight.getDoubleValue();
            double desiredHeightVelocity = 0.0;

            // Compute the required torque to stay at the desired leg length
            double torque = supportLegLengthController.compute(bodyHeight, desiredHeight, bodyHeightVelocity, desiredHeightVelocity);
            supportLegFeedbackKneeForce.set(torque);
            supportLegDesiredKneeForce.set(torque + force.norm());

            // Calculate the X component of the leg force
//            ReferenceFrame comFrame = controllerRobot.getCenterOfMassFrame();
//            ReferenceFrame footFrame = controllerRobot.getFootFrame(supportSide);
//
//            FramePoint3D comPoint = new FramePoint3D(comFrame);
//            comPoint.setToZero();
//            comPoint.changeFrame(footFrame);

//            FrameVector3D legForceVector = new FrameVector3D(footFrame, comPoint.getX(),comPoint.getY(), comPoint.getZ());
//            legForceVector.normalize();
//            legForceVector.scale(-torque);


            controllerRobot.getKneeJoint(supportSide).setTau(-supportLegDesiredKneeForce.getDoubleValue());

//            controllerRobot.getHipJoint(supportSide).setTau(0.0);

            FrameQuaternion base = new FrameQuaternion(controllerRobot.getFloatingJoint().getFrameAfterJoint());
            base.changeFrame(controllerRobot.getWorldFrame());
            torque = supportPitchController.compute(base.getPitch(), 0.0, controllerRobot.getFloatingJoint().getFrameAfterJoint().getTwistOfFrame().getAngularPartY(), 0.0);
            controllerRobot.getHipJoint(supportSide).setTau(-torque);

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
