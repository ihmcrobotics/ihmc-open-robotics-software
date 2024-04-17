package us.ihmc.exampleSimulations.planarWalker;

import org.bytedeco.javacv.Frame;
import org.lwjgl.system.linux.Stat;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
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

import static us.ihmc.scs2.definition.visual.ColorDefinitions.Blue;
import static us.ihmc.scs2.definition.visual.ColorDefinitions.Red;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicArrow3D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint3D;

public class BPWPlanarWalkingController implements Controller, SCS2YoGraphicHolder
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

    private final YoDouble swingHipRollKp = new YoDouble("swingHipRollKp", registry);
    private final YoDouble swingHipRollKd = new YoDouble("swingHipRollKd", registry);

    private final YoDouble supportPitchKp = new YoDouble("supportPitchKp", registry);
    private final YoDouble supportPitchKd = new YoDouble("supportPitchKd", registry);

    private final YoDouble supportHipRollKp = new YoDouble("supportHipRollKp", registry);
    private final YoDouble supportHipRollKd = new YoDouble("supportHipRollKd", registry);

    private enum LegStateName {Swing, Support};
    private final SideDependentList<StateMachine<LegStateName, State>> legStateMachines = new SideDependentList<>();
    private final SideDependentList<YoBoolean> hasFootHitGround = new SideDependentList<>();

    private YoDouble desiredWalkingSpeed = new YoDouble("desiredWalkingSpeed", registry);
    private YoDouble desiredSideWalkingSpeed = new YoDouble("desiredSideWalkingSpeed", registry);

    private YoDouble desiredFootPositionXFromHip = new YoDouble("desiredFootPositionXFromHip", registry);

    private static SideDependentList<YoDouble> desiredHipPosition = new SideDependentList<>();

    private static SideDependentList<YoFrameVector3D> supportStateLegForce = new SideDependentList<>();
    private final YoFrameVector3D desiredLIPMForce;

    private final YoFrameVector3D desiredLIPMAcceleration;

    private final SideDependentList<YoFramePoint3D> desiredTouchdownPositions = new SideDependentList<>();
    private final SideDependentList<YoFramePoint3D> desiredFootPositions = new SideDependentList<>();
    private final SideDependentList<YoFrameVector3D> desiredLegForceVectors = new SideDependentList<>();
    private final SideDependentList<YoFramePoint3D> currentFootPositions = new SideDependentList<>();

    private final YoDouble swingFootStepAdjustmentGain = new YoDouble("swingFootStepAdjustmentGain", registry);
    private final YoDouble swingFootSideStepAdjustmentGain = new YoDouble("swingFootSideStepAdjustmentGain", registry);

    private final YoDouble debug1 = new YoDouble("debug1", registry);
    private final YoDouble debug2 = new YoDouble("debug2", registry);
    private final YoDouble debug3 = new YoDouble("debug3", registry);
    private final YoDouble debug4 = new YoDouble("debug4", registry);

    private final YoDouble WalkingGain = new YoDouble("WalkingGain", registry);
    private final YoDouble SideWalkingGain = new YoDouble("SideWalkingGain", registry);




    public BPWPlanarWalkingController( BPWPLanarWalkingRobot controllerRobot, RobotSide initialSwingSide)
    {
        this.controllerRobot = controllerRobot;
        desiredLIPMForce = new YoFrameVector3D("desiredLIPMForce", controllerRobot.getWorldFrame(), registry);
        desiredLIPMAcceleration = new YoFrameVector3D("desiredLIPMAcceleration", controllerRobot.getWorldFrame(), registry);

        // Set up the leg length controller gains
        supportingLegLengthKp.set(1000.0);
        supportingLegLengthKd.set(750.0);

        // Setting gains for swing
        swingFootHeightKp.set(1000.0);
        swingFootHeightKd.set(150.0);

        desiredSupportLegLength.set((BPWPlanarWalkingRobotDefinition.shinLength + BPWPlanarWalkingRobotDefinition.thighLength) / 2.0);
        desiredSwingHeight.set(0.1);
        desiredSwingDuration.set(0.5);
        desiredBodyHeight.set(0.75);

        swingHipPitchKp.set(100.0); // 200
        swingHipPitchKd.set(2.0); // 1.0

        supportPitchKp.set(250.0);
        supportPitchKd.set(100.0);

        supportHipRollKp.set(500.0);
        supportHipRollKd.set(100.0);

        swingHipRollKp.set(800.0);
        swingHipRollKd.set(120.0);



        swingFootStepAdjustmentGain.set(0.55); // 0.65
        swingFootSideStepAdjustmentGain.set(1.0); // 0.65
        // Setting desired Walking Speed to zero for now
        desiredWalkingSpeed.set(0.0);
        WalkingGain.set(0.06); //0.06
        SideWalkingGain.set(0.06);


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

            desiredTouchdownPositions.put(robotSide, new YoFramePoint3D(robotSide.getLowerCaseName() + "DesiredTouchdownPosition", controllerRobot.getWorldFrame(), registry));
            desiredFootPositions.put(robotSide, new YoFramePoint3D(robotSide.getLowerCaseName() + "DesiredFootPositions", controllerRobot.getWorldFrame(), registry));
            currentFootPositions.put(robotSide, new YoFramePoint3D(robotSide.getLowerCaseName() + "CurrentFootPositions", controllerRobot.getWorldFrame(), registry));
            desiredLegForceVectors.put(robotSide, new YoFrameVector3D(robotSide.getLowerCaseName() + "DesiredLegForceVector", controllerRobot.getWorldFrame(), registry));

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

        private final PDController swingFootHeightController;
        private final PDController swingHipPitchController;
        private final PDController swingHipRollController;

        private final YoDouble swingLegForce;
        private final YoDouble swingHipForce;
        private final YoDouble swingHipRollForce;

        private final YoPolynomial swingFootHeightTraj;
        private final YoPolynomial swingFootXTraj;
        private final YoPolynomial swingFootYTraj;


        private final YoFramePoint3D footPositionAtStart;
        private final YoDouble footTouchdownPositionX;
        private final YoDouble footDesiredPositionX;

        private final YoDouble footTouchdownPositionY;
        private final YoDouble footDesiredPositionY;


        public SwingFootState(RobotSide swingSide)
        {
            this.swingSide = swingSide;

            swingFootHeightController = new PDController(swingFootHeightKp, swingFootHeightKd, swingSide.getLowerCaseName() + "SwingFootHeightController", registry);
            swingHipPitchController = new PDController(swingHipPitchKp, swingHipPitchKd, swingSide.getLowerCaseName() + "swingHipPitchController", registry);
            swingHipRollController = new PDController( swingHipRollKp, swingHipRollKd, swingSide.getLowerCaseName() + "swingHipRollController", registry);

            swingLegForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredLegForce", registry);
            swingHipForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredHipForce", registry);
            swingHipRollForce = new YoDouble(swingSide.getLowerCaseName() + "DesiredHipRollForce", registry);

            swingFootHeightTraj = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootHeight", 5, registry);
            swingFootXTraj = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootX", 4, registry);
            swingFootYTraj = new YoPolynomial(swingSide.getLowerCaseName() + "SwingFootY", 4, registry);

            footPositionAtStart = new YoFramePoint3D(swingSide.getLowerCaseName() + "FootPositionAtStart", controllerRobot.getCenterOfMassFrame(), registry);
            footTouchdownPositionX = new YoDouble(swingSide.getLowerCaseName() + "footTouchdownPositionX", registry);
            footDesiredPositionX = new YoDouble(swingSide.getLowerCaseName() + "footDesiredPositionX", registry);

            footTouchdownPositionY = new YoDouble(swingSide.getLowerCaseName() + "footTouchdownPositionY", registry);
            footDesiredPositionY = new YoDouble(swingSide.getLowerCaseName() + "footDesiredPositionY", registry);



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

            swingFootXTraj.setCubic(0.0, swingDuration, footPositionAtStart.getX(), computeDesiredTouchdownPositionX());
            swingFootYTraj.setCubic(0.0, swingDuration, footPositionAtStart.getY(), computeDesiredTouchdownPositionY(swingSide));

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

            footTouchdownPositionX.set(computeDesiredTouchdownPositionX());
            swingFootXTraj.setCubic(0.0, desiredSwingDuration.getDoubleValue(), footPositionAtStart.getX(), footTouchdownPositionX.getDoubleValue());
            swingFootXTraj.compute(timeInState);


            double desiredFootPositionX = swingFootXTraj.getValue();
            footDesiredPositionX.set(desiredFootPositionX);
            double desiredFootVelocityX = swingFootXTraj.getVelocity();

            footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
            double currentFootPositionX = footPosition.getX();

            double currentFootVelocityX = -controllerRobot.getHipJoint(swingSide).getQd() * controllerRobot.getLegLength(swingSide);
            // why this calculation
            double trial = controllerRobot.getFootVelocityRelativeToCOM(swingSide).getX();

//            velocityDebugR.set(currentFootVelocityX);
//
            double hipForce = swingHipPitchController.compute(currentFootPositionX, desiredFootPositionX, currentFootVelocityX, desiredFootVelocityX);
            swingHipForce.set(-hipForce);

            controllerRobot.getHipJoint(swingSide).setTau(swingHipForce.getDoubleValue());

            footPosition.setX(desiredFootPositionX);
            footPosition.changeFrame(controllerRobot.getWorldFrame());
            footPosition.setZ(desiredHeight);
            desiredFootPositions.get(swingSide).setMatchingFrame(footPosition);

            footPosition.setZ(0.0);
            footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
            footPosition.setX(footTouchdownPositionX.getDoubleValue());
            desiredTouchdownPositions.get(swingSide).setMatchingFrame(footPosition);


            // Swing hip roll torque calculator
            footTouchdownPositionY.set(computeDesiredTouchdownPositionY(swingSide));
            swingFootYTraj.setCubic(0.0, desiredSwingDuration.getDoubleValue(), footPositionAtStart.getY(), footTouchdownPositionY.getDoubleValue());
            swingFootYTraj.compute(timeInState);


            double desiredFootPositionY = swingFootYTraj.getValue();
            footDesiredPositionY.set(desiredFootPositionY);
            double desiredFootVelocityY = swingFootYTraj.getVelocity();

            footPosition.changeFrame(controllerRobot.getCenterOfMassFrame());
            double currentFootPositionY = footPosition.getY();

            debug1.set(currentFootPositionY);

//            double currentFootVelocityY = controllerRobot.getHipRollJoint(swingSide).getQd() * controllerRobot.getLegLength(swingSide);
            double currentFootVelocityY = controllerRobot.getFootVelocityRelativeToCOM(swingSide).getY();
            debug2.set(currentFootVelocityY);
            // why this calculation
            double trial2 = controllerRobot.getFootVelocityRelativeToCOM(swingSide).getX();

//            velocityDebugR.set(currentFootVelocityX);
//            velocityDebugC.set(trial);
            double hipRollForce = swingHipRollController.compute(currentFootPositionY, desiredFootPositionY, currentFootVelocityY, desiredFootVelocityY);
//            swingHipRollForce.set(swingSide.negateIfRightSide(hipRollForce));
            swingHipRollForce.set(hipRollForce);

            controllerRobot.getHipRollJoint(swingSide).setTau(swingHipRollForce.getDoubleValue());


        }
        private double computeDesiredTouchdownPositionX()
        {
            double currentCoMVelocityX = -controllerRobot.getFootVelocityRelativeToCOM(swingSide.getOppositeSide()).getX();

            // why not do this
            double trial = controllerRobot.getCenterOfMassVelocity().getX();

//            velocityDebugR.set(currentCoMVelocity);
//            velocityDebugC.set(trial);
            double omega = Math.sqrt(9.81 / controllerRobot.getCenterOfMassPoint().getZ());
            double velocityError = currentCoMVelocityX - desiredWalkingSpeed.getDoubleValue() ;
            return swingFootStepAdjustmentGain.getDoubleValue() / omega * currentCoMVelocityX + WalkingGain.getDoubleValue() * velocityError;
//            return 1 / omega * currentCoMVelocity;
        }

        private double computeDesiredTouchdownPositionY(RobotSide robotSide)
        {
//            double currentCoMVelocityY = controllerRobot.getCenterOfMassVelocity().getY();
            double currentCoMVelocityY = -controllerRobot.getFootVelocityRelativeToCOM(swingSide.getOppositeSide()).getY();

            // why not do this
            double trial = controllerRobot.getCenterOfMassVelocity().getY();

            double omega = Math.sqrt(9.81 / controllerRobot.getCenterOfMassPoint().getZ());
            double velocityError = currentCoMVelocityY - desiredSideWalkingSpeed.getDoubleValue() ;

            return swingFootSideStepAdjustmentGain.getDoubleValue() / omega *  currentCoMVelocityY + robotSide.negateIfRightSide(0.05) +  SideWalkingGain.getDoubleValue() * velocityError;
//          return swingFootSideStepAdjustmentGain.getDoubleValue() / omega *  currentCoMVelocityY + robotSide.negateIfRightSide(0.05);
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
        private final PDController supportRollController;
        private final YoDouble supportLegDesiredKneeForce;
        private final YoDouble supportLegFeedbackKneeForce;
        private final YoDouble supportHipPitchForce;
        private final YoDouble supportHipRollForce;

        public SupportFootState(RobotSide supportSide)
        {
            this.supportSide = supportSide;

            supportLegDesiredKneeForce = new YoDouble(supportSide.getShortLowerCaseName() + "SupportLegDesiredKneeForce", registry);
            supportLegFeedbackKneeForce = new YoDouble(supportSide.getLowerCaseName() + "SupportLegFeedbackKneeForce", registry);
            supportHipPitchForce = new YoDouble(supportSide.getLowerCaseName() + "supportHipPitchForce", registry);
            supportHipRollForce = new YoDouble(supportSide.getLowerCaseName() + "supportHipRollForce", registry);
            supportLegLengthController = new PDController(supportingLegLengthKp, supportingLegLengthKd, supportSide.getLowerCaseName() + "SupportLegLengthController", registry);
            supportPitchController = new PDController(supportPitchKp, supportPitchKd, supportSide.getLowerCaseName() + "supportPitchController", registry);
            supportRollController = new PDController(supportHipRollKp, supportHipRollKd, supportSide.getLowerCaseName() + "supportHipRollController", registry);
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

//            debug1.set(footPosition.getZ());

            FramePoint3D comPosition = new FramePoint3D(controllerRobot.getCenterOfMassFrame());
            comPosition.changeFrame(controllerRobot.getFootFrame(supportSide));

//            debug2.set(comPosition.getZ());

            FrameVector3D force = new FrameVector3D(footPosition);
            FrameVector3D force2 = new FrameVector3D(comPosition);
            force.setY(0.0);
            force.normalize();

//            debug3.set(force.getZ());
//            debug4.set(force2.getZ());

            double weight = 9.81 * controllerRobot.getMass();
            force.scale(0.75 * weight / force.getZ()); // TODO fake scal
            desiredLIPMForce.setMatchingFrame(force);

            double bodyHeight = controllerRobot.getCenterOfMassPoint().getZ();
            double bodyHeightVelocity = controllerRobot.getCenterOfMassVelocity().getZ();

            double desiredHeight = desiredBodyHeight.getDoubleValue();
            double desiredHeightVelocity = 0.0;

            // Compute the required torque to stay at the desired leg length
            double torque_knee = supportLegLengthController.compute(bodyHeight, desiredHeight, bodyHeightVelocity, desiredHeightVelocity);
            supportLegFeedbackKneeForce.set(torque_knee);
            supportLegDesiredKneeForce.set(torque_knee + force.norm());

            desiredLegForceVectors.get(supportSide).set(desiredLIPMForce);

            // Calculate the X component of the leg force
//            ReferenceFrame comFrame = controllerRobot.getCenterOfMassFrame();
//            ReferenceFrame footFrame = controllerRobot.getFootFrame(supportSide);
//
//            FramePoint3D comPoint = new FramePoint3D(comFrame);
//            comPoint.setToZero();
//            comPoint.changeFrame(footFrame);
//
//            FrameVector3D legForceVector = new FrameVector3D(footFrame, comPoint.getX(),comPoint.getY(), comPoint.getZ());
//            legForceVector.normalize();
//            legForceVector.scale(25.0);
//
//            desiredLegForceVectors.get(supportSide).set(legForceVector);


            controllerRobot.getKneeJoint(supportSide).setTau(-supportLegDesiredKneeForce.getDoubleValue());

//            controllerRobot.getHipJoint(supportSide).setTau(0.0);

            FrameQuaternion base = new FrameQuaternion(controllerRobot.getFloatingJoint().getFrameAfterJoint());
            base.changeFrame(controllerRobot.getWorldFrame());
            double torque_pitch = supportPitchController.compute(base.getPitch(), 0.0, controllerRobot.getFloatingJoint().getFrameAfterJoint().getTwistOfFrame().getAngularPartY(), 0.0);
            supportHipPitchForce.set(-torque_pitch);
            controllerRobot.getHipJoint(supportSide).setTau(supportHipPitchForce.getDoubleValue());

            double torque_roll = supportRollController.compute(base.getRoll(), 0.0, controllerRobot.getFloatingJoint().getFrameAfterJoint().getTwistOfFrame().getAngularPartX(), 0.0);
            supportHipRollForce.set(-torque_roll);
//            controllerRobot.getHipRollJoint(supportSide).setTau(supportSide.negateIfRightSide(supportHipRollForce.getDoubleValue()));
            controllerRobot.getHipRollJoint(supportSide).setTau(supportHipRollForce.getDoubleValue());




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

    @Override
    public YoGraphicDefinition getSCS2YoGraphics()
    {
        YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
        for(RobotSide robotSide : RobotSide.values)
        {
            group.addChild(newYoGraphicPoint3D(robotSide.getLowerCaseName() + "TouchdownPosition", desiredTouchdownPositions.get(robotSide), 0.015, Red()));
            group.addChild(newYoGraphicPoint3D(robotSide.getLowerCaseName() + "CurrentFootPosition", desiredFootPositions.get(robotSide), 0.015, Blue()));
            group.addChild(newYoGraphicArrow3D(robotSide.getLowerCaseName() + "GroundReactionForce", currentFootPositions.get(robotSide), desiredLegForceVectors.get(robotSide), 0.01, Red()));

        }
        group.setVisible(true);

        return group;

    }

}
