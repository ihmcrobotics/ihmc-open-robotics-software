package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.FixedMovingReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCoordinateSystem3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.scs2.definition.visual.ColorDefinitions.DarkOrange;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint3D;

public class BPWPLanarWalkingRobot implements SCS2YoGraphicHolder
{
    private final SideDependentList<SimPrismaticJoint> kneeJoints;
    private final SideDependentList<SimRevoluteJoint> hipJoints;

//    private final SimRigidBody torso;

    private final SideDependentList<YoDouble> legLengths = new SideDependentList<YoDouble>();
    private final SideDependentList<MovingReferenceFrame> footFrames = new SideDependentList<>();

    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final DoubleProvider time;
    private final ReferenceFrame worldFrame;
    private double mass;

    private final MovingReferenceFrame centerOfMassFrame;
    private final YoFramePoint3D centerOfMassPoint;
    private final YoFrameVector3D centerOfMassVelocity;
    private final SideDependentList<YoFrameVector3D> footVelocities = new SideDependentList<>();
    private final SimFloatingJointBasics floatingJoint;


    public BPWPLanarWalkingRobot(Robot robot, DoubleProvider time)
    {
        this.time = time;

        floatingJoint = robot.getFloatingRootJoint();
        floatingJoint.setJointPosition(new Vector3D(0.0, 0.0, 0.75));
        mass = TotalMassCalculator.computeSubTreeMass(robot.getRootBody());

        worldFrame = robot.getInertialFrame();
        kneeJoints = new SideDependentList<>();
        hipJoints = new SideDependentList<>();

        centerOfMassPoint = new YoFramePoint3D("centerOfMassPoint", worldFrame, registry);
        centerOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);


        // FIXME use the center of mass jacobian calculator for this.
        centerOfMassFrame = robot.getJoint(BPWPlanarWalkingRobotDefinition.baseJointName).getFrameAfterJoint();

        for(RobotSide robotSide : RobotSide.values)
        {
            SimPrismaticJoint kneeJoint = (SimPrismaticJoint) robot.getJoint(BPWPlanarWalkingRobotDefinition.kneeNames.get(robotSide));
            SimRevoluteJoint hipJoint = (SimRevoluteJoint) robot.getJoint(BPWPlanarWalkingRobotDefinition.hipPitchNames.get(robotSide));
            kneeJoints.put(robotSide, kneeJoint);
            hipJoints.put(robotSide, hipJoint);

            footVelocities.put(robotSide, new YoFrameVector3D(robotSide.getLowerCaseName() + "FootVelocity", centerOfMassFrame, registry));

            YoDouble legLength = new YoDouble( robotSide.getLowerCaseName() + "LegLength", registry);
            legLengths.put(robotSide, legLength);
            Vector3D footTranslationFromKnee = new Vector3D();
            footTranslationFromKnee.setZ(-BPWPlanarWalkingRobotDefinition.shinLength / 2.0);
            MovingReferenceFrame footFrame = new FixedMovingReferenceFrame(robotSide.getLowerCaseName() + "FootFrame", kneeJoint.getFrameAfterJoint(), footTranslationFromKnee);
            footFrames.put(robotSide, footFrame);

        }
        kneeJoints.get(RobotSide.LEFT).setQ(0.25);
//        torso = (SimRigidBody) robot.getJoint(BPWPlanarWalkingRobotDefinition.baseJointName);


    }
    
    public YoRegistry getYoRegistry()
    {
        return registry;
    }

    public ReferenceFrame getWorldFrame()
    {
        return worldFrame;
    }

    public double getLegLength(RobotSide robotSide)
    {
        return legLengths.get(robotSide).getDoubleValue();
    }

    public SimPrismaticJoint getKneeJoint(RobotSide robotSide)
    {
        return kneeJoints.get(robotSide);
    }

    public DoubleProvider getTime()
    {
        return time;
    }

    public void update()
    {
        for(RobotSide robotSide : RobotSide.values)
        {
            // update current leg length
            double restingLegLength = (BPWPlanarWalkingRobotDefinition.thighLength + BPWPlanarWalkingRobotDefinition.shinLength) / 2.0;
            double currentLegLength = restingLegLength - kneeJoints.get(robotSide).getQ();

            legLengths.get(robotSide).set(currentLegLength);
            footFrames.get(robotSide).update();

            Twist footTwist = new Twist();
            footFrames.get(robotSide).getTwistRelativeToOther(centerOfMassFrame, footTwist);
            footVelocities.get(robotSide).setMatchingFrame(footTwist.getLinearPart());

        }

        centerOfMassPoint.setFromReferenceFrame(centerOfMassFrame);
        centerOfMassVelocity.setMatchingFrame(centerOfMassFrame.getTwistOfFrame().getLinearPart());
    }

    public MovingReferenceFrame getFootFrame(RobotSide robotSide)
    {
        return footFrames.get(robotSide);
    }

    public GroundContactPoint getGroundContactPoint(RobotSide robotSide)
    {
        return kneeJoints.get(robotSide).getAuxiliaryData().getGroundContactPoints().get(0);
    }

    public double getMass()
    {
        return mass;
    }

    public SimRevoluteJoint getHipJoint(RobotSide robotSide)
    {
        return hipJoints.get(robotSide);
    }

//    public SimRigidBody getTorso()
//    {
//        return torso;
//    }

    public FramePoint3DReadOnly getCenterOfMassPoint()
    {
        return centerOfMassPoint;
    }

    public FrameVector3DReadOnly getCenterOfMassVelocity()
    {
        return centerOfMassVelocity;
    }

    public FrameVector3DReadOnly getFootVelocityRelativeToCOM( RobotSide robotSide)
    {
        return footVelocities.get(robotSide);
    }
    public MovingReferenceFrame getCenterOfMassFrame()
    {
        return centerOfMassFrame;
    }

    @Override
    public YoGraphicDefinition getSCS2YoGraphics() {

        YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
        group.addChild(newYoGraphicCoordinateSystem3D("BasePoint", floatingJoint.getAuxiliaryData().getKinematicPoints().get(0).getPose(), 0.25));

        for(RobotSide robotSide : RobotSide.values)
        {
            group.addChild(newYoGraphicPoint3D(robotSide.getLowerCaseName() + "GroundPoint", kneeJoints.get(robotSide).getAuxiliaryData().getGroundContactPoints().get(0).getPose().getPosition(), 0.01, DarkOrange()));
            group.addChild(newYoGraphicCoordinateSystem3D(robotSide.getLowerCaseName() + "KneeFrame", kneeJoints.get(robotSide).getAuxiliaryData().getKinematicPoints().get(0).getPose(), 0.075));
            group.addChild(newYoGraphicCoordinateSystem3D(robotSide.getLowerCaseName() + "HipFrame", hipJoints.get(robotSide).getAuxiliaryData().getKinematicPoints().get(0).getPose(), 0.075));
        }
        group.setVisible(true);
        return group;

    }

    public SimFloatingJointBasics getFloatingJoint()
    {
        return floatingJoint;
    }

//    public FrameVector3DReadOnly getCenterOfMassVelocity()
//    {
//        Twist twist = new Twist();
//        centerOfMassFrame.getTwistRelativeToOther(worldFrame, twist);
//        return twist.getLinearPart();
//    }

}
