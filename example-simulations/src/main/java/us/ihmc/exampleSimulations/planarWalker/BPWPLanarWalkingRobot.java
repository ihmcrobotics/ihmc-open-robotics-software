package us.ihmc.exampleSimulations.planarWalker;

import org.apache.poi.ss.formula.functions.T;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRigidBody;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.DoublePredicate;

public class BPWPLanarWalkingRobot
{
    private final SideDependentList<SimPrismaticJoint> kneeJoints;
    private final SideDependentList<SimRevoluteJoint> hipJoints;

//    private final SimRigidBody torso;

    private final SideDependentList<YoDouble> legLengths = new SideDependentList<YoDouble>();
    private final SideDependentList<ReferenceFrame> footFrames = new SideDependentList<>();
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final DoubleProvider time;
    private final ReferenceFrame worldFrame;

    private final MovingReferenceFrame centerOfMassFrame;

    public BPWPLanarWalkingRobot(Robot robot, DoubleProvider time)
    {
        this.time = time;
        robot.getFloatingRootJoint().setJointPosition(new Vector3D(0.0, 0.0, 0.75));

        worldFrame = robot.getInertialFrame();
        kneeJoints = new SideDependentList<>();
        hipJoints = new SideDependentList<>();

        // FIXME use the center of mass jacobian calculator for this.
        centerOfMassFrame = robot.getJoint(BPWPlanarWalkingRobotDefinition.baseJointName).getFrameAfterJoint();

        for(RobotSide robotSide : RobotSide.values)
        {
            SimPrismaticJoint kneeJoint = (SimPrismaticJoint) robot.getJoint(BPWPlanarWalkingRobotDefinition.kneeNames.get(robotSide));
            SimRevoluteJoint hipJoint = (SimRevoluteJoint) robot.getJoint(BPWPlanarWalkingRobotDefinition.hipNames.get(robotSide));
            kneeJoints.put(robotSide, kneeJoint);
            hipJoints.put(robotSide, hipJoint);

            YoDouble legLength = new YoDouble( robotSide.getLowerCaseName() + "LegLength", registry);
            legLengths.put(robotSide, legLength);
            Vector3D footTranslationFromKnee = new Vector3D();
            footTranslationFromKnee.setZ(-BPWPlanarWalkingRobotDefinition.shinLength / 2.0);
            ReferenceFrame footFrame = new FixedReferenceFrame(robotSide.getLowerCaseName() + "FootFrame", kneeJoint.getFrameAfterJoint(), footTranslationFromKnee);
            footFrames.put(robotSide, footFrame);

        }
        kneeJoints.get(RobotSide.LEFT).setQ(0.25);
//        hipJoints.get(RobotSide.LEFT).setQ(-0.3);
//        hipJoints.get(RobotSide.RIGHT).setQ(0.3);
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
        }
    }

    public ReferenceFrame getFootFrame(RobotSide robotSide)
    {
        return footFrames.get(robotSide);
    }

    public SimRevoluteJoint getHipJoint(RobotSide robotSide)
    {
        return hipJoints.get(robotSide);
    }

//    public SimRigidBody getTorso()
//    {
//        return torso;
//    }

    public ReferenceFrame getCenterOfMassFrame()
    {
        return centerOfMassFrame;
    }

    public FrameVector3DReadOnly getCenterOfMassVelocity()
    {
        Twist twist = new Twist();
        centerOfMassFrame.getTwistRelativeToOther(worldFrame, twist);
        return twist.getLinearPart();
    }

}
