package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFInertia;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.function.DoublePredicate;

public class BPWPLanarWalkingRobot
{
    private final SideDependentList<SimPrismaticJoint> kneeJoints;
    private final SideDependentList<YoDouble> legLengths = new SideDependentList<YoDouble>();
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final DoubleProvider time;

    public BPWPLanarWalkingRobot(Robot robot, DoubleProvider time)
    {
        this.time = time;
        robot.getFloatingRootJoint().setJointPosition(new Vector3D(0.0, 0.0, 0.75));

        kneeJoints = new SideDependentList<>();

        for(RobotSide robotSide : RobotSide.values)
        {
            SimPrismaticJoint kneeJoint = (SimPrismaticJoint) robot.getJoint(BPWPlanarWalkingRobotDefinition.kneeNames.get(robotSide));
            kneeJoints.put(robotSide, kneeJoint);

            YoDouble legLength = new YoDouble( robotSide.getLowerCaseName() + "LegLength", registry);
            legLengths.put(robotSide, legLength);
        }
//        kneeJoints.get(RobotSide.LEFT).setQ(0.25);

    }

    public YoRegistry getYoRegistry()
    {
        return registry;
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
        }
    }
}
