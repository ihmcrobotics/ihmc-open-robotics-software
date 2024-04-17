package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class simplePendulumRobot
{
    private final SimRevoluteJoint pendulumJoint;

    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

    private final YoDouble pendulumAngle = new YoDouble("pendulumAngle", registry);

    public simplePendulumRobot(Robot robot)
    {
        pendulumJoint = (SimRevoluteJoint) robot.getJoint(simplePendulumDefinition.pendulumJointName);
        pendulumAngle.set(pendulumJoint.getQ());

//        pendulumJoint.setQ(1.2);


    }


    public SimRevoluteJoint getPendulumJoint() {
        return pendulumJoint;
    }

    public double getPendulumAngle()
    {
        return pendulumJoint.getQ();
    }
    public YoRegistry getYoRegistry()
    {
        return registry;
    }


}
