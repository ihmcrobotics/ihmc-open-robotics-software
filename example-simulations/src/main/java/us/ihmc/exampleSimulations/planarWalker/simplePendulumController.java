package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.robotics.controllers.PDController;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;

public class simplePendulumController implements Controller {


    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final simplePendulumRobot controllerRobot;

    private final YoDouble pendulumAngleSetPoint = new YoDouble("angleSetPoint", registry);
    private final YoDouble pendulumJointKp = new YoDouble("pendulumJointKp", registry);
    private final YoDouble pendulumJointKd = new YoDouble("pendulumJointKd", registry);

    private final YoDouble desTau = new YoDouble("desiredTorque", registry);

    private final PDController pendulumJointController;



    public simplePendulumController(simplePendulumRobot controllerRobot)
    {
        this.controllerRobot = controllerRobot;

        pendulumAngleSetPoint.set(0.5);

        pendulumJointKp.set(500.0);
        pendulumJointKd.set(10.0);

        pendulumJointController = new PDController(pendulumJointKp, pendulumJointKd, "pendulumJointController", registry);

        registry.addChild(controllerRobot.getYoRegistry());

    }

    @Override
    public void doControl() {

        double currentAngle = controllerRobot.getPendulumAngle();
        double currentVelocity  = controllerRobot.getPendulumJoint().getQd();

        double desiredTorque = pendulumJointController.compute(currentAngle, pendulumAngleSetPoint.getDoubleValue(), currentVelocity, 0.0);

        desTau.set(desiredTorque);

        controllerRobot.getPendulumJoint().setTau(desTau.getDoubleValue());



    }

    @Override
    public YoRegistry getYoRegistry() {
        return registry;
    }
}
