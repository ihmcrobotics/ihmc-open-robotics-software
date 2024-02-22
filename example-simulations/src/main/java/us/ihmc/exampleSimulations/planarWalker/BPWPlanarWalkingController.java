package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BPWPlanarWalkingController implements Controller
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
    private final BPWPLanarWalkingRobot controllerRobot;

    private final SideDependentList<PDController> kneeLengthControllers = new SideDependentList<>();
    private final SideDependentList<YoDouble> desiredKneeForces = new SideDependentList<>();
    private final YoDouble desiredLegLength = new YoDouble("desiredLegLength", registry);

    public BPWPlanarWalkingController( BPWPLanarWalkingRobot controllerRobot)
    {
        this.controllerRobot = controllerRobot;

        // Set up the leg length controller gains
        YoDouble legLengthKp = new YoDouble("legLengthKp", registry);
        YoDouble legLengthKd = new YoDouble("legLengthKd", registry);
        legLengthKp.set(1500.0);
        legLengthKd.set(50.0);

        desiredLegLength.set((BPWPlanarWalkingRobotDefinition.shinLength + BPWPlanarWalkingRobotDefinition.thighLength) / 2.0);

        // Setting up the leg length controllers
        PDController leftLegLengthController = new PDController(legLengthKp, legLengthKd, "leftLegLengthController", registry);
        PDController rightLegLengthController = new PDController(legLengthKp, legLengthKd, "rightLegLengthController", registry);
        YoDouble leftLegDesKneeForces = new YoDouble("leftLegDesKneeForces", registry);
        YoDouble rightLegDesKneeForces = new YoDouble("rightLegDesKneeForces", registry);
        kneeLengthControllers.put(RobotSide.LEFT, leftLegLengthController);
        kneeLengthControllers.put(RobotSide.RIGHT, rightLegLengthController);
        desiredKneeForces.put(RobotSide.LEFT, leftLegDesKneeForces);
        desiredKneeForces.put(RobotSide.RIGHT, rightLegDesKneeForces);

        registry.addChild(controllerRobot.getYoRegistry());



    }

    @Override
    public void doControl()
    {
        controllerRobot.update();

        for( RobotSide robotSide : RobotSide.values)
        {
            double legLength = controllerRobot.getLegLength(robotSide);
            double kneeVelocity = -controllerRobot.getKneeJoint(robotSide).getQd();

            double desLegLength = desiredLegLength.getDoubleValue();
            double desKneeVelocity = 0.0;

            // Cal the torque required to hold the leg at the des length
            double torque = kneeLengthControllers.get(robotSide).compute(legLength, desLegLength, kneeVelocity, desKneeVelocity);
            desiredKneeForces.get(robotSide).set(torque);

            // Set the desired torque to the knee joint to hold the leg at the desired length
            controllerRobot.getKneeJoint(robotSide).setTau(-torque);
        }

    }


    @Override
    public YoRegistry getYoRegistry() {
        return registry;
    }

}
