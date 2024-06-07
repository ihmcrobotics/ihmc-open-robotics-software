package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.exampleSimulations.planarWalker.SlopeGroundDefinition;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.robot.Robot;

public class BPWPlanarWalkingSimulation {

    public BPWPlanarWalkingSimulation()
    {
        int simTicksPerControlTick = 3;

        // Instantiate sim and its environment
        SimulationConstructionSet2 scs = new SimulationConstructionSet2();
        scs.setBufferRecordTickPeriod(simTicksPerControlTick);
        scs.addTerrainObject(new SlopeGroundDefinition(0.0));
        double controllerDt = scs.getDT() * simTicksPerControlTick;

        // Create robot definition for our bipedal walker
        BPWPlanarWalkingRobotDefinition robotDefinition = new BPWPlanarWalkingRobotDefinition();

        // Creating sim robot and robot definition
        RobotDefinition simRobotDefinition = new RobotDefinition(robotDefinition);
        simRobotDefinition.setName(robotDefinition.getName() + "_SIM");
        Robot simRobot = new Robot(simRobotDefinition, scs.getInertialFrame());

        // Creating controller robot and robot definition
        RobotDefinition ctrlRobotDefinition = new RobotDefinition(robotDefinition);
        ctrlRobotDefinition.setName(robotDefinition.getName() + "_CONTROLLER");
        Robot controllerRobot = new Robot(ctrlRobotDefinition, ReferenceFrame.getWorldFrame());

        // Create bipedal walker robot and its controller
        BPWPLanarWalkingRobot bpwPlanarWalkingRobot = new BPWPLanarWalkingRobot(controllerRobot, scs.getTime());
        BPWPlanarWalkingController controller = new BPWPlanarWalkingController(bpwPlanarWalkingRobot, controllerDt, RobotSide.LEFT);

        // Make sure robot starts in proper configuration
        MultiBodySystemTools.copyJointsState(controllerRobot.getAllJoints(), simRobot.getAllJoints(), JointStateType.CONFIGURATION);

        // Add sim robot to sim, and controller robot registry to sim
        scs.addRobot(simRobot);
        scs.getRootRegistry().addChild(controllerRobot.getRegistry());

        // Add YoGraphics
        scs.addYoGraphic(bpwPlanarWalkingRobot.getSCS2YoGraphics());
        scs.addYoGraphic(controller.getSCS2YoGraphics());

        // Add sensor reader to transfer states of sim robot to controller robot
        simRobot.addController(() ->
                               { // Sensor reader
                                   for (JointStateType state : JointStateType.values())
                                       MultiBodySystemTools.copyJointsState(simRobot.getAllJoints(), controllerRobot.getAllJoints(), state);
                                   controllerRobot.updateFrames();
                               });

        // Add our controller robot's controller to the simulation
        simRobot.addThrottledController(controller, controllerDt);

        // Add output writer to add control commands of controller robot to sim robot
        simRobot.addController(() ->
                               { // Output writer
                                   MultiBodySystemTools.copyJointsState(controllerRobot.getAllJoints(), simRobot.getAllJoints(), JointStateType.EFFORT);
                               });

        scs.changeBufferSize(28000);
        scs.startSimulationThread();
    }

    public static void main(String[] args)
    {
        new BPWPlanarWalkingSimulation();
    }
}
