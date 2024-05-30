package us.ihmc.exampleSimulations.simpleMPCTouchdownCalculatorTesting;

import us.ihmc.exampleSimulations.planarWalker.SlopeGroundDefinition;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;

public class BPWPlanarWalkingSimulation {

    public BPWPlanarWalkingSimulation()
    {
        int simTicksPerControlTick = 3;

        SimulationConstructionSet2 scs = new SimulationConstructionSet2();
        scs.setBufferRecordTickPeriod(simTicksPerControlTick);
//        scs.getGravity().setToZero();

        // Creating a robot and adding it to zero
        BPWPlanarWalkingRobotDefinition robotDefinition = new BPWPlanarWalkingRobotDefinition();
        Robot robot = new Robot(robotDefinition, scs.getInertialFrame());
        scs.addRobot(robot);

        // Setting a Slope ground definition
        scs.addTerrainObject(new SlopeGroundDefinition(0.0));

        // Adding the Ground Contact Parameters.
//        ContactPointBasedPhysicsEngine physicsEngine = (ContactPointBasedPhysicsEngine) scs.getSimulationSession().getPhysicsEngine();
//        ContactPointBasedContactParameters parameters = ContactPointBasedContactParameters.defaultParameters();
//        parameters.setKz(650.0);
//        parameters.setBz(500.0);
//        parameters.setKxy(15000.0);
//        parameters.setBxy(600.0);
//        physicsEngine.setGroundContactParameters(parameters);



        // Set up the controller robot with some convenience method
        BPWPLanarWalkingRobot controllerRobot = new BPWPLanarWalkingRobot(robot, scs.getTime());

        scs.addYoGraphic(controllerRobot.getSCS2YoGraphics());

        // controller
        BPWPlanarWalkingController controller = new BPWPlanarWalkingController(controllerRobot, RobotSide.LEFT);
        scs.addYoGraphic(controller.getSCS2YoGraphics());
//         add the controller
//        robot.addController(controller);
        robot.addThrottledController(controller, scs.getDT() * simTicksPerControlTick);

        scs.startSimulationThread();
        //scs.simulate();
    }


    public static void main(String[] args) {
        new BPWPlanarWalkingSimulation();
    }
}
