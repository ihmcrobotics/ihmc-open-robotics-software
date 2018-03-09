package us.ihmc.exampleSimulations.cart;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.newtonsCradle.GroundAsABoxRobot;
import us.ihmc.exampleSimulations.newtonsCradle.PileOfRandomObjectsRobot;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionVisualizer;

public class TriWheelCartSimulation
{
   public TriWheelCartSimulation()
   {
      double dt = 0.001;

      TriWheelCartDescription robotDescription = new TriWheelCartDescription("triWheel");

      Vector3D startingPoint = new Vector3D(-2.0, 0.0, robotDescription.getInitialHeight());

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(8000);

      SimulationConstructionSet scs = new SimulationConstructionSet(parameters);

      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(staticLinkGraphics);

      // create robot.
      Robot triWheelRobot = scs.addRobot(robotDescription);
      FloatingJoint floatingJoint = (FloatingJoint) triWheelRobot.getRootJoints().get(0);
      floatingJoint.setPosition(startingPoint);

      // robot controller.
      TriWheelCartController controller = new TriWheelCartController(triWheelRobot);
      triWheelRobot.setController(controller);

      // obstacles.
      PileOfRandomObjectsRobot pileOfRandomObjectsRobot = new PileOfRandomObjectsRobot();
      pileOfRandomObjectsRobot.setSizeScale(2.0);
      pileOfRandomObjectsRobot.setGroupAndCollisionMask(0xffff, 0xffff);
      pileOfRandomObjectsRobot.setNumberOfObjects(100);
      pileOfRandomObjectsRobot.setXYExtents(1.0, 1.0);
      pileOfRandomObjectsRobot.setZMinAndMax(1.0, 2.0);
      ArrayList<Robot> robots = pileOfRandomObjectsRobot.createAndGetRobots();
      for (Robot robot : robots)
         scs.addRobot(robot);

      // ground.
      int estimatedNumberOfContactPoints = 100;
      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setFloorLength(15.0);
      groundAsABoxRobot.setFloorWidth(15.0);
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      groundAsABoxRobot.setAddWalls(false);
      groundAsABoxRobot.setCollisionMask(0xff);

      scs.addRobot(groundAsABoxRobot.createRobot());
      scs.setGroundVisible(false);

      // simulate.
      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, 0.01, scs, 1000);
      double coefficientOfRestitution = 0.2;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);

      scs.setDT(dt, 1);
      scs.setFastSimulate(true);

      scs.setCameraFix(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, -8.0, 1.4);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new TriWheelCartSimulation();
   }
}
