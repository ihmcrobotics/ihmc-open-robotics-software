package us.ihmc.exampleSimulations.cart;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationConstructionSetTools.util.environments.CartRobotRacingEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.simple.CollisionManager;

public class CartRobotEnvironmentSimulation
{
   public CartRobotEnvironmentSimulation()
   {
      double dt = 0.001;

      Vector3D startingPoint = new Vector3D(0.0, 0.0, 0.3);

      List<Robot> allSimulatedRobotList = new ArrayList<Robot>();

      CartRobotDescription robotDescription = new CartRobotDescription("rollingRobot");
      CartRobotCollisionMeshDefinitionDataHolder collisionMeshData = new CartRobotCollisionMeshDefinitionDataHolder();
      robotDescription.addCollisionMeshDefinitionData(collisionMeshData);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(8000);

      // create robot.
      Robot cartRobot = new RobotFromDescription(robotDescription);
      FloatingJoint floatingJoint = (FloatingJoint) cartRobot.getRootJoints().get(0);
      floatingJoint.setPosition(startingPoint);
      allSimulatedRobotList.add(cartRobot);

      // robot controller.
      CartRobotController controller = new CartRobotController(cartRobot);
      cartRobot.setController(controller);

      // Stair Env Robot
      CartRobotRacingEnvironment environment = new CartRobotRacingEnvironment(false);

      // Scs
      SimulationConstructionSet scs = new SimulationConstructionSet(allSimulatedRobotList.toArray(new Robot[0]), parameters);
      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(staticLinkGraphics);
      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());

      // simulate.
      double coefficientOfRestitution = 0.2;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      CollisionManager collisionManager = new CollisionManager(environment.getTerrainObject3D(), collisionHandler);
      scs.initializeShapeCollision(collisionManager);

      scs.setDT(dt, 1);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      scs.setCameraFix(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, -8.0, 1.4);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new CartRobotEnvironmentSimulation();
   }
}
