package us.ihmc.exampleSimulations.cart;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationConstructionSetTools.util.environments.CartRobotRacingEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SmallStepDownEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionVisualizer;

public class CartRobotEnvironmentSimulation
{
   public CartRobotEnvironmentSimulation()
   {
      System.out.println("Hello World.");

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
      CartRobotController controller = new CartRobotController(cartRobot, dt);
      cartRobot.setController(controller);

      // Stair Env Robot
      //SmallStepDownEnvironment environment = createEnvironment();
      CartRobotRacingEnvironment environment = new CartRobotRacingEnvironment(false);
      //      ArrayList<Robot> environmentRobots = environment.getEnvironmentRobots();
      //      allSimulatedRobotList.addAll(environmentRobots);

      // Scs
      SimulationConstructionSet scs = new SimulationConstructionSet(allSimulatedRobotList.toArray(new Robot[0]), parameters);
      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(staticLinkGraphics);
      scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());

      //      List<? extends Shape3D> simpleShapes = environment.getTerrainObject3D().getSimpleShapes();
      //      for(int i=0;i<simpleShapes.size();i++)
      //      {
      //         Graphics3DObject graphicsObject = new Graphics3DObject(simpleShapes.get(i), YoAppearance.Aqua());
      //         scs.addStaticLinkGraphics(graphicsObject);
      //      }

      // simulate.
      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, 0.01, scs, 1000);
      double coefficientOfRestitution = 0.2;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      // OLD //     scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.initializeCollisionDetector(collisionVisualizer, collisionHandler);
      scs.addEnvironmentCollisionShapes(environment.getTerrainObject3D().getSimpleShapes());
      scs.initializeCollisionHandler(collisionVisualizer, collisionHandler);

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

   private SmallStepDownEnvironment createEnvironment()
   {
      double stepLength = 0.35;
      double stepDownHeight = 0.15;
      int stepsBeforeDrop = 1;
      double dropHeight = -stepDownHeight;

      int numberOfDrops = 4;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();

      double currentHeight = 0.0;

      for (int i = 0; i < numberOfDrops; i++)
      {
         for (int j = 0; j < stepsBeforeDrop; j++)
         {
            stepHeights.add(currentHeight);
            stepLengths.add(stepLength);
         }

         currentHeight += dropHeight;

         stepHeights.add(currentHeight);
         stepLengths.add(stepLength);
      }

      double starterLength = 0.35;
      return new SmallStepDownEnvironment(stepHeights, stepLengths, starterLength, 0.0, currentHeight);
   }

}
