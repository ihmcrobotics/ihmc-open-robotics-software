package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.exampleSimulations.collidingArms.SingleBallRobotDescription;
import us.ihmc.exampleSimulations.collidingArms.SingleBoxRobotDescription;
import us.ihmc.exampleSimulations.collidingArms.SingleCylinderRobotDescription;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.HybridImpulseSpringDamperCollisionHandler;
import us.ihmc.simulationconstructionset.physics.visualize.DefaultCollisionVisualizer;
import us.ihmc.simulationconstructionset.util.LinearStickSlipGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.tools.thread.ThreadTools;

public class NewtonsCradleSimulation
{
   private static CollisionHandler createCollisionHandler(double coefficientOfRestitution, double coefficientOfFriction, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
//      CollisionHandler collisionHandler =  new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      CollisionHandler collisionHandler =  new HybridImpulseSpringDamperCollisionHandler(coefficientOfRestitution, coefficientOfFriction, registry, yoGraphicsListRegistry);
      
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      
      return collisionHandler;
   }

   public static void createNewtonsCradleSimulation()
   {
      NewtonsCradleRobot robot = new NewtonsCradleRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      CollisionHandler collisionHandler = createCollisionHandler(0.99, 0.15, scs.getRootRegistry(), yoGraphicsListRegistry);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(4.0, 4.0, 0.01, scs, 100);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.startOnAThread();
   }

   public static void createSpinningCoinSimulation()
   {
      SpinningCoinRobot spinningCoinRobot = new SpinningCoinRobot();
      ArrayList<Robot> robots = spinningCoinRobot.getRobots();

      int estimatedNumberOfContactPoints = 32;
      Robot groundRobot = new GroundAsABoxRobot(estimatedNumberOfContactPoints);
      robots.add(groundRobot);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(10000);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 1);
      scs.setGroundVisible(false);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double epsilon = 0.3;
      double mu = 0.7;
      CollisionHandler collisionHandler = createCollisionHandler(epsilon, mu, scs.getRootRegistry(), yoGraphicsListRegistry);
      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(10.0, 10.0, 0.01, scs, 100);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();

      scs.setSimulateDuration(60.0);
      scs.simulate();
   }

   public static void createStackOfBouncyBallsSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();
      
      // Set to true for a large stack of same sized balls for object stacking evaluations.
      boolean stackOfSameSizedBalls = false;
      
      int numberOfBalls = 4;
      double radiusScaleFactor = 0.6;
      double massScaleFactor = 0.2;
      
      if (stackOfSameSizedBalls)
      {
         numberOfBalls = 40;
         radiusScaleFactor = 1.0;
         massScaleFactor = 1.0;
      }
      
      StackOfBouncyBallsRobot robot = new StackOfBouncyBallsRobot(numberOfBalls, radiusScaleFactor, massScaleFactor);
      robots.add(robot);

      int estimatedNumberOfContactPoints = 4;
      Robot groundRobot = new GroundAsABoxRobot(estimatedNumberOfContactPoints);
      robots.add(groundRobot);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray);
      scs.setDT(0.0001, 100);
      scs.setGroundVisible(false);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(0.1, 0.1, 0.01, scs, 100);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.9;
      double coefficientOfFriction = 0.0;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(), yoGraphicsListRegistry);
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.98, 0.1, robot);
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.3, 0.7, robot);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }
   
   public static void createBoxDownRampSimulation()
   {
      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.4;

      ArrayList<Robot> robots = new ArrayList<>();
      double mass = 1.0;
      double xLength = 0.2;
      double yWidth = 0.2;
      double zHeight = 0.1;
      SingleBoxRobotDescription singleBoxRobotDescription = new SingleBoxRobotDescription("BoxOne", mass, xLength, yWidth, zHeight, 0xff, 0xff);
      RobotFromDescription boxRobot = new RobotFromDescription(singleBoxRobotDescription);
      
      FloatingJoint rootJoint = (FloatingJoint) boxRobot.getRootJoints().get(0);
      rootJoint.setPosition(0.0, 0.0, 0.15);
      robots.add(boxRobot);

      double groundAngle = Math.PI/8.0;
      
      int estimatedNumberOfContactPoints = 32;
      Robot groundRobot = new GroundAsABoxRobot(estimatedNumberOfContactPoints, groundAngle , false, 0xffff, 0xffff);
      robots.add(groundRobot);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      CombinedTerrainObject3D boxTerrain = new CombinedTerrainObject3D("BoxTerrain");
      Box3d box = new Box3d(2.0, 1.0, 0.1);
      box.setPosition(new Point3d(0.0, 1.0, 0.0));
      box.setYawPitchRoll(0.0, groundAngle, 0.0);
      boxTerrain.addRotatableBox(box, YoAppearance.Blue());
      
      LinearStickSlipGroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(boxRobot, boxRobot.getRobotsYoVariableRegistry());
      groundContactModel.setGroundProfile3D(boxTerrain);

      groundContactModel.setAlphaStickSlip(coefficientOfFriction, coefficientOfFriction);
      groundContactModel.enableSlipping();
      groundContactModel.enableSurfaceNormal();
      boxRobot.setGroundContactModel(groundContactModel);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 1);
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(boxTerrain.getLinkGraphics());

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(0.1, 0.1, 0.01, scs, 100);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(), yoGraphicsListRegistry);
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.98, 0.1, robot);
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.3, 0.7, robot);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }
   
   public static void createRollingObjectsSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();

      double ballRadius = 0.2;
      double cylinderRadius = 0.2;
      double cylinderHeight = 0.5;
      
      SingleBallRobotDescription ballDescription = new SingleBallRobotDescription();
      ballDescription.setName("ball");
      ballDescription.setMass(1.0);
      ballDescription.setRadius(ballRadius);
      ballDescription.setCollisionGroup(0xffff);
      ballDescription.setCollisionMask(0xffff);
      ballDescription.setAppearance(YoAppearance.DarkCyan());
      ballDescription.setAddStripes(true);
      ballDescription.setStripeAppearance(YoAppearance.Gold());

      RobotFromDescription ballRobot = new RobotFromDescription(ballDescription.createRobotDescription());
      robots.add(ballRobot);
      FloatingJoint ballRootJoint = (FloatingJoint) ballRobot.getRootJoints().get(0);
      ballRootJoint.setPosition(0.0, 0.0, ballRadius * 1.02);
      ballRootJoint.setVelocity(0.2, 0.0, 0.0);
      
      SingleCylinderRobotDescription cylinderDescription = new SingleCylinderRobotDescription();
      cylinderDescription.setName("cylinder");
      cylinderDescription.setMass(1.0);
      cylinderDescription.setRadius(cylinderRadius);
      cylinderDescription.setHeight(cylinderHeight);
      cylinderDescription.setCollisionGroup(0xffff);
      cylinderDescription.setCollisionMask(0xffff);
      cylinderDescription.setAppearance(YoAppearance.DarkCyan());
      cylinderDescription.setAddStripes(true);
      cylinderDescription.setStripeAppearance(YoAppearance.Gold());

      RobotFromDescription cylinderRobot = new RobotFromDescription(cylinderDescription.createRobotDescription());
      robots.add(cylinderRobot);
      FloatingJoint cylinderRootJoint = (FloatingJoint) cylinderRobot.getRootJoints().get(0);
      cylinderRootJoint.setPosition(0.0, cylinderHeight * 2.0, cylinderRadius * 1.02);
      cylinderRootJoint.setVelocity(0.2, 0.0, 0.0);
      cylinderRootJoint.setYawPitchRoll(0.0, 0.0, Math.PI/2.00);

      int estimatedNumberOfContactPoints = 200;
      double groundAngle = 0.0;
      boolean addWalls = false;
      int collisionGroup = 0xffff;
      int collisionMask = 0xffff;
      
      Robot groundRobot = new GroundAsABoxRobot(estimatedNumberOfContactPoints, groundAngle, addWalls, collisionGroup, collisionMask);
      robots.add(groundRobot);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(4000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0002, 10);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(50.0, 100.0, 0.003, scs, 100);
//      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(), yoGraphicsListRegistry);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   public static void createRowOfDominosSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();
      int numberOfDominos = 30;
      RowOfDominosRobot robot = new RowOfDominosRobot(numberOfDominos);
      robots.add(robot);

      int estimatedNumberOfContactPoints = 200;
      Robot groundRobot = new GroundAsABoxRobot(estimatedNumberOfContactPoints);
      robots.add(groundRobot);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(4000);
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0002, 10);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(50.0, 100.0, 0.003, scs, 100);
//      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      //      CollisionHandler handler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new DefaultCollisionHandler(0.98, 0.1, robot);
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(), yoGraphicsListRegistry);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   public static void createStackOfBlocksSimulation()
   {
      int numberOfBlocks = 6;
      StackOfBlocksRobot stackOfBlocksRobot = new StackOfBlocksRobot(numberOfBlocks);
      ArrayList<Robot> robots = stackOfBlocksRobot.getRobots();

      int estimatedNumberOfContactPoints = 16;
      Robot groundRobot = new GroundAsABoxRobot(estimatedNumberOfContactPoints);
      robots.add(groundRobot);

      boolean showGUI = true;

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(showGUI);
      parameters.setDataBufferSize(32000);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 100);
      scs.setFastSimulate(false);
      scs.setGroundVisible(false);
      scs.setSimulateDuration(2.0);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, 0.01, scs, 1000);
//      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(), yoGraphicsListRegistry);
      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.startOnAThread();
   }

   public static void createPileOfRandomObjectsSimulation()
   {
      PileOfRandomObjectsRobot pileOfRandomObjectsRobot = new PileOfRandomObjectsRobot();
      pileOfRandomObjectsRobot.setNumberOfObjects(50);
      ArrayList<Robot> robots = pileOfRandomObjectsRobot.createAndGetRobots();

      int estimatedNumberOfContactPoints = 200;
      Robot groundRobot = new GroundAsABoxRobot(estimatedNumberOfContactPoints, true);
      robots.add(groundRobot);

      boolean showGUI = true;

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(4000);
      parameters.setCreateGUI(showGUI);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);

//    DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, scs, 1000);
      DefaultCollisionVisualizer collisionVisualizer = null;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = createCollisionHandler(coefficientOfRestitution, coefficientOfFriction, scs.getRootRegistry(), yoGraphicsListRegistry);
      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      
      scs.setDT(0.00025, 10);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      //      scs.simulate();

      long wallStartTime = System.currentTimeMillis();
      while (true)
      {
         ThreadTools.sleep(5000);

         double simTime = scs.getTime();
         long wallTime = System.currentTimeMillis();

         double wallTimeElapsed = ((double) (wallTime - wallStartTime)) * 0.001;
         double realTimeRate = simTime / wallTimeElapsed;
         System.out.println("Real Time Rate = " + realTimeRate);
      }
   }

   public static void main(String[] args)
   {
//            createNewtonsCradleSimulation();
//    createSpinningCoinSimulation();
//            createStackOfBouncyBallsSimulation();
//            createBoxDownRampSimulation();
//            createRowOfDominosSimulation();
//      createStackOfBlocksSimulation();
      createRollingObjectsSimulation();
//      createPileOfRandomObjectsSimulation();
   }

}
