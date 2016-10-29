package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.visualize.DefaultCollisionVisualizer;
import us.ihmc.tools.thread.ThreadTools;

public class NewtonsCradleSimulation
{
   public static void createNewtonsCradleSimulation()
   {
      NewtonsCradleRobot robot = new NewtonsCradleRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);
      scs.startOnAThread();

      //      CollisionHandler handler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      CollisionHandler handler = new DefaultCollisionHandler(0.99, 0.15);

      DefaultCollisionVisualizer visualize = new DefaultCollisionVisualizer(4.0, 4.0, scs, 100);

      handler.addListener(visualize);
      ScsCollisionDetector collisionDetector = robot.getCollisionDetector();
      collisionDetector.initialize();

      scs.initPhysics(new ScsPhysics(null, collisionDetector, handler, visualize));
   }

   public static void createSpinningCoinSimulation()
   {
      SpinningCoinRobot spinningCoinRobot = new SpinningCoinRobot();
      ArrayList<Robot> robots = spinningCoinRobot.getRobots();

      ScsCollisionDetector collisionDetector = spinningCoinRobot.getCollisionDetector();
      Robot groundRobot = new GroundAsABoxRobot(collisionDetector);
      robots.add(groundRobot);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(100000);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 1);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      double epsilon = 0.3;
      double mu = 0.7;
      CollisionHandler handler = new DefaultCollisionHandler(epsilon, mu);
      DefaultCollisionVisualizer visualize = new DefaultCollisionVisualizer(10.0, 10.0, scs, 100);

      handler.addListener(visualize);
      collisionDetector.initialize();

      scs.initPhysics(new ScsPhysics(null, collisionDetector, handler, visualize));

      scs.setSimulateDuration(60.0);
      scs.simulate();
   }

   public static void createStackOfBouncyBallsSimulation()
   {
      StackOfBouncyBallsRobot robot = new StackOfBouncyBallsRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);
      scs.startOnAThread();

      //      CollisionHandler handler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new DefaultCollisionHandler(0.98, 0.1, robot);
      CollisionHandler handler = new DefaultCollisionHandler(1.0, 0.0);
      //      CollisionHandler handler = new DefaultCollisionHandler(0.3, 0.7, robot);

      DefaultCollisionVisualizer visualize = new DefaultCollisionVisualizer(0.1, 0.1, scs, 100);

      handler.addListener(visualize);
      ScsCollisionDetector collisionDetector = robot.getCollisionDetector();
      collisionDetector.initialize();

      scs.initPhysics(new ScsPhysics(null, collisionDetector, handler, visualize));
   }

   public static void createRowOfDominosSimulation()
   {
      RowOfDominosRobot robot = new RowOfDominosRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      //      CollisionHandler handler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new DefaultCollisionHandler(0.98, 0.1, robot);
      CollisionHandler handler = new DefaultCollisionHandler(0.3, 0.7);

      //      DefaultCollisionVisualize visualize = new DefaultCollisionVisualize(100.0, 100.0, scs, 100);
      //
      //      handler.addListener(visualize);
      DefaultCollisionVisualizer visualize = null;

      ScsCollisionDetector collisionDetector = robot.getCollisionDetector();
      collisionDetector.initialize();

      scs.initPhysics(new ScsPhysics(null, collisionDetector, handler, visualize));
   }

   public static void createStackOfBlocksSimulation()
   {
      int numberOfBlocks = 10;
      StackOfBlocksRobot stackOfBlocksRobot = new StackOfBlocksRobot(numberOfBlocks);
      ArrayList<Robot> robots = stackOfBlocksRobot.getRobots();

      ScsCollisionDetector collisionDetector = stackOfBlocksRobot.getCollisionDetector();
      Robot groundRobot = new GroundAsABoxRobot(collisionDetector);
      robots.add(groundRobot);

      boolean showGUI = true;

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(showGUI);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.00025, 1);
      scs.setFastSimulate(false);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      scs.setSimulateDuration(2.0);
      CollisionHandler handler = new DefaultCollisionHandler(0.3, 0.7);

      DefaultCollisionVisualizer visualize = new DefaultCollisionVisualizer(100.0, 100.0, scs, 1000);
//      DefaultCollisionVisualizer visualize = null;
      handler.addListener(visualize);

      collisionDetector.initialize();

      scs.initPhysics(new ScsPhysics(null, collisionDetector, handler, visualize));
   }

   public static void createPileOfRandomObjectsSimulation()
   {
      PileOfRandomObjectsRobot pileOfRandomObjectsRobot = new PileOfRandomObjectsRobot();
      ArrayList<Robot> robots = pileOfRandomObjectsRobot.getRobots();

      ScsCollisionDetector collisionDetector = pileOfRandomObjectsRobot.getCollisionDetector();
      Robot groundRobot = new GroundAsABoxRobot(collisionDetector, true);
      robots.add(groundRobot);

      boolean showGUI = true;

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(4000);
      parameters.setCreateGUI(showGUI);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.00025, 10);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      CollisionHandler handler = new DefaultCollisionHandler(0.3, 0.7);

//          DefaultCollisionVisualizer visualize = new DefaultCollisionVisualizer(100.0, 100.0, scs, 1000);
      DefaultCollisionVisualizer visualize = null;
//            handler.addListener(visualize);

      collisionDetector.initialize();

      scs.initPhysics(new ScsPhysics(null, collisionDetector, handler, visualize));

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
//            createStackOfBouncyBallsSimulation();
//            createRowOfDominosSimulation();
      createPileOfRandomObjectsSimulation();
//            createSpinningCoinSimulation();
//      createStackOfBlocksSimulation();
   }
}
