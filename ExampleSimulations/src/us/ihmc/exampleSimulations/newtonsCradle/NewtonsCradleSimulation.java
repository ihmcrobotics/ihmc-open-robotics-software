package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
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

      //      CollisionHandler collisionHandler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.99, 0.15);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(4.0, 4.0, scs, 100);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.startOnAThread();
   }

   public static void createSpinningCoinSimulation()
   {
      SpinningCoinRobot spinningCoinRobot = new SpinningCoinRobot();
      ArrayList<Robot> robots = spinningCoinRobot.getRobots();

      Robot groundRobot = new GroundAsABoxRobot();
      robots.add(groundRobot);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(100000);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 1);
      scs.setGroundVisible(false);

      double epsilon = 0.3;
      double mu = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(epsilon, mu);
      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(10.0, 10.0, scs, 100);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.startOnAThread();

      scs.setSimulateDuration(60.0);
      scs.simulate();
   }

   public static void createStackOfBouncyBallsSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();
      StackOfBouncyBallsRobot robot = new StackOfBouncyBallsRobot();
      robots.add(robot);

      Robot groundRobot = new GroundAsABoxRobot();
      robots.add(groundRobot);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray);
      scs.setDT(0.0001, 100);
      scs.setGroundVisible(false);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(0.1, 0.1, scs, 100);

      double coefficientOfRestitution = 0.9;
      double coefficientOfFriction = 0.0;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.98, 0.1, robot);
      //      CollisionHandler collisionHandler = new DefaultCollisionHandler(0.3, 0.7, robot);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.startOnAThread();
   }

   public static void createRowOfDominosSimulation()
   {
      ArrayList<Robot> robots = new ArrayList<>();
      RowOfDominosRobot robot = new RowOfDominosRobot();
      robots.add(robot);

      Robot groundRobot = new GroundAsABoxRobot();
      robots.add(groundRobot);

      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray);
      scs.setDT(0.0001, 100);
      scs.setGroundVisible(false);

      //      DefaultCollisionVisualize collisionVisualizer = new DefaultCollisionVisualize(100.0, 100.0, scs, 100);
      DefaultCollisionVisualizer collisionVisualizer = null;

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      //      CollisionHandler handler = new SpringCollisionHandler(2.0, 1.1, 1.1, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new SpringCollisionHandler(1, 1000, 10.0, robot.getRobotsYoVariableRegistry());
      //      CollisionHandler handler = new DefaultCollisionHandler(0.98, 0.1, robot);
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);

      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.startOnAThread();
   }

   public static void createStackOfBlocksSimulation()
   {
      int numberOfBlocks = 10;
      StackOfBlocksRobot stackOfBlocksRobot = new StackOfBlocksRobot(numberOfBlocks);
      ArrayList<Robot> robots = stackOfBlocksRobot.getRobots();

      Robot groundRobot = new GroundAsABoxRobot();
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
      scs.setSimulateDuration(2.0);

      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, scs, 1000);
//      DefaultCollisionVisualizer collisionVisualizer = null;

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
      scs.startOnAThread();
   }

   public static void createPileOfRandomObjectsSimulation()
   {
      PileOfRandomObjectsRobot pileOfRandomObjectsRobot = new PileOfRandomObjectsRobot();
      ArrayList<Robot> robots = pileOfRandomObjectsRobot.getRobots();

      Robot groundRobot = new GroundAsABoxRobot(true);
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

      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);

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
//            createRowOfDominosSimulation();
//      createStackOfBlocksSimulation();
      createPileOfRandomObjectsSimulation();
   }

}
