package us.ihmc.exampleSimulations.newtonsCradle;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.visualize.DefaultCollisionVisualize;
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
      CollisionHandler handler = new DefaultCollisionHandler(1.0, 0.0);

      DefaultCollisionVisualize visualize = new DefaultCollisionVisualize();

      handler.addListener(visualize);
      ScsCollisionDetector collisionDetector = robot.getCollisionDetector();

      handler.initialize(collisionDetector);
      collisionDetector.initialize(handler);

      scs.initPhysics(new ScsPhysics(null, collisionDetector, visualize));
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

      DefaultCollisionVisualize visualize = new DefaultCollisionVisualize();

      handler.addListener(visualize);
      ScsCollisionDetector collisionDetector = robot.getCollisionDetector();

      handler.initialize(collisionDetector);
      collisionDetector.initialize(handler);

      scs.initPhysics(new ScsPhysics(null, collisionDetector, visualize));
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

      DefaultCollisionVisualize visualize = new DefaultCollisionVisualize();

      handler.addListener(visualize);
      ScsCollisionDetector collisionDetector = robot.getCollisionDetector();

      handler.initialize(collisionDetector);
      collisionDetector.initialize(handler);

      scs.initPhysics(new ScsPhysics(null, collisionDetector, visualize));
   }
   
   public static void createPileOfRandomObjectsSimulation()
   {
      PileOfRandomObjectsRobot pileOfRandomObjectsRobot = new PileOfRandomObjectsRobot();
      ArrayList<Robot> robots = pileOfRandomObjectsRobot.getRobots();
      
      boolean showGUI = true;
      
      Robot[] robotArray = new Robot[robots.size()];
      robots.toArray(robotArray);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(showGUI);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArray, parameters);
      scs.setDT(0.0001, 100);
      scs.setGroundVisible(false);
      scs.startOnAThread();

      CollisionHandler handler = new DefaultCollisionHandler(0.3, 0.7);

      DefaultCollisionVisualize visualize = new DefaultCollisionVisualize();

      handler.addListener(visualize);
      ScsCollisionDetector collisionDetector = pileOfRandomObjectsRobot.getCollisionDetector();

      handler.initialize(collisionDetector);
      collisionDetector.initialize(handler);

      scs.initPhysics(new ScsPhysics(null, collisionDetector, visualize));
      
      scs.simulate();
      
      long wallStartTime = System.currentTimeMillis();
      while(true)
      {
         ThreadTools.sleep(1000);

         double simTime = scs.getTime();
         long wallTime = System.currentTimeMillis();

         double wallTimeElapsed = ((double) (wallTime - wallStartTime)) * 0.001;
         double realTimeRate = simTime / wallTimeElapsed;
         System.out.println("Real Time Rate = " + realTimeRate);
      }
   }

   public static void main(String[] args)
   {
      createNewtonsCradleSimulation();
//      createStackOfBouncyBallsSimulation();
//      createRowOfDominosSimulation();
//      createPileOfRandomObjectsSimulation();
   }
}
