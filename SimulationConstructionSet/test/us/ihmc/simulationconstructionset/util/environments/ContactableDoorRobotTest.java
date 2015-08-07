package us.ihmc.simulationconstructionset.util.environments;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class ContactableDoorRobotTest
{   
   private YoVariableRegistry doorTestRegistry;
   
   private Robot[] robots;
   
   private final SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromEnvironmentVariables();
   private final SimulationTestingParameters testingParams = new SimulationTestingParameters();
   
   RigidBodyTransform doorToWorldTransform = new RigidBodyTransform();
   
   @EstimatedDuration
   @Test(timeout = 100000)
   public void testPointIsClose()
   {      
      double epsilon = 1e-4;
      Point3d pos = new Point3d(5.0, -5.0, 10.0);

      ContactableDoorRobot door = createDefaultDoor(pos, 0);
      
      Point3d tempPos = new Point3d(pos);
      
      assertFalse(door.isClose(new Point3d()));
      assertFalse(door.isClose(new Point3d(1e4, -1e4, 1e4)));
      assertTrue(door.isPointOnOrInside(tempPos));
      Point3d diagonal = new Point3d(
            tempPos.x + (1.0 - epsilon)*ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.x, 
            tempPos.y + (1.0 - epsilon)*ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.y, 
            tempPos.z + (1.0 - epsilon)*ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.z);
      assertTrue(door.isPointOnOrInside(diagonal));
   }
   
   @EstimatedDuration
   @Test(timeout=300000)
   public void testDoorIsClosing()
   {
      doorTestRegistry = new YoVariableRegistry("doorTestRegistry");
      robots = new Robot[1];
      
      // create door   
      Point3d pos = new Point3d(2.0, 1.0, 0.0);
      int id = 1;
      ContactableDoorRobot door = createDefaultDoor(pos, id);
      
      robots[0] = door;
      door.setYaw(0.3);
      door.setKdDoor(1.0);
      door.setKpDoor(0.1);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robots, parameters);
      
      scs.addYoVariableRegistry(doorTestRegistry);
      
      Thread myThread = new Thread(scs);
      myThread.start();
      
      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0);
      try
      {
         blockingSimulationRunner.simulateAndBlock(60.0);
      }
      catch (Exception e)
      {
         System.err.println("Caught exception in SimulationTestHelper.simulateAndBlockAndCatchExceptions. Exception = " + e);
      }
       
      assertTrue(Math.abs(door.getHingeYaw()) < 1e-3);
      
      if(testingParams.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }
   }

   private ContactableDoorRobot createDefaultDoor(Point3d positionInWorld, int id)
   {
      return new ContactableDoorRobot("robotDoor" + id, positionInWorld);
   }

}
