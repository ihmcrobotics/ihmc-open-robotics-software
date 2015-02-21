package us.ihmc.simulationconstructionset.util.environments;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class ContactableDoorRobotTest
{   
   private YoVariableRegistry doorTestRegistry;
   
   private Robot[] robots;
   
   private final SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromEnvironmentVariables();
   private final SimulationTestingParameters testingParams = new SimulationTestingParameters();
   
   RigidBodyTransform doorToWorldTransform = new RigidBodyTransform();
   
   @AverageDuration
   @Test(timeout = 100000)
   public void testPointIsClose()
   {      
      Vector3d pos = new Vector3d(5.0, -5.0, 10.0);

      ContactableDoorRobot door = createDefaultDoor(pos, 0);
      
      Point3d tempPos = new Point3d(pos);
      
      assertFalse(door.isClose(new Point3d()));
      assertFalse(door.isClose(new Point3d(1e4, -1e4, 1e4)));
      assertTrue(door.isClose(tempPos));
      Point3d diagonal = new Point3d(
            tempPos.x + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.x, 
            tempPos.y + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.y, 
            tempPos.z + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.z);
      assertTrue(door.isClose(diagonal));
   }
   
   @AverageDuration
   @Test(timeout=300000)
   public void testDoorIsClosing()
   {
      doorTestRegistry = new YoVariableRegistry("doorTestRegistry");
      robots = new Robot[1];
      
      // create door   
      Vector3d pos = new Vector3d(2.0, 1.0, 0.0);
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
      
      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 50.0);
      try
      {
         blockingSimulationRunner.simulateAndBlock(50.0);
      }
      catch (Exception e)
      {
         System.err.println("Caught exception in SimulationTestHelper.simulateAndBlockAndCatchExceptions. Exception = " + e);
      }
       
      assertTrue(Math.abs(door.getYaw()) < 1e-3);
      
      if(testingParams.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }
   }

   private ContactableDoorRobot createDefaultDoor(Vector3d positionInWorld, int id)
   {
      return new ContactableDoorRobot("robotDoor" + id, positionInWorld);
   }

}
