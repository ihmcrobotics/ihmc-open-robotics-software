package us.ihmc.simulationConstructionSetTools.util.environments;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.PrintTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.ContactableDoorRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.commons.thread.ThreadTools;

public class ContactableDoorRobotTest
{   
   private YoRegistry doorTestRegistry;
   
   private Robot[] robots;
   
   private final SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
   private final SimulationTestingParameters testingParams = new SimulationTestingParameters();
   
   RigidBodyTransform doorToWorldTransform = new RigidBodyTransform();
   
   @Test
   public void testPointIsClose()
   {      
      double epsilon = 1e-4;
      Point3D pos = new Point3D(5.0, -5.0, 10.0);

      ContactableDoorRobot door = createDefaultDoor(pos, 0);
      
      Point3D tempPos = new Point3D(pos);
      
      assertFalse(door.isClose(new Point3D()));
      assertFalse(door.isClose(new Point3D(1e4, -1e4, 1e4)));
      assertTrue(door.isPointOnOrInside(tempPos));
      Point3D diagonal = new Point3D(
            tempPos.getX() + (1.0 - epsilon)*ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), 
            tempPos.getY() + (1.0 - epsilon)*ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getY(), 
            tempPos.getZ() + (1.0 - epsilon)*ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getZ());
      assertTrue(door.isPointOnOrInside(diagonal));
   }
   
   @Test
   public void testDoorIsClosing()
   {
      doorTestRegistry = new YoRegistry("doorTestRegistry");
      robots = new Robot[1];
      
      // create door   
      Point3D pos = new Point3D(2.0, 1.0, 0.0);
      int id = 1;
      ContactableDoorRobot door = createDefaultDoor(pos, id);
      
      robots[0] = door;
      door.setYaw(0.3);
      door.setKdDoor(1.0);
      door.setKpDoor(0.1);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robots, parameters);
      
      scs.addYoRegistry(doorTestRegistry);
      
      Thread myThread = new Thread(scs);
      myThread.start();
      
      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0);
      try
      {
         blockingSimulationRunner.simulateAndBlock(60.0);
      }
      catch (Exception e)
      {
         PrintTools.error(this, e.getMessage());
      }
       
      assertTrue(Math.abs(door.getHingeYaw()) < 1e-3);
      
      if(testingParams.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }
   }

   private ContactableDoorRobot createDefaultDoor(Point3D positionInWorld, int id)
   {
      return new ContactableDoorRobot("robotDoor" + id, positionInWorld);
   }

}
