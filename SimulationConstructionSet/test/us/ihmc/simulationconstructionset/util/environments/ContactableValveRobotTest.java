package us.ihmc.simulationconstructionset.util.environments;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.thread.ThreadTools;

public class ContactableValveRobotTest
{
   private Robot[] robots = new Robot[2]; // 0 == floatingRobot ; 1 == valveRobot 

   private RobotController floatingRobotController;

   private SliderJoint horizontalJoint;
   private SliderJoint verticalJoint;

   private YoVariableRegistry valveTestRegistry;

   @ContinuousIntegrationTest(estimatedDuration = 2.3)
   @Test(timeout = 30000)
   public void testValveIsClosing()
   {
      boolean isValveClosed = false;
      valveTestRegistry = new YoVariableRegistry("valveTestRegistry");

      createValveRobot();
      createFloatingRobot();
      floatingRobotController.initialize();

      createContactPoints(robots[0]);

      SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      SimulationConstructionSet scs = new SimulationConstructionSet(robots, simulationTestingParameters);

      scs.addYoVariableRegistry(valveTestRegistry);

      Thread myThread = new Thread(scs);
      myThread.start();

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 4.0);
      try
      {
         blockingSimulationRunner.simulateAndBlock(4.0);
      }
      catch (Exception e)
      {
         PrintTools.error(this, e.getMessage());
      }

      if (robots[1].getVariable("valveClosePercentage").getValueAsDouble() >= 99.0)
         isValveClosed = true;

      assertTrue(isValveClosed);
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }
      
      blockingSimulationRunner.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetValveTransformToWorld()
   {
      Random random = new Random(1235125L);

      double valveX = RandomNumbers.nextDouble(random, -2.0, 2.0);
      double valveY = RandomNumbers.nextDouble(random, -2.0, 2.0);
      double valveZ = RandomNumbers.nextDouble(random, 0.1, 2.0);
      double valveYaw_degrees = RandomNumbers.nextDouble(random, -100, 100);

      ContactableValveRobot valveRobot = createValveRobot(valveX, valveY, valveZ, valveYaw_degrees);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame(), valveTransformToWorld);

      assertEquals(valveX, valvePose.getX(), 1e-7);
      assertEquals(valveY, valvePose.getY(), 1e-7);
      assertEquals(valveZ, valvePose.getZ(), 1e-7);

      assertEquals(valveYaw_degrees, Math.toDegrees(valvePose.getYaw()), 1e-7);
   }

   private void createFloatingRobot()
   {

      Robot floatingRobot = new Robot("floatingRobot");
      Vector3D position = new Vector3D(0.0, 0.02, 1.1);
      double length = 0.01;

      floatingRobot.setGravity(0.0, 0.0, 0.0);

      horizontalJoint = new SliderJoint("y", position, floatingRobot, Axis.Y);

      floatingRobot.addRootJoint(horizontalJoint);

      Link linkHorizontal = new Link("linkHorizontal");
      linkHorizontal.setMass(0.5);
      linkHorizontal.setComOffset(length / 2.0, 0.0, 0.0);
      linkHorizontal.setMomentOfInertia(0.0, 0.01, 0.0);

      Graphics3DObject linkHorizontalGraphics = new Graphics3DObject();
      linkHorizontalGraphics.addCylinder(length * 10, 0.005, YoAppearance.Orange());
      linkHorizontal.setLinkGraphics(linkHorizontalGraphics);

      horizontalJoint.setLink(linkHorizontal);

      verticalJoint = new SliderJoint("z", new Vector3D(0.0, 0.0, 0.0), floatingRobot, Axis.Z);

      Link linkVertical = new Link("linkVertical");
      linkVertical.setMass(0.5);
      linkVertical.setComOffset(length / 2.0, 0.0, 0.0);
      linkVertical.setMomentOfInertia(0.0, 0.01, 0.0);

      Graphics3DObject linkVerticalGraphics = new Graphics3DObject();
      linkVerticalGraphics.addCylinder(length, 0.005, YoAppearance.Blue());
      linkVertical.setLinkGraphics(linkVerticalGraphics);

      verticalJoint.setLink(linkVertical);

      horizontalJoint.addJoint(verticalJoint);

      createFloatingRobotController();
      robots[0] = floatingRobot;
      robots[0].setController(floatingRobotController);
   }

   private void createContactPoints(Robot floatingRobot)
   {
      GroundContactPoint contactPoint1 = new GroundContactPoint("contactPoint1", new Vector3D(0.0, 0.0, 0.0), floatingRobot);
      verticalJoint.addGroundContactPoint(1, contactPoint1);

      GroundContactPoint contactPoint2 = new GroundContactPoint("contactPoint2", new Vector3D(-0.002, 0.0, 0.0), floatingRobot);
      verticalJoint.addGroundContactPoint(1, contactPoint2);

      GroundContactPoint contactPoint3 = new GroundContactPoint("contactPoint3", new Vector3D(0.002, 0.0, 0.0), floatingRobot);
      verticalJoint.addGroundContactPoint(1, contactPoint3);

      ContactController contactController = new ContactController();
      contactController.setContactParameters(10000.0, 1000.0, 0.5, 0.3);
      contactController.addContactPoints(robots[0].getAllGroundContactPoints());

      ArrayList<Contactable> robotList = new ArrayList<Contactable>();
      robotList.add((Contactable) robots[1]);
      contactController.addContactables(robotList);
      robots[1].setController(contactController);
   }

   private void createFloatingRobotController()
   {
      floatingRobotController = new RobotController()
      {
         private YoVariableRegistry robotControllerRegistry = new YoVariableRegistry("robotControllerRegistry");

         @Override
         public void initialize()
         {
            horizontalJoint.setKp(1000);
            horizontalJoint.setKd(50);
            verticalJoint.setKp(1000);
            verticalJoint.setKd(50);
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return robotControllerRegistry;
         }

         @Override
         public String getName()
         {
            return "floatingRobotController";
         }

         @Override
         public String getDescription()
         {
            return floatingRobotController.getDescription();
         }

         @Override
         public void doControl()
         {
            if (robots[0].getTime() < 1.0)
            {
               horizontalJoint.setqDesired(-0.05);
               verticalJoint.setqDesired(0.0);
            }
            else if (robots[0].getTime() >= 1.0 && robots[0].getTime() < 2.0)
            {
               horizontalJoint.setqDesired(-0.1);
               verticalJoint.setqDesired(-0.05);
            }
         }
      };
   }

   private void createValveRobot()
   {
      ContactableValveRobot valveRobot = createValveRobot(0.0, 0.0, 1.0, 0.0);
      robots[1] = valveRobot;
   }

   private ContactableValveRobot createValveRobot(double x, double y, double z, double yaw_degrees)
   {
      double forceVectorScale = 1.0 / 50.0;

      FramePose valvePoseInWorld = createFramePose(x, y, z, yaw_degrees);

      ContactableValveRobot valveRobot = new ContactableValveRobot("valveRobot", ValveType.SMALL_VALVE, 0.125, valvePoseInWorld);
      valveRobot.createValveRobot();
      valveRobot.createAvailableContactPoints(1, 30, forceVectorScale, true);

      return valveRobot;
   }

   private FramePose createFramePose(double x, double y, double z, double yaw_degrees)
   {
      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame());

      Point3D position = new Point3D(x, y, z);
      Quaternion orientation = new Quaternion();

      orientation.setYawPitchRoll(Math.toRadians(yaw_degrees), Math.toRadians(0), Math.toRadians(0));
      framePose.setPose(position, orientation);

      return framePose;
   }
}
