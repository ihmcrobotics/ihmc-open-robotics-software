package us.ihmc.simulationconstructionset.util.environments;

import static org.junit.Assert.*;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.robotController.ContactController;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class ContactableValveRobotTest
{
   private boolean showGUI = false;

   private Robot[] robots = new Robot[2]; // 0 == floatingRobot ; 1 == valveRobot 

   private RobotController floatingRobotController;

   private SliderJoint horizontalJoint;
   private SliderJoint verticalJoint;

   private YoVariableRegistry valveTestRegistry;

   @Test(timeout=300000)
   public void testValveIsClosing()
   {
      boolean isValveClosed = true;
      valveTestRegistry = new YoVariableRegistry("valveTestRegistry");

      createValveRobot();
      createFloatingRobot();
      floatingRobotController.initialize();

      createContactPoints(robots[0]);

      SimulationConstructionSet scs = new SimulationConstructionSet(robots, showGUI);

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
         System.err.println("Caught exception in SimulationTestHelper.simulateAndBlockAndCatchExceptions. Exception = " + e);
      }
      
      if (robots[1].getVariable("valveClosePercentage").getValueAsDouble() >= 99.0)
         isValveClosed = true;
      
      assertTrue(isValveClosed);
   }

   private void createFloatingRobot()
   {

      Robot floatingRobot = new Robot("floatingRobot");
      Vector3d position = new Vector3d(0.0, 0.02, 1.1);
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

      verticalJoint = new SliderJoint("z", new Vector3d(0.0, 0.0, 0.0), floatingRobot, Axis.Z);

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
      GroundContactPoint contactPoint1 = new GroundContactPoint("contactPoint1", new Vector3d(0.0, 0.0, 0.0), floatingRobot);
      verticalJoint.addGroundContactPoint(1, contactPoint1);

      GroundContactPoint contactPoint2 = new GroundContactPoint("contactPoint2", new Vector3d(-0.002, 0.0, 0.0), floatingRobot);
      verticalJoint.addGroundContactPoint(1, contactPoint2);

      GroundContactPoint contactPoint3 = new GroundContactPoint("contactPoint3", new Vector3d(0.002, 0.0, 0.0), floatingRobot);
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
      double forceVectorScale = 1.0 / 50.0;
      FramePose valvePoseInWorld = new FramePose(ReferenceFrame.getWorldFrame());
      valvePoseInWorld.setPose(new Point3d(0.0, 0.0, 1.0), new Quat4d(0.0, 0.0, 0.0, 1.0));
     
      ContactableValveRobot valveRobot = new ContactableValveRobot("valveRobot", ValveType.SMALL_VALVE, 0.125, valvePoseInWorld);
      valveRobot.createValveRobot();
      valveRobot.createAvailableContactPoints(1, 30, forceVectorScale, true);
      
      robots[1] = valveRobot;
   }
}
