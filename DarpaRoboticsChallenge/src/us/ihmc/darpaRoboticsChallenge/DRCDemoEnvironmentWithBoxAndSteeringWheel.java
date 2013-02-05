package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.environments.SelectableObject;
import com.yobotics.simulationconstructionset.util.environments.SelectableObjectListener;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.CombinedTerrainObject;
import com.yobotics.simulationconstructionset.util.ground.Contactable;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject;

public class DRCDemoEnvironmentWithBoxAndSteeringWheel implements CommonAvatarEnvironmentInterface
{
   private static final boolean debug = false;
   private static final double BOX_LENGTH = 0.5;
   private static final double BOX_WIDTH = 0.5;
   private static final double BOX_HEIGHT = 0.51;
   private final CombinedTerrainObject combinedTerrainObject;

   private final ArrayList<Robot> boxRobots = new ArrayList<Robot>();
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   private final ArrayList<Contactable> contactables = new ArrayList<Contactable>();

   Vector3d wheelLoc = new Vector3d(0.0, 0.25+(2*ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS), 0.75);

   public DRCDemoEnvironmentWithBoxAndSteeringWheel()
   {
      combinedTerrainObject = createCombinedTerrainObject();

      ContactableSelectableSteeringWheelRobot bot = new ContactableSelectableSteeringWheelRobot("steeringWheel", wheelLoc);
      boxRobots.add(bot);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(bot, 36000);
      scs.setDT(0.001, 1);

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
      scs.addStaticLinkGraphics(combinedTerrainObject.getLinkGraphics());

      scs.addDynamicGraphicObjectListRegistries(dynamicGraphicObjectsListRegistry);
      scs.setGroundVisible(true);

      scs.startOnAThread();
   }



   private CombinedTerrainObject createCombinedTerrainObject()
   {
      CombinedTerrainObject terrainObject = new CombinedTerrainObject("carSeatBox");

      // seat
      terrainObject.addBox(-BOX_LENGTH / 2.0, -BOX_WIDTH / 2.0, BOX_LENGTH / 2.0, BOX_WIDTH / 2.0, BOX_HEIGHT);

      // ground
      terrainObject.addBox(-100.0, -100.0, 100.0, 100.0, -0.05, 0.0, YoAppearance.DarkGray());

      return terrainObject;
   }


   public TerrainObject getTerrainObject()
   {
      return combinedTerrainObject;
   }

   public List<Robot> getEnvironmentRobots()
   {
      return new ArrayList<Robot>(boxRobots);
   }

   public void addContactPoints(ExternalForcePoint[] contactPoints)
   {
      for (ExternalForcePoint contactPoint : contactPoints)
      {
         this.contactPoints.add(contactPoint);
      }
   }

   public void createAndSetContactControllerToARobot()
   {
      // add contact controller to any robot so it gets called
      ContactController contactController = new ContactController();
      contactController.addContactPoints(contactPoints);
      contactController.addContactables(contactables);
      boxRobots.get(0).setController(contactController);
   }


   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
      for (Contactable contactable : contactables)
      {
         if (contactable instanceof SelectableObject)
         {
            ((SelectableObject) contactable).addSelectedListeners(selectedListener);
         }
      }
   }

   public static void main(String[] args)
   {
      DRCDemoEnvironmentWithBoxAndSteeringWheel env = new DRCDemoEnvironmentWithBoxAndSteeringWheel();
      if (debug)
      {
         env.testWheelContactPoints();
      }
   }
   
   
   private void testWheelContactPoints()
   {
      for (Robot bot : boxRobots)
      {
         if (bot instanceof ContactableSelectableSteeringWheelRobot)
         {
            // Z is top, X is left/right side, Y is toward/away from driver
            Point3d centerLoc = new Point3d(wheelLoc.x, wheelLoc.y, wheelLoc.z);
            boolean gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, centerLoc, false);

            Point3d topPoint = new Point3d(wheelLoc.x,wheelLoc.y,wheelLoc.z+ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS);
            gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, topPoint, true);

            Point3d bottomEdgePoint = new Point3d(wheelLoc.x,wheelLoc.y,wheelLoc.z-(ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS+ContactableSelectableSteeringWheelRobot.DEFAULT_THICKNESS));
            gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, bottomEdgePoint, true);

            Point3d leftPoint = new Point3d(wheelLoc.x-ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS,wheelLoc.y,wheelLoc.z);
            gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, leftPoint, true);

            Point3d rightEdgePoint = new Point3d(wheelLoc.x+(ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS+ContactableSelectableSteeringWheelRobot.DEFAULT_THICKNESS),wheelLoc.y,wheelLoc.z);
            gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, rightEdgePoint, true);

            Point3d aboveTop = new Point3d(wheelLoc.x,wheelLoc.y, wheelLoc.z+(2.5*ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS));
            gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, aboveTop, false);

            Point3d barelyTooFarLeft = new Point3d(wheelLoc.x -(ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS+ContactableSelectableSteeringWheelRobot.DEFAULT_THICKNESS+0.001),wheelLoc.y, wheelLoc.z);
            gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, barelyTooFarLeft, false);

            Point3d tooFarRight = new Point3d(wheelLoc.x + (1.7*ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS),wheelLoc.y, wheelLoc.z);
            gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, tooFarRight, false);

            Point3d tooCloseToDriver = new Point3d(wheelLoc.x, wheelLoc.y-(1.2*ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS), wheelLoc.z);
            gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, tooCloseToDriver, false);

            Point3d tooFarFromDriver = new Point3d(wheelLoc.x, wheelLoc.y + (1.1*ContactableSelectableSteeringWheelRobot.DEFAULT_RADIUS), wheelLoc.z);
            gotExpectedResult = testContactPoint((ContactableSelectableSteeringWheelRobot)bot, tooFarFromDriver, false);

         }
      }

   }
   public  boolean testContactPoint(ContactableSelectableSteeringWheelRobot bot, Point3d point, boolean expectedResult)
   {
      boolean result = bot.isPointOnOrInside(point);
      if (result == expectedResult)
      {
         System.out.println("point in wheel test, pass " + point);
      }   
      else
      {
         System.out.println("point in wheel test, fail " + point);
      }
      drawTestPoint(combinedTerrainObject, point, 0.011, result == true?YoAppearance.Green():YoAppearance.Red());
      return result == expectedResult;
   }
   
   private void drawTestPoint(CombinedTerrainObject combinedTerrainObject, Point3d point, double radius, AppearanceDefinition appearance)
   {
      Graphics3DObject linkGraphics = combinedTerrainObject.getLinkGraphics();//bot.getWheelLink().getLinkGraphics();
      linkGraphics.identity();
      linkGraphics.translate(point.x, point.y, point.z);
      linkGraphics.addSphere(radius, appearance);
   }
   
   

}
