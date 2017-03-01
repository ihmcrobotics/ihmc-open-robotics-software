package us.ihmc.footstepPlanning.graphSearch;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;

public class PlanarRegionBaseOfCliffAvoiderTest
{
   private final boolean visualize = false;
   private final boolean doAsserts = true;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testBaseOfCliffAvoiderWithSimpleQueriesOnABlock()
   {
      double stepHeight = 0.2;
      double boxSize = 1.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(1.0 + boxSize / 2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(5.0, 5.0); // floor plane
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      YoVariableRegistry registry = new YoVariableRegistry("Test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      PlanarRegionBaseOfCliffAvoider avoider = new PlanarRegionBaseOfCliffAvoider(registry, yoGraphicsListRegistry);

      BipedalFootstepPlannerParameters parameters = new BipedalFootstepPlannerParameters(registry);
      parameters.setCliffHeightToShiftAwayFrom(0.01);
      double minimumDistanceFromCliffBottom = 0.22;
      parameters.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottom);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("TestRobot"));
         scs.addYoVariableRegistry(registry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         staticLinkGraphics.addCoordinateSystem(1.0);
         staticLinkGraphics.addPlanarRegionsList(planarRegionsList, YoAppearance.Green(), YoAppearance.Beige(), YoAppearance.Yellow(), YoAppearance.Orange());
         scs.addStaticLinkGraphics(staticLinkGraphics);
         scs.startOnAThread();
      }

      RobotSide footstepSide = RobotSide.LEFT;
      double x = 0.9;
      double y = 0.0;
      doAQuery(planarRegionsList, avoider, parameters, minimumDistanceFromCliffBottom, scs, footstepSide, x, y);
      
      x = 0.9;
      y = -0.375;
      doAQuery(planarRegionsList, avoider, parameters, minimumDistanceFromCliffBottom, scs, footstepSide, x, y);

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testBaseOfCliffAvoider()
   {
      double stepHeight = 0.2;
      double boxSize = 1.0;
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(1.0 + boxSize / 2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(5.0, 5.0); // floor plane
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      YoVariableRegistry registry = new YoVariableRegistry("Test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      PlanarRegionBaseOfCliffAvoider avoider = new PlanarRegionBaseOfCliffAvoider(registry, yoGraphicsListRegistry);

      BipedalFootstepPlannerParameters parameters = new BipedalFootstepPlannerParameters(registry);
      parameters.setCliffHeightToShiftAwayFrom(0.01);
      double minimumDistanceFromCliffBottom = 0.22;
      parameters.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottom);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("TestRobot"));
         scs.addYoVariableRegistry(registry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         staticLinkGraphics.addCoordinateSystem(1.0);
         staticLinkGraphics.addPlanarRegionsList(planarRegionsList, YoAppearance.Green(), YoAppearance.Beige(), YoAppearance.Yellow(), YoAppearance.Orange());
         scs.addStaticLinkGraphics(staticLinkGraphics);
         scs.startOnAThread();
      }

      RobotSide footstepSide = RobotSide.LEFT;

      for (double x = 0.0; x < 2.0; x = x + 0.025)
      {
         for (double y = -1.0; y < 1.0; y = y + 0.025)
         {
            doAQuery(planarRegionsList, avoider, parameters, minimumDistanceFromCliffBottom, scs, footstepSide, x, y);
         }
      }

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   private void doAQuery(PlanarRegionsList planarRegionsList, PlanarRegionBaseOfCliffAvoider avoider, BipedalFootstepPlannerParameters parameters,
                         double minimumDistanceFromCliffBottom, SimulationConstructionSet scs, RobotSide footstepSide, double x, double y)
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      soleTransform.setTranslation(x, y, 0.0);
      BipedalFootstepPlannerNode node = new BipedalFootstepPlannerNode(footstepSide, soleTransform);
      avoider.shiftAwayFromCliffBottoms(parameters, planarRegionsList, node);

      RigidBodyTransform newSoleTransform = new RigidBodyTransform();
      node.getSoleTransform(newSoleTransform);

      if ((x > 1.0 - minimumDistanceFromCliffBottom) && (x < 1.0 - 0.1 - 1e-6) && (Math.abs(y) < 0.5-1e-6)) // Close to first cliff
      {
         Point3D solePosition = node.getSolePosition();

         if (doAsserts)
            assertEquals("x = " + x + ", y = " + y, 1.0 - minimumDistanceFromCliffBottom, solePosition.getX(), 1e-7);

      }
      
      if ((y > -0.5 - minimumDistanceFromCliffBottom) && (y < -0.5 - 0.1 - 1e-6) && (Math.abs(x - 1.5) < 0.5-1e-6)) // Close to left side cliff
      {
         Point3D solePosition = node.getSolePosition();

         if (doAsserts)
            assertEquals("x = " + x + ", y = " + y, -0.5 - minimumDistanceFromCliffBottom, solePosition.getY(), 1e-7);

      }
      
      if ((y < 0.5 + minimumDistanceFromCliffBottom) && (y > 0.5 + 0.1 + 1e-6) && (Math.abs(x - 1.5) < 0.5-1e-6)) // Close to right side cliff
      {
         Point3D solePosition = node.getSolePosition();

         if (doAsserts)
            assertEquals("x = " + x + ", y = " + y, 0.5 + minimumDistanceFromCliffBottom, solePosition.getY(), 1e-7);
      }

      if (visualize)
      {
         scs.setTime(scs.getTime() + 0.01);
         scs.tickAndUpdate();
      }
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionBaseOfCliffAvoider.class, PlanarRegionBaseOfCliffAvoiderTest.class);
   }
}
