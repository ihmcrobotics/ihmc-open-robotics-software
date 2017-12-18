package us.ihmc.footstepPlanning.graphSearch;

import static junit.framework.TestCase.assertFalse;
import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.AxisAngleTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.PlanarRegionBaseOfCliffAvoider;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class PlanarRegionBaseOfCliffAvoiderTest
{
   private final boolean visualize = false;
   private final boolean doAsserts = true;
   private final Random random = new Random(4587L);

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBaseOfCliffAvoiderWithSimpleQueriesOnABlock()
   {
      double stepHeight = 0.2;
      double boxSize = 1.0;
      double edgeOfBoxX = 1.0;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(edgeOfBoxX + boxSize / 2.0, 0.0, 0.0);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      generator.translate(0.0, 0.0, 0.001);
      generator.addRectangle(5.0, 5.0); // floor plane
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      YoVariableRegistry registry = new YoVariableRegistry("Test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double epsilon = 1e-6;
      double minimumDistanceFromCliffBottom = 0.2 - epsilon;
      YoFootstepPlannerParameters parameters = new YoFootstepPlannerParameters(registry, new DefaultFootstepPlanningParameters()
      {
         @Override
         public double getCliffHeightToAvoid()
         {
            return 0.01;
         }

         @Override
         public double getMinimumDistanceFromCliffBottoms()
         {
            return minimumDistanceFromCliffBottom;
         }
      });

      double footLength = 0.2;
      double footWidth = 0.1;
      SideDependentList<ConvexPolygon2D> footPolygons = PlanningTestTools.createFootPolygons(footLength, footWidth);
      PlanarRegionBaseOfCliffAvoider avoider = new PlanarRegionBaseOfCliffAvoider(parameters, footPolygons);
      avoider.setPlanarRegions(planarRegionsList);

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
         ThreadTools.sleepForever();
      }

      double closestNodeDistanceToCliff = edgeOfBoxX - 0.5 * footLength - minimumDistanceFromCliffBottom;

      RobotSide footstepSide = RobotSide.LEFT;
      double x = closestNodeDistanceToCliff;
      double y = 0.0;
      FootstepNode node = new FootstepNode(x, y, 0.0, footstepSide);
      assertTrue(avoider.isNodeValid(node, null));

      x = closestNodeDistanceToCliff + FootstepNode.gridSizeXY;
      node = new FootstepNode(x, y, 0.0, footstepSide);
      assertFalse(avoider.isNodeValid(node, null));

      x = closestNodeDistanceToCliff - FootstepNode.gridSizeXY;
      node = new FootstepNode(x, y, 0.0, footstepSide);
      assertTrue(avoider.isNodeValid(node, null));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000000)
   public void testAvoidingRotatedAndElevatedCliff()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      double cliffHeightToAvoid = 0.2;
      double minimumDistanceFromCliffBottom = 0.1;
      double footLength = 0.2;
      double footWidth = 0.1;

      YoFootstepPlannerParameters parameters = new YoFootstepPlannerParameters(registry, new DefaultFootstepPlanningParameters()
      {
         @Override
         public double getCliffHeightToAvoid()
         {
            return cliffHeightToAvoid;
         }

         @Override
         public double getMinimumDistanceFromCliffBottoms()
         {
            return minimumDistanceFromCliffBottom;
         }
      });

      SideDependentList<ConvexPolygon2D> footPolygons = PlanningTestTools.createFootPolygons(footLength, footWidth);
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(parameters, footPolygons);

      double centerX = 1.0;
      double centerY = 1.0;
      double groundHeight = 0.5;
      double cliffHeight = cliffHeightToAvoid + 0.1;
      double rotation = 0.25 * Math.PI;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(5.0, 5.0);
      generator.translate(centerX, centerY, groundHeight + cliffHeight);
      generator.rotate(rotation, Axis.Z);
      double boxWidth = 0.5;
      generator.addRectangle(boxWidth, boxWidth);

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      cliffAvoider.setPlanarRegions(planarRegionsList);

      Vector2D frontNearNodeOffset = new Vector2D(0.5 * boxWidth + minimumDistanceFromCliffBottom + 0.5 * footLength - FootstepNode.gridSizeXY, 0.0);
      Vector2D frontFarNodeOffset = new Vector2D(0.5 * boxWidth + minimumDistanceFromCliffBottom + 0.5 * footLength + FootstepNode.gridSizeXY, 0.0);
      Vector2D sideNearNodeOffset = new Vector2D(0.0, 0.5 * boxWidth + 0.5 * footWidth + minimumDistanceFromCliffBottom - FootstepNode.gridSizeXY);
      Vector2D sideFarNodeOffset = new Vector2D(0.0, 0.5 * boxWidth + 0.5 * footWidth + minimumDistanceFromCliffBottom + FootstepNode.gridSizeXY);

      AxisAngle rotationTransform = new AxisAngle(rotation, 0.0, 0.0);
      rotationTransform.transform(frontNearNodeOffset);
      rotationTransform.transform(frontFarNodeOffset);
      rotationTransform.transform(sideNearNodeOffset);
      rotationTransform.transform(sideFarNodeOffset);

      frontNearNodeOffset.add(centerX, centerY);
      frontFarNodeOffset.add(centerX, centerY);
      sideNearNodeOffset.add(centerX, centerY);
      sideFarNodeOffset.add(centerX, centerY);

      FootstepNode frontNearNode = new FootstepNode(frontNearNodeOffset.getX(), frontNearNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));
      FootstepNode frontFarNode = new FootstepNode(frontFarNodeOffset.getX(), frontFarNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));
      FootstepNode sideNearNode = new FootstepNode(sideNearNodeOffset.getX(), sideNearNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));
      FootstepNode sideFarNode = new FootstepNode(sideFarNodeOffset.getX(), sideFarNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));

      assertFalse(cliffAvoider.isNodeValid(frontNearNode, null));
      assertFalse(cliffAvoider.isNodeValid(sideNearNode, null));
      assertTrue(cliffAvoider.isNodeValid(frontFarNode, null));
      assertTrue(cliffAvoider.isNodeValid(sideFarNode, null));
   }

      public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionBaseOfCliffAvoider.class, PlanarRegionBaseOfCliffAvoiderTest.class);
   }
}
