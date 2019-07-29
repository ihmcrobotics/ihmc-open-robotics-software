package us.ihmc.footstepPlanning.graphSearch;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.PlanarRegionBaseOfCliffAvoider;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.YoFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class PlanarRegionBaseOfCliffAvoiderTest
{
   private boolean visualize = true;
   private final Random random = new Random(4587L);

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @Test
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
      YoFootstepPlannerParameters parameters = new YoFootstepPlannerParameters(registry, new FootstepPlanningParameters()
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
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createFootPolygons(footLength, footWidth);
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      PlanarRegionBaseOfCliffAvoider avoider = new PlanarRegionBaseOfCliffAvoider(parameters, snapper, footPolygons);
      avoider.setPlanarRegions(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("TestRobot"));
         scs.addYoVariableRegistry(registry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         staticLinkGraphics.addCoordinateSystem(1.0);
         Graphics3DObjectTools.addPlanarRegionsList(staticLinkGraphics, planarRegionsList, YoAppearance.Green(), YoAppearance.Beige(), YoAppearance.Yellow(),
                                                    YoAppearance.Orange());
         scs.addStaticLinkGraphics(staticLinkGraphics);
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }

      double closestNodeDistanceToCliff = edgeOfBoxX - 0.5 * footLength - minimumDistanceFromCliffBottom;

      RobotSide footstepSide = RobotSide.LEFT;
      double x = closestNodeDistanceToCliff;
      double y = 0.0;
      FootstepNode node = new FootstepNode(x, y, 0.0, footstepSide);
      snapper.snapFootstepNode(node);
      assertTrue(avoider.isNodeValid(node, null));

      x = closestNodeDistanceToCliff + LatticeNode.gridSizeXY;
      node = new FootstepNode(x, y, 0.0, footstepSide);
      snapper.snapFootstepNode(node);
      assertFalse(avoider.isNodeValid(node, null));

      x = closestNodeDistanceToCliff - LatticeNode.gridSizeXY;
      node = new FootstepNode(x, y, 0.0, footstepSide);
      snapper.snapFootstepNode(node);
      assertTrue(avoider.isNodeValid(node, null));
   }

   @Test
   public void testAvoidingRotatedAndElevatedCliff()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      double cliffHeightToAvoid = 0.2;
      double minimumDistanceFromCliffBottom = 0.1;
      double footLength = 0.2;
      double footWidth = 0.1;

      YoFootstepPlannerParameters parameters = new YoFootstepPlannerParameters(registry, new FootstepPlanningParameters()
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

      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createFootPolygons(footLength, footWidth);
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(parameters, snapper, footPolygons);

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
      snapper.setPlanarRegions(planarRegionsList);

      Vector2D frontNearNodeOffset = new Vector2D(0.5 * boxWidth + minimumDistanceFromCliffBottom + 0.5 * footLength - LatticeNode.gridSizeXY, 0.0);
      Vector2D frontFarNodeOffset = new Vector2D(0.5 * boxWidth + minimumDistanceFromCliffBottom + 0.5 * footLength + LatticeNode.gridSizeXY, 0.0);
      Vector2D sideNearNodeOffset = new Vector2D(0.0, 0.5 * boxWidth + 0.5 * footWidth + minimumDistanceFromCliffBottom - LatticeNode.gridSizeXY);
      Vector2D sideFarNodeOffset = new Vector2D(0.0, 0.5 * boxWidth + 0.5 * footWidth + minimumDistanceFromCliffBottom + LatticeNode.gridSizeXY);

      AxisAngle rotationTransform = new AxisAngle(rotation, 0.0, 0.0);
      rotationTransform.transform(frontNearNodeOffset);
      rotationTransform.transform(frontFarNodeOffset);
      rotationTransform.transform(sideNearNodeOffset);
      rotationTransform.transform(sideFarNodeOffset);

      frontNearNodeOffset.add(centerX, centerY);
      frontFarNodeOffset.add(centerX, centerY);
      sideNearNodeOffset.add(centerX, centerY);
      sideFarNodeOffset.add(centerX, centerY);

      FootstepNode frontNearNode = new FootstepNode(frontNearNodeOffset.getX(), frontNearNodeOffset.getY(), rotation,
                                                    RobotSide.generateRandomRobotSide(random));
      FootstepNode frontFarNode = new FootstepNode(frontFarNodeOffset.getX(), frontFarNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));
      FootstepNode sideNearNode = new FootstepNode(sideNearNodeOffset.getX(), sideNearNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));
      FootstepNode sideFarNode = new FootstepNode(sideFarNodeOffset.getX(), sideFarNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));

      snapper.snapFootstepNode(frontNearNode);
      snapper.snapFootstepNode(sideNearNode);
      snapper.snapFootstepNode(frontFarNode);
      snapper.snapFootstepNode(sideFarNode);

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
