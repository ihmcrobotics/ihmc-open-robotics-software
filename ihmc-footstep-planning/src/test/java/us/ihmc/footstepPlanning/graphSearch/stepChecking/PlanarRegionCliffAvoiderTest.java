package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.YoVariablesForFootstepPlannerParameters;
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
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class PlanarRegionCliffAvoiderTest
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

      YoRegistry registry = new YoRegistry("Test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double epsilon = 1e-6;
      double minimumDistanceFromCliffBottom = 0.2 - epsilon;
      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();
      new YoVariablesForFootstepPlannerParameters(registry, parameters);
      parameters.setCliffBaseHeightToAvoid(0.01);
      parameters.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottom);

      double footLength = 0.2;
      double footWidth = 0.1;
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createFootPolygons(footLength, footWidth);
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      PlanarRegionCliffAvoider avoider = new PlanarRegionCliffAvoider(parameters, snapper, footPolygons);
      avoider.setPlanarRegionsList(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("TestRobot"));
         scs.addYoRegistry(registry);
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
      DiscreteFootstep node = new DiscreteFootstep(x, y, 0.0, footstepSide);
      snapper.snapFootstep(node);
      assertTrue(avoider.isStepValid(node));

      x = closestNodeDistanceToCliff + LatticePoint.gridSizeXY;
      node = new DiscreteFootstep(x, y, 0.0, footstepSide);
      snapper.snapFootstep(node);
      assertFalse(avoider.isStepValid(node));

      x = closestNodeDistanceToCliff - LatticePoint.gridSizeXY;
      node = new DiscreteFootstep(x, y, 0.0, footstepSide);
      snapper.snapFootstep(node);
      assertTrue(avoider.isStepValid(node));
   }

   @Test
   public void testAvoidingRotatedAndElevatedCliff()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      double cliffHeightToAvoid = 0.2;
      double minimumDistanceFromCliffBottom = 0.1;
      double footLength = 0.2;
      double footWidth = 0.1;

      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();
      new YoVariablesForFootstepPlannerParameters(registry, parameters);
      parameters.setCliffBaseHeightToAvoid(cliffHeightToAvoid);
      parameters.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottom);

      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createFootPolygons(footLength, footWidth);
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      PlanarRegionCliffAvoider cliffAvoider = new PlanarRegionCliffAvoider(parameters, snapper, footPolygons);

      double centerX = 1.0;
      double centerY = 1.0;
      double groundHeight = 0.5;
      double cliffHeight = cliffHeightToAvoid + 0.1;
      double rotation = 0.25 * Math.PI;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(5.0, 5.0);
      generator.translate(centerX, centerY, groundHeight + cliffHeight);
      generator.rotate(rotation, Axis3D.Z);
      double boxWidth = 0.5;
      generator.addRectangle(boxWidth, boxWidth);

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();
      cliffAvoider.setPlanarRegionsList(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);

      Vector2D frontNearNodeOffset = new Vector2D(0.5 * boxWidth + minimumDistanceFromCliffBottom + 0.5 * footLength - LatticePoint.gridSizeXY, 0.0);
      Vector2D frontFarNodeOffset = new Vector2D(0.5 * boxWidth + minimumDistanceFromCliffBottom + 0.5 * footLength + LatticePoint.gridSizeXY, 0.0);
      Vector2D sideNearNodeOffset = new Vector2D(0.0, 0.5 * boxWidth + 0.5 * footWidth + minimumDistanceFromCliffBottom - LatticePoint.gridSizeXY);
      Vector2D sideFarNodeOffset = new Vector2D(0.0, 0.5 * boxWidth + 0.5 * footWidth + minimumDistanceFromCliffBottom + LatticePoint.gridSizeXY);

      AxisAngle rotationTransform = new AxisAngle(rotation, 0.0, 0.0);
      rotationTransform.transform(frontNearNodeOffset);
      rotationTransform.transform(frontFarNodeOffset);
      rotationTransform.transform(sideNearNodeOffset);
      rotationTransform.transform(sideFarNodeOffset);

      frontNearNodeOffset.add(centerX, centerY);
      frontFarNodeOffset.add(centerX, centerY);
      sideNearNodeOffset.add(centerX, centerY);
      sideFarNodeOffset.add(centerX, centerY);

      DiscreteFootstep frontNearNode = new DiscreteFootstep(frontNearNodeOffset.getX(), frontNearNodeOffset.getY(), rotation,
                                                            RobotSide.generateRandomRobotSide(random));
      DiscreteFootstep frontFarNode = new DiscreteFootstep(frontFarNodeOffset.getX(), frontFarNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));
      DiscreteFootstep sideNearNode = new DiscreteFootstep(sideNearNodeOffset.getX(), sideNearNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));
      DiscreteFootstep sideFarNode = new DiscreteFootstep(sideFarNodeOffset.getX(), sideFarNodeOffset.getY(), rotation, RobotSide.generateRandomRobotSide(random));

      snapper.snapFootstep(frontNearNode);
      snapper.snapFootstep(sideNearNode);
      snapper.snapFootstep(frontFarNode);
      snapper.snapFootstep(sideFarNode);

      assertFalse(cliffAvoider.isStepValid(frontNearNode));
      assertFalse(cliffAvoider.isStepValid(sideNearNode));
      assertTrue(cliffAvoider.isStepValid(frontFarNode));
      assertTrue(cliffAvoider.isStepValid(sideFarNode));
   }

   @Test
   public void testTopOfCliffAvoiderWithSimpleQueriesOnABlock()
   {
      double stepHeight = 0.2;
      double boxSize = 1.0;
      double edgeOfBoxX = 1.0;

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(2.0 * edgeOfBoxX, 5.0);
      generator.translate(edgeOfBoxX + boxSize / 2.0, 0.0, -2.0 * stepHeight);
      generator.addCubeReferencedAtBottomMiddle(boxSize, boxSize, stepHeight);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      YoRegistry registry = new YoRegistry("Test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double epsilon = 1e-6;
      double minimumDistanceFromCliffTop = 0.2 - epsilon;
      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();
      new YoVariablesForFootstepPlannerParameters(registry, parameters);
      parameters.setCliffTopHeightToAvoid(0.01);
      parameters.setMinimumDistanceFromCliffTops(minimumDistanceFromCliffTop);

      double footLength = 0.2;
      double footWidth = 0.1;
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createFootPolygons(footLength, footWidth);
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      PlanarRegionCliffAvoider avoider = new PlanarRegionCliffAvoider(parameters, snapper, footPolygons);
      avoider.setPlanarRegionsList(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("TestRobot"));
         scs.addYoRegistry(registry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         staticLinkGraphics.addCoordinateSystem(1.0);
         Graphics3DObjectTools.addPlanarRegionsList(staticLinkGraphics, planarRegionsList, YoAppearance.Green(), YoAppearance.Beige(), YoAppearance.Yellow(),
                                                    YoAppearance.Orange());
         scs.addStaticLinkGraphics(staticLinkGraphics);
         scs.startOnAThread();
         scs.setGroundVisible(false);

         ThreadTools.sleepForever();
      }

      double closestNodeDistanceToCliff = edgeOfBoxX - 0.5 * footLength - minimumDistanceFromCliffTop;

      RobotSide footstepSide = RobotSide.LEFT;
      double x = closestNodeDistanceToCliff;
      double y = 0.0;
      DiscreteFootstep node = new DiscreteFootstep(x, y, 0.0, footstepSide);
      snapper.snapFootstep(node);
      assertTrue(avoider.isStepValid(node));

      x = closestNodeDistanceToCliff + LatticePoint.gridSizeXY;
      node = new DiscreteFootstep(x, y, 0.0, footstepSide);
      snapper.snapFootstep(node);
      assertFalse(avoider.isStepValid(node));

      x = closestNodeDistanceToCliff - LatticePoint.gridSizeXY;
      node = new DiscreteFootstep(x, y, 0.0, footstepSide);
      snapper.snapFootstep(node);
      assertTrue(avoider.isStepValid(node));
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionCliffAvoider.class, PlanarRegionCliffAvoiderTest.class);
   }
}
