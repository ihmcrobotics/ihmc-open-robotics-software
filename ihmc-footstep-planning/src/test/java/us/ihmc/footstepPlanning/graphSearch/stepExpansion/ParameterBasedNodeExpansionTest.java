package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;

import java.util.*;
import java.util.function.ToDoubleFunction;

import static org.junit.jupiter.api.Assertions.*;

public class ParameterBasedNodeExpansionTest
{
   private static final double epsilon = 1e-6;
   private DefaultFootstepPlannerParameters parameters;

   @BeforeEach
   public void setupParameters()
   {
      // We create default parameters for the tests
      parameters = new DefaultFootstepPlannerParameters();
   }

   @Test
   public void testExpansionAlongBoundsFromOriginWithRight()
   {
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, null, PlannerTools.createDefaultFootPolygons());
      expansion.initialize();

      // Set up the feet to test moving a right foot
      DiscreteFootstep stanceStep = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep startOfSwingStep = new DiscreteFootstep(0.0, 0.3, 0.0, RobotSide.RIGHT);

      // Do full expansion of the steps
      List<FootstepGraphNode> childNodes = new ArrayList<>();
      expansion.doFullExpansion(new FootstepGraphNode(startOfSwingStep, stanceStep), childNodes);

      // Check the edges of where steps can reach in the x and y directions
      DiscreteFootstep mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      DiscreteFootstep mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      DiscreteFootstep mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));
      DiscreteFootstep mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));

      assertTrue(mostForward.getX() <= parameters.getMaxStepReach());
      assertTrue(mostBackward.getX() >= parameters.getMinStepLength());
      assertTrue(mostInward.getY() <= -parameters.getMinStepWidth());
      assertTrue(mostOutward.getY() >= -parameters.getMaxStepWidth());

      // Check the min and max a step can yaw
      DiscreteFootstep mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      DiscreteFootstep mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

      // The yaw is a little tricky because we don't have any notion of things being negative.
      // So if the yaw is clockwise (which should be negative) it will be ~5.7 or something just less then 2 PI radians
      // To account for this, we need to subtract the yaw we got from 2 PI.
      double outwardYawFromZero = Math.PI * 2 - mostOutwardYawed.getYaw();
      assertTrue(outwardYawFromZero <= parameters.getMaxStepYaw());
      assertTrue(mostInwardYawed.getYaw() >= parameters.getMinStepYaw());

      // Get the footstep closest to the ideal step and ensure that it's less than the max reach
      DiscreteFootstep idealReach = getExtremumNode(childNodes, Comparator.comparingDouble(node -> getReachAtNode(node, parameters.getIdealFootstepWidth())));
      assertTrue(getReachAtNode(idealReach, parameters.getIdealFootstepWidth()) < parameters.getMaxStepReach());
   }

   @Test
   public void testExpansionAlongBoundsFromOriginWithLeft()
   {
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, null, PlannerTools.createDefaultFootPolygons());
      expansion.initialize();

      // Set up the feet to test moving a right foot
      DiscreteFootstep stanceStep = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT);
      DiscreteFootstep startOfSwingStep = new DiscreteFootstep(0.0, -0.3, 0.0, RobotSide.LEFT);

      // Do full expansion of the steps
      List<FootstepGraphNode> childNodes = new ArrayList<>();
      expansion.doFullExpansion(new FootstepGraphNode(startOfSwingStep, stanceStep), childNodes);

      // Check the edges of where steps can reach in the x and y directions
      DiscreteFootstep mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      DiscreteFootstep mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      DiscreteFootstep mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));
      DiscreteFootstep mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));

      assertTrue(mostForward.getX() <= parameters.getMaxStepReach());
      assertTrue(mostBackward.getX() >= parameters.getMinStepLength());
      assertTrue(mostInward.getY() >= parameters.getMinStepWidth());
      assertTrue(mostOutward.getY() <= parameters.getMaxStepWidth());

      // Check the min and max a step can yaw
      DiscreteFootstep mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      DiscreteFootstep mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

      // The yaw is a little tricky because we don't have any notion of things being negative.
      // So if the yaw is clockwise (which should be negative) it will be ~5.7 or something just less then 2 PI radians
      // To account for this, we need to subtract the yaw we got from 2 PI.
      double outwardYawFromZero = Math.PI * 2 - mostOutwardYawed.getYaw();
      assertTrue(outwardYawFromZero <= parameters.getMaxStepYaw());
      assertTrue(mostInwardYawed.getYaw() >= parameters.getMinStepYaw());

      // Get the footstep closest to the ideal step and ensure that it's less than the max reach
      DiscreteFootstep idealReach = getExtremumNode(childNodes, Comparator.comparingDouble(node -> getReachAtNode(node, parameters.getIdealFootstepWidth())));
      assertTrue(getReachAtNode(idealReach, parameters.getIdealFootstepWidth()) < parameters.getMaxStepReach());
   }

   @Test
   public void testExpansionAlongBoundsFromOrigin()
   {
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, null, PlannerTools.createDefaultFootPolygons());
      expansion.initialize();

      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaxStepYaw(maxYaw);
      parameters.setMinStepYaw(minYaw);

      double maxYawAtFullLength = (1.0 - yawReduction) * maxYaw;
      double minYawAtFullLength = (1.0 - yawReduction) * minYaw;

      DiscreteFootstep stanceStep = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep startOfSwingStep = new DiscreteFootstep(0.0, 0.3, 0.0, RobotSide.RIGHT);

      List<FootstepGraphNode> childNodes = new ArrayList<>();
      expansion.doFullExpansion(new FootstepGraphNode(startOfSwingStep, stanceStep), childNodes);
      DiscreteFootstep mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      DiscreteFootstep furthestReach = getExtremumNode(childNodes,
                                                       Comparator.comparingDouble(node -> getReachAtNode(node, parameters.getIdealFootstepWidth())));
      DiscreteFootstep mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      DiscreteFootstep mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));
      DiscreteFootstep mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));
      DiscreteFootstep mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      DiscreteFootstep mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

      assertTrue(mostForward.getX() < parameters.getMaxStepReach() + epsilon);
      assertTrue(mostBackward.getX() > parameters.getMinStepLength() - epsilon);
      assertTrue(mostInward.getY() < -parameters.getMinStepWidth() + epsilon);
      assertTrue(mostOutward.getY() > -parameters.getMaxStepWidth() - epsilon);

      double mostOutwardYawedReach = getReachAtNode(mostOutwardYawed, parameters.getIdealFootstepWidth());
      double mostInwardYawedReach = getReachAtNode(mostOutwardYawed, parameters.getIdealFootstepWidth());
      double mostOutwardYawMax = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, mostOutwardYawedReach / parameters.getMaxStepReach());
      double mostInwardYawMin = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, mostInwardYawedReach / parameters.getMaxStepReach());
      double minOutwardYaw = snapToYawGrid(-mostOutwardYawMax - epsilon);
      double maxInwardYaw = snapToYawGrid(-mostInwardYawMin + epsilon);
      assertTrue(mostOutwardYawed.getYaw() > minOutwardYaw);
      assertTrue(mostInwardYawed.getYaw() < maxInwardYaw + epsilon);
      assertTrue(getReachAtNode(furthestReach, parameters.getIdealFootstepWidth()) < parameters.getMaxStepReach());
   }

   private static double getReachAtNode(DiscreteFootstep node, double idealWidth)
   {
      double relativeYToIdeal = node.getY() - node.getRobotSide().negateIfRightSide(idealWidth);
      return EuclidCoreTools.normSquared(node.getX(), relativeYToIdeal);
   }

   private static double snapToCircle(double yaw)
   {
      if (yaw < Math.PI)
         return yaw;
      else
         return -(2.0 * Math.PI - yaw);
   }

   private static double snapToYawGrid(double yaw)
   {
      return LatticePoint.gridSizeYaw * Math.floorMod((int) (Math.round((yaw) / LatticePoint.gridSizeYaw)), LatticePoint.yawDivisions);
   }

   private DiscreteFootstep getExtremumNode(Collection<FootstepGraphNode> nodes, Comparator<DiscreteFootstep> comparator)
   {
      DiscreteFootstep extremumNode = null;
      for (FootstepGraphNode node : nodes)
      {
         if (extremumNode == null)
            extremumNode = node.getSecondStep();
         else if (comparator.compare(node.getSecondStep(), extremumNode) == 1)
            extremumNode = node.getSecondStep();
      }

      return extremumNode;
   }

   @Test
   public void testPartialExpansionSize()
   {
      int branchFactor = 100;
      parameters.setMaxBranchFactor(branchFactor);

      IdealStepCalculatorInterface idealStepCalculator = (stance, startOfSwing) -> new DiscreteFootstep(stance.getLatticePoint(),
                                                                                                        stance.getRobotSide().getOppositeSide());
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, idealStepCalculator, PlannerTools.createDefaultFootPolygons());

      expansion.initialize();

      List<FootstepGraphNode> expansionList = new ArrayList<>();
      FootstepGraphNode graphNode = new FootstepGraphNode(new DiscreteFootstep(0, -6, 0, RobotSide.RIGHT), new DiscreteFootstep(0, 0, 0, RobotSide.LEFT));
      expansion.doFullExpansion(graphNode, expansionList);
      int fullExpansionSize = expansionList.size();

      int numberOfIterativeExpansions = fullExpansionSize / branchFactor + 1;
      for (int i = 0; i < numberOfIterativeExpansions - 1; i++)
      {
         boolean containsMoreNodes = expansion.doIterativeExpansion(graphNode, expansionList);
         Assertions.assertTrue(containsMoreNodes);
         Assertions.assertEquals(expansionList.size(), branchFactor);
      }

      boolean containsMoreNodes = expansion.doIterativeExpansion(graphNode, expansionList);
      Assertions.assertFalse(containsMoreNodes);
      Assertions.assertEquals(expansionList.size(), fullExpansionSize % branchFactor);

      containsMoreNodes = expansion.doIterativeExpansion(graphNode, expansionList);
      Assertions.assertFalse(containsMoreNodes);
      Assertions.assertTrue(expansionList.isEmpty());
   }

   /**
    * This test is meant to check if the full expansion returns a sorted list or not.
    * We don't always sort the full expansion, so by default, this test is
    * disabled.
    */
   @Test
   @Disabled
   public void testFullExpansionReturnsSortedOrder()
   {
      Random random = new Random(329032);
      int numberOfGraphNodes = 5;
      int numberOfChildNodes = 5;
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();

      int branchFactor = 100;
      parameters.setMaxBranchFactor(branchFactor);

      for (int i = 0; i < numberOfGraphNodes; i++)
      {
         DiscreteFootstep stanceStep = DiscreteFootstep.generateRandomFootstep(random, 5.0);
         DiscreteFootstep startOfSwingStep = DiscreteFootstepTools.constructStepInPreviousStepFrame(0.0, 0.3, 0.0, stanceStep);
         FootstepGraphNode node = new FootstepGraphNode(startOfSwingStep, stanceStep);

         for (int j = 0; j < numberOfChildNodes; j++)
         {
            DiscreteFootstep idealStep = DiscreteFootstep.generateRandomFootstep(random, 5.0, stanceStep.getRobotSide().getOppositeSide());
            IdealStepCalculatorInterface idealStepCalculator = (stance, startOfSwing) -> idealStep;
            ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, idealStepCalculator, PlannerTools.createDefaultFootPolygons());
            expansion.initialize();

            List<FootstepGraphNode> fullExpansion = new ArrayList<>();
            expansion.doFullExpansion(node, fullExpansion);
            List<FootstepGraphNode> fullExpansionSorted = new ArrayList<>(fullExpansion);

            ToDoubleFunction<FootstepGraphNode> stepDistance = step -> ParameterBasedStepExpansion.IdealStepProximityComparator.calculateStepProximity(step.getSecondStep(),
                                                                                                                                                       idealStep);
            Comparator<FootstepGraphNode> sorter = Comparator.comparingDouble(stepDistance);
            fullExpansionSorted.sort(sorter);

            for (int k = 0; k < fullExpansion.size(); k++)
            {
               Assertions.assertTrue(fullExpansion.get(i).getSecondStep().equalPosition(fullExpansionSorted.get(i).getSecondStep()));
            }
         }
      }
   }

   @Test
   public void testIterativeExpansionReturnsSortedOrder()
   {
      Random random = new Random(329032);
      int numberOfGraphNodes = 5;
      int numberOfChildNodes = 5;
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();

      int branchFactor = 100;
      parameters.setMaxBranchFactor(branchFactor);

      for (int i = 0; i < numberOfGraphNodes; i++)
      {
         DiscreteFootstep stanceStep = DiscreteFootstep.generateRandomFootstep(random, 5.0);
         DiscreteFootstep startOfSwingStep = DiscreteFootstepTools.constructStepInPreviousStepFrame(0.0, 0.3, 0.0, stanceStep);
         FootstepGraphNode node = new FootstepGraphNode(startOfSwingStep, stanceStep);

         for (int j = 0; j < numberOfChildNodes; j++)
         {
            DiscreteFootstep idealStep = DiscreteFootstep.generateRandomFootstep(random, 5.0, stanceStep.getRobotSide().getOppositeSide());
            IdealStepCalculatorInterface idealStepCalculator = (stance, startOfSwing) -> idealStep;
            ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, idealStepCalculator, PlannerTools.createDefaultFootPolygons());
            expansion.initialize();

            List<FootstepGraphNode> fullExpansion = new ArrayList<>();
            expansion.doFullExpansion(node, fullExpansion);

            int numberOfIterativeExpansions = fullExpansion.size() / branchFactor + 1;
            for (int k = 0; k < numberOfIterativeExpansions; k++)
            {
               List<FootstepGraphNode> iterativeExpansion = new ArrayList<>();
               expansion.doIterativeExpansion(node, iterativeExpansion);

               for (int l = 0; l < iterativeExpansion.size(); l++)
               {
                  FootstepGraphNode stepFromFullExpansion = fullExpansion.get(branchFactor * k + l);
                  FootstepGraphNode stepFromIterativeExpansion = iterativeExpansion.get(l);
                  Assertions.assertEquals(stepFromFullExpansion, stepFromIterativeExpansion);
               }
            }
         }
      }
   }

   @Test
   public void testSelfIntersection()
   {
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      double clearance = 0.01;

      // set width so expansion will step on stance foot if not prevented
      parameters.setMinStepWidth(0.0);
      parameters.setMinStepLength(-0.2);
      parameters.setEnableExpansionMask(false);
      parameters.setMinClearanceFromStance(clearance);
      parameters.setMaxBranchFactor(Integer.MAX_VALUE);

      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, null, footPolygons);
      expansion.initialize();

      List<FootstepGraphNode> expansionList = new ArrayList<>();
      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
      ConvexPolygon2D intersectionPolygon = new ConvexPolygon2D();
      Point2D pointA = new Point2D();
      Point2D pointB = new Point2D();

      DiscreteFootstep stanceStep = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep startOfSwingStep = DiscreteFootstepTools.constructStepInPreviousStepFrame(0.0, 0.3, 0.0, stanceStep);
      FootstepGraphNode node = new FootstepGraphNode(startOfSwingStep, stanceStep);

      expansion.doFullExpansion(node, expansionList);

      ConvexPolygon2D stanceStepPolygon = new ConvexPolygon2D();
      DiscreteFootstepTools.getFootPolygon(stanceStep, footPolygons.get(stanceStep.getRobotSide()), stanceStepPolygon);

      for (int i = 0; i < expansionList.size(); i++)
      {
         FootstepGraphNode childNode = expansionList.get(i);
         ConvexPolygon2D childNodePolygon = new ConvexPolygon2D();
         DiscreteFootstepTools.getFootPolygon(childNode.getSecondStep(), footPolygons.get(childNode.getSecondStepSide()), childNodePolygon);

         boolean intersectionDetected = convexPolygonTools.computeIntersectionOfPolygons(stanceStepPolygon, childNodePolygon, intersectionPolygon);
         Assertions.assertFalse(intersectionDetected, "Intersection detected in footstep node expansion");

         convexPolygonTools.computeMinimumDistancePoints(stanceStepPolygon, childNodePolygon, 1e-3, pointA, pointB);
         double distance = pointA.distance(pointB);
         Assertions.assertTrue(distance >= clearance, "Intersection detected in footstep node expansion");
      }
   }
}
