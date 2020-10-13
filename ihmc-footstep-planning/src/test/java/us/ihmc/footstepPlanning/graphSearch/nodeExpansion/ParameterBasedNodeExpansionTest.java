package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.*;
import java.util.function.ToDoubleFunction;
import java.util.function.UnaryOperator;

import static us.ihmc.robotics.Assert.assertTrue;

public class ParameterBasedNodeExpansionTest
{
   private static final double epsilon = 1e-6;

   @Test
   public void testExpansionAlongBoundsFromOriginDefaultParametersWithRight()
   {
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, null, PlannerTools.createDefaultFootPolygons());
      expansion.initialize();

      double maxYaw = parameters.getMaximumStepYaw();
      double minYaw = parameters.getMinimumStepYaw();
      double yawReduction = parameters.getStepYawReductionFactorAtMaxReach();

      double maxYawAtFullLength = (1.0 - yawReduction) * maxYaw;
      double minYawAtFullLength = (1.0 - yawReduction) * minYaw;

      List<FootstepNode> childNodes = new ArrayList<>();
      expansion.doFullExpansion(new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT), childNodes);
      FootstepNode mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      FootstepNode furthestReach = getExtremumNode(childNodes, Comparator.comparingDouble(node -> getReachAtNode(node, parameters.getIdealFootstepWidth())));
      FootstepNode mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      FootstepNode mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));
      FootstepNode mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));
      FootstepNode mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      FootstepNode mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

      assertTrue(mostForward.getX() < parameters.getMaximumStepReach() + epsilon);
      assertTrue(mostBackward.getX() > parameters.getMinimumStepLength() - epsilon);
      assertTrue(mostInward.getY() < -parameters.getMinimumStepWidth() + epsilon);
      assertTrue(mostOutward.getY() > -parameters.getMaximumStepWidth() - epsilon);

      double mostOutwardYawedReach = getReachAtNode(mostOutwardYawed, parameters.getIdealFootstepWidth());
      double mostInwardYawedReach = getReachAtNode(mostOutwardYawed, parameters.getIdealFootstepWidth());
      double mostOutwardYawMax = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, mostOutwardYawedReach / parameters.getMaximumStepReach());
      double mostInwardYawMin = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, mostInwardYawedReach / parameters.getMaximumStepReach());
      double minOutwardYaw = snapToYawGrid(-mostOutwardYawMax - epsilon);
      double maxInwardYaw = snapToYawGrid(-mostInwardYawMin + epsilon);
      assertTrue(mostOutwardYawed.getYaw() > minOutwardYaw - epsilon);
      assertTrue(mostInwardYawed.getYaw() < maxInwardYaw + epsilon);
      assertTrue(getReachAtNode(furthestReach, parameters.getIdealFootstepWidth()) < parameters.getMaximumStepReach());
   }

   @Test
   public void testExpansionAlongBoundsFromOriginDefaultParametersWithLeft()
   {
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, null, PlannerTools.createDefaultFootPolygons());
      expansion.initialize();

      double maxYaw = parameters.getMaximumStepYaw();
      double minYaw = parameters.getMinimumStepYaw();
      double yawReduction = parameters.getStepYawReductionFactorAtMaxReach();

      double maxYawAtFullLength = (1.0 - yawReduction) * maxYaw;
      double minYawAtFullLength = (1.0 - yawReduction) * minYaw;

      List<FootstepNode> childNodes = new ArrayList<>();
      expansion.doFullExpansion(new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT), childNodes);
      FootstepNode mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      FootstepNode furthestReach = getExtremumNode(childNodes, Comparator.comparingDouble(node -> getReachAtNode(node, parameters.getIdealFootstepWidth())));
      FootstepNode mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      FootstepNode mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));
      FootstepNode mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));
      FootstepNode mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));
      FootstepNode mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));

      assertTrue(mostForward.getX() < parameters.getMaximumStepReach() + epsilon);
      assertTrue(mostBackward.getX() > parameters.getMinimumStepLength() - epsilon);
      assertTrue(mostInward.getY() > parameters.getMinimumStepWidth() - epsilon);
      assertTrue(mostOutward.getY() < parameters.getMaximumStepWidth() + epsilon);

      double mostOutwardYawedReach = getReachAtNode(mostOutwardYawed, parameters.getIdealFootstepWidth());
      double mostInwardYawedReach = getReachAtNode(mostInwardYawed, parameters.getIdealFootstepWidth());
      double mostOutwardYawMax = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, mostOutwardYawedReach / parameters.getMaximumStepReach());
      double mostInwardYawMin = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, mostInwardYawedReach / parameters.getMaximumStepReach());
      double maxOutwardYaw = snapToYawGrid(mostOutwardYawMax + epsilon);
      double maxInwardYaw = snapToYawGrid(mostInwardYawMin + epsilon);
      assertTrue(mostOutwardYawed.getYaw() < maxOutwardYaw + epsilon);
      assertTrue(mostInwardYawed.getYaw() > maxInwardYaw - epsilon);
      assertTrue(getReachAtNode(furthestReach, parameters.getIdealFootstepWidth()) < parameters.getMaximumStepReach());
   }

   @Test
   public void testExpansionAlongBoundsFromOrigin()
   {
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, null, PlannerTools.createDefaultFootPolygons());
      expansion.initialize();

      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = (1.0 - yawReduction) * maxYaw;
      double minYawAtFullLength = (1.0 - yawReduction) * minYaw;


      List<FootstepNode> childNodes = new ArrayList<>();
      expansion.doFullExpansion(new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT), childNodes);
      FootstepNode mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      FootstepNode furthestReach = getExtremumNode(childNodes, Comparator.comparingDouble(node -> getReachAtNode(node, parameters.getIdealFootstepWidth())));
      FootstepNode mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      FootstepNode mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));
      FootstepNode mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));
      FootstepNode mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      FootstepNode mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

      assertTrue(mostForward.getX() < parameters.getMaximumStepReach() + epsilon);
      assertTrue(mostBackward.getX() > parameters.getMinimumStepLength() - epsilon);
      assertTrue(mostInward.getY() < -parameters.getMinimumStepWidth() + epsilon);
      assertTrue(mostOutward.getY() > -parameters.getMaximumStepWidth() - epsilon);

      double mostOutwardYawedReach = getReachAtNode(mostOutwardYawed, parameters.getIdealFootstepWidth());
      double mostInwardYawedReach = getReachAtNode(mostOutwardYawed, parameters.getIdealFootstepWidth());
      double mostOutwardYawMax = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, mostOutwardYawedReach / parameters.getMaximumStepReach());
      double mostInwardYawMin = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, mostInwardYawedReach / parameters.getMaximumStepReach());
      double minOutwardYaw = snapToYawGrid(-mostOutwardYawMax - epsilon);
      double maxInwardYaw = snapToYawGrid(-mostInwardYawMin + epsilon);
      assertTrue(mostOutwardYawed.getYaw() > minOutwardYaw);
      assertTrue(mostInwardYawed.getYaw() < maxInwardYaw + epsilon);
      assertTrue(getReachAtNode(furthestReach, parameters.getIdealFootstepWidth()) < parameters.getMaximumStepReach());
   }

   private static double getReachAtNode(FootstepNode node, double idealWidth)
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
      return LatticeNode.gridSizeYaw * Math.floorMod((int) (Math.round((yaw) / LatticeNode.gridSizeYaw)), LatticeNode.yawDivisions);
   }

   private FootstepNode getExtremumNode(Collection<FootstepNode> nodes, Comparator<FootstepNode> comparator)
   {
      FootstepNode extremumNode = null;
      for (FootstepNode node : nodes)
      {
         if (extremumNode == null)
            extremumNode = node;
         else if (comparator.compare(node, extremumNode) == 1)
            extremumNode = node;
      }

      return extremumNode;
   }

   @Test
   public void testPartialExpansionSize()
   {
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      int branchFactor = 100;
      parameters.setMaximumBranchFactor(branchFactor);

      UnaryOperator<FootstepNode> idealStepSupplier = step -> new FootstepNode(step.getX(), step.getY(), step.getYaw(), step.getRobotSide().getOppositeSide());
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, idealStepSupplier, PlannerTools.createDefaultFootPolygons());

      expansion.initialize();

      List<FootstepNode> expansionList = new ArrayList<>();
      FootstepNode stanceNode = new FootstepNode(0, 0, 0, RobotSide.LEFT);
      expansion.doFullExpansion(stanceNode, expansionList);
      int fullExpansionSize = expansionList.size();

      int numberOfIterativeExpansions = fullExpansionSize / branchFactor + 1;
      for (int i = 0; i < numberOfIterativeExpansions - 1; i++)
      {
         boolean containsMoreNodes = expansion.doIterativeExpansion(stanceNode, expansionList);
         Assertions.assertTrue(containsMoreNodes);
         Assertions.assertEquals(expansionList.size(), branchFactor);
      }

      boolean containsMoreNodes = expansion.doIterativeExpansion(stanceNode, expansionList);
      Assertions.assertFalse(containsMoreNodes);
      Assertions.assertEquals(expansionList.size(), fullExpansionSize % branchFactor);

      containsMoreNodes = expansion.doIterativeExpansion(stanceNode, expansionList);
      Assertions.assertFalse(containsMoreNodes);
      Assertions.assertTrue(expansionList.isEmpty());
   }

   @Test
   public void testFullExpansionReturnsSortedOrder()
   {
      Random random = new Random(329032);
      int numberOfStanceNodes = 5;
      int numberOfIdealSteps = 5;
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();

      int branchFactor = 100;
      parameters.setMaximumBranchFactor(branchFactor);

      for (int i = 0; i < numberOfStanceNodes; i++)
      {
         FootstepNode stanceNode = FootstepNode.generateRandomFootstepNode(random, 5.0);
         for (int j = 0; j < numberOfIdealSteps; j++)
         {
            FootstepNode randomIdealStep = FootstepNode.generateRandomFootstepNode(random, 5.0, stanceNode.getRobotSide().getOppositeSide());
            UnaryOperator<FootstepNode> idealStepSupplier = step -> randomIdealStep;
            ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, idealStepSupplier, PlannerTools.createDefaultFootPolygons());
            expansion.initialize();

            List<FootstepNode> fullExpansion = new ArrayList<>();
            expansion.doFullExpansion(stanceNode, fullExpansion);
            List<FootstepNode> fullExpansionSorted = new ArrayList<>(fullExpansion);

            ToDoubleFunction<FootstepNode> stepDistance = step -> ParameterBasedNodeExpansion.IdealStepProximityComparator.calculateStepProximity(step, randomIdealStep);
            Comparator<FootstepNode> sorter = Comparator.comparingDouble(stepDistance);
            fullExpansionSorted.sort(sorter);

            for (int k = 0; k < fullExpansion.size(); k++)
            {
               Assertions.assertTrue(fullExpansion.get(i).equalPosition(fullExpansionSorted.get(i)));
            }
         }
      }
   }

   @Test
   public void testIterativeExpansionReturnsSortedOrder()
   {
      Random random = new Random(329032);
      int numberOfStanceNodes = 5;
      int numberOfIdealSteps = 5;
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();

      int branchFactor = 100;
      parameters.setMaximumBranchFactor(branchFactor);

      for (int i = 0; i < numberOfStanceNodes; i++)
      {
         FootstepNode stanceNode = FootstepNode.generateRandomFootstepNode(random, 5.0);
         for (int j = 0; j < numberOfIdealSteps; j++)
         {
            FootstepNode randomIdealStep = FootstepNode.generateRandomFootstepNode(random, 5.0, stanceNode.getRobotSide().getOppositeSide());
            UnaryOperator<FootstepNode> idealStepSupplier = step -> randomIdealStep;
            ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, idealStepSupplier, PlannerTools.createDefaultFootPolygons());
            expansion.initialize();

            List<FootstepNode> fullExpansion = new ArrayList<>();
            expansion.doFullExpansion(stanceNode, fullExpansion);

            int numberOfIterativeExpansions = fullExpansion.size() / branchFactor + 1;
            for (int k = 0; k < numberOfIterativeExpansions; k++)
            {
               List<FootstepNode> iterativeExpansion = new ArrayList<>();
               expansion.doIterativeExpansion(stanceNode, iterativeExpansion);

               for (int l = 0; l < iterativeExpansion.size(); l++)
               {
                  FootstepNode stepFromFullExpansion = fullExpansion.get(branchFactor * k + l);
                  FootstepNode stepFromIterativeExpansion = iterativeExpansion.get(l);
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
      parameters.setMinimumStepWidth(0.0);
      parameters.setMinimumStepLength(-0.2);
      parameters.setEnableExpansionMask(false);
      parameters.setMinClearanceFromStance(clearance);
      parameters.setMaximumBranchFactor(Integer.MAX_VALUE);

      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters, null, footPolygons);
      expansion.initialize();

      List<FootstepNode> expansionList = new ArrayList<>();
      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
      ConvexPolygon2D intersectionPolygon = new ConvexPolygon2D();
      Point2D pointA = new Point2D();
      Point2D pointB = new Point2D();

      FootstepNode stanceNode = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      expansion.doFullExpansion(stanceNode, expansionList);

      ConvexPolygon2D stanceNodePolygon = new ConvexPolygon2D();
      FootstepNodeTools.getFootPolygon(stanceNode, footPolygons.get(stanceNode.getRobotSide()), stanceNodePolygon);

      for (int i = 0; i < expansionList.size(); i++)
      {
         FootstepNode childNode = expansionList.get(i);
         ConvexPolygon2D childNodePolygon = new ConvexPolygon2D();
         FootstepNodeTools.getFootPolygon(childNode, footPolygons.get(childNode.getRobotSide()), childNodePolygon);

         boolean intersectionDetected = convexPolygonTools.computeIntersectionOfPolygons(stanceNodePolygon, childNodePolygon, intersectionPolygon);
         Assertions.assertFalse(intersectionDetected, "Intersection detected in footstep node expansion");

         convexPolygonTools.computeMinimumDistancePoints(stanceNodePolygon, childNodePolygon, 1e-3, pointA, pointB);
         double distance = pointA.distance(pointB);
         Assertions.assertTrue(distance >= clearance, "Intersection detected in footstep node expansion");
      }
   }
}
