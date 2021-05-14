package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import org.junit.jupiter.api.Assertions;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.*;
import java.util.function.ToDoubleFunction;

import static us.ihmc.robotics.Assert.assertTrue;

public class ParameterBasedNodeExpansionTest
{
   private static final double epsilon = 1e-6;

   @Test
   public void testExpansionAlongBoundsFromOriginDefaultParametersWithRight()
   {
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, null, PlannerTools.createDefaultFootPolygons());
      expansion.initialize();

      double maxYaw = parameters.getMaximumStepYaw();
      double minYaw = parameters.getMinimumStepYaw();
      double yawReduction = parameters.getStepYawReductionFactorAtMaxReach();

      double maxYawAtFullLength = (1.0 - yawReduction) * maxYaw;
      double minYawAtFullLength = (1.0 - yawReduction) * minYaw;

      List<FootstepGraphNode> childNodes = new ArrayList<>();

      DiscreteFootstep stanceStep = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep startOfSwingStep = new DiscreteFootstep(0.0, 0.3, 0.0, RobotSide.RIGHT);

      expansion.doFullExpansion(new FootstepGraphNode(startOfSwingStep, stanceStep), childNodes);
      DiscreteFootstep mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      DiscreteFootstep furthestReach = getExtremumNode(childNodes, Comparator.comparingDouble(node -> getReachAtNode(node, parameters.getIdealFootstepWidth())));
      DiscreteFootstep mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      DiscreteFootstep mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));
      DiscreteFootstep mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));
      DiscreteFootstep mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      DiscreteFootstep mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

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
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, null, PlannerTools.createDefaultFootPolygons());
      expansion.initialize();

      double maxYaw = parameters.getMaximumStepYaw();
      double minYaw = parameters.getMinimumStepYaw();
      double yawReduction = parameters.getStepYawReductionFactorAtMaxReach();

      double maxYawAtFullLength = (1.0 - yawReduction) * maxYaw;
      double minYawAtFullLength = (1.0 - yawReduction) * minYaw;

      DiscreteFootstep stanceStep = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT);
      DiscreteFootstep startOfSwingStep = new DiscreteFootstep(0.0, -0.3, 0.0, RobotSide.LEFT);

      List<FootstepGraphNode> childNodes = new ArrayList<>();
      expansion.doFullExpansion(new FootstepGraphNode(startOfSwingStep, stanceStep), childNodes);
      DiscreteFootstep mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      DiscreteFootstep furthestReach = getExtremumNode(childNodes, Comparator.comparingDouble(node -> getReachAtNode(node, parameters.getIdealFootstepWidth())));
      DiscreteFootstep mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      DiscreteFootstep mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));
      DiscreteFootstep mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));
      DiscreteFootstep mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));
      DiscreteFootstep mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));

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
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(parameters, null, PlannerTools.createDefaultFootPolygons());
      expansion.initialize();

      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = (1.0 - yawReduction) * maxYaw;
      double minYawAtFullLength = (1.0 - yawReduction) * minYaw;

      DiscreteFootstep stanceStep = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep startOfSwingStep = new DiscreteFootstep(0.0, 0.3, 0.0, RobotSide.RIGHT);

      List<FootstepGraphNode> childNodes = new ArrayList<>();
      expansion.doFullExpansion(new FootstepGraphNode(startOfSwingStep, stanceStep), childNodes);
      DiscreteFootstep mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      DiscreteFootstep furthestReach = getExtremumNode(childNodes, Comparator.comparingDouble(node -> getReachAtNode(node, parameters.getIdealFootstepWidth())));
      DiscreteFootstep mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      DiscreteFootstep mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));
      DiscreteFootstep mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));
      DiscreteFootstep mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      DiscreteFootstep mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

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
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      int branchFactor = 100;
      parameters.setMaximumBranchFactor(branchFactor);

      IdealStepCalculatorInterface idealStepCalculator = (stance, startOfSwing) -> new DiscreteFootstep(stance.getLatticePoint(), stance.getRobotSide().getOppositeSide());
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

   @Test
   public void testFullExpansionReturnsSortedOrder()
   {
      Random random = new Random(329032);
      int numberOfGraphNodes = 5;
      int numberOfChildNodes = 5;
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();

      int branchFactor = 100;
      parameters.setMaximumBranchFactor(branchFactor);

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

            ToDoubleFunction<FootstepGraphNode> stepDistance = step -> ParameterBasedStepExpansion.IdealStepProximityComparator.calculateStepProximity(step.getSecondStep(), idealStep);
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
      parameters.setMaximumBranchFactor(branchFactor);

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
      parameters.setMinimumStepWidth(0.0);
      parameters.setMinimumStepLength(-0.2);
      parameters.setEnableExpansionMask(false);
      parameters.setMinClearanceFromStance(clearance);
      parameters.setMaximumBranchFactor(Integer.MAX_VALUE);

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
