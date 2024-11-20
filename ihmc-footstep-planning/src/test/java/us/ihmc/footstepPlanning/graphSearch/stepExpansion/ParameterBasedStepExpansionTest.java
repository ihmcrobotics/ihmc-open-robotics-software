package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import gnu.trove.list.array.TDoubleArrayList;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerHeuristicCalculator;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepCheckerInterface;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostCalculator;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostCalculatorInterface;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.*;
import java.util.function.Function;
import java.util.function.ToDoubleFunction;

import static org.junit.jupiter.api.Assertions.*;

public class ParameterBasedStepExpansionTest
{
   private static final double epsilon = 1e-6;
   private DefaultFootstepPlannerParametersBasics footstepPlannerParameters;

   private FootstepSnapAndWiggler snapper;
   private FootstepCostCalculator stepCostCalculator;
   private FootstepPlannerHeuristicCalculator distanceAndYawHeuristics;

   private final SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();
   private final FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();

   private final YoRegistry registry = new YoRegistry("ParameterBasedStepExpansionTest");

   @BeforeEach
   public void setupParameters()
   {
      // We create default parameters for the tests
      footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      snapper = new FootstepSnapAndWiggler(defaultFootPolygons, footstepPlannerParameters, environmentHandler);

      distanceAndYawHeuristics = new FootstepPlannerHeuristicCalculator(footstepPlannerParameters, bodyPathPlanHolder, registry);

      // NOTE: This may affect new tests that are added, be careful when setting up future tests
      FootstepCheckerInterface footstepChecker = (candidateStep, stanceStep, startOfSwing) -> true;

      stepCostCalculator = new FootstepCostCalculator(footstepPlannerParameters,
                                                      snapper,
                                                      distanceAndYawHeuristics::compute,
                                                      registry,
                                                      footstepChecker,
                                                      bodyPathPlanHolder,
                                                      environmentHandler);

      // We set up these foot poses to get passed into the bodyPathPlanHolder; however, the value of these doesn't affect the test
      Pose3D startMidFootPose = new Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      Pose3D goalMidFootPose = new Pose3D(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      bodyPathPlanHolder.setPoseWaypoints(Arrays.asList(startMidFootPose, goalMidFootPose));

      SideDependentList<Pose3D> goalStepPoses = PlannerTools.createSquaredUpFootsteps(goalMidFootPose, footstepPlannerParameters.getIdealFootstepWidth());
      SideDependentList<DiscreteFootstep> goalSteps = createGoalSteps(goalStepPoses::get);

      stepCostCalculator.initialize(goalSteps);
   }

   @Test
   public void testExpectedYawOffsetsForParameterBasedStepExpansion()
   {
      // Here we set the minimum step yaw to the yaw size of the lattice point multiplied by the number of offsets we want per side
      double numberOfYawOffsetsForASide = 2;
      double latticePointYaw = LatticePoint.gridSizeYaw;
      footstepPlannerParameters.setMinStepYaw(latticePointYaw * -numberOfYawOffsetsForASide);
      footstepPlannerParameters.setMaxStepYaw(latticePointYaw * numberOfYawOffsetsForASide);

      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(footstepPlannerParameters, null, defaultFootPolygons);
      expansion.initialize();

      // Here is the expected number of yaw offsets we are trying to plan with
      double minYaw = footstepPlannerParameters.getMinStepYaw();
      double maxYaw = footstepPlannerParameters.getMaxStepYaw();
      double totalYawOffset = Math.abs(maxYaw) + Math.abs(minYaw);

      int expectedYawOffsets = 0;
      for (double i = 0; i <= totalYawOffset; )
      {
         expectedYawOffsets++;
         i += LatticePoint.gridSizeYaw;
      }
      // Convert the yaw offsets to a HastMap so we can get each unique item to see how many different yaw offsets we are working with
      // This doesn't mean that each step considers all the yaw offsets, but we should expect some that have the option to use all the yaw offsets
      HashSet<Double> uniqueYawOffsets = new HashSet<>();
      TDoubleArrayList yawOffsets = new TDoubleArrayList();
      expansion.getYawOffsets(yawOffsets);
      for (int i = 0; i < yawOffsets.size(); i++)
      {
         uniqueYawOffsets.add(yawOffsets.get(i));
      }

      assertEquals(expectedYawOffsets, uniqueYawOffsets.size());
   }

   @Test
   public void testExpansionAlongBoundsFromOriginWithRight()
   {
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(footstepPlannerParameters, null, defaultFootPolygons);
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

      assertTrue(mostForward.getX() <= footstepPlannerParameters.getMaxStepReach());
      assertTrue(mostBackward.getX() >= footstepPlannerParameters.getMinStepLength());
      assertTrue(mostInward.getY() <= -footstepPlannerParameters.getMinStepWidth());
      assertTrue(mostOutward.getY() >= -footstepPlannerParameters.getMaxStepWidth());

      // Check the min and max a step can yaw
      DiscreteFootstep mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      DiscreteFootstep mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

      // The yaw is a little tricky because we don't have any notion of things being negative.
      // So if the yaw is clockwise (which should be negative) it will be ~5.7 or something just less then 2 PI radians
      // To account for this, we need to subtract the yaw we got from 2 PI.
      double outwardYawFromZero = Math.PI * 2 - mostOutwardYawed.getYaw();
      assertTrue(outwardYawFromZero <= footstepPlannerParameters.getMaxStepYaw());
      assertTrue(mostInwardYawed.getYaw() >= footstepPlannerParameters.getMinStepYaw());

      // Get the footstep closest to the ideal step and ensure that it's less than the max reach
      DiscreteFootstep idealReach = getExtremumNode(childNodes,
                                                    Comparator.comparingDouble(node -> getReachAtNode(node,
                                                                                                      footstepPlannerParameters.getIdealFootstepWidth())));
      assertTrue(getReachAtNode(idealReach, footstepPlannerParameters.getIdealFootstepWidth()) < footstepPlannerParameters.getMaxStepReach());
   }

   @Test
   public void testExpansionAlongBoundsFromOriginWithLeft()
   {
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(footstepPlannerParameters, null, defaultFootPolygons);
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

      assertTrue(mostForward.getX() <= footstepPlannerParameters.getMaxStepReach());
      assertTrue(mostBackward.getX() >= footstepPlannerParameters.getMinStepLength());
      assertTrue(mostInward.getY() >= footstepPlannerParameters.getMinStepWidth());
      assertTrue(mostOutward.getY() <= footstepPlannerParameters.getMaxStepWidth());

      // Check the min and max a step can yaw
      DiscreteFootstep mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      DiscreteFootstep mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

      // The yaw is a little tricky because we don't have any notion of things being negative.
      // So if the yaw is clockwise (which should be negative) it will be ~5.7 or something just less then 2 PI radians
      // To account for this, we need to subtract the yaw we got from 2 PI.
      double outwardYawFromZero = Math.PI * 2 - mostOutwardYawed.getYaw();
      assertTrue(outwardYawFromZero <= footstepPlannerParameters.getMaxStepYaw());
      assertTrue(mostInwardYawed.getYaw() >= footstepPlannerParameters.getMinStepYaw());

      // Get the footstep closest to the ideal step and ensure that it's less than the max reach
      DiscreteFootstep idealReach = getExtremumNode(childNodes,
                                                    Comparator.comparingDouble(node -> getReachAtNode(node,
                                                                                                      footstepPlannerParameters.getIdealFootstepWidth())));
      assertTrue(getReachAtNode(idealReach, footstepPlannerParameters.getIdealFootstepWidth()) < footstepPlannerParameters.getMaxStepReach());
   }

   @Test
   public void testExpansionAlongBoundsFromOrigin()
   {
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(footstepPlannerParameters, null, defaultFootPolygons);
      expansion.initialize();

      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      footstepPlannerParameters.setMaxStepYaw(maxYaw);
      footstepPlannerParameters.setMinStepYaw(minYaw);

      double maxYawAtFullLength = (1.0 - yawReduction) * maxYaw;
      double minYawAtFullLength = (1.0 - yawReduction) * minYaw;

      DiscreteFootstep stanceStep = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep startOfSwingStep = new DiscreteFootstep(0.0, 0.3, 0.0, RobotSide.RIGHT);

      List<FootstepGraphNode> childNodes = new ArrayList<>();
      expansion.doFullExpansion(new FootstepGraphNode(startOfSwingStep, stanceStep), childNodes);
      DiscreteFootstep mostForward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getX()));
      DiscreteFootstep furthestReach = getExtremumNode(childNodes,
                                                       Comparator.comparingDouble(node -> getReachAtNode(node,
                                                                                                         footstepPlannerParameters.getIdealFootstepWidth())));
      DiscreteFootstep mostBackward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getX()));
      DiscreteFootstep mostInward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> node.getY()));
      DiscreteFootstep mostOutward = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -node.getY()));
      DiscreteFootstep mostOutwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> -snapToCircle(node.getYaw())));
      DiscreteFootstep mostInwardYawed = getExtremumNode(childNodes, Comparator.comparingDouble(node -> snapToCircle(node.getYaw())));

      assertTrue(mostForward.getX() < footstepPlannerParameters.getMaxStepReach() + epsilon);
      assertTrue(mostBackward.getX() > footstepPlannerParameters.getMinStepLength() - epsilon);
      assertTrue(mostInward.getY() < -footstepPlannerParameters.getMinStepWidth() + epsilon);
      assertTrue(mostOutward.getY() > -footstepPlannerParameters.getMaxStepWidth() - epsilon);

      double mostOutwardYawedReach = getReachAtNode(mostOutwardYawed, footstepPlannerParameters.getIdealFootstepWidth());
      double mostInwardYawedReach = getReachAtNode(mostOutwardYawed, footstepPlannerParameters.getIdealFootstepWidth());
      double mostOutwardYawMax = InterpolationTools.linearInterpolate(maxYaw,
                                                                      maxYawAtFullLength,
                                                                      mostOutwardYawedReach / footstepPlannerParameters.getMaxStepReach());
      double mostInwardYawMin = InterpolationTools.linearInterpolate(minYaw,
                                                                     minYawAtFullLength,
                                                                     mostInwardYawedReach / footstepPlannerParameters.getMaxStepReach());
      double minOutwardYaw = snapToYawGrid(-mostOutwardYawMax - epsilon);
      double maxInwardYaw = snapToYawGrid(-mostInwardYawMin + epsilon);
      assertTrue(mostOutwardYawed.getYaw() > minOutwardYaw);
      assertTrue(mostInwardYawed.getYaw() < maxInwardYaw + epsilon);
      assertTrue(getReachAtNode(furthestReach, footstepPlannerParameters.getIdealFootstepWidth()) < footstepPlannerParameters.getMaxStepReach());
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
      footstepPlannerParameters.setMaxBranchFactor(branchFactor);

      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(footstepPlannerParameters, stepCostCalculator, defaultFootPolygons);

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

   public static DiscreteFootstep idealStep;

   /**
    * This test is meant to check if the full expansion returns a sorted list or not.
    * We don't always sort the full expansion, but by changing {@link ParameterBasedStepExpansion#SORT_FULL_EXPANSION} to true we can sort the expansion
    */
   @Test
   public void testFullExpansionReturnsSortedOrder()
   {
      Random random = new Random(329032);
      int numberOfGraphNodes = 5;
      int numberOfChildNodes = 5;

      int branchFactor = 100;
      footstepPlannerParameters.setMaxBranchFactor(branchFactor);

      for (int i = 0; i < numberOfGraphNodes; i++)
      {
         DiscreteFootstep stanceStep = DiscreteFootstep.generateRandomFootstep(random, 5.0);
         DiscreteFootstep startOfSwingStep = DiscreteFootstepTools.constructStepInPreviousStepFrame(0.0, 0.3, 0.0, stanceStep);
         FootstepGraphNode node = new FootstepGraphNode(startOfSwingStep, stanceStep);

         for (int j = 0; j < numberOfChildNodes; j++)
         {
            ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(footstepPlannerParameters, stepCostCalculator, defaultFootPolygons);
            expansion.initialize();

            //Get the full expansion without sorting it
            ParameterBasedStepExpansion.SORT_FULL_EXPANSION = false;
            List<FootstepGraphNode> unsortedFullExpansion = new ArrayList<>();
            expansion.doFullExpansion(node, unsortedFullExpansion);

            // Because we set a field to true, this gets sorted inside the doFullExpansion method
            ParameterBasedStepExpansion.SORT_FULL_EXPANSION = true;
            List<FootstepGraphNode> actualFullExpansion = new ArrayList<>();
            expansion.doFullExpansion(node, actualFullExpansion);

            DiscreteFootstep idealStep = stepCostCalculator.computeIdealStep(node.getSecondStep(), node.getFirstStep());

            ToDoubleFunction<FootstepGraphNode> stepDistance = step -> ParameterBasedStepExpansion.IdealStepProximityComparator.calculateStepProximity(step.getSecondStep(),
                                                                                                                                                       idealStep);

            List<FootstepGraphNode> expectedExpansionSorted = new ArrayList<>(unsortedFullExpansion);
            Comparator<FootstepGraphNode> sorter = Comparator.comparingDouble(stepDistance);
            expectedExpansionSorted.sort(sorter);

            for (int k = 0; k < actualFullExpansion.size(); k++)
            {
               Assertions.assertTrue(actualFullExpansion.get(i).getSecondStep().equalPosition(expectedExpansionSorted.get(i).getSecondStep()));
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

      int branchFactor = 100;
      footstepPlannerParameters.setMaxBranchFactor(branchFactor);

      for (int i = 0; i < numberOfGraphNodes; i++)
      {
         DiscreteFootstep stanceStep = DiscreteFootstep.generateRandomFootstep(random, 5.0);
         DiscreteFootstep startOfSwingStep = DiscreteFootstepTools.constructStepInPreviousStepFrame(0.0, 0.3, 0.0, stanceStep);
         FootstepGraphNode node = new FootstepGraphNode(startOfSwingStep, stanceStep);

         for (int j = 0; j < numberOfChildNodes; j++)
         {
            ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(footstepPlannerParameters, stepCostCalculator, defaultFootPolygons);
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
      double clearance = 0.01;

      // Set width so expansion will step on stance foot if not prevented
      footstepPlannerParameters.setMinStepWidth(0.0);
      footstepPlannerParameters.setMinStepLength(-0.2);
      footstepPlannerParameters.setEnableExpansionMask(false);
      footstepPlannerParameters.setMinClearanceFromStance(clearance);
      footstepPlannerParameters.setMaxBranchFactor(Integer.MAX_VALUE);

      SideDependentList<ConvexPolygon2D> footPolygons = defaultFootPolygons;
      ParameterBasedStepExpansion expansion = new ParameterBasedStepExpansion(footstepPlannerParameters, null, footPolygons);
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

   private SideDependentList<DiscreteFootstep> createGoalSteps(Function<RobotSide, Pose3D> poses)
   {
      return new SideDependentList<>(side ->
                                     {
                                        Pose3DReadOnly goalFootPose = poses.apply(side);
                                        return new DiscreteFootstep(goalFootPose.getX(), goalFootPose.getY(), goalFootPose.getYaw(), side);
                                     });
   }
}
