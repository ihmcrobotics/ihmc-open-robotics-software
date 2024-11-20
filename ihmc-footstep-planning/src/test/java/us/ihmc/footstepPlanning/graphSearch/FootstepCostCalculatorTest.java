package us.ihmc.footstepPlanning.graphSearch;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepCheckerInterface;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostCalculator;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Arrays;
import java.util.Random;
import java.util.function.Function;

public class FootstepCostCalculatorTest
{
   private final double EPSILON = 1e-10;
   private final int NUMBER_OF_TESTS = 25;
   private final Random random = new Random(1776L);

   private DefaultFootstepPlannerParametersBasics footstepPlannerParameters;
   private FootstepSnapAndWiggler snapper;
   private FootstepCostCalculator stepCostCalculator;
   private FootstepPlannerHeuristicCalculator distanceAndYawHeuristics;

   private final SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();
   private final FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();

   private final YoRegistry registry = new YoRegistry("FootstepCostCalculatorTest");

   private DiscreteFootstep step1;
   private DiscreteFootstep step0;
   private DiscreteFootstep idealStepNode;
   private FramePose3D stanceFoot;
   private FramePose3D idealStep;

   @BeforeEach
   public void setupPosesAndSteps()
   {
      // We create new footstep parameters so that changes from the last test don't affect the parameters in the next test
      // We create the other objects before each test too, because we want the most up to date parameters and nothing left over from the other test
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
      Pose3D goalMidFootPose = new Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      bodyPathPlanHolder.setPoseWaypoints(Arrays.asList(startMidFootPose, goalMidFootPose));

      SideDependentList<Pose3D> goalStepPoses = PlannerTools.createSquaredUpFootsteps(goalMidFootPose, footstepPlannerParameters.getIdealFootstepWidth());
      SideDependentList<DiscreteFootstep> goalSteps = createGoalSteps(goalStepPoses::get);

      stepCostCalculator.initialize(goalSteps);

      // Create a height map that can be used by the snapper
      HeightMapParameters heightMapParameters = RapidHeightMapExtractor.getHeightMapParameters();
      double gridResolution = heightMapParameters.getGridResolutionXY();
      double gridSizeXY = heightMapParameters.getGridSizeXY();
      double gridCenterXY = 0.0;

      HeightMapData localHeightMapData = new HeightMapData(gridResolution, gridSizeXY, 0.0, 0.0);
      int centerIndex = localHeightMapData.getCenterIndex();
      int cellsPerAxis = 2 * centerIndex + 1;
      int N = cellsPerAxis * cellsPerAxis;

      for (int key = 0; key < N; key++)
      {
         double x = HeightMapTools.keyToXCoordinate(key, gridCenterXY, gridResolution, centerIndex);
         double y = HeightMapTools.keyToYCoordinate(key, gridCenterXY, gridResolution, centerIndex);
         double z = 0.0;
         localHeightMapData.setHeightAt(x, y, z);
      }

      // Set the heightmap so we can use it in the snapper
      environmentHandler.setHeightMap(localHeightMapData);
   }

   /**
    * This test sets the ideal step to the ideal step node.
    * Meaning that there shouldn't be any extra cost to get to the ideal step outside the parameter
    * that defines the cost per each step.
    * This test is rather quick, so we can run it a bunch of times with random steps to ensure reliability of the test
    */
   @Test
   public void testIdealStepEqualsStepCost()
   {
      for (int i = 0; i < NUMBER_OF_TESTS; i++)
      {
         setupNextRandomStanceAndIdealStep();

         // Test ideal step cost equals base step cost
         snapper.addSnapData(idealStepNode, new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(idealStepNode, idealStep), new ConvexPolygon2D()));
         double actualCost = stepCostCalculator.computeCost(idealStepNode, step1, step0);

         double expectedCost = footstepPlannerParameters.getCostPerStep();
         Assertions.assertTrue(MathTools.epsilonEquals(actualCost, expectedCost, EPSILON),
                               "Ideal step cost doesn't equal per step cost! Expected value was: " + expectedCost + " and the actual value was: " + actualCost);
      }
   }

   /**
    * When planning footsteps, we allow the planner to compute the cost of steps that don't completely rest on the ground.
    * This could be because it's hanging off the edge of something, or the surface its standing on has gaps in it detected by perception data.
    * When this happens, we want the planner to punish those steps as they aren't the greatest option.
    * Test that the cost of the given step is equal to (costPerStep + (1 - (footholdAreaWeight * percentFoothold))
    */
   @Test
   public void testPartialAreaCost()
   {
      for (int i = 0; i < NUMBER_OF_TESTS; i++)
      {
         setupNextRandomStanceAndIdealStep();

         // Set up the foothold area weight so that it's not zero
         footstepPlannerParameters.setFootholdAreaWeight(1.0);
         // Set up the remaining parameters to make sure these edge costs will be zero
         footstepPlannerParameters.setForwardWeight(0.0);
         footstepPlannerParameters.setLateralWeight(0.0);
         footstepPlannerParameters.setStepDownWeight(0.0);
         footstepPlannerParameters.setStepUpWeight(0.0);
         footstepPlannerParameters.setYawWeight(0.0);
         footstepPlannerParameters.setRollWeight(0.0);
         footstepPlannerParameters.setPitchWeight(0.0);

         // Get the partial area cost for the ideal step
         ConvexPolygon2D foothold = PlannerTools.createDefaultFootPolygon();
         double percentAreaCost = random.nextDouble();
         double percentFoothold = EuclidCoreTools.interpolate(1.0, footstepPlannerParameters.getMinFootholdPercent(), percentAreaCost);
         foothold.scale(Math.sqrt(percentFoothold));

         // Compute the snap data with the partial area cost for the step, pack all this into the idealStepNode
         FootstepSnapData snapDataWithTransform = new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(idealStepNode, idealStep),
                                                                       new ConvexPolygon2D());
         RotationMatrix expectedRotation = EuclidCoreRandomTools.nextRotationMatrix(random);
         Vector3D expectedTranslation = EuclidCoreRandomTools.nextVector3D(random);
         snapDataWithTransform.getSnapTransform().set(expectedRotation, expectedTranslation);
         snapDataWithTransform.setSnapAreaFraction(percentFoothold);
         idealStepNode.setSnapData(snapDataWithTransform);

         double actualCost = stepCostCalculator.computeCost(idealStepNode, step1, step0);

         double expectedCost = footstepPlannerParameters.getCostPerStep() + (1 - footstepPlannerParameters.getFootholdAreaWeight() * percentFoothold);
         Assertions.assertTrue(MathTools.epsilonEquals(actualCost, expectedCost, EPSILON),
                               "Area based cost does not equal expected value!  Expected value was: " + expectedCost + " and the actual value was: "
                               + actualCost);
      }
   }

   /**
    * This test covers the case where we have a random footstep.
    * Not looking for a special case where we have a partial foothold, or that we already at the ideal step.
    * Testing that, when given a random pose, we know what the expected cost is.
    * Because the footstep planner has a lot of moving parts.
    * We have to use methods inside to confirm that we get the expected cost.
    */
   @Test
   public void testEdgeCostPipelineWithRandomPose()
   {
      for (int i = 0; i < NUMBER_OF_TESTS; i++)
      {
         setupNextRandomStanceAndIdealStep();

         // Test random footstep cost
         DiscreteFootstep randomStep = new DiscreteFootstep(random.nextDouble(-2.0, 2.0),
                                                            random.nextDouble(-2.0, 2.0),
                                                            random.nextDouble(-Math.PI, Math.PI),
                                                            idealStepNode.getRobotSide());

         double randomStepHeight = random.nextDouble(-10.0, 10.0);
         double randomStepPitch = random.nextDouble(-0.25 * Math.PI, 0.25 * Math.PI);
         double randomStepRoll = random.nextDouble(-0.25 * Math.PI, 0.25 * Math.PI);
         FramePose3D randomStepPose = new FramePose3D();
         randomStepPose.getPosition().set(randomStep.getX(), randomStep.getY(), randomStepHeight);
         randomStepPose.getOrientation().set(new Quaternion(randomStep.getYaw(), randomStepPitch, randomStepRoll));

         FramePose3D stanceFootZUpPose = new FramePose3D();
         stanceFootZUpPose.getPosition().set(stanceFoot.getPosition());
         stanceFootZUpPose.getOrientation().setYawPitchRoll(step1.getYaw(), 0.0, 0.0);
         PoseReferenceFrame stanceFootZUpFrame = new PoseReferenceFrame("stanceNodeFrame", stanceFootZUpPose);

         randomStepPose.changeFrame(stanceFootZUpFrame);
         idealStep.changeFrame(stanceFootZUpFrame);

         double deltaX = randomStepPose.getX() - idealStep.getX();
         double deltaY = randomStepPose.getY() - idealStep.getY();
         double deltaZ = randomStepPose.getZ() - idealStep.getZ();
         double deltaYaw = AngleTools.computeAngleDifferenceMinusPiToPi(randomStepPose.getYaw(), idealStep.getYaw());
         double deltaPitch = AngleTools.computeAngleDifferenceMinusPiToPi(randomStepPose.getPitch(), idealStep.getPitch());
         double deltaRoll = AngleTools.computeAngleDifferenceMinusPiToPi(randomStepPose.getRoll(), idealStep.getRoll());

         // Expected cost based on the current parameter values with the location of the step pose
         double expectedCost = 0;
         expectedCost += Math.abs(deltaX * footstepPlannerParameters.getForwardWeight());
         expectedCost += Math.abs(deltaY * footstepPlannerParameters.getLateralWeight());
         expectedCost += Math.abs(deltaZ * (deltaZ > 0.0 ? footstepPlannerParameters.getStepUpWeight() : footstepPlannerParameters.getStepDownWeight()));
         expectedCost += Math.abs(deltaYaw * footstepPlannerParameters.getYawWeight());
         expectedCost += Math.abs(deltaPitch * footstepPlannerParameters.getPitchWeight());
         expectedCost += Math.abs(deltaRoll * footstepPlannerParameters.getRollWeight());
         expectedCost += footstepPlannerParameters.getCostPerStep();

         randomStepPose.changeFrame(ReferenceFrame.getWorldFrame());
         snapper.addSnapData(randomStep, new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(randomStep, randomStepPose), new ConvexPolygon2D()));
         randomStepPose.changeFrame(stanceFootZUpFrame);

         // Get the cost of the expectedIdealStep; this is what we expect to happen in the FootstepCostCalculator
         DiscreteFootstep expectedIdealStep = stepCostCalculator.computeIdealStep(step1, step0);
         double idealStepHeuristicCost = distanceAndYawHeuristics.compute(new FootstepGraphNode(step1, expectedIdealStep));
         double heuristicCost = distanceAndYawHeuristics.compute(new FootstepGraphNode(step1, randomStep));

         // This can be negative, have to check this the same way its being done in the FootstepCostCalculator
         double deltaHeuristics = idealStepHeuristicCost - heuristicCost;
         if (deltaHeuristics > 0.0)
         {
            expectedCost += deltaHeuristics;
         }
         else
         {
            expectedCost = (Math.max(0.0, expectedCost - deltaHeuristics));
         }

         double stepCost = stepCostCalculator.computeCost(randomStep, step1, step0);
         Assertions.assertTrue(MathTools.epsilonEquals(stepCost, expectedCost, EPSILON),
                               "Pose based cost does not equal expected value! Expected value was: " + expectedCost + " and the actual value was: " + stepCost);
      }
   }

   /**
    * Test that when planning on flat ground, and when the goal steps are out of reach.
    * That the ideal step matches the parameter for the ideal step length.
    * Because the ground is flat, and it's not possible to reach the goal, we should be taking the ideal step
    */
   @Test
//   @Disabled
   public void testStanceWidthAndLength()
   {
      // Since the goal poses are out of reach, this is the expected step length
      double expectedStepLength = 0.3;
      double expectedStepWidth = 0.4;
      footstepPlannerParameters.setIdealFootstepLength(expectedStepLength);
      footstepPlannerParameters.setIdealFootstepWidth(expectedStepWidth);

      // Set the heightmap to null to create the flat ground environment
      environmentHandler.setHeightMap(null);

      // Set the goal pose to be out of reach of the start pose
      double pathLength = 20.0;
      Pose3D startNidFootPose = new Pose3D(-pathLength, 0.0, 0.0, 0.0, 0.0, 0.0);
      Pose3D goalMidFootPose = new Pose3D(pathLength, 0.0, 0.0, 0.0, 0.0, 0.0);
      bodyPathPlanHolder.setPoseWaypoints(Arrays.asList(startNidFootPose, goalMidFootPose));

      // Give the goal poses to the FootstepCostCalculator
      SideDependentList<Pose3D> goalStepPoses = PlannerTools.createSquaredUpFootsteps(goalMidFootPose, footstepPlannerParameters.getIdealFootstepWidth());
      SideDependentList<DiscreteFootstep> goalPoses = createGoalSteps(goalStepPoses::get);
      stepCostCalculator.initialize(goalPoses);

      for (int i = 0; i < NUMBER_OF_TESTS; i++)
      {
         RobotSide side = RobotSide.generateRandomRobotSide(random);
         double startX = EuclidCoreRandomTools.nextDouble(random, 0.5 * pathLength - 1.0);

         DiscreteFootstep testStanceNode = new DiscreteFootstep(startX, 0.5 * side.negateIfRightSide(expectedStepWidth), 0.0, side);
         DiscreteFootstep testSwingNode = new DiscreteFootstep(0.0, 0.0, 0.0, side.getOppositeSide());

         DiscreteFootstep idealStep = stepCostCalculator.computeIdealStep(testStanceNode, testSwingNode);

         double actualStepLength = idealStep.getX() - testStanceNode.getX();
         double actualStepWidth = side.negateIfLeftSide(idealStep.getY() - testStanceNode.getY());

         Assertions.assertTrue(MathTools.epsilonEquals(actualStepLength, expectedStepLength, EPSILON),
                               " Expected step length was: " + expectedStepLength + ", but the acutal was: " + actualStepLength);
         Assertions.assertTrue(MathTools.epsilonEquals(actualStepWidth, expectedStepWidth, EPSILON),
                               " Expected step width was: " + expectedStepWidth + ", but the acutal was: " + actualStepWidth);
      }
   }

   /**
    * Get the next random stance and ideal step.
    * Nothing special with this method.
    * We perform operations that need to happen after each loop in each test.
    * This is because we want to get random steps each time, that is what this method does, and it resets the snapper.
    */
   private void setupNextRandomStanceAndIdealStep()
   {
      step1 = DiscreteFootstep.generateRandomFootstep(random, 10.0);
      step0 = DiscreteFootstepTools.constructStepInPreviousStepFrame(0.0, 0.3, 0.0, step1);

      idealStepNode = stepCostCalculator.computeIdealStep(step1, step0);

      double stanceHeight = random.nextDouble(-10.0, 10.0);
      double stancePitch = random.nextDouble(-0.25 * Math.PI, 0.25 * Math.PI);
      double stanceRoll = random.nextDouble(-0.25 * Math.PI, 0.25 * Math.PI);

      stanceFoot = new FramePose3D();
      stanceFoot.getPosition().set(step1.getX(), step1.getY(), stanceHeight);
      stanceFoot.getOrientation().set(new Quaternion(step1.getYaw(), stancePitch, stanceRoll));

      // The ideal step location is the same place as the ideal step node
      idealStep = new FramePose3D();
      idealStep.getPosition().set(idealStepNode.getX(), idealStepNode.getY(), stanceFoot.getZ());
      idealStep.getOrientation().setYawPitchRoll(idealStepNode.getYaw(), 0.0, 0.0);

      snapper.reset();
      snapper.addSnapData(step1, new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(step1, stanceFoot), new ConvexPolygon2D()));
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
