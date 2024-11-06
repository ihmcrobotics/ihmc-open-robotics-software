package us.ihmc.footstepPlanning.graphSearch.parameters;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

/**
 * This class is meant to test that we can get and set all the parameters for the footstep planner.
 * More tests could be added in the future.
 * But we want to ensure that we can set and get all the parameters that we want to use.
 */
public class FootstepPlannerParametersTest
{
   private static final double epsilon = 1e-5;
   private final Random random = new Random(1738L);
   private DefaultFootstepPlannerParametersBasics parameters;

   @BeforeEach
   public void setUp()
   {
      parameters = new TestFootstepPlannerParameters();
   }

   @Test
   public void testAStarHeuristicsWeight()
   {
      double aStarHeuristicWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setAStarHeuristicsWeight(aStarHeuristicWeight);
      assertEquals(aStarHeuristicWeight, parameters.getAStarHeuristicsWeight(), epsilon);
   }

   @Test
   public void testMaxBranchFactor()
   {
      int maxBranchFactor = RandomNumbers.nextInt(random, 0, 100);
      parameters.setMaxBranchFactor(maxBranchFactor);
      assertEquals(maxBranchFactor, parameters.getMaxBranchFactor(), epsilon);
   }

   @Test
   public void testEnableExpansionMask()
   {
      boolean enableExpansionMask = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setEnableExpansionMask(enableExpansionMask);
      assertEquals(enableExpansionMask, parameters.getEnableExpansionMask());
   }

   @Test
   public void testUseReachabilityMap()
   {
      boolean useReachabilityMap = random.nextBoolean();
      parameters.setUseReachabilityMap(useReachabilityMap);
      assertEquals(useReachabilityMap, parameters.getUseReachabilityMap());
   }

   @Test
   public void testSolutionQualityThreshold()
   {
      double solutionQualityThreshold = RandomNumbers.nextDouble(random, 10.0);
      parameters.setSolutionQualityThreshold(solutionQualityThreshold);
      assertEquals(solutionQualityThreshold, parameters.getSolutionQualityThreshold(), epsilon);
   }

   @Test
   public void testIdealFootstepWidth()
   {
      double idealFootstepWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setIdealFootstepWidth(idealFootstepWidth);
      assertEquals(idealFootstepWidth, parameters.getIdealFootstepWidth(), epsilon);
   }

   @Test
   public void testIdealFootstepLength()
   {
      double idealFootstepLength = RandomNumbers.nextDouble(random, 10.0);
      parameters.setIdealFootstepLength(idealFootstepLength);
      assertEquals(idealFootstepLength, parameters.getIdealFootstepLength(), epsilon);
   }

   @Test
   public void testIdealSideStepWidth()
   {
      double idealSideStepWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setIdealSideStepWidth(idealSideStepWidth);
      assertEquals(idealSideStepWidth, parameters.getIdealSideStepWidth(), epsilon);
   }

   @Test
   public void testIdealBackStepLength()
   {
      double idealBackStepLength = RandomNumbers.nextDouble(random, 10.0);
      parameters.setIdealBackStepLength(idealBackStepLength);
      assertEquals(idealBackStepLength, parameters.getIdealBackStepLength(), epsilon);
   }

   @Test
   public void testMinFootholdPercent()
   {
      double minFootholdPercent = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinFootholdPercent(minFootholdPercent);
      assertEquals(minFootholdPercent, parameters.getMinFootholdPercent(), epsilon);
   }

   @Test
   public void testMinClearanceFromStance()
   {
      double minClearanceFromStance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinClearanceFromStance(minClearanceFromStance);
      assertEquals(minClearanceFromStance, parameters.getMinClearanceFromStance(), epsilon);
   }

   @Test
   public void testMinSurfaceIncline()
   {
      double minSurfaceIncline = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinSurfaceInclineRadians(minSurfaceIncline);
      assertEquals(minSurfaceIncline, parameters.getMinSurfaceIncline(), epsilon);
   }

   @Test
   public void testMinStepWidth()
   {
      double minStepWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinStepWidth(minStepWidth);
      assertEquals(minStepWidth, parameters.getMinStepWidth(), epsilon);
   }

   @Test
   public void testMinStepLength()
   {
      double minStepLength = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinStepLength(minStepLength);
      assertEquals(minStepLength, parameters.getMinStepLength(), epsilon);
   }

   @Test
   public void testMinStepYaw()
   {
      double minStepYaw = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinStepYaw(minStepYaw);
      assertEquals(minStepYaw, parameters.getMinStepYaw(), epsilon);
   }

   @Test
   public void testMaxStepWidth()
   {
      double maxStepWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxStepWidth(maxStepWidth);
      assertEquals(maxStepWidth, parameters.getMaxStepWidth(), epsilon);
   }

   @Test
   public void testMaxStepReach()
   {
      double maxStepReach = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxStepReach(maxStepReach);
      assertEquals(maxStepReach, parameters.getMaxStepReach(), epsilon);
   }

   @Test
   public void testMaxStepYaw()
   {
      double maxStepYaw = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxStepYaw(maxStepYaw);
      assertEquals(maxStepYaw, parameters.getMaxStepYaw(), epsilon);
   }

   @Test
   public void testMaxStepZ()
   {
      double maxStepZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxStepZ(maxStepZ);
      assertEquals(maxStepZ, parameters.getMaxStepZ(), epsilon);
   }

   @Test
   public void testMaxSwingZ()
   {
      double maxSwingZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxSwingZ(maxSwingZ);
      assertEquals(maxSwingZ, parameters.getMaxSwingZ(), epsilon);
   }

   @Test
   public void testMaxSwingReach()
   {
      double maxSwingReach = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxSwingReach(maxSwingReach);
      assertEquals(maxSwingReach, parameters.getMaxSwingReach(), epsilon);
   }

   @Test
   public void testCostPerStep()
   {
      double costPerStep = RandomNumbers.nextDouble(random, 10.0);
      parameters.setCostPerStep(costPerStep);
      assertEquals(costPerStep, parameters.getCostPerStep(), epsilon);
   }

   @Test
   public void testForwardWeight()
   {
      double forwardWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setForwardWeight(forwardWeight);
      assertEquals(forwardWeight, parameters.getForwardWeight(), epsilon);
   }

   @Test
   public void testLateralWeight()
   {
      double lateralWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setLateralWeight(lateralWeight);
      assertEquals(lateralWeight, parameters.getLateralWeight(), epsilon);
   }

   @Test
   public void testStepUpWeight()
   {
      double stepUpWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setStepUpWeight(stepUpWeight);
      assertEquals(stepUpWeight, parameters.getStepUpWeight(), epsilon);
   }

   @Test
   public void testStepDownWeight()
   {
      double stepDownWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setStepDownWeight(stepDownWeight);
      assertEquals(stepDownWeight, parameters.getStepDownWeight(), epsilon);
   }

   @Test
   public void testYawWeight()
   {
      double yawWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setYawWeight(yawWeight);
      assertEquals(yawWeight, parameters.getYawWeight(), epsilon);
   }

   @Test
   public void testRollWeight()
   {
      double rollWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setRollWeight(rollWeight);
      assertEquals(rollWeight, parameters.getRollWeight(), epsilon);
   }

   @Test
   public void testPitchWeight()
   {
      double pitchWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setPitchWeight(pitchWeight);
      assertEquals(pitchWeight, parameters.getPitchWeight(), epsilon);
   }

   @Test
   public void testFootholdAreaWeight()
   {
      double footholdAreaWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setFootholdAreaWeight(footholdAreaWeight);
      assertEquals(footholdAreaWeight, parameters.getFootholdAreaWeight(), epsilon);
   }

   @Test
   public void testRMSErrorThreshold()
   {
      double errorThreshold = RandomNumbers.nextDouble(random, 10.0);
      parameters.setRMSErrorThreshold(errorThreshold);
      assertEquals(errorThreshold, parameters.getRMSErrorThreshold(), epsilon);
   }

   @Test
   public void testRMSErrorCost()
   {
      double errorCost = RandomNumbers.nextDouble(random, 10.0);
      parameters.setRMSErrorCost(errorCost);
      assertEquals(errorCost, parameters.getRMSErrorCost(), epsilon);
   }

   @Test
   public void testRMSMinErrorToPenalize()
   {
      double minErrorToPenalize = RandomNumbers.nextDouble(random, 10.0);
      parameters.setRMSMinErrorToPenalize(minErrorToPenalize);
      assertEquals(minErrorToPenalize, parameters.getRMSMinErrorToPenalize(), epsilon);
   }

   @Test
   public void testHeightMapSnapThreshold()
   {
      double heightMapSnapThreshold = RandomNumbers.nextDouble(random, 10.0);
      parameters.setHeightMapSnapThreshold(heightMapSnapThreshold);
      assertEquals(heightMapSnapThreshold, parameters.getHeightMapSnapThreshold(), epsilon);
   }

   @Test
   public void testReferencePlanAlpha()
   {
      double referencePlanAlpha = RandomNumbers.nextDouble(random, 10.0);
      parameters.setReferencePlanAlpha(referencePlanAlpha);
      assertEquals(referencePlanAlpha, parameters.getReferencePlanAlpha(), epsilon);
   }

   @Test
   public void testWiggleInsideDeltaTarget()
   {
      double wiggleInsideDeltaTarget = RandomNumbers.nextDouble(random, 10.0);
      parameters.setWiggleInsideDeltaTarget(wiggleInsideDeltaTarget);
      assertEquals(wiggleInsideDeltaTarget, parameters.getWiggleInsideDeltaTarget(), epsilon);
   }

   @Test
   public void testWiggleInsideDeltaMinimum()
   {
      double wiggleInsideDeltaMinimum = RandomNumbers.nextDouble(random, 10.0);
      parameters.setWiggleInsideDeltaMinimum(wiggleInsideDeltaMinimum);
      assertEquals(wiggleInsideDeltaMinimum, parameters.getWiggleInsideDeltaMinimum(), epsilon);
   }

   @Test
   public void testEnableConcaveHullWiggler()
   {
      boolean enableConcaveHullWiggler = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setEnableConcaveHullWiggler(enableConcaveHullWiggler);
      assertEquals(enableConcaveHullWiggler, parameters.getEnableConcaveHullWiggler());
   }

   @Test
   public void testWiggleWhilePlanning()
   {
      boolean wiggleWhilePlanning = random.nextBoolean();
      parameters.setWiggleWhilePlanning(wiggleWhilePlanning);
      assertEquals(wiggleWhilePlanning, parameters.getWiggleWhilePlanning());
   }

   @Test
   public void testMaxXYWiggleDistance()
   {
      double maximumXYWiggleDistance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxXYWiggleDistance(maximumXYWiggleDistance);
      assertEquals(maximumXYWiggleDistance, parameters.getMaxXYWiggleDistance(), epsilon);
   }

   @Test
   public void testMaxYawWiggle()
   {
      double maximumYawWiggle = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxYawWiggle(maximumYawWiggle);
      assertEquals(maximumYawWiggle, parameters.getMaxYawWiggle(), epsilon);
   }

   @Test
   public void testMaxZPenetrationOnValleyRegions()
   {
      double maxZpenetration = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxZPenetrationOnValleyRegions(maxZpenetration);
      assertEquals(maxZpenetration, parameters.getMaxZPenetrationOnValleyRegions(), epsilon);
   }

   @Test
   public void testMaximumSnapHeight()
   {
      double maximumSnapHeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumSnapHeight(maximumSnapHeight);
      assertEquals(maximumSnapHeight, parameters.getMaximumSnapHeight(), epsilon);
   }

   @Test
   public void testFinalTurnProximity()
   {
      double finalTurnProximity = RandomNumbers.nextDouble(random, 10.0);
      parameters.setFinalTurnProximity(finalTurnProximity);
      assertEquals(finalTurnProximity, parameters.getFinalTurnProximity(), epsilon);
   }

   @Test
   public void testDistanceFromPathTolerance()
   {
      double distanceFromPathTolerance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setDistanceFromPathTolerance(distanceFromPathTolerance);
      assertEquals(distanceFromPathTolerance, parameters.getDistanceFromPathTolerance(), epsilon);
   }

   @Test
   public void testDeltaYawFromReferenceTolerance()
   {
      double deltaYawFromReferenceTolerance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setDeltaYawFromReferenceTolerance(deltaYawFromReferenceTolerance);
      assertEquals(deltaYawFromReferenceTolerance, parameters.getDeltaYawFromReferenceTolerance(), epsilon);
   }

   @Test
   public void testCheckForBodyBoxCollisions()
   {
      boolean checkForBodyBoxCollisions = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setCheckForBodyBoxCollisions(checkForBodyBoxCollisions);
      assertEquals(checkForBodyBoxCollisions, parameters.getCheckForBodyBoxCollisions());
   }

   @Test
   public void testBodyBoxWidth()
   {
      double bodyBoxWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxWidth(bodyBoxWidth);
      assertEquals(bodyBoxWidth, parameters.getBodyBoxWidth(), epsilon);
   }

   @Test
   public void testBodyBoxHeight()
   {
      double bodyBoxHeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxHeight(bodyBoxHeight);
      assertEquals(bodyBoxHeight, parameters.getBodyBoxHeight(), epsilon);
   }

   @Test
   public void testBodyBoxDepth()
   {
      double bodyBoxDepth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxDepth(bodyBoxDepth);
      assertEquals(bodyBoxDepth, parameters.getBodyBoxDepth(), epsilon);
   }

   @Test
   public void testBodyBoxBaseX()
   {
      double bodyBoxBaseX = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxBaseX(bodyBoxBaseX);
      assertEquals(bodyBoxBaseX, parameters.getBodyBoxBaseX(), epsilon);
   }

   @Test
   public void testBodyBoxBaseY()
   {
      double bodyBoxBaseY = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxBaseY(bodyBoxBaseY);
      assertEquals(bodyBoxBaseY, parameters.getBodyBoxBaseY(), epsilon);
   }

   @Test
   public void testBodyBoxBaseZ()
   {
      double bodyBoxBaseZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxBaseZ(bodyBoxBaseZ);
      assertEquals(bodyBoxBaseZ, parameters.getBodyBoxBaseZ(), epsilon);
   }

   @Test
   public void testIntermediateBodyBoxChecks()
   {
      int numberOfBoundingBoxChecks = RandomNumbers.nextInt(random, -10, 10);
      parameters.setIntermediateBodyBoxChecks(numberOfBoundingBoxChecks);
      assertEquals(numberOfBoundingBoxChecks, parameters.getIntermediateBodyBoxChecks());
   }

   @Test
   public void testEnableShinCollisionCheck()
   {
      boolean enableShinCollision = random.nextBoolean();
      parameters.setEnableShinCollisionCheck(enableShinCollision);
      assertEquals(enableShinCollision, parameters.getEnableShinCollisionCheck());
   }

   @Test
   public void testShinLength()
   {
      double shinLength = RandomNumbers.nextDouble(random, 10.0);
      parameters.setShinLength(shinLength);
      assertEquals(shinLength, parameters.getShinLength(), epsilon);
   }

   @Test
   public void testShinToeClearance()
   {
      double shinToeClearance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setShinToeClearance(shinToeClearance);
      assertEquals(shinToeClearance, parameters.getShinToeClearance(), epsilon);
   }

   @Test
   public void testShinHeelClearance()
   {
      double shinHeelClearance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setShinHeelClearance(shinHeelClearance);
      assertEquals(shinHeelClearance, parameters.getShinHeelClearance(), epsilon);
   }

   @Test
   public void testShinHeightOffset()
   {
      double shinHeightOffset = RandomNumbers.nextDouble(random, 10.0);
      parameters.setShinHeightOffset(shinHeightOffset);
      assertEquals(shinHeightOffset, parameters.getShinHeightOffset(), epsilon);
   }

   @Test
   public void testCheckForPathCollisions()
   {
      boolean checkForPathCollisions = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setCheckForPathCollisions(checkForPathCollisions);
      assertEquals(checkForPathCollisions, parameters.getCheckForPathCollisions());
   }

   @Test
   public void testCliffBottomHeightToAvoid()
   {
      double cliffHeightToAvoid = RandomNumbers.nextDouble(random, 10.0);
      parameters.setCliffBottomHeightToAvoid(cliffHeightToAvoid);
      assertEquals(cliffHeightToAvoid, parameters.getCliffBottomHeightToAvoid(), epsilon);
   }

   @Test
   public void testMinDistanceFromCliffBottoms()
   {
      double minimumDistanceFromCliff = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinDistanceFromCliffBottoms(minimumDistanceFromCliff);
      assertEquals(minimumDistanceFromCliff, parameters.getMinDistanceFromCliffBottoms(), epsilon);
   }

   @Test
   public void testCliffTopHeightToAvoid()
   {
      double cliffTopHeightToAvoid = RandomNumbers.nextDouble(random, 10.0);
      parameters.setCliffTopHeightToAvoid(cliffTopHeightToAvoid);
      assertEquals(cliffTopHeightToAvoid, parameters.getCliffTopHeightToAvoid(), epsilon);
   }

   @Test
   public void testMinDistanceFromCliffTops()
   {
      double minimumDistanceFromCliff = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinDistanceFromCliffTops(minimumDistanceFromCliff);
      assertEquals(minimumDistanceFromCliff, parameters.getMinDistanceFromCliffTops(), epsilon);
   }

   @Test
   public void testScaledFootPolygonPercentage()
   {
      double scaledFootPolygonPercentage = RandomNumbers.nextDouble(random, 10.0);
      parameters.setScaledFootPolygonPercentage(scaledFootPolygonPercentage);
      assertEquals(scaledFootPolygonPercentage, parameters.getScaledFootPolygonPercentage(), epsilon);
   }

   @Test
   public void testCliffHeightThreshold()
   {
      double cliffHeightThreshold = RandomNumbers.nextDouble(random, 10.0);
      parameters.setCliffHeightThreshold(cliffHeightThreshold);
      assertEquals(cliffHeightThreshold, parameters.getCliffHeightThreshold(), epsilon);
   }
}