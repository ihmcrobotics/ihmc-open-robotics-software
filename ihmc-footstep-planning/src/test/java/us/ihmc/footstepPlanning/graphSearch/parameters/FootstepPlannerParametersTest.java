package us.ihmc.footstepPlanning.graphSearch.parameters;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class FootstepPlannerParametersTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-5;

   @Test
   public void testSettingParameters()
   {
      Random random = new Random(1738L);
      DefaultFootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();

      for (int iter = 0; iter < iters; iter++)
      {
         testSettingParameters(random, parameters);
      }
   }

   private static void testSettingParameters(Random random, DefaultFootstepPlannerParametersBasics parameters)
   {
      boolean checkForBodyBoxCollisions = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setCheckForBodyBoxCollisions(checkForBodyBoxCollisions);
      assertEquals(checkForBodyBoxCollisions, parameters.getCheckForBodyBoxCollisions());

      boolean checkForPathCollisions = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setCheckForPathCollisions(checkForPathCollisions);
      assertEquals(checkForPathCollisions, parameters.getCheckForPathCollisions());

      double idealFootstepWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setIdealFootstepWidth(idealFootstepWidth);
      assertEquals(idealFootstepWidth, parameters.getIdealFootstepWidth(), epsilon);

      double idealFootstepLength = RandomNumbers.nextDouble(random, 10.0);
      parameters.setIdealFootstepLength(idealFootstepLength);
      assertEquals(idealFootstepLength, parameters.getIdealFootstepLength(), epsilon);

      double idealStepLengthAtMaxStepZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setIdealStepLengthAtMaxStepZ(idealStepLengthAtMaxStepZ);
      assertEquals(idealStepLengthAtMaxStepZ, parameters.getIdealStepLengthAtMaxStepZ(), epsilon);

      double maxStepReach = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxStepReach(maxStepReach);
      assertEquals(maxStepReach, parameters.getMaxStepReach(), epsilon);

      double minStepLength = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMinStepLength(minStepLength);
      assertEquals(minStepLength, parameters.getMinStepLength(), epsilon);

      double minSTepYaw = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinStepYaw(minSTepYaw);
      assertEquals(minSTepYaw, parameters.getMinStepYaw(), epsilon);

      double minStepWidth = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMinStepWidth(minStepWidth);
      assertEquals(minStepWidth, parameters.getMinStepWidth(), epsilon);

      boolean useReachabilityMap = random.nextBoolean();
      parameters.setUseReachabilityMap(useReachabilityMap);
      assertEquals(useReachabilityMap, parameters.getUseReachabilityMap());

      double solutionQualityThreshold = RandomNumbers.nextDouble(random, 10.00);
      parameters.setSolutionQualityThreshold(solutionQualityThreshold);
      assertEquals(solutionQualityThreshold, parameters.getSolutionQualityThreshold(), epsilon);

      double maxStepWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxStepWidth(maxStepWidth);
      assertEquals(maxStepWidth, parameters.getMaxStepWidth(), epsilon);

      double maxStepZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxStepZ(maxStepZ);
      assertEquals(maxStepZ, parameters.getMaxStepZ(), epsilon);

      double maxSwingZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxSwingZ(maxSwingZ);
      assertEquals(maxSwingZ, parameters.getMaxSwingZ(), epsilon);

      double maxSwingReach = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxSwingReach(maxSwingReach);
      assertEquals(maxSwingReach, parameters.getMaxSwingReach(), epsilon);

      double maxStepXWhenForwardAndDown = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxStepXWhenForwardAndDown(maxStepXWhenForwardAndDown);
      assertEquals(maxStepXWhenForwardAndDown, parameters.getMaxStepXWhenForwardAndDown(), epsilon);

      double maxStepZWhenForwardAndDown = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxStepZWhenForwardAndDown(maxStepZWhenForwardAndDown);
      assertEquals(maxStepZWhenForwardAndDown, parameters.getMaxStepZWhenForwardAndDown(), epsilon);

      double wiggleInsideDeltaTarget = RandomNumbers.nextDouble(random, 10.0);
      parameters.setWiggleInsideDeltaTarget(wiggleInsideDeltaTarget);
      assertEquals(wiggleInsideDeltaTarget, parameters.getWiggleInsideDeltaTarget(), epsilon);

      double wiggleInsideDeltaMinimum = RandomNumbers.nextDouble(random, 10.0);
      parameters.setWiggleInsideDeltaMinimum(wiggleInsideDeltaMinimum);
      assertEquals(wiggleInsideDeltaMinimum, parameters.getWiggleInsideDeltaMinimum(), epsilon);

      double maximumStepReachWhenSteppingUp = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaxStepReachWhenSteppingUp(maximumStepReachWhenSteppingUp);
      assertEquals(maximumStepReachWhenSteppingUp, parameters.getMaxStepReachWhenSteppingUp(), epsilon);

      double maximumStepZWhenSteppingUp = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaxStepZWhenSteppingUp(maximumStepZWhenSteppingUp);
      assertEquals(maximumStepZWhenSteppingUp, parameters.getMaxStepZWhenSteppingUp(), epsilon);

      double minFootholdPercent = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinFootholdPercent(minFootholdPercent);
      assertEquals(minFootholdPercent, parameters.getMinFootholdPercent(), epsilon);

      double minSurfaceIncline = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMinSurfaceInclineRadians(minSurfaceIncline);
      assertEquals(minSurfaceIncline, parameters.getMinSurfaceIncline(), epsilon);

      boolean wiggleWhilePlanning = random.nextBoolean();
      parameters.setWiggleWhilePlanning(wiggleWhilePlanning);
      assertEquals(wiggleWhilePlanning, parameters.getWiggleWhilePlanning());

      boolean enableConcaveHullWiggler = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setEnableConcaveHullWiggler(enableConcaveHullWiggler);
      assertEquals(enableConcaveHullWiggler, parameters.getEnableConcaveHullWiggler());

      double maximumXYWiggleDistance = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaxXYWiggleDistance(maximumXYWiggleDistance);
      assertEquals(maximumXYWiggleDistance, parameters.getMaxXYWiggleDistance(), epsilon);

      double maximumYawWiggle = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaxYawWiggle(maximumYawWiggle);
      assertEquals(maximumYawWiggle, parameters.getMaxYawWiggle(), epsilon);

      double maxZpenetration = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaxZPenetrationOnValleyRegions(maxZpenetration);
      assertEquals(maxZpenetration, parameters.getMaxZPenetrationOnValleyRegions(), epsilon);

      double cliffHeightToAvoid = RandomNumbers.nextDouble(random, 10.00);
      parameters.setCliffBaseHeightToAvoid(cliffHeightToAvoid);
      assertEquals(cliffHeightToAvoid, parameters.getCliffBottomHeightToAvoid());

      double minimumDistanceFromCliff = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinDistanceFromCliffBottoms(minimumDistanceFromCliff);
      assertEquals(minimumDistanceFromCliff, parameters.getMinDistanceFromCliffBottoms(), epsilon);

      double minClearanceFromStance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinClearanceFromStance(minClearanceFromStance);
      assertEquals(minClearanceFromStance, parameters.getMinClearanceFromStance(), epsilon);

      double bodyBoxWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxWidth(bodyBoxWidth);
      assertEquals(bodyBoxWidth, parameters.getBodyBoxWidth(), epsilon);

      double bodyBoxHeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxHeight(bodyBoxHeight);
      assertEquals(bodyBoxHeight, parameters.getBodyBoxHeight(), 10.0);

      double bodyBoxDepth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxDepth(bodyBoxDepth);
      assertEquals(bodyBoxDepth, parameters.getBodyBoxDepth(), epsilon);

      double bodyBoxBaseX = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxBaseX(bodyBoxBaseX);
      assertEquals(bodyBoxBaseX, parameters.getBodyBoxBaseX(), epsilon);

      double bodyBOxBaseY = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxBaseY(bodyBOxBaseY);
      assertEquals(bodyBOxBaseY, parameters.getBodyBoxBaseY(), epsilon);

      double bodyBoxBaseZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setBodyBoxBaseZ(bodyBoxBaseZ);
      assertEquals(bodyBoxBaseZ, parameters.getBodyBoxBaseZ(), epsilon);

      double maximumSnapHeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumSnapHeight(maximumSnapHeight);
      assertEquals(maximumSnapHeight, parameters.getMaximumSnapHeight(), epsilon);

      double finalTurnProximity = RandomNumbers.nextDouble(random, 10.0);
      parameters.setFinalTurnProximity(finalTurnProximity);
      assertEquals(finalTurnProximity, parameters.getFinalTurnProximity(), epsilon);

      int numberOfBoundingBoxChecks = RandomNumbers.nextInt(random, -10, 10);
      parameters.setIntermediateBodyBoxChecks(numberOfBoundingBoxChecks);
      assertEquals(numberOfBoundingBoxChecks, parameters.getIntermediateBodyBoxChecks());

      double aStarHeuristicWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setAStarHeuristicsWeight(aStarHeuristicWeight);
      assertEquals(aStarHeuristicWeight, parameters.getAStarHeuristicsWeight(), epsilon);

      double yawWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setYawWeight(yawWeight);
      assertEquals(yawWeight, parameters.getYawWeight(), epsilon);

      double forwardWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setForwardWeight(forwardWeight);
      assertEquals(forwardWeight, parameters.getForwardWeight(), epsilon);

      double laterlaWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setLateralWeight(laterlaWeight);
      assertEquals(laterlaWeight, parameters.getLateralWeight(), epsilon);

      double costPerStep = RandomNumbers.nextDouble(random, 10.0);
      parameters.setCostPerStep(costPerStep);
      assertEquals(costPerStep, parameters.getCostPerStep(), epsilon);

      double stepUpWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setStepUpWeight(stepUpWeight);
      assertEquals(stepUpWeight, parameters.getStepUpWeight(), epsilon);

      double stepDownWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setStepDownWeight(stepDownWeight);
      assertEquals(stepDownWeight, parameters.getStepDownWeight(), epsilon);

      double rollWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setRollWeight(rollWeight);
      assertEquals(rollWeight, parameters.getRollWeight(), epsilon);

      double pitchWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setPitchWeight(pitchWeight);
      assertEquals(pitchWeight, parameters.getPitchWeight(), epsilon);

      double footholdAreaWeight = RandomNumbers.nextDouble(random, 10.00);
      parameters.setFootholdAreaWeight(footholdAreaWeight);
      assertEquals(footholdAreaWeight, parameters.getFootholdAreaWeight(), epsilon);

      double distanceFrompathTolerance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setDistanceFromPathTolerance(distanceFrompathTolerance);
      assertEquals(distanceFrompathTolerance, parameters.getDistanceFromPathTolerance(), epsilon);

      double deltaYawFromReferenceTolerance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setDeltaYawFromReferenceTolerance(deltaYawFromReferenceTolerance);
      assertEquals(deltaYawFromReferenceTolerance, parameters.getDeltaYawFromReferenceTolerance(), epsilon);

      boolean enableShinCollision = random.nextBoolean();
      parameters.setEnableShinCollisionCheck(enableShinCollision);
      assertEquals(enableShinCollision, parameters.getEnableShinCollisionCheck());

      double shinToeClearance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setShinToeClearance(shinToeClearance);
      assertEquals(shinToeClearance, parameters.getShinToeClearance());

      double shinHeelClearance = RandomNumbers.nextDouble(random, 10.0);
      parameters.setShinHeelClearance(shinHeelClearance);
      assertEquals(shinHeelClearance, parameters.getShinHeelClearance());

      double shinLength = RandomNumbers.nextDouble(random, 10.0);
      parameters.setShinLength(shinLength);
      assertEquals(shinLength, parameters.getShinLength());

      double shinHeightOffset = RandomNumbers.nextDouble(random, 10.0);
      parameters.setShinHeightOffset(shinHeightOffset);
      assertEquals(shinHeightOffset, parameters.getShinHeightOffset());
   }
}
