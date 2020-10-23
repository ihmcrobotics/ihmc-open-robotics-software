package us.ihmc.footstepPlanning.graphSearch.parameters;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.robotSide.RobotSide;

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
      FootstepPlannerParametersBasics parameters = new DefaultFootstepPlannerParameters();

      for (int iter = 0; iter < iters; iter++)
      {
         testSettingParameters(random, parameters);
      }
   }

   private static void testSettingParameters(Random random, FootstepPlannerParametersBasics parameters)
   {
      boolean checkForBodyBoxCollisions = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setCheckForBodyBoxCollisions(checkForBodyBoxCollisions);
      assertEquals(checkForBodyBoxCollisions, parameters.checkForBodyBoxCollisions());

      boolean checkForPathCollisions = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setCheckForPathCollisions(checkForPathCollisions);
      assertEquals(checkForPathCollisions, parameters.checkForPathCollisions());

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
      parameters.setMaximumStepReach(maxStepReach);
      assertEquals(maxStepReach, parameters.getMaximumStepReach(), epsilon);

      double minStepLength = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMinimumStepLength(minStepLength);
      assertEquals(minStepLength, parameters.getMinimumStepLength(), epsilon);

      double minSTepYaw = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinimumStepYaw(minSTepYaw);
      assertEquals(minSTepYaw, parameters.getMinimumStepYaw(), epsilon);

      double minStepWidth = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMinimumStepWidth(minStepWidth);
      assertEquals(minStepWidth, parameters.getMinimumStepWidth(), epsilon);

      double maxStepWidth = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumStepWidth(maxStepWidth);
      assertEquals(maxStepWidth, parameters.getMaximumStepWidth(), epsilon);

      double maxStepZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumStepZ(maxStepZ);
      assertEquals(maxStepZ, parameters.getMaxStepZ(), epsilon);

      double maxSwingZ = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumSwingZ(maxSwingZ);
      assertEquals(maxSwingZ, parameters.getMaxSwingZ(), epsilon);

      double maxSwingReach = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumSwingReach(maxSwingReach);
      assertEquals(maxSwingReach, parameters.getMaxSwingReach(), epsilon);

      double maxStepXWhenForwardAndDown = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumStepXWhenForwardAndDown(maxStepXWhenForwardAndDown);
      assertEquals(maxStepXWhenForwardAndDown, parameters.getMaximumStepXWhenForwardAndDown(), epsilon);

      double maxStepZWhenForwardAndDown = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumStepZWhenForwardAndDown(maxStepZWhenForwardAndDown);
      assertEquals(maxStepZWhenForwardAndDown, parameters.getMaximumStepZWhenForwardAndDown(), epsilon);

      double wiggleInsideDeltaTarget = RandomNumbers.nextDouble(random, 10.0);
      parameters.setWiggleInsideDeltaTarget(wiggleInsideDeltaTarget);
      assertEquals(wiggleInsideDeltaTarget, parameters.getWiggleInsideDeltaTarget(), epsilon);

      double wiggleInsideDeltaMinimum = RandomNumbers.nextDouble(random, 10.0);
      parameters.setWiggleInsideDeltaMinimum(wiggleInsideDeltaMinimum);
      assertEquals(wiggleInsideDeltaMinimum, parameters.getWiggleInsideDeltaMinimum(), epsilon);

      double maximumStepReachWhenSteppingUp = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaximumStepReachWhenSteppingUp(maximumStepReachWhenSteppingUp);
      assertEquals(maximumStepReachWhenSteppingUp, parameters.getMaximumStepReachWhenSteppingUp(), epsilon);

      double maximumStepZWhenSteppingUp = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaximumStepZWhenSteppingUp(maximumStepZWhenSteppingUp);
      assertEquals(maximumStepZWhenSteppingUp, parameters.getMaximumStepZWhenSteppingUp(), epsilon);

      double minFootholdPercent = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinimumFootholdPercent(minFootholdPercent);
      assertEquals(minFootholdPercent, parameters.getMinimumFootholdPercent(), epsilon);

      double minSurfaceIncline = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMinimumSurfaceInclineRadians(minSurfaceIncline);
      assertEquals(minSurfaceIncline, parameters.getMinimumSurfaceInclineRadians(), epsilon);

      boolean wiggleWhilePlanning = random.nextBoolean();
      parameters.setWiggleWhilePlanning(wiggleWhilePlanning);
      assertEquals(wiggleWhilePlanning, parameters.getWiggleWhilePlanning());

      boolean enableConcaveHullWiggler = RandomNumbers.nextBoolean(random, 0.5);
      parameters.setEnableConcaveHullWiggler(enableConcaveHullWiggler);
      assertEquals(enableConcaveHullWiggler, parameters.getEnableConcaveHullWiggler());

      double maximumXYWiggleDistance = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaximumXYWiggleDistance(maximumXYWiggleDistance);
      assertEquals(maximumXYWiggleDistance, parameters.getMaximumXYWiggleDistance(), epsilon);

      double maximumYawWiggle = RandomNumbers.nextDouble(random, 10.00);
      parameters.setMaximumYawWiggle(maximumYawWiggle);
      assertEquals(maximumYawWiggle, parameters.getMaximumYawWiggle(), epsilon);

      double maxZpenetration = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximumZPenetrationOnValleyRegions(maxZpenetration);
      assertEquals(maxZpenetration, parameters.getMaximumZPenetrationOnValleyRegions(), epsilon);

      double cliffHeightToAvoid = RandomNumbers.nextDouble(random, 10.00);
      parameters.setCliffBaseHeightToAvoid(cliffHeightToAvoid);
      assertEquals(cliffHeightToAvoid, parameters.getCliffBaseHeightToAvoid());

      double minimumDistanceFromCliff = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliff);
      assertEquals(minimumDistanceFromCliff, parameters.getMinimumDistanceFromCliffBottoms(), epsilon);

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
      parameters.setNumberOfBoundingBoxChecks(numberOfBoundingBoxChecks);
      assertEquals(numberOfBoundingBoxChecks, parameters.getNumberOfBoundingBoxChecks());

      double aStarHeuristicWeight = RandomNumbers.nextDouble(random, 10.0);
      parameters.setAStarHeuristicsWeight(aStarHeuristicWeight);
      assertEquals(aStarHeuristicWeight, parameters.getAStarHeuristicsWeight().getValue(), epsilon);

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

      double maximumDistanceFromBoundingBox = RandomNumbers.nextDouble(random, 10.0);
      parameters.setMaximum2dDistanceFromBoundingBoxToPenalize(maximumDistanceFromBoundingBox);
      assertEquals(maximumDistanceFromBoundingBox, parameters.getMaximum2dDistanceFromBoundingBoxToPenalize(), epsilon);

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
