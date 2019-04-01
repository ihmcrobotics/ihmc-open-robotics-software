package us.ihmc.footstepPlanning;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepPlanningTestTools
{
   private final static double epsilon = 1e-7;

   public static FootstepPlannerParameters createRandomParameters(Random random)
   {
      FootstepPlannerParameters parameters = new FootstepPlannerParameters()
      {
         private final double idealWidth = RandomNumbers.nextDouble(random, 0.01, 1.0);

         @Override
         public double getIdealFootstepWidth()
         {
            return idealWidth;
         }

         private final double idealLength = RandomNumbers.nextDouble(random, 0.01, 1.0);

         @Override
         public double getIdealFootstepLength()
         {
            return idealLength;
         }

         private final double wiggleInsideDelta = RandomNumbers.nextDouble(random, 0.01, 1.0);

         @Override
         public double getWiggleInsideDelta()
         {
            return wiggleInsideDelta;
         }

         private final double maxReach = RandomNumbers.nextDouble(random, 0.01, 1.0);

         @Override
         public double getMaximumStepReach()
         {
            return maxReach;
         }

         private final double maxYaw = RandomNumbers.nextDouble(random, 0.01, Math.PI);

         @Override
         public double getMaximumStepYaw()
         {
            return maxYaw;
         }

         private final double minStepWidth = RandomNumbers.nextDouble(random, 0.0, 1.0);

         @Override
         public double getMinimumStepWidth()
         {
            return minStepWidth;
         }

         private final double minStepLength = RandomNumbers.nextDouble(random, 0.01, 1.0);

         @Override
         public double getMinimumStepLength()
         {
            return minStepLength;
         }

         private final double minStepYaw = RandomNumbers.nextDouble(random, 0.0, Math.PI);

         @Override
         public double getMinimumStepYaw()
         {
            return minStepYaw;
         }

         private final double maxStepReachWhenSteppingUp = RandomNumbers.nextDouble(random, 0.0, 0.5);

         @Override
         public double getMaximumStepReachWhenSteppingUp()
         {
            return maxStepReachWhenSteppingUp;
         }

         private final double maxStepZWhenSteppingUp = RandomNumbers.nextDouble(random, 0.0, 0.5);

         @Override
         public double getMaximumStepZWhenSteppingUp()
         {
            return maxStepZWhenSteppingUp;
         }

         private final double maxStepXForwardAndDown = RandomNumbers.nextDouble(random, 0.0, 0.5);

         @Override
         public double getMaximumStepXWhenForwardAndDown()
         {
            return maxStepXForwardAndDown;
         }

         private final double maxStepZForwardAndDown = RandomNumbers.nextDouble(random, 0.0, 5.0);

         @Override
         public double getMaximumStepZWhenForwardAndDown()
         {
            return maxStepZForwardAndDown;
         }

         private final double maxStepZ = RandomNumbers.nextDouble(random, 0.01, 1.5);

         @Override
         public double getMaximumStepZ()
         {
            return maxStepZ;
         }

         private final double minFootholdPercent = RandomNumbers.nextDouble(random, 0.0, 1.0);

         @Override
         public double getMinimumFootholdPercent()
         {
            return minFootholdPercent;
         }

         private final double minSurfaceIncline = RandomNumbers.nextDouble(random, 0.0, 2.0);

         @Override
         public double getMinimumSurfaceInclineRadians()
         {
            return minSurfaceIncline;
         }

         private final boolean wiggleInto = RandomNumbers.nextBoolean(random, 0.5);

         @Override
         public boolean getWiggleIntoConvexHullOfPlanarRegions()
         {
            return wiggleInto;
         }

         private final boolean rejectIfNoWiggle = RandomNumbers.nextBoolean(random, 0.5);

         @Override
         public boolean getRejectIfCannotFullyWiggleInside()
         {
            return rejectIfNoWiggle;
         }

         private final double maxXYWiggle = RandomNumbers.nextDouble(random, 0.1, 1.5);

         @Override
         public double getMaximumXYWiggleDistance()
         {
            return maxXYWiggle;
         }

         private final double maxYawWiggle = RandomNumbers.nextDouble(random, 0.1, Math.PI);

         @Override
         public double getMaximumYawWiggle()
         {
            return maxYawWiggle;
         }

         private final double maxZPenetration = RandomNumbers.nextDouble(random, 0.05, 0.4);

         @Override
         public double getMaximumZPenetrationOnValleyRegions()
         {
            return maxZPenetration;
         }

         private final double maxStepWidth = RandomNumbers.nextDouble(random, 0.1, 0.5);

         @Override
         public double getMaximumStepWidth()
         {
            return maxStepWidth;
         }

         private final double cliffHeightToAvoid = RandomNumbers.nextDouble(random, 0.01, 1.0);

         @Override
         public double getCliffHeightToAvoid()
         {
            return cliffHeightToAvoid;
         }

         private final double minDistanceFromCliff = RandomNumbers.nextDouble(random, 0.05, 1.0);

         @Override
         public double getMinimumDistanceFromCliffBottoms()
         {
            return minDistanceFromCliff;
         }

         private final boolean returnBestEffort = RandomNumbers.nextBoolean(random, 0.5);

         @Override
         public boolean getReturnBestEffortPlan()
         {
            return returnBestEffort;
         }

         private final int minSteps = RandomNumbers.nextInt(random, 1, 10);

         @Override
         public int getMinimumStepsForBestEffortPlan()
         {
            return minSteps;
         }

         private final double bodyBoxHeight = RandomNumbers.nextDouble(random, 0.1, 0.5);

         @Override
         public double getBodyBoxHeight()
         {
            return bodyBoxHeight;
         }

         private final double bodyGroundClearance = RandomNumbers.nextDouble(random, 0.1, 0.5);

         @Override
         public double getBodyGroundClearance()
         {
            return bodyGroundClearance;
         }

         private final double bodyBoxDepth = RandomNumbers.nextDouble(random, 0.1, 0.5);

         @Override
         public double getBodyBoxDepth()
         {
            return bodyBoxDepth;
         }

         private final double bodyBoxWidth = RandomNumbers.nextDouble(random, 0.1, 0.5);

         @Override
         public double getBodyBoxWidth()
         {
            return bodyBoxWidth;
         }

         private final double bodyBoxBaseX = RandomNumbers.nextDouble(random, 0.1, 0.5);

         @Override
         public double getBodyBoxBaseX()
         {
            return bodyBoxBaseX;
         }

         private final double bodyBoxBaseY = RandomNumbers.nextDouble(random, 0.1, 0.5);

         @Override
         public double getBodyBoxBaseY()
         {
            return bodyBoxBaseY;
         }

         private final double bodyBoxBaseZ = RandomNumbers.nextDouble(random, 0.1, 0.5);

         @Override
         public double getBodyBoxBaseZ()
         {
            return bodyBoxBaseZ;
         }

         private final boolean checkForBodyCollisions = random.nextBoolean();

         @Override
         public boolean checkForBodyBoxCollisions()
         {
            return checkForBodyCollisions;
         }

         private final boolean performHeuristicSearchPolicies = random.nextBoolean();

         @Override
         public boolean performHeuristicSearchPolicies()
         {
            return performHeuristicSearchPolicies;
         }

         private final double minXClearance = RandomNumbers.nextDouble(random, 0.01, 1.0);

         @Override
         public double getMinXClearanceFromStance()
         {
            return minXClearance;
         }

         private final double minYClearance = RandomNumbers.nextDouble(random, 0.01, 1.0);

         @Override
         public double getMinYClearanceFromStance()
         {
            return minYClearance;
         }

         private final FootstepPlannerCostParameters costParameters = getRandomCostParameters(random);

         @Override
         public FootstepPlannerCostParameters getCostParameters()
         {
            return costParameters;
         }

      };

      return parameters;
   }

   public static FootstepPlannerCostParameters getRandomCostParameters(Random random)
   {
      return new FootstepPlannerCostParameters()
      {
         private final boolean useQuadraticDistanceCost = RandomNumbers.nextBoolean(random, 0.5);

         @Override
         public boolean useQuadraticDistanceCost()
         {
            return useQuadraticDistanceCost;
         }

         private final boolean useQuadraticHeightCost = RandomNumbers.nextBoolean(random, 0.5);

         @Override
         public boolean useQuadraticHeightCost()
         {
            return useQuadraticHeightCost;
         }

         private final double aStarHeuristicsWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);
         private final double visGraphWithAStarHeuristicsWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);
         private final double depthFirstHeuristicsWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);
         private final double bodyPathBasedHeuristicsWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);

         @Override
         public DoubleProvider getAStarHeuristicsWeight()
         {
            return () -> aStarHeuristicsWeight;
         }

         @Override
         public DoubleProvider getVisGraphWithAStarHeuristicsWeight()
         {
            return () -> visGraphWithAStarHeuristicsWeight;
         }

         @Override
         public DoubleProvider getDepthFirstHeuristicsWeight()
         {
            return () -> depthFirstHeuristicsWeight;
         }

         @Override
         public DoubleProvider getBodyPathBasedHeuristicsWeight()
         {
            return () -> bodyPathBasedHeuristicsWeight;
         }

         private final double yawWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);

         @Override
         public double getYawWeight()
         {
            return yawWeight;
         }

         private final double forwardWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);

         @Override
         public double getForwardWeight()
         {
            return forwardWeight;
         }

         private final double lateralWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);

         @Override
         public double getLateralWeight()
         {
            return lateralWeight;
         }

         private final double costPerStep = RandomNumbers.nextDouble(random, 0.01, 10.0);

         @Override
         public double getCostPerStep()
         {
            return costPerStep;
         }

         private final double stepUpWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);

         @Override
         public double getStepUpWeight()
         {
            return stepUpWeight;
         }

         private final double stepDownWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);

         @Override
         public double getStepDownWeight()
         {
            return stepDownWeight;
         }

         private final double rollWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);

         @Override
         public double getRollWeight()
         {
            return rollWeight;
         }

         private final double pitchWeight = RandomNumbers.nextDouble(random, 0.01, 10.0);

         @Override
         public double getPitchWeight()
         {
            return pitchWeight;
         }
      };
   }


   public static void assertCostParametersEqual(FootstepPlannerCostParameters parameters, FootstepPlannerCostParameters other)
   {
      assertEquals(parameters.useQuadraticDistanceCost(), other.useQuadraticDistanceCost());
      assertEquals(parameters.useQuadraticHeightCost(), other.useQuadraticHeightCost());
      assertEquals(parameters.getAStarHeuristicsWeight().getValue(), other.getAStarHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getVisGraphWithAStarHeuristicsWeight().getValue(), other.getVisGraphWithAStarHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getDepthFirstHeuristicsWeight().getValue(), other.getDepthFirstHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getBodyPathBasedHeuristicsWeight().getValue(), other.getBodyPathBasedHeuristicsWeight().getValue(), epsilon);
      assertEquals(parameters.getYawWeight(), other.getYawWeight(), epsilon);
      assertEquals(parameters.getForwardWeight(), other.getForwardWeight(), epsilon);
      assertEquals(parameters.getLateralWeight(), other.getLateralWeight(), epsilon);
      assertEquals(parameters.getCostPerStep(), other.getCostPerStep(), epsilon);
      assertEquals(parameters.getStepUpWeight(), other.getStepUpWeight(), epsilon);
      assertEquals(parameters.getStepDownWeight(), other.getStepDownWeight(), epsilon);
      assertEquals(parameters.getRollWeight(), other.getRollWeight(), epsilon);
      assertEquals(parameters.getPitchWeight(), other.getPitchWeight(), epsilon);
   }

   public static void assertParametersEqual(FootstepPlannerParameters parameters, FootstepPlannerParameters other)
   {
      assertEquals(parameters.checkForBodyBoxCollisions(), other.checkForBodyBoxCollisions());
      assertEquals(parameters.performHeuristicSearchPolicies(), other.performHeuristicSearchPolicies());
      assertEquals(parameters.getIdealFootstepWidth(), other.getIdealFootstepWidth(), epsilon);
      assertEquals(parameters.getIdealFootstepLength(), other.getIdealFootstepLength(), epsilon);
      assertEquals(parameters.getWiggleInsideDelta(), other.getWiggleInsideDelta(), epsilon);
      assertEquals(parameters.getMaximumStepReach(), other.getMaximumStepReach(), epsilon);
      assertEquals(parameters.getMaximumStepYaw(), other.getMaximumStepYaw(), epsilon);
      assertEquals(parameters.getMinimumStepWidth(), other.getMinimumStepWidth(), epsilon);
      assertEquals(parameters.getMinimumStepLength(), other.getMinimumStepLength(), epsilon);
      assertEquals(parameters.getMinimumStepYaw(), other.getMinimumStepYaw(), epsilon);
      assertEquals(parameters.getMaximumStepReachWhenSteppingUp(), other.getMaximumStepReachWhenSteppingUp(), epsilon);
      assertEquals(parameters.getMaximumStepZWhenSteppingUp(), other.getMaximumStepZWhenSteppingUp(), epsilon);
      assertEquals(parameters.getMaximumStepXWhenForwardAndDown(), other.getMaximumStepXWhenForwardAndDown(), epsilon);
      assertEquals(parameters.getMaximumStepZWhenForwardAndDown(), other.getMaximumStepZWhenForwardAndDown(), epsilon);
      assertEquals(parameters.getMaximumStepZ(), other.getMaximumStepZ(), epsilon);
      assertEquals(parameters.getMinimumFootholdPercent(), other.getMinimumFootholdPercent(), epsilon);
      assertEquals(parameters.getMinimumSurfaceInclineRadians(), other.getMinimumSurfaceInclineRadians(), epsilon);
      assertEquals(parameters.getWiggleIntoConvexHullOfPlanarRegions(), other.getWiggleIntoConvexHullOfPlanarRegions());
      assertEquals(parameters.getRejectIfCannotFullyWiggleInside(), other.getRejectIfCannotFullyWiggleInside());
      assertEquals(parameters.getMaximumXYWiggleDistance(), other.getMaximumXYWiggleDistance(), epsilon);
      assertEquals(parameters.getMaximumYawWiggle(), other.getMaximumYawWiggle(), epsilon);
      assertEquals(parameters.getMaximumZPenetrationOnValleyRegions(), other.getMaximumZPenetrationOnValleyRegions(), epsilon);
      assertEquals(parameters.getMaximumStepWidth(), other.getMaximumStepWidth(), epsilon);
      assertEquals(parameters.getCliffHeightToAvoid(), other.getCliffHeightToAvoid(), epsilon);
      assertEquals(parameters.getMinimumDistanceFromCliffBottoms(), other.getMinimumDistanceFromCliffBottoms(), epsilon);
      assertEquals(parameters.getReturnBestEffortPlan(), other.getReturnBestEffortPlan());
      assertEquals(parameters.getMinimumStepsForBestEffortPlan(), other.getMinimumStepsForBestEffortPlan());
      assertEquals(parameters.getBodyGroundClearance(), other.getBodyGroundClearance(), epsilon);
      assertEquals(parameters.getBodyBoxHeight(), other.getBodyBoxHeight(), epsilon);
      assertEquals(parameters.getBodyBoxDepth(), other.getBodyBoxDepth(), epsilon);
      assertEquals(parameters.getBodyBoxWidth(), other.getBodyBoxWidth(), epsilon);
      assertEquals(parameters.getBodyBoxBaseX(), other.getBodyBoxBaseX(), epsilon);
      assertEquals(parameters.getBodyBoxBaseY(), other.getBodyBoxBaseY(), epsilon);
      assertEquals(parameters.getBodyBoxBaseZ(), other.getBodyBoxBaseZ(), epsilon);
      assertEquals(parameters.getMinXClearanceFromStance(), other.getMinXClearanceFromStance(), epsilon);
      assertEquals(parameters.getMinYClearanceFromStance(), other.getMinYClearanceFromStance(), epsilon);

      assertCostParametersEqual(parameters.getCostParameters(), other.getCostParameters());
   }
}
