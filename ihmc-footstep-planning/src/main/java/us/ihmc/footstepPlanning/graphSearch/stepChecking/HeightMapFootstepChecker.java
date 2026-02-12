package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepPlannerBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.perception.gpuMapping.SnapResult;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class HeightMapFootstepChecker implements FootstepCheckerInterface
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   public static final String rejectionReasonVariable = "rejectionReason";

   private static final double traversabilityThresholdCenter = 0.08;
   private static final double traversabilityThresholdPerimeter = 0.02;

   private final DefaultFootstepPlannerParametersReadOnly parameters;
   final FootstepSnapperReadOnly snapper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private TerrainMapData terrainMapData;

   private final ObstacleBetweenStepsChecker obstacleBetweenStepsChecker;
   private final FootstepPlannerBodyCollisionDetector collisionDetector;
   private final FootstepPoseHeuristicChecker heuristicPoseChecker;
   private final FootstepPoseReachabilityChecker reachabilityChecker;

   private boolean assumeFlatGround = false;

   private final FootstepSnapData candidateStepSnapData = FootstepSnapData.identityData();
   private final YoEnum<BipedalFootstepPlannerNodeRejectionReason> rejectionReason = new YoEnum<>(rejectionReasonVariable, "", registry, BipedalFootstepPlannerNodeRejectionReason.class, true);
   private final YoDouble footAreaPercentage = new YoDouble("footAreaPercentage", registry);
   private final YoInteger footstepIndex = new YoInteger("footstepIndex", registry);
   private final YoDouble achievedDeltaInside = new YoDouble("achievedDeltaInside", registry);
   private final YoDouble rmsError = new YoDouble("rmsError", registry);

   private final List<CustomFootstepChecker> customFootstepCheckers = new ArrayList<>();

   public HeightMapFootstepChecker(DefaultFootstepPlannerParametersReadOnly parameters,
                                   SideDependentList<ConvexPolygon2D> footPolygons,
                                   FootstepSnapperReadOnly snapper,
                                   StepReachabilityData stepReachabilityData,
                                   YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.footPolygons = footPolygons;
      this.obstacleBetweenStepsChecker = new ObstacleBetweenStepsChecker(parameters, snapper);
      this.collisionDetector = new FootstepPlannerBodyCollisionDetector(parameters);
      this.heuristicPoseChecker = new FootstepPoseHeuristicChecker(parameters, snapper, registry);
      this.reachabilityChecker = new FootstepPoseReachabilityChecker(parameters, snapper, stepReachabilityData, registry);
      parentRegistry.addChild(registry);
   }

   @Override
   public boolean isStepValid(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep, DiscreteFootstep startOfSwing)
   {
      if (stanceStep != null && candidateStep.getRobotSide() == stanceStep.getRobotSide())
      {
         throw new RuntimeException(getClass().getSimpleName() + " stance and next steps have the same side");
      }
      if (startOfSwing != null && candidateStep.getRobotSide() != startOfSwing.getRobotSide())
      {
         throw new RuntimeException(getClass().getSimpleName() + " start of swing and touchdown steps have different sides");
      }

      clearLoggedVariables();
      doValidityCheck(candidateStep, stanceStep, startOfSwing);

      return rejectionReason.getValue() == null;
   }

   // TODO compute step index

   private void doValidityCheck(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep, DiscreteFootstep startOfSwing)
   {
      // Do pre-snap validity checks here -- any validity checks before snapping will have big speedups

      FootstepSnapDataReadOnly snapData = snapper.snapFootstep(candidateStep, stanceStep, parameters.getWiggleWhilePlanning());
      candidateStepSnapData.set(snapData);
      heuristicPoseChecker.setApproximateStepDimensions(candidateStep, stanceStep);
      achievedDeltaInside.set(snapData.getAchievedInsideDelta());

      // Check the quality of the snap data.
      if (!doValidityCheckForSnap(candidateStep))
         return;

      if (terrainMapData != null)
      {
         // Check height map rejection reasons
         BipedalFootstepPlannerNodeRejectionReason poseRejectionReason = doValidityCheckForTerrainMap(candidateStep);

         if (poseRejectionReason != null)
         {
            rejectionReason.set(poseRejectionReason);
            return;
         }
      }

      // Check step placement
      if (!assumeFlatGround && !isStepPlacementValid())
      {
         return;
      }

      BipedalFootstepPlannerNodeRejectionReason poseRejectionReason;
      // Check snapped footstep placement
      if (parameters.getUseReachabilityMap())
      {
         poseRejectionReason = reachabilityChecker.checkStepValidity(candidateStep, stanceStep);
      }
      else
      {
         poseRejectionReason = heuristicPoseChecker.snapAndCheckValidity(candidateStep, stanceStep, startOfSwing);
      }
      if (poseRejectionReason != null)
      {
         rejectionReason.set(poseRejectionReason);
         return;
      }

      // Programmatically added custom checks
      for (CustomFootstepChecker customFootstepChecker : customFootstepCheckers)
      {
         if (!customFootstepChecker.isStepValid(candidateStep, stanceStep))
         {
            rejectionReason.set(customFootstepChecker.getRejectionReason());
            return;
         }
      }

      // Check collisions
      isCollisionFree(candidateStep, stanceStep, startOfSwing);
   }

   private BipedalFootstepPlannerNodeRejectionReason doValidityCheckForTerrainMap(DiscreteFootstep candidateStep)
   {
      SnapResult snapResult = terrainMapData.getTraversabilityClass(candidateStep.getX(), candidateStep.getY());
      if (snapResult == null)
         return null;

      return switch (snapResult)
            {
               case SQUARED_ERROR -> BipedalFootstepPlannerNodeRejectionReason.RMS_ERROR_TOO_HIGH;
               case TOO_STEEP -> BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP;
               case NOT_ENOUGH_AREA -> BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA;
               case SNAP_FAILED -> BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP;
               default -> null;
            };
   }


   private boolean doValidityCheckForSnap(DiscreteFootstep candidateStep)
   {
      // Area
      double areaFraction = candidateStepSnapData.getSnapAreaFraction();
      if (Double.isFinite(areaFraction))
         footAreaPercentage.set(areaFraction);
      else
         footAreaPercentage.set(1.0);

      double epsilonAreaPercentage = 1e-4;
      if (footAreaPercentage.getValue() < (parameters.getMinFootholdPercent() - epsilonAreaPercentage))
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
         return false;
      }

      // Root-mean-squared error
      rmsError.set(candidateStepSnapData.getSnapRMSError());
      if (candidateStepSnapData.getSnapRMSError() > parameters.getRMSErrorThreshold())
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.RMS_ERROR_TOO_HIGH);
         return false;
      }

      return true;
   }

   private boolean isStepPlacementValid()
   {
      // Check valid snap
      if (candidateStepSnapData.getSnapTransform().containsNaN())
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return false;
      }

      return true;
   }

   private void isCollisionFree(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep, DiscreteFootstep startOfSwing)
   {
      if (stanceStep == null)
      {
         return;
      }

      // Check for obstacle collisions (vertically extruded line between steps)
      if (parameters.getCheckForPathCollisions())
      {
         try
         {
            obstacleBetweenStepsChecker.setTerrainMapData(terrainMapData);

            if (!obstacleBetweenStepsChecker.isFootstepValid(candidateStep, stanceStep))
            {
               rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_BODY);
               return;
            }
         }
         catch(Exception e)
         {
            e.printStackTrace();
         }
      }

      // Check for bounding box collisions
      if (parameters.getCheckForBodyBoxCollisions())
      {
         if (boundingBoxCollisionDetected(candidateStep, stanceStep))
         {
            rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY);
         }
      }
   }

   private boolean boundingBoxCollisionDetected(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep)
   {
      FootstepSnapDataReadOnly stanceStepSnapData = snapper.snapFootstep(stanceStep, null, parameters.getWiggleWhilePlanning());
      if (stanceStepSnapData == null)
      {
         return false;
      }

      double candidateStepHeight = DiscreteFootstepTools.getSnappedStepHeight(candidateStep, candidateStepSnapData.getSnapTransform());
      double stanceStepHeight = DiscreteFootstepTools.getSnappedStepHeight(stanceStep, stanceStepSnapData.getSnapTransform());
      collisionDetector.setTerrainMapData(terrainMapData);
      boolean collisionDetected = collisionDetector.checkForCollision(candidateStep,
                                                                      stanceStep,
                                                                      candidateStepHeight,
                                                                      stanceStepHeight,
                                                                      parameters.getIntermediateBodyBoxChecks());
      return collisionDetected;
   }

   private void clearLoggedVariables()
   {
      footAreaPercentage.setToNaN();
      rejectionReason.set(null);
      footstepIndex.set(-1);
      achievedDeltaInside.setToNaN();

      candidateStepSnapData.clear();
      heuristicPoseChecker.clearLoggedVariables();
   }

   public void setAssumeFlatGround(boolean assumeFlatGround)
   {
      this.assumeFlatGround = assumeFlatGround;
   }

   private static void checkWiggleParameters(DefaultFootstepPlannerParametersReadOnly parameters)
   {
      double epsilon = 1e-7;
      if (parameters.getWiggleInsideDeltaMinimum() > parameters.getWiggleInsideDeltaTarget() + epsilon)
      {
         throw new RuntimeException(
               "Illegal wiggle parameters, target should be greater or equal to minimum. Target: " + parameters.getWiggleInsideDeltaTarget() + ", Minimum: "
               + parameters.getWiggleInsideDeltaMinimum());
      }
   }

   public void clearCustomFootstepCheckers()
   {
      customFootstepCheckers.clear();
   }

   public void attachCustomFootstepChecker(CustomFootstepChecker customFootstepChecker)
   {
      customFootstepCheckers.add(customFootstepChecker);
   }

   public void setTerrainMapData(TerrainMapData terrainMapData)
   {
      this.terrainMapData = terrainMapData;
   }

   public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
   {
      return rejectionReason.getValue();
   }
}
