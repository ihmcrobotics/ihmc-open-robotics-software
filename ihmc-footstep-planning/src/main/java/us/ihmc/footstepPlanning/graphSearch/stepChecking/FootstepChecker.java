package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepPlannerBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class FootstepChecker implements FootstepCheckerInterface
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepSnapAndWiggler snapper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private final PlanarRegionCliffAvoider cliffAvoider;
   private final ObstacleBetweenStepsChecker obstacleBetweenStepsChecker;
   private final FootstepPlannerBodyCollisionDetector collisionDetector;
   private final FootstepPoseChecker goodPositionChecker;

   private PlanarRegionsList planarRegionsList = null;

   private final FootstepSnapData candidateStepSnapData = FootstepSnapData.identityData();
   private final YoEnum<BipedalFootstepPlannerNodeRejectionReason> rejectionReason = new YoEnum<>("rejectionReason", "", registry, BipedalFootstepPlannerNodeRejectionReason.class, true);
   private final YoDouble footAreaPercentage = new YoDouble("footAreaPercentage", registry);
   private final YoInteger footstepIndex = new YoInteger("footstepIndex", registry);
   private final YoDouble achievedDeltaInside = new YoDouble("achievedDeltaInside", registry);

   private final List<CustomFootstepChecker> customFootstepCheckers = new ArrayList<>();

   public FootstepChecker(FootstepPlannerParametersReadOnly parameters,
                          SideDependentList<ConvexPolygon2D> footPolygons,
                          FootstepSnapAndWiggler snapper,
                          YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.footPolygons = footPolygons;
      this.cliffAvoider = new PlanarRegionCliffAvoider(parameters, snapper, footPolygons);
      this.obstacleBetweenStepsChecker = new ObstacleBetweenStepsChecker(parameters, snapper);
      this.collisionDetector = new FootstepPlannerBodyCollisionDetector(parameters);
      this.goodPositionChecker = new FootstepPoseChecker(parameters, snapper, registry);
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
      FootstepSnapData snapData = snapper.snapFootstep(candidateStep, stanceStep, parameters.getWiggleWhilePlanning());
      candidateStepSnapData.set(snapData);
      goodPositionChecker.setApproximateStepDimensions(candidateStep, stanceStep);
      achievedDeltaInside.set(snapData.getAchievedInsideDelta());

      if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return;
      }

      // Check valid snap
      if (candidateStepSnapData.getSnapTransform().containsNaN())
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return;
      }

      // Check wiggle parameters satisfied
      if (parameters.getWiggleWhilePlanning())
      {
         checkWiggleParameters(parameters);
         if (snapData.getAchievedInsideDelta() < parameters.getWiggleInsideDeltaMinimum())
         {
            rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.WIGGLE_CONSTRAINT_NOT_MET);
            return;
         }
      }

      // Check incline
      RigidBodyTransform snappedSoleTransform = candidateStepSnapData.getSnappedStepTransform(candidateStep);
      double minimumSurfaceNormalZ = Math.cos(parameters.getMinimumSurfaceInclineRadians());
      if (snappedSoleTransform.getM22() < minimumSurfaceNormalZ)
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);
         return;
      }

      // Check snap area
      ConvexPolygon2D footholdAfterSnap = candidateStepSnapData.getCroppedFoothold();
      double croppedFootArea = footholdAfterSnap.getArea();
      double fullFootArea = footPolygons.get(candidateStep.getRobotSide()).getArea();
      footAreaPercentage.set(croppedFootArea / fullFootArea);

      double epsilonAreaPercentage = 1e-4;
      if (!footholdAfterSnap.isEmpty() && footAreaPercentage.getValue() < (parameters.getMinimumFootholdPercent() - epsilonAreaPercentage))
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
         return;
      }

      // Check for ankle collision
      cliffAvoider.setPlanarRegionsList(planarRegionsList);
      if(!cliffAvoider.isStepValid(candidateStep))
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.AT_CLIFF_BOTTOM);
         return;
      }

      if (stanceStep == null)
      {
         return;
      }

      // Check snapped footstep placement
      BipedalFootstepPlannerNodeRejectionReason poseRejectionReason = goodPositionChecker.checkStepValidity(candidateStep, stanceStep, startOfSwing);
      if (poseRejectionReason != null)
      {
         rejectionReason.set(poseRejectionReason);
         return;
      }

      // Check for obstacle collisions (vertically extruded line between steps)
      if (parameters.checkForPathCollisions())
      {
         obstacleBetweenStepsChecker.setPlanarRegions(planarRegionsList);
         try
         {
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
      if (parameters.checkForBodyBoxCollisions())
      {
         if (boundingBoxCollisionDetected(candidateStep, stanceStep))
         {
            rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY);
            return;
         }
      }

      for (CustomFootstepChecker customFootstepChecker : customFootstepCheckers)
      {
         if (!customFootstepChecker.isStepValid(candidateStep, stanceStep))
         {
            rejectionReason.set(customFootstepChecker.getRejectionReason());
            return;
         }
      }
   }

   private boolean boundingBoxCollisionDetected(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep)
   {
      FootstepSnapData stanceStepSnapData = snapper.snapFootstep(stanceStep, null, parameters.getWiggleWhilePlanning());
      if (stanceStepSnapData == null)
      {
         return false;
      }

      double candidateStepHeight = DiscreteFootstepTools.getSnappedStepHeight(candidateStep, candidateStepSnapData.getSnapTransform());
      double stanceStepHeight = DiscreteFootstepTools.getSnappedStepHeight(stanceStep, stanceStepSnapData.getSnapTransform());
      List<BodyCollisionData> collisionData = collisionDetector.checkForCollision(candidateStep,
                                                                                  stanceStep,
                                                                                  candidateStepHeight,
                                                                                  stanceStepHeight,
                                                                                  parameters.getNumberOfBoundingBoxChecks());
      for (int i = 0; i < collisionData.size(); i++)
      {
         if (collisionData.get(i).isCollisionDetected())
         {
            return true;
         }
      }

      return false;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      collisionDetector.setPlanarRegionsList(planarRegionsList);
   }

   private void clearLoggedVariables()
   {
      footAreaPercentage.setToNaN();
      rejectionReason.set(null);
      footstepIndex.set(-1);
      achievedDeltaInside.setToNaN();

      candidateStepSnapData.clear();
      goodPositionChecker.clearLoggedVariables();
   }

   private static void checkWiggleParameters(FootstepPlannerParametersReadOnly parameters)
   {
      double epsilon = 1e-7;
      if (parameters.getWiggleInsideDeltaMinimum() > parameters.getWiggleInsideDeltaTarget() + epsilon)
      {
         throw new RuntimeException(
               "Illegal wiggle parameters, target should be greater or equal to minimum. Target: " + parameters.getWiggleInsideDeltaTarget() + ", Minimum: "
               + parameters.getWiggleInsideDeltaMinimum());
      }
   }

   public void attachCustomFootstepChecker(CustomFootstepChecker customFootstepChecker)
   {
      customFootstepCheckers.add(customFootstepChecker);
   }
}
