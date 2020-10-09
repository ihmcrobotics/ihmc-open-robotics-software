package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
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

public class FootstepNodeChecker implements FootstepNodeCheckerInterface
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapAndWiggler snapper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private final PlanarRegionCliffAvoider cliffAvoider;
   private final ObstacleBetweenNodesChecker obstacleBetweenNodesChecker;
   private final FootstepNodeBodyCollisionDetector collisionDetector;
   private final FootstepPoseChecker goodPositionChecker;

   private PlanarRegionsList planarRegionsList = null;

   private final FootstepNodeSnapData candidateNodeSnapData = FootstepNodeSnapData.identityData();
   private final YoEnum<BipedalFootstepPlannerNodeRejectionReason> rejectionReason = new YoEnum<>("rejectionReason", "", registry, BipedalFootstepPlannerNodeRejectionReason.class, true);
   private final YoDouble footAreaPercentage = new YoDouble("footAreaPercentage", registry);
   private final YoInteger footstepIndex = new YoInteger("footstepIndex", registry);
   private final YoDouble achievedDeltaInside = new YoDouble("achievedDeltaInside", registry);

   private final List<CustomNodeChecker> customNodeCheckers = new ArrayList<>();

   public FootstepNodeChecker(FootstepPlannerParametersReadOnly parameters,
                              SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapAndWiggler snapper, YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.footPolygons = footPolygons;
      this.cliffAvoider = new PlanarRegionCliffAvoider(parameters, snapper, footPolygons);
      this.obstacleBetweenNodesChecker = new ObstacleBetweenNodesChecker(parameters, snapper);
      this.collisionDetector = new FootstepNodeBodyCollisionDetector(parameters);
      this.goodPositionChecker = new FootstepPoseChecker(parameters, snapper, registry);
      parentRegistry.addChild(registry);
   }

   @Override
   public boolean isNodeValid(FootstepNode candidateStep, FootstepNode stanceStep, FootstepNode startOfSwing)
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

   private void doValidityCheck(FootstepNode candidateStep, FootstepNode stanceStep, FootstepNode startOfSwing)
   {
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(candidateStep, stanceStep, parameters.getWiggleWhilePlanning());
      candidateNodeSnapData.set(snapData);
      goodPositionChecker.setApproximateStepDimensions(candidateStep, stanceStep);
      achievedDeltaInside.set(snapData.getAchievedInsideDelta());

      if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return;
      }

      // Check valid snap
      if (candidateNodeSnapData.getSnapTransform().containsNaN())
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
      RigidBodyTransform snappedSoleTransform = candidateNodeSnapData.getSnappedNodeTransform(candidateStep);
      double minimumSurfaceNormalZ = Math.cos(parameters.getMinimumSurfaceInclineRadians());
      if (snappedSoleTransform.getM22() < minimumSurfaceNormalZ)
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP);
         return;
      }

      // Check snap area
      ConvexPolygon2D footholdAfterSnap = candidateNodeSnapData.getCroppedFoothold();
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
      if(!cliffAvoider.isNodeValid(candidateStep))
      {
         rejectionReason.set(BipedalFootstepPlannerNodeRejectionReason.AT_CLIFF_BOTTOM);
         return;
      }

      if (stanceStep == null)
      {
         return;
      }

      // Check snapped footstep placement
      BipedalFootstepPlannerNodeRejectionReason poseRejectionReason = goodPositionChecker.checkStepValidity(candidateStep, stanceStep);
      if (poseRejectionReason != null)
      {
         rejectionReason.set(poseRejectionReason);
         return;
      }

      // Check for obstacle collisions (vertically extruded line between steps)
      if (parameters.checkForPathCollisions())
      {
         obstacleBetweenNodesChecker.setPlanarRegions(planarRegionsList);
         try
         {
            if (!obstacleBetweenNodesChecker.isNodeValid(candidateStep, stanceStep))
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

      for (CustomNodeChecker customNodeChecker : customNodeCheckers)
      {
         if (!customNodeChecker.isNodeValid(candidateStep, stanceStep))
         {
            rejectionReason.set(customNodeChecker.getRejectionReason());
            return;
         }
      }
   }

   private boolean boundingBoxCollisionDetected(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      FootstepNodeSnapData stanceNodeSnapData = snapper.snapFootstepNode(stanceNode, null, parameters.getWiggleWhilePlanning());
      if (stanceNodeSnapData == null)
      {
         return false;
      }

      double candidateNodeHeight = FootstepNodeTools.getSnappedNodeHeight(candidateNode, candidateNodeSnapData.getSnapTransform());
      double stanceNodeHeight = FootstepNodeTools.getSnappedNodeHeight(stanceNode, stanceNodeSnapData.getSnapTransform());
      List<BodyCollisionData> collisionData = collisionDetector.checkForCollision(candidateNode,
                                                                                  stanceNode,
                                                                                  candidateNodeHeight,
                                                                                  stanceNodeHeight,
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

      candidateNodeSnapData.clear();
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

   public void attachCustomNodeChecker(CustomNodeChecker customNodeChecker)
   {
      customNodeCheckers.add(customNodeChecker);
   }
}
