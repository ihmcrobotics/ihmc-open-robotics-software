package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;
import java.util.function.UnaryOperator;

public class FootstepNodeValidityChecker
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapper snapper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private final PlanarRegionBaseOfCliffAvoider cliffAvoider;
   private final ObstacleBetweenNodesChecker obstacleBetweenNodesChecker;
   private final FootstepNodeBodyCollisionDetector collisionDetector;
   private final GoodFootstepPositionChecker goodPositionChecker;

   private PlanarRegionsList planarRegionsList = null;
   private BipedalFootstepPlannerListener listener = null;

   public FootstepNodeValidityChecker(FootstepPlannerParametersReadOnly parameters,
                                      SideDependentList<ConvexPolygon2D> footPolygons,
                                      FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.footPolygons = footPolygons;
      this.cliffAvoider = new PlanarRegionBaseOfCliffAvoider(parameters, snapper, footPolygons);
      this.obstacleBetweenNodesChecker = new ObstacleBetweenNodesChecker(parameters, snapper);
      this.collisionDetector = new FootstepNodeBodyCollisionDetector(parameters);
      this.goodPositionChecker = new GoodFootstepPositionChecker(parameters, snapper);
   }

   public boolean isNodeValid(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      if (stanceNode != null && candidateNode.getRobotSide() == stanceNode.getRobotSide())
      {
         throw new RuntimeException(getClass().getSimpleName() + " received a pair stance and swing footsteps both on the " + candidateNode.getRobotSide() + " side");
      }

      BipedalFootstepPlannerNodeRejectionReason rejectionReason = isNodeValidInternal(candidateNode, stanceNode);
      if (listener != null && rejectionReason != null)
      {
         listener.rejectNode(candidateNode, stanceNode, rejectionReason);
      }

      return rejectionReason == null;
   }

   private BipedalFootstepPlannerNodeRejectionReason isNodeValidInternal(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      FootstepNodeSnapData candidateNodeSnapData = snapper.snapFootstepNode(candidateNode);

      // Check valid snap
      if (candidateNodeSnapData.getSnapTransform().containsNaN())
      {
         return BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP;
      }

      // Check incline
      RigidBodyTransform snappedSoleTransform = candidateNodeSnapData.getOrComputeSnappedNodeTransform(candidateNode);
      double minimumSurfaceNormalZ = Math.cos(parameters.getMinimumSurfaceInclineRadians());
      if (snappedSoleTransform.getM22() < minimumSurfaceNormalZ)
      {
         return BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP;
      }

      // Check snap area
      ConvexPolygon2D footholdAfterSnap = candidateNodeSnapData.getCroppedFoothold();
      double croppedFootArea = footholdAfterSnap.getArea();
      double fullFootArea = footPolygons.get(candidateNode.getRobotSide()).getArea();
      double footAreaPercentage = croppedFootArea / fullFootArea;

      double epsilonAreaPercentage = 1e-4;
      if (!footholdAfterSnap.isEmpty() && footAreaPercentage < (parameters.getMinimumFootholdPercent() - epsilonAreaPercentage))
      {
         return BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA;
      }

      // Check for ankle collision
      cliffAvoider.setPlanarRegions(planarRegionsList);
      if(!cliffAvoider.isNodeValid(candidateNode, null))
      {
         return BipedalFootstepPlannerNodeRejectionReason.AT_CLIFF_BOTTOM;
      }

      if(stanceNode == null)
      {
         return null;
      }

      // Check snapped footstep placement
      if (!goodPositionChecker.isNodeValid(candidateNode, stanceNode))
      {
         return goodPositionChecker.getRejectionReason();
      }

      // Check for obstacle collisions (vertically extruded line between steps)
      if (parameters.checkForPathCollisions())
      {
         obstacleBetweenNodesChecker.setPlanarRegions(planarRegionsList);
         try
         {
            if (!obstacleBetweenNodesChecker.isNodeValid(candidateNode, stanceNode))
            {
               return BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_BODY;
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
         FootstepNodeSnapData stanceNodeSnapData = snapper.snapFootstepNode(stanceNode);

         double candidateNodeHeight = FootstepNodeTools.getSnappedNodeHeight(candidateNode, candidateNodeSnapData.getSnapTransform());
         double stanceNodeHeight = FootstepNodeTools.getSnappedNodeHeight(stanceNode, stanceNodeSnapData.getSnapTransform());
         collisionDetector.setPlanarRegionsList(planarRegionsList);
         List<BodyCollisionData> collisionData = collisionDetector.checkForCollision(candidateNode,
                                                                                     stanceNode,
                                                                                     candidateNodeHeight,
                                                                                     stanceNodeHeight,
                                                                                     parameters.getNumberOfBoundingBoxChecks());
         for (int i = 0; i < collisionData.size(); i++)
         {
            if (collisionData.get(i).isCollisionDetected())
            {
               return BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY;
            }
         }
      }

      return null;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void setParentNodeSupplier(UnaryOperator<FootstepNode> parentNodeSupplier)
   {
      this.goodPositionChecker.setParentNodeSupplier(parentNodeSupplier);
   }

   public void setListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
   }

   public BipedalFootstepPlannerListener getListener()
   {
      return listener;
   }
}
