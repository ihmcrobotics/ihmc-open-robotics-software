package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;
import java.util.function.UnaryOperator;

public class FootstepNodeChecker
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepNodeSnapAndWiggler snapper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepPlannerEdgeData edgeData;

   private final PlanarRegionCliffAvoider cliffAvoider;
   private final ObstacleBetweenNodesChecker obstacleBetweenNodesChecker;
   private final FootstepNodeBodyCollisionDetector collisionDetector;
   private final GoodFootstepPositionChecker goodPositionChecker;

   private PlanarRegionsList planarRegionsList = null;

   // Variables to log
   private final FootstepNodeSnapData candidateNodeSnapData = FootstepNodeSnapData.identityData();
   private double footAreaPercentage;
   private int footstepIndex = -1;

   public FootstepNodeChecker(FootstepPlannerParametersReadOnly parameters,
                              SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapAndWiggler snapper, FootstepPlannerEdgeData edgeData)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.edgeData = edgeData;
      this.footPolygons = footPolygons;
      this.cliffAvoider = new PlanarRegionCliffAvoider(parameters, snapper, footPolygons);
      this.obstacleBetweenNodesChecker = new ObstacleBetweenNodesChecker(parameters, snapper);
      this.collisionDetector = new FootstepNodeBodyCollisionDetector(parameters);
      this.goodPositionChecker = new GoodFootstepPositionChecker(parameters, snapper, edgeData);
   }

   public boolean isNodeValid(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      if (stanceNode != null && candidateNode.getRobotSide() == stanceNode.getRobotSide())
      {
         throw new RuntimeException(getClass().getSimpleName() + " received a pair stance and swing footsteps both on the " + candidateNode.getRobotSide() + " side");
      }

      clearLoggedVariables();
      BipedalFootstepPlannerNodeRejectionReason rejectionReason = isNodeValidInternal(candidateNode, stanceNode);
      logVariables(candidateNode, stanceNode, rejectionReason);

      return rejectionReason == null;
   }

   public void onIterationStart(FootstepNode footstepNode)
   {
      footstepIndex = footstepNode.getChildNodes().size() - 1;
   }

   private BipedalFootstepPlannerNodeRejectionReason isNodeValidInternal(FootstepNode candidateNode, FootstepNode stanceNode)
   {
      footstepIndex++;

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(candidateNode, stanceNode, parameters.getWiggleWhilePlanning());
      candidateNodeSnapData.set(snapData);

      if (planarRegionsList == null || planarRegionsList.isEmpty())
      {
         return null;
      }

      // Check valid snap
      if (candidateNodeSnapData.getSnapTransform().containsNaN())
      {
         return BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP;
      }

      // Check incline
      RigidBodyTransform snappedSoleTransform = candidateNodeSnapData.getSnappedNodeTransform(candidateNode);
      double minimumSurfaceNormalZ = Math.cos(parameters.getMinimumSurfaceInclineRadians());
      if (snappedSoleTransform.getM22() < minimumSurfaceNormalZ)
      {
         return BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP;
      }

      // Check snap area
      ConvexPolygon2D footholdAfterSnap = candidateNodeSnapData.getCroppedFoothold();
      double croppedFootArea = footholdAfterSnap.getArea();
      double fullFootArea = footPolygons.get(candidateNode.getRobotSide()).getArea();
      footAreaPercentage = croppedFootArea / fullFootArea;

      double epsilonAreaPercentage = 1e-4;
      if (!footholdAfterSnap.isEmpty() && footAreaPercentage < (parameters.getMinimumFootholdPercent() - epsilonAreaPercentage))
      {
         return BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA;
      }

      // Check for ankle collision
      cliffAvoider.setPlanarRegionsList(planarRegionsList);
      if(!cliffAvoider.isNodeValid(candidateNode))
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
         if (boundingBoxCollisionDetected(candidateNode, stanceNode))
         {
            return BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_HITTING_BODY;
         }
     }

      return null;
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

   public void setParentNodeSupplier(UnaryOperator<FootstepNode> parentNodeSupplier)
   {
      this.goodPositionChecker.setParentNodeSupplier(parentNodeSupplier);
   }

   private void logVariables(FootstepNode candidateNode, FootstepNode stanceNode, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      if (edgeData != null)
      {
         edgeData.setStanceNode(stanceNode);
         edgeData.setCandidateNode(candidateNode);
         edgeData.setCandidateNodeSnapData(candidateNodeSnapData);
         edgeData.setFootAreaPercentage(footAreaPercentage);
         edgeData.setRejectionReason(rejectionReason);
         edgeData.setStepIndex(footstepIndex);
         goodPositionChecker.logVariables();
      }
   }

   private void clearLoggedVariables()
   {
      footAreaPercentage = Double.NaN;
      candidateNodeSnapData.clear();
      goodPositionChecker.clearLoggedVariables();
   }
}
