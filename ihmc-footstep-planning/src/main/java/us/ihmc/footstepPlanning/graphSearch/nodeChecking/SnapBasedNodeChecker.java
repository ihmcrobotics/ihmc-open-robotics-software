package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SnapBasedNodeChecker implements FootstepNodeChecker
{
   private static final boolean DEBUG = false;

   private final FootstepPlannerParameters parameters;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FootstepNodeSnapper snapper;

   private PlanarRegionsList planarRegions;
   private BipedalFootstepPlannerListener listener;

   public SnapBasedNodeChecker(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.footPolygons = footPolygons;
      this.snapper = snapper;
   }

   public void addPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      snapper.setPlanarRegions(planarRegions);
      this.planarRegions = planarRegions;
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode != null && node.equals(previousNode))
      {
         throw new RuntimeException("Checking node assuming it is follwoing itself.");
      }

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(node);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();
      if (snapTransform.containsNaN())
      {
         if (DEBUG)
         {
            PrintTools.debug("Was not able to snap node:\n" + node);
         }
         notifyPlannerListenerThatNodeIsRejected(node, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
         return false;
      }

      ConvexPolygon2D footholdAfterSnap = snapData.getCroppedFoothold();
      double area = footholdAfterSnap.getArea();
      double footArea = footPolygons.get(node.getRobotSide()).getArea();
      if (!footholdAfterSnap.isEmpty() && area < parameters.getMinimumFootholdPercent() * footArea)
      {
         if (DEBUG)
         {
            PrintTools.debug("Node does not have enough foothold area. It only has " + Math.floor(100.0 * area / footArea) + "% foothold:\n" + node);
         }
         notifyPlannerListenerThatNodeIsRejected(node, BipedalFootstepPlannerNodeRejectionReason.NOT_ENOUGH_AREA);
         return false;
      }

      if (previousNode == null)
      {
         return true;
      }

      FootstepNodeSnapData previousNodeSnapData = snapper.snapFootstepNode(previousNode);
      RigidBodyTransform previousSnapTransform = previousNodeSnapData.getSnapTransform();
      Point3D nodePosition = new Point3D(DistanceAndYawBasedCost.computeMidFootPoint(node, parameters.getIdealFootstepWidth()));
      snapTransform.transform(nodePosition);
      Point3D previousNodePosition = new Point3D(DistanceAndYawBasedCost.computeMidFootPoint(previousNode, parameters.getIdealFootstepWidth()));
      previousSnapTransform.transform(previousNodePosition);

      double heightChange = Math.abs(nodePosition.getZ() - previousNodePosition.getZ());
      if (heightChange > parameters.getMaximumStepZ())
      {
         if (DEBUG)
         {
            PrintTools.debug("Too much height difference (" + Math.round(100.0 * heightChange) + "cm) to previous node:\n" + node);
         }
         notifyPlannerListenerThatNodeIsRejected(node, BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW);
         return false;
      }

      if (planarRegions != null && isObstacleBetweenNodes(nodePosition, previousNodePosition, planarRegions, parameters.getBodyGroundClearance()))
      {
         if (DEBUG)
         {
            PrintTools.debug("Found a obstacle between the nodes " + node + " and " + previousNode);
         }
         notifyPlannerListenerThatNodeIsRejected(node, BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_BODY);
         return false;
      }

      return true;
   }

   /**
    * This is meant to test if there is a wall that the body of the robot would run into when shifting
    * from one step to the next. It is not meant to eliminate swing overs.
    */
   private static boolean isObstacleBetweenNodes(Point3D nodePosition, Point3D previousNodePosition, PlanarRegionsList planarRegions, double groundClearance)
   {
      PlanarRegion bodyPath = createBodyRegionFromNodes(nodePosition, previousNodePosition, groundClearance, 2.0);

      for (PlanarRegion region : planarRegions.getPlanarRegionsAsList())
      {
         List<LineSegment3D> intersections = region.intersect(bodyPath);
         if (!intersections.isEmpty())
         {
            return true;
         }
      }

      return false;
   }

   /**
    * Given two footstep position this will create a vertical planar region above the nodes. The region
    * will be aligned with the vector connecting the nodes. It's lower edge will be the specified
    * distance above the higher of the two nodes and the plane will have the specified hight.
    */
   public static PlanarRegion createBodyRegionFromNodes(Point3D nodeA, Point3D nodeB, double clearance, double height)
   {
      double lowerZ = Math.max(nodeA.getZ(), nodeB.getZ()) + clearance;
      Point3D point0 = new Point3D(nodeA.getX(), nodeA.getY(), lowerZ);
      Point3D point1 = new Point3D(nodeA.getX(), nodeA.getY(), lowerZ + height);
      Point3D point2 = new Point3D(nodeB.getX(), nodeB.getY(), lowerZ);
      Point3D point3 = new Point3D(nodeB.getX(), nodeB.getY(), lowerZ + height);

      Vector3D xAxisInPlane = new Vector3D();
      xAxisInPlane.sub(point2, point0);
      xAxisInPlane.normalize();
      Vector3D yAxisInPlane = new Vector3D(0.0, 0.0, 1.0);
      Vector3D zAxis = new Vector3D();
      zAxis.cross(xAxisInPlane, yAxisInPlane);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotation(xAxisInPlane.getX(), xAxisInPlane.getY(), xAxisInPlane.getZ(), yAxisInPlane.getX(), yAxisInPlane.getY(), yAxisInPlane.getZ(),
                            zAxis.getX(), zAxis.getY(), zAxis.getZ());
      transform.setTranslation(point0);
      transform.invertRotation();

      point0.applyInverseTransform(transform);
      point1.applyInverseTransform(transform);
      point2.applyInverseTransform(transform);
      point3.applyInverseTransform(transform);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(point0.getX(), point0.getY());
      polygon.addVertex(point1.getX(), point1.getY());
      polygon.addVertex(point2.getX(), point2.getY());
      polygon.addVertex(point3.getX(), point3.getY());
      polygon.update();

      return new PlanarRegion(transform, polygon);
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
      snapper.addSnapData(startNode, new FootstepNodeSnapData(startNodeTransform));
   }

   private void notifyPlannerListenerThatNodeIsRejected(FootstepNode node, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      if(listener != null)
         listener.nodeUnderConsiderationWasRejected(node, rejectionReason);
   }
}
