package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class ObstacleBetweenNodesChecker implements SnapBasedCheckerComponent
{
   private static final boolean DEBUG = false;

   private PlanarRegionsList planarRegionsList;
   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapper snapper;

   private BipedalFootstepPlannerNodeRejectionReason rejectionReason;

   public ObstacleBetweenNodesChecker(FootstepPlannerParameters parameters, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public void setFootstepGraph(FootstepGraph graph)
   {

   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
   }

   boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      FootstepNodeSnapData snapData = snapper.getSnapData(node);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      FootstepNodeSnapData previousNodeSnapData = snapper.snapFootstepNode(previousNode);
      RigidBodyTransform previousSnapTransform = previousNodeSnapData.getSnapTransform();

      Point3D nodePosition = new Point3D(node.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
      snapTransform.transform(nodePosition);
      Point3D previousNodePosition = new Point3D(previousNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth()));
      previousSnapTransform.transform(previousNodePosition);

      if (hasPlanarRegions() && isObstacleBetweenNodes(nodePosition, previousNodePosition, planarRegionsList.getPlanarRegionsAsList(),
                                                        parameters.getBodyGroundClearance()))
      {
         if (DEBUG)
         {
            PrintTools.debug("Found a obstacle between the nodes " + node + " and " + previousNode);
         }
         rejectionReason = BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_BODY;
         return false;
      }

      rejectionReason = null;
      return true;
   }

   /**
    * This is meant to test if there is a wall that the body of the robot would run into when shifting
    * from one step to the next. It is not meant to eliminate swing overs.
    */
   private static boolean isObstacleBetweenNodes(Point3D nodePosition, Point3D previousNodePosition, List<PlanarRegion> planarRegions, double groundClearance)
   {
      PlanarRegion bodyPath = createBodyRegionFromNodes(nodePosition, previousNodePosition, groundClearance, 2.0);

      for (PlanarRegion region : planarRegions)
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
   public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
   {
      return rejectionReason;
   }
}
