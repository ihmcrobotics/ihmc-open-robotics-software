package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ObstacleBetweenNodesChecker implements SnapBasedCheckerComponent
{
   private static final boolean DEBUG = false;

   private PlanarRegionsList planarRegionsList;
   private final FootstepNodeSnapper snapper;
   private final BooleanSupplier checkForPathCollisions;
   private final DoubleSupplier idealFootstepWidth;
   private final DoubleSupplier heightOffset;
   private final DoubleSupplier heightExtrusion;

   public ObstacleBetweenNodesChecker(FootstepPlannerParametersReadOnly parameters, FootstepNodeSnapper snapper)
   {
      this(snapper, parameters::checkForPathCollisions, parameters::getIdealFootstepWidth, parameters::getBodyBoxBaseZ, parameters::getBodyBoxHeight);
   }

   public ObstacleBetweenNodesChecker(FootstepNodeSnapper snapper,
                                      BooleanSupplier checkForPathCollisions,
                                      DoubleSupplier idealFootstepWidth,
                                      DoubleSupplier heightOffset,
                                      DoubleSupplier heightExtrusion)
   {
      this.snapper = snapper;
      this.checkForPathCollisions = checkForPathCollisions;
      this.idealFootstepWidth = idealFootstepWidth;
      this.heightOffset = heightOffset;
      this.heightExtrusion = heightExtrusion;
   }

   @Override
   public void setFootstepGraph(DirectedGraph graph)
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
      if (previousNode == null || !checkForPathCollisions.getAsBoolean())
         return true;

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(node);
      RigidBodyTransform snapTransform = snapData.getSnapTransform();

      FootstepNodeSnapData previousNodeSnapData = snapper.snapFootstepNode(previousNode);
      RigidBodyTransform previousSnapTransform = previousNodeSnapData.getSnapTransform();

      Point3D nodePosition = new Point3D(node.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble()));
      Point3D previousNodePosition = new Point3D(previousNode.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble()));

      snapTransform.transform(nodePosition);
      previousSnapTransform.transform(previousNodePosition);

      if (hasPlanarRegions() && isObstacleBetweenNodes(nodePosition, previousNodePosition, planarRegionsList.getPlanarRegionsAsList()))
      {
         if (DEBUG)
         {
            LogTools.debug("Found a obstacle between the nodes " + node + " and " + previousNode);
         }
         return false;
      }

      return true;
   }

   /**
    * This is meant to test if there is a wall that the body of the robot would run into when shifting
    * from one step to the next. It is not meant to eliminate swing overs.
    */
   private boolean isObstacleBetweenNodes(Point3D nodePosition, Point3D previousNodePosition, List<PlanarRegion> planarRegions)
   {
      double groundClearance = heightOffset.getAsDouble();
      double regionHeight = heightExtrusion.getAsDouble();

      try
      {
         PlanarRegion bodyPath = createBodyRegionFromNodes(nodePosition, previousNodePosition, groundClearance, regionHeight);

         for (PlanarRegion region : planarRegions)
         {
            List<LineSegment3D> intersections = region.intersect(bodyPath);
            if (!intersections.isEmpty())
            {
               return true;
            }
         }
      }
      catch(Exception e)
      {
         e.printStackTrace();
      }

      return false;
   }

   /**
    * Given two footstep position this will create a vertical planar region above the nodes. The region
    * will be aligned with the vector connecting the nodes. It's lower edge will be the specified
    * distance above the higher of the two nodes and the plane will have the specified hight.
    */
   public static PlanarRegion createBodyRegionFromNodes(Point3DReadOnly nodeA, Point3DReadOnly nodeB, double clearance, double height)
   {
      double lowerZ = Math.max(nodeA.getZ(), nodeB.getZ()) + clearance;
      double higherZ = lowerZ + height;

      Point3D point0 = new Point3D(nodeA.getX(), nodeA.getY(), lowerZ);
      Point3D point1 = new Point3D(nodeA.getX(), nodeA.getY(), higherZ);
      Point3D point2 = new Point3D(nodeB.getX(), nodeB.getY(), lowerZ);
      Point3D point3 = new Point3D(nodeB.getX(), nodeB.getY(), higherZ);

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
      return BipedalFootstepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_BODY;
   }
}
