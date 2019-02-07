package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.List;

public class SnapBasedNodeChecker extends FootstepNodeChecker
{
   private static final boolean DEBUG = false;

   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapper snapper;

   public SnapBasedNodeChecker(FootstepPlannerParameters parameters, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.snapper = snapper;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      super.setPlanarRegions(planarRegions);
      snapper.setPlanarRegions(planarRegions);
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode != null && node.equals(previousNode))
      {
         throw new IllegalArgumentException("Checking node assuming it is following itself.");
      }

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(node);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform snapTransform = snapData.getSnapTransform(robotQuadrant);
         if (snapTransform.containsNaN())
         {
            if (DEBUG)
            {
               PrintTools.debug("Was not able to snap node:\n" + node);
            }
            rejectNode(node, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
            return false;
         }
      }


      if (previousNode == null)
      {
         return true;
      }

      FootstepNodeSnapData previousNodeSnapData = snapper.snapFootstepNode(previousNode);



      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform previousFootSnapTransform = previousNodeSnapData.getSnapTransform(robotQuadrant);
         RigidBodyTransform snapTransform = snapData.getSnapTransform(robotQuadrant);

         Point3D stepPosition = new Point3D(node.getX(robotQuadrant), node.getY(robotQuadrant), 0.0);
         Point3D previousStepPosition = new Point3D(previousNode.getX(robotQuadrant), previousNode.getY(robotQuadrant), 0.0);

         snapTransform.transform(stepPosition);
         previousFootSnapTransform.transform(previousStepPosition);

         double heightChange = Math.abs(stepPosition.getZ() - previousStepPosition.getZ());
         if (heightChange > parameters.getMaximumStepChangeZ())
         {
            if (DEBUG)
            {
               PrintTools.debug("Too much height difference (" + Math.round(100.0 * heightChange) + "cm) to previous node:\n" + node);
            }
            rejectNode(node, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.STEP_TOO_HIGH_OR_LOW);
            return false;
         }


         if (hasPlanarRegions() && isObstacleBetweenNodes(stepPosition, previousStepPosition,
                                                          snapper.getOrCreateNearbyRegions(node.getRoundedX(robotQuadrant), node.getRoundedY(robotQuadrant)),
                                                          parameters.getBodyGroundClearance()))
         {
            if (DEBUG)
            {
               PrintTools.debug("Found a obstacle between the nodes " + node + " and " + previousNode);
            }
            rejectNode(node, previousNode, QuadrupedFootstepPlannerNodeRejectionReason.OBSTACLE_BLOCKING_BODY);
            return false;
         }
      }





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
   public void addStartNode(FootstepNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransforms)
   {
      snapper.addSnapData(startNode, new FootstepNodeSnapData(startNodeTransforms));
   }
}
