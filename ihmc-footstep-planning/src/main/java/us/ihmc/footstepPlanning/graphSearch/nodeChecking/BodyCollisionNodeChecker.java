package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

public class BodyCollisionNodeChecker extends FootstepNodeChecker
{
   private final BoundingBoxCollisionDetector collisionChecker;
   private final FootstepPlannerParameters parameters;
   private final FootstepNodeSnapper snapper;

   private final Point3D tempPoint1 = new Point3D();
   private final Point3D tempPoint2 = new Point3D();

   public BodyCollisionNodeChecker(BoundingBoxCollisionDetector collisionChecker, FootstepPlannerParameters parameters, FootstepNodeSnapper snapper)
   {
      this.parameters = parameters;
      this.collisionChecker = collisionChecker;
      this.snapper = snapper;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      super.setPlanarRegions(planarRegions);
      collisionChecker.setPlanarRegionsList(planarRegionsList);
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode)
   {
      if (previousNode == null || !parameters.checkForBodyBoxCollisions() || !hasPlanarRegions())
      {
         return true;
      }

      tempPoint1.set(node.getX(), node.getY(), 0.0);
      tempPoint2.set(previousNode.getX(), previousNode.getY(), 0.0);
      tempPoint1.interpolate(tempPoint2, 0.5);
      double yaw = AngleTools.computeAngleAverage(node.getYaw(), previousNode.getYaw());

      FootstepNodeSnapData footstepNodeSnapData = snapper.snapFootstepNode(new FootstepNode(tempPoint1.getX(), tempPoint1.getY(), yaw, RobotSide.LEFT));
      double height = footstepNodeSnapData.getSnapTransform().getTranslationZ();

      if(Double.isNaN(height))
      {
         // fall back on height of upcoming step
         height = snapper.getSnapData(node).getSnapTransform().getTranslationZ();
      }

      collisionChecker.setBoxPose(tempPoint1.getX(), tempPoint1.getY(), height, yaw);
      BodyCollisionData collisionData = collisionChecker.checkForCollision();
      return !collisionData.isCollisionDetected();
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
   }
}
