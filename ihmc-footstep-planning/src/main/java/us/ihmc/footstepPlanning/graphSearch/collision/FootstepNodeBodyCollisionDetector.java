package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.HashMap;

public class FootstepNodeBodyCollisionDetector
{
   private final BoundingBoxCollisionDetector collisionDetector;
   private final FootstepPlannerParameters parameters;
   private final HashMap<LatticeNode, BodyCollisionData> collisionDataHolder = new HashMap<>();

   public FootstepNodeBodyCollisionDetector(FootstepPlannerParameters parameters)
   {
      this.collisionDetector = new BoundingBoxCollisionDetector(parameters);
      this.parameters = parameters;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      collisionDetector.setPlanarRegionsList(planarRegionsList);
      collisionDataHolder.clear();
   }

   public BodyCollisionData checkForCollision(LatticeNode node, double snappedNodeHeight)
   {
      if (collisionDataHolder.containsKey(node))
      {
         return collisionDataHolder.get(node);
      }
      else
      {
         double offsetX = parameters.getBodyBoxBaseX() * Math.cos(node.getYaw()) - parameters.getBodyBoxBaseY() * Math.sin(node.getYaw());
         double offsetY = parameters.getBodyBoxBaseX() * Math.sin(node.getYaw()) + parameters.getBodyBoxBaseY() * Math.cos(node.getYaw());

         collisionDetector.setBoxPose(offsetX + node.getX(), offsetY + node.getY(), snappedNodeHeight + parameters.getBodyBoxBaseZ(), node.getYaw());
         BodyCollisionData collisionData = collisionDetector.checkForCollision();
         collisionDataHolder.put(node, collisionData);
         return collisionData;
      }
   }
}
