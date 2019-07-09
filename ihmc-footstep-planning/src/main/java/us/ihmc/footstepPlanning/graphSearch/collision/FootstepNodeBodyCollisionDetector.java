package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
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
      this.collisionDetector = new BoundingBoxCollisionDetector(parameters.getBodyBoxDepth(), parameters.getBodyBoxWidth(), parameters.getBodyBoxHeight(),
                                                                parameters.getCostParameters().getMaximum2dDistanceFromBoundingBoxToPenalize());
      this.parameters = parameters;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      if(planarRegionsList != null)
         collisionDetector.setPlanarRegionsList(planarRegionsList);
      collisionDataHolder.clear();
   }

   public BodyCollisionData checkForCollision(FootstepNode footstepNode, double snappedNodeHeight)
   {
      LatticeNode latticeNode = createNodeForCollisionCheck(footstepNode);

      if (collisionDataHolder.containsKey(latticeNode))
      {
         return collisionDataHolder.get(latticeNode);
      }
      else
      {
         collisionDetector.setBoxPose(latticeNode.getX(), latticeNode.getY(), snappedNodeHeight + parameters.getBodyBoxBaseZ(), latticeNode.getYaw());
         BodyCollisionData collisionData = collisionDetector.checkForCollision();
         collisionDataHolder.put(latticeNode, collisionData);
         return collisionData;
      }
   }

   private LatticeNode createNodeForCollisionCheck(FootstepNode node)
   {
      double lateralOffsetSign = node.getRobotSide().negateIfLeftSide(1.0);
      double offsetX = parameters.getBodyBoxBaseX() * Math.cos(node.getYaw()) - lateralOffsetSign * parameters.getBodyBoxBaseY() * Math.sin(node.getYaw());
      double offsetY = parameters.getBodyBoxBaseX() * Math.sin(node.getYaw()) + lateralOffsetSign * parameters.getBodyBoxBaseY() * Math.cos(node.getYaw());
      return new LatticeNode(node.getX() + offsetX, node.getY() + offsetY, node.getYaw());
   }
}
