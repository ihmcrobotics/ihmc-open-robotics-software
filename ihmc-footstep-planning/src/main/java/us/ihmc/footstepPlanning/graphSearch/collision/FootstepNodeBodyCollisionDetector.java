package us.ihmc.footstepPlanning.graphSearch.collision;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class FootstepNodeBodyCollisionDetector
{
   private static final int nodeChecks = 3; // includes end node

   private final BoundingBoxCollisionDetector collisionDetector;
   private final FootstepPlannerParameters parameters;
   private final HashMap<LatticeNode, BodyCollisionData> collisionDataHolder = new HashMap<>();

   private final List<LatticeNode> nodesToCheck = new ArrayList<>();

   public FootstepNodeBodyCollisionDetector(FootstepPlannerParameters parameters)
   {
      this.collisionDetector = new BoundingBoxCollisionDetector(parameters);
      this.parameters = parameters;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      if(planarRegionsList != null)
         collisionDetector.setPlanarRegionsList(planarRegionsList);
      collisionDataHolder.clear();
   }

   public boolean checkForCollision(FootstepNode footstepNode, FootstepNode grandParentNode, double snappedNodeHeight)
   {
      calculateNodesToCheck(footstepNode, grandParentNode);

      boolean collisionDetected = false;
      for (int i = 0; i < nodesToCheck.size(); i++)
      {
         BodyCollisionData collisionData = collisionDataHolder.computeIfAbsent(nodesToCheck.get(i), latticeNode ->
         {
            collisionDetector.setBoxPose(latticeNode.getX(), latticeNode.getY(), snappedNodeHeight + parameters.getBodyBoxBaseZ(), latticeNode.getYaw());
            return collisionDetector.checkForCollision();
         });

         collisionDetected = collisionDetected || collisionData.isCollisionDetected();
      }

      return collisionDetected;
   }

   private void calculateNodesToCheck(FootstepNode node, FootstepNode grandParentNode)
   {
      nodesToCheck.clear();

      if(grandParentNode == null || nodeChecks <= 1)
      {
         nodesToCheck.add(createNodeForCollisionCheck(node));
      }
      else
      {
         LatticeNode startNode = createNodeForCollisionCheck(grandParentNode);
         LatticeNode endNode = createNodeForCollisionCheck(node);

         for (int i = 0; i < nodeChecks; i++)
         {
            double alpha = i / ((double) nodeChecks);
            LatticeNode latticeNode = FootstepNodeTools.interpolate(startNode, endNode, alpha);

            if(!nodesToCheck.contains(latticeNode))
               nodesToCheck.add(latticeNode);
         }

         Collections.reverse(nodesToCheck);
      }
   }

   public BodyCollisionData getCollisionData(FootstepNode node)
   {
      return collisionDataHolder.get(createNodeForCollisionCheck(node));
   }

   private LatticeNode createNodeForCollisionCheck(FootstepNode node)
   {
      double lateralOffsetSign = node.getRobotSide().negateIfLeftSide(1.0);
      double offsetX = parameters.getBodyBoxBaseX() * Math.cos(node.getYaw()) - lateralOffsetSign * parameters.getBodyBoxBaseY() * Math.sin(node.getYaw());
      double offsetY = parameters.getBodyBoxBaseX() * Math.sin(node.getYaw()) + lateralOffsetSign * parameters.getBodyBoxBaseY() * Math.cos(node.getYaw());
      return new LatticeNode(node.getX() + offsetX, node.getY() + offsetY, node.getYaw());
   }
}
