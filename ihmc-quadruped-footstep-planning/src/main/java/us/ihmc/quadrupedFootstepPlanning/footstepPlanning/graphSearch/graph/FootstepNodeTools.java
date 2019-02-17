package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootstepNodeTools
{
   /**
    * Computes a node-to-world RigidBodyTransform from the node's x and y position. This transform
    * will always have no z translation..
    */
   public static void getNodeTransformToWorld(RobotQuadrant robotQuadrant, FootstepNode node, RigidBodyTransform nodeTransformToWorldToPack)
   {
      getNodeTransformToWorld(node.getXIndex(robotQuadrant), node.getYIndex(robotQuadrant), nodeTransformToWorldToPack);
   }

   /**
    * Computes a node-to-world RigidBodyTransform from the node's x and y position. This transform
    * will always have no z translation..
    */
   public static void getNodeTransformToWorld(int xIndex, int yIndex, RigidBodyTransform nodeTransformToWorldToPack)
   {
      nodeTransformToWorldToPack.setTranslation(FootstepNode.gridSizeXY * xIndex, FootstepNode.gridSizeXY * yIndex, 0.0);
   }


   /**
    * Computes a node-to-world RigidBodyTransform which transforms from node frame to world frame
    * by applying snapTransform to the given node's transform
    *
    * @param snapTransform pre-snap to post-snap transform
    */
   public static void getSnappedNodeTransformToWorld(RobotQuadrant robotQuadrant, FootstepNode node, RigidBodyTransform snapTransform,
                                                     RigidBodyTransform nodeTransformToWorldToPack)
   {
      getNodeTransformToWorld(robotQuadrant, node, nodeTransformToWorldToPack);
      snapTransform.transform(nodeTransformToWorldToPack);
   }

   /**
    * Computes a node-to-world RigidBodyTransform which transforms from node frame to world frame
    * by applying snapTransform to the given node's transform
    *
    * @param snapTransform pre-snap to post-snap transform
    */
   public static void getSnappedNodeTransformToWorld(int xIndex, int yIndex, RigidBodyTransform snapTransform, RigidBodyTransform nodeTransformToWorldToPack)
   {
      getNodeTransformToWorld(xIndex, yIndex, nodeTransformToWorldToPack);
      snapTransform.transform(nodeTransformToWorldToPack);
   }

   /**
    * Computes the foot position in world frame that corresponds to the give footstep node
    */
   public static void getFootPosition(RobotQuadrant robotQuadrant, FootstepNode node, Point2DBasics footPositionToPack)
   {
      getFootPosition(node.getXIndex(robotQuadrant), node.getYIndex(robotQuadrant), footPositionToPack);
   }

   public static void getFootPosition(int xIndex, int yIndex, Point2DBasics footPositionToPack)
   {
      footPositionToPack.setToZero();

      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransformToWorld(xIndex, yIndex, nodeTransform);

      footPositionToPack.applyTransform(nodeTransform);
   }
}
