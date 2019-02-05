package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootstepNodeTools
{
   /**
    * Computes a node-to-world RigidBodyTransform from the node's x and y position. This transform
    * will always have no z translation..
    */
   public static void getNodeTransform(RobotQuadrant robotQuadrant, FootstepNode node, RigidBodyTransform nodeToWorldTransformToPack)
   {
      Point3D solePosition = new Point3D(node.getX(robotQuadrant), node.getY(robotQuadrant), 0.0);
      nodeToWorldTransformToPack.setTranslation(solePosition);
   }

   /**
    * Computes a node-to-world RigidBodyTransform which transforms from node frame to world frame
    * by applying snapTransform to the given node's transform
    *
    * @param snapTransform pre-snap to post-snap transform
    */
   public static void getSnappedNodeTransform(RobotQuadrant robotQuadrant, FootstepNode node, RigidBodyTransform snapTransform,
                                              RigidBodyTransform transformToPack)
   {
      getNodeTransform(robotQuadrant, node, transformToPack);
      snapTransform.transform(transformToPack);
   }

   /**
    * Computes the foot position in world frame that corresponds to the give footstep node
    */
   public static void getFootPosition(RobotQuadrant robotQuadrant, FootstepNode node, Point2DBasics footPositionToPack)
   {
      footPositionToPack.setToZero();

      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(robotQuadrant, node, nodeTransform);

      footPositionToPack.applyTransform(nodeTransform);
   }
}
