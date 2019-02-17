package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootstepNodeSnappingTools
{
   /**
    * Transforms footPositionInRegionFrame from planarRegion frame to the snapped footstep node frame
    */
   public static void changeFromPlanarRegionToSoleFrame(PlanarRegion planarRegion, RobotQuadrant robotQuadrant, FootstepNode footstepNode,
                                                        RigidBodyTransform snapTransform, Point2DBasics footPositionInRegionFrame)
   {
      RigidBodyTransform regionToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(regionToWorld);

      RigidBodyTransform soleTransform = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransformToWorld(robotQuadrant, footstepNode, snapTransform, soleTransform);

      RigidBodyTransform regionToSole = new RigidBodyTransform();
      regionToSole.setAndInvert(soleTransform);
      regionToSole.multiply(regionToWorld);

      footPositionInRegionFrame.applyTransform(regionToSole, false);
   }

   /**
    * Computes the snap transform which snaps the given node to the given position
    *
    * @param node
    * @param footstepPosition
    * @return
    */
   public static RigidBodyTransform computeSnapTransform(RobotQuadrant robotQuadrant, FootstepNode node, Point3DReadOnly footstepPosition)
   {
      RigidBodyTransform snapTransform = new RigidBodyTransform();
      RigidBodyTransform stepTransform = new RigidBodyTransform();
      stepTransform.setTranslation(footstepPosition);

      FootstepNodeTools.getNodeTransformToWorld(robotQuadrant, node, snapTransform);
      snapTransform.preMultiplyInvertThis(stepTransform);

      return snapTransform;
   }

   /**
    * Computes the snap transform which snaps the given node to the given position
    *
    * @param node
    * @param footstepPosition
    * @return
    */
   public static RigidBodyTransform computeSnapTransform(int xIndex, int yIndex, Point3DReadOnly footstepPosition)
   {
      RigidBodyTransform snapTransform = new RigidBodyTransform();
      RigidBodyTransform stepTransform = new RigidBodyTransform();
      stepTransform.setTranslation(footstepPosition);

      FootstepNodeTools.getNodeTransformToWorld(xIndex, yIndex, snapTransform);
      snapTransform.preMultiplyInvertThis(stepTransform);

      return snapTransform;
   }
}
