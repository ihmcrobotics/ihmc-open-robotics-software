package us.ihmc.footstepPlanning.aStar.implementations;

import java.util.HashSet;

import javax.vecmath.Vector3d;

import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeExpansion;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class SimpleSideBasedExpansion implements FootstepNodeExpansion
{
   @Override
   public HashSet<FootstepNode> expandNode(FootstepNode node)
   {
      RobotSide stepSide = node.getRobotSide().getOppositeSide();
      double yOffset = stepSide.negateIfRightSide(FootstepNode.gridSizeY);
      double stanceYaw = node.getYaw();

      HashSet<FootstepNode> neighbors = new HashSet<>();
      double yawGrid = FootstepNode.gridSizeYaw;
      double[] neighborYaws = new double[] {stanceYaw - yawGrid, stanceYaw, stanceYaw + yawGrid};

      for (int i = 0; i < neighborYaws.length; i++)
      {
         double neighborYaw = neighborYaws[i];
         Vector3d offset1 = new Vector3d(FootstepNode.gridSizeX, yOffset, 0.0);
         Vector3d offset2 = new Vector3d(-FootstepNode.gridSizeX, yOffset, 0.0);
         Vector3d offset3 = new Vector3d(0.0, yOffset, 0.0);

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setRotationYawAndZeroTranslation(neighborYaw);
         transform.transform(offset1);
         transform.transform(offset2);
         transform.transform(offset3);

         neighbors.add(new FootstepNode(node.getX() + offset1.getX(), node.getY() + offset1.getY(), neighborYaw, stepSide));
         neighbors.add(new FootstepNode(node.getX() + offset2.getX(), node.getY() + offset2.getY(), neighborYaw, stepSide));
         neighbors.add(new FootstepNode(node.getX() + offset3.getX(), node.getY() + offset3.getY(), neighborYaw, stepSide));
      }

      return neighbors;
   }
}
