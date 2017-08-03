package us.ihmc.footstepPlanning.aStar.implementations;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeSnapper;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FlatGroundFootstepNodeSnapper implements FootstepNodeSnapper
{
   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
   }

   @Override
   public boolean snapFootstepNode(FootstepNode footstepNode)
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      soleTransform.setTranslation(footstepNode.getX(), footstepNode.getY(), 0.0);
      soleTransform.setRotation(new AxisAngle(0.0, 0.0, 1.0, footstepNode.getYaw()));
      footstepNode.setSoleTransform(soleTransform);
      return true;
   }
}
