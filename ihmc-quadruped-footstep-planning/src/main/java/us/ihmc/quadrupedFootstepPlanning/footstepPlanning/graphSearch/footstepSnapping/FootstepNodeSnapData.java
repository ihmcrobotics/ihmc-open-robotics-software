package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootstepNodeSnapData
{
   private final RigidBodyTransform snapTransform;

   public FootstepNodeSnapData(RigidBodyTransform snapTransform)
   {
      this.snapTransform = snapTransform;
   }

   public RigidBodyTransform getSnapTransform()
   {
      return snapTransform;
   }

   private static final FootstepNodeSnapData EMPTY_SNAP_DATA;
   private static final FootstepNodeSnapData IDENTITY_SNAP_DATA;

   static
   {
      IDENTITY_SNAP_DATA = new FootstepNodeSnapData(new RigidBodyTransform());

      RigidBodyTransform emptySnapTransform = new RigidBodyTransform();
      emptySnapTransform.setToNaN();

      EMPTY_SNAP_DATA = new FootstepNodeSnapData(emptySnapTransform);
   }

   public static FootstepNodeSnapData emptyData()
   {
      return EMPTY_SNAP_DATA;
   }

   public static FootstepNodeSnapData identityData()
   {
      return IDENTITY_SNAP_DATA;
   }
}
