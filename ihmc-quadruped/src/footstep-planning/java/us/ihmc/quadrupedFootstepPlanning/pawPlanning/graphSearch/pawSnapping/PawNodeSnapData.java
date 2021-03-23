package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class PawNodeSnapData
{
   private final RigidBodyTransform snapTransform;

   public PawNodeSnapData(RigidBodyTransform snapTransform)
   {
      this.snapTransform = snapTransform;
   }

   public RigidBodyTransform getSnapTransform()
   {
      return snapTransform;
   }

   private static final PawNodeSnapData EMPTY_SNAP_DATA;
   private static final PawNodeSnapData IDENTITY_SNAP_DATA;

   static
   {
      IDENTITY_SNAP_DATA = new PawNodeSnapData(new RigidBodyTransform());

      RigidBodyTransform emptySnapTransform = new RigidBodyTransform();
      emptySnapTransform.setToNaN();

      EMPTY_SNAP_DATA = new PawNodeSnapData(emptySnapTransform);
   }

   public static PawNodeSnapData emptyData()
   {
      return EMPTY_SNAP_DATA;
   }

   public static PawNodeSnapData identityData()
   {
      return IDENTITY_SNAP_DATA;
   }
}
