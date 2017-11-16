package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class FootstepNodeSnapData
{
   private final RigidBodyTransform snapTransform;
   private final ConvexPolygon2D croppedFoothold;

   public FootstepNodeSnapData(RigidBodyTransform snapTransform)
   {
      this(snapTransform, new ConvexPolygon2D());
   }

   public FootstepNodeSnapData(RigidBodyTransform snapTransform, ConvexPolygon2D croppedFoothold)
   {
      this.snapTransform = snapTransform;
      this.croppedFoothold = croppedFoothold;
   }

   public ConvexPolygon2D getCroppedFoothold()
   {
      return croppedFoothold;
   }

   public RigidBodyTransform getSnapTransform()
   {
      return snapTransform;
   }

   private static final FootstepNodeSnapData EMPTY_SNAP_DATA;
   private static final FootstepNodeSnapData IDENTITY_SNAP_DATA;

   static
   {
      IDENTITY_SNAP_DATA = new FootstepNodeSnapData(new RigidBodyTransform(), new ConvexPolygon2D());

      RigidBodyTransform snapTransform = new RigidBodyTransform();
      ConvexPolygon2D croppedFoothold = new ConvexPolygon2D();

      snapTransform.setToNaN();
      croppedFoothold.setToNaN();

      EMPTY_SNAP_DATA = new FootstepNodeSnapData(snapTransform, croppedFoothold);
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
