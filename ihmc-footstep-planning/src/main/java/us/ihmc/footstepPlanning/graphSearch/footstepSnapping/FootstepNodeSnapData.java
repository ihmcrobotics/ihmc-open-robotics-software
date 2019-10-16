package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;

public class FootstepNodeSnapData
{
   private final RigidBodyTransform snapTransform;
   private RigidBodyTransform snappedNodeTransform = null;
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

   public RigidBodyTransform getOrComputeSnappedNodeTransform(FootstepNode node)
   {
      if (snappedNodeTransform == null)
         snappedNodeTransform = computeSnappedNodeTransform(node, this);
      return snappedNodeTransform;
   }

   private static RigidBodyTransform computeSnappedNodeTransform(FootstepNode node, FootstepNodeSnapData snapData)
   {
      RigidBodyTransform snappedNodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getSnappedNodeTransform(node, snapData.getSnapTransform(), snappedNodeTransform);

      return snappedNodeTransform;
   }

   private static final FootstepNodeSnapData EMPTY_SNAP_DATA;

   static
   {
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
      return new FootstepNodeSnapData(new RigidBodyTransform(), new ConvexPolygon2D());
   }
}
