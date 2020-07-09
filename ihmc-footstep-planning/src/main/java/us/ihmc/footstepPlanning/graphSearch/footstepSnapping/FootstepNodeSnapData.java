package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class FootstepNodeSnapData implements FootstepNodeSnapDataReadOnly
{
   private final RigidBodyTransform snapTransform = new RigidBodyTransform();
   private final RigidBodyTransform wiggleTransformInWorld = new RigidBodyTransform();
   private final RigidBodyTransform snappedNodeTransform = new RigidBodyTransform();
   private final ConvexPolygon2D croppedFoothold = new ConvexPolygon2D();
   private int regionIndex = -1;
   private boolean snappedNodeTransformIncludesWiggleTransform = false;

   public FootstepNodeSnapData()
   {
      this(null);
   }

   public FootstepNodeSnapData(RigidBodyTransformReadOnly snapTransform)
   {
      this(snapTransform, null);
   }

   public FootstepNodeSnapData(RigidBodyTransformReadOnly snapTransform, ConvexPolygon2DReadOnly croppedFoothold)
   {
      this.wiggleTransformInWorld.setToNaN();
      this.snappedNodeTransform.setToNaN();

      if (snapTransform == null)
      {
         this.snapTransform.setToNaN();
      }
      else
      {
         this.snapTransform.set(snapTransform);
      }

      if (croppedFoothold != null)
      {
         this.croppedFoothold.set(croppedFoothold);
      }
   }

   /** {@inheritDoc} */
   @Override
   public ConvexPolygon2D getCroppedFoothold()
   {
      return croppedFoothold;
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyTransform getSnapTransform()
   {
      return snapTransform;
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyTransform getWiggleTransformInWorld()
   {
      return wiggleTransformInWorld;
   }

   /** {@inheritDoc} */
   @Override
   public int getRegionIndex()
   {
      return regionIndex;
   }

   /** {@inheritDoc} */
   @Override
   public RigidBodyTransform getSnappedNodeTransform(FootstepNode node)
   {
      updateSnappedNodeTransform(node);
      return snappedNodeTransform;
   }

   public void setRegionIndex(int regionIndex)
   {
      this.regionIndex = regionIndex;
   }

   private void updateSnappedNodeTransform(FootstepNode node)
   {
      if (snappedNodeTransform.containsNaN())
      {
         FootstepNodeTools.getSnappedNodeTransform(node, snapTransform, snappedNodeTransform);
      }

      if (!snappedNodeTransformIncludesWiggleTransform && !wiggleTransformInWorld.containsNaN())
      {
         snappedNodeTransform.preMultiply(wiggleTransformInWorld);
         snappedNodeTransformIncludesWiggleTransform = true;
      }
   }

   public void set(FootstepNodeSnapData other)
   {
      this.snapTransform.set(other.snapTransform);
      this.croppedFoothold.set(other.croppedFoothold);
      this.wiggleTransformInWorld.set(other.wiggleTransformInWorld);
      this.snappedNodeTransform.set(other.snappedNodeTransform);
      this.regionIndex = other.regionIndex;
      this.snappedNodeTransformIncludesWiggleTransform = other.snappedNodeTransformIncludesWiggleTransform;
   }

   public void clear()
   {
      this.snapTransform.setToNaN();
      this.wiggleTransformInWorld.setToNaN();
      this.snappedNodeTransform.setToNaN();
      this.croppedFoothold.clearAndUpdate();
      this.regionIndex = PlanarRegion.NO_REGION_ID;
      this.snappedNodeTransformIncludesWiggleTransform = false;
   }

   private static final FootstepNodeSnapData EMPTY_SNAP_DATA;

   static
   {
      EMPTY_SNAP_DATA = new FootstepNodeSnapData();
   }

   public static FootstepNodeSnapData emptyData()
   {
      return EMPTY_SNAP_DATA;
   }

   public static FootstepNodeSnapData identityData()
   {
      FootstepNodeSnapData snapData = new FootstepNodeSnapData();
      snapData.getSnapTransform().setIdentity();
      snapData.getWiggleTransformInWorld().setIdentity();
      snapData.getCroppedFoothold().clearAndUpdate();
      return snapData;
   }
}
