package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.ConcavePolygonWiggler;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListPolygonSnapper;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class FootstepNodeSnapData
{
   /**
    * Snap transform describing the projection of a 2D step onto a planar region.
    * The relationship between the unsnapped planar footstep transform T_planar, snapped step transform T_snapped and this transform S is:
    * T_snapped = S * T_planar
    *
    * {@link FootstepNodeTools#getSnappedNodeTransform}
    * {@link PlanarRegionsListPolygonSnapper#snapPolygonToPlanarRegionsList}
    */
   private final RigidBodyTransform snapTransform = new RigidBodyTransform();

   /**
    * Transform for wiggling a step inside a planar region.
    * The relationship between the unsnapped planar footstep transform T_planar, snapped step transform T_snapped, snap transform S,
    * the planar region's transform from local to world P, the wiggler solver output W_local, and this transform W_world:
    * T_snapped = (P * W_local * P_inv) * S * T_planar = W_world * S * T_planar
    *
    * The wiggle transform is set to NaN by default.
    * {@link PolygonWiggler#findWiggleTransform}
    * {@link ConcavePolygonWiggler#wigglePolygon}
    */
   private final RigidBodyTransform wiggleTransformInWorld = new RigidBodyTransform();

   /**
    * Transform of the snapped footstep, T_snapped in the formaulas above
    * This transform includes the wiggle transform if it's available.
    */
   private final RigidBodyTransform snappedNodeTransform = new RigidBodyTransform();

   /**
    * Cropped foothold polygon in sole frame.
    */
   private final ConvexPolygon2D croppedFoothold = new ConvexPolygon2D();

   /**
    * Planar region ID that the step is snapped to
    */
   private int planarRegionId = PlanarRegion.NO_REGION_ID;

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

   public ConvexPolygon2D getCroppedFoothold()
   {
      return croppedFoothold;
   }

   public RigidBodyTransform getSnapTransform()
   {
      return snapTransform;
   }

   public RigidBodyTransform getWiggleTransform()
   {
      return wiggleTransformInWorld;
   }

   public void setPlanarRegionId(int planarRegionId)
   {
      this.planarRegionId = planarRegionId;
   }

   public int getPlanarRegionId()
   {
      return planarRegionId;
   }

   public RigidBodyTransform getSnappedNodeTransform(FootstepNode node)
   {
      updateSnappedNodeTransform(node);
      return snappedNodeTransform;
   }

   public void packSnapAndWiggleTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(snapTransform);
      if (!wiggleTransformInWorld.containsNaN())
      {
         transformToPack.preMultiply(wiggleTransformInWorld);
      }
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
      this.planarRegionId = other.planarRegionId;
      this.snappedNodeTransformIncludesWiggleTransform = other.snappedNodeTransformIncludesWiggleTransform;
   }

   public void clear()
   {
      this.snapTransform.setToNaN();
      this.wiggleTransformInWorld.setToNaN();
      this.snappedNodeTransform.setToNaN();
      this.croppedFoothold.clearAndUpdate();
      this.planarRegionId = PlanarRegion.NO_REGION_ID;
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
      snapData.getWiggleTransform().setIdentity();
      snapData.getCroppedFoothold().clearAndUpdate();
      return snapData;
   }
}
