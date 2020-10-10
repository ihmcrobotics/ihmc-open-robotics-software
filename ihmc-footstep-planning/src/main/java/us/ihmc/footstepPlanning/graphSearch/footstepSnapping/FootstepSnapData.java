package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class FootstepSnapData implements FootstepSnapDataReadOnly
{
   private final RigidBodyTransform snapTransform = new RigidBodyTransform();
   private final RigidBodyTransform wiggleTransformInWorld = new RigidBodyTransform();
   private final RigidBodyTransform snappedFootstepTransform = new RigidBodyTransform();
   private final ConvexPolygon2D croppedFoothold = new ConvexPolygon2D();
   private int regionIndex = -1;
   private double achievedInsideDelta = Double.NaN;
   private boolean snappedFootstepTransformIncludesWiggleTransform = false;

   public FootstepSnapData()
   {
      this(null);
   }

   public FootstepSnapData(RigidBodyTransformReadOnly snapTransform)
   {
      this(snapTransform, null);
   }

   public FootstepSnapData(RigidBodyTransformReadOnly snapTransform, ConvexPolygon2DReadOnly croppedFoothold)
   {
      this.wiggleTransformInWorld.setToNaN();
      this.snappedFootstepTransform.setToNaN();

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
   public RigidBodyTransform getSnappedStepTransform(DiscreteFootstep footstep)
   {
      updateSnappedStepTransform(footstep);
      return snappedFootstepTransform;
   }

   @Override
   public double getAchievedInsideDelta()
   {
      return achievedInsideDelta;
   }

   public void setRegionIndex(int regionIndex)
   {
      this.regionIndex = regionIndex;
   }

   public void setAchievedInsideDelta(double achievedInsideDelta)
   {
      this.achievedInsideDelta = achievedInsideDelta;
   }

   private void updateSnappedStepTransform(DiscreteFootstep footstep)
   {
      if (snappedFootstepTransform.containsNaN())
      {
         DiscreteFootstepTools.getSnappedStepTransform(footstep, snapTransform, snappedFootstepTransform);
      }

      if (!snappedFootstepTransformIncludesWiggleTransform && !wiggleTransformInWorld.containsNaN())
      {
         snappedFootstepTransform.preMultiply(wiggleTransformInWorld);
         snappedFootstepTransformIncludesWiggleTransform = true;
      }
   }

   public void set(FootstepSnapData other)
   {
      this.snapTransform.set(other.snapTransform);
      this.croppedFoothold.set(other.croppedFoothold);
      this.wiggleTransformInWorld.set(other.wiggleTransformInWorld);
      this.snappedFootstepTransform.set(other.snappedFootstepTransform);
      this.regionIndex = other.regionIndex;
      this.achievedInsideDelta = other.achievedInsideDelta;
      this.snappedFootstepTransformIncludesWiggleTransform = other.snappedFootstepTransformIncludesWiggleTransform;
   }

   public void clear()
   {
      this.snapTransform.setToNaN();
      this.wiggleTransformInWorld.setToNaN();
      this.snappedFootstepTransform.setToNaN();
      this.croppedFoothold.clearAndUpdate();
      this.regionIndex = PlanarRegion.NO_REGION_ID;
      this.achievedInsideDelta = Double.NaN;
      this.snappedFootstepTransformIncludesWiggleTransform = false;
   }

   private static final FootstepSnapData EMPTY_SNAP_DATA;

   static
   {
      EMPTY_SNAP_DATA = new FootstepSnapData();
   }

   public static FootstepSnapData emptyData()
   {
      return EMPTY_SNAP_DATA;
   }

   public static FootstepSnapData identityData()
   {
      FootstepSnapData snapData = new FootstepSnapData();
      snapData.getSnapTransform().setIdentity();
      snapData.getWiggleTransformInWorld().setIdentity();
      snapData.getCroppedFoothold().clearAndUpdate();
      return snapData;
   }
}
