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
   private double rmsErrorHeightMap = Double.NaN;
   private double heightMapSnapArea = Double.NaN;
   private boolean snappedFootstepTransformIncludesWiggleTransform = false;
   private boolean snappedToPlanarRegions = false;
   private boolean snappedToHeightMap = false;

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

   public void setRMSErrorHeightMap(double rSquaredHeightMap)
   {
      this.rmsErrorHeightMap = rSquaredHeightMap;
   }

   public void setHeightMapArea(double area)
   {
      this.heightMapSnapArea = area;
   }

   public double getHeightMapArea()
   {
      return heightMapSnapArea;
   }

   public void setSnappedToPlanarRegions(boolean snappedToPlanarRegions)
   {
      this.snappedToPlanarRegions = snappedToPlanarRegions;
   }

   public void setSnappedToHeightMap(boolean snappedToHeightMap)
   {
      this.snappedToHeightMap = snappedToHeightMap;
   }

   @Override
   public boolean getSnappedToPlanarRegions()
   {
      return snappedToPlanarRegions;
   }

   @Override
   public boolean getSnappedToHeightMap()
   {
      return snappedToHeightMap;
   }

   @Override
   public double getRMSErrorHeightMap()
   {
      return rmsErrorHeightMap;
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

   public void set(FootstepSnapDataReadOnly other)
   {
      this.snapTransform.set(other.getSnapTransform());
      this.croppedFoothold.set(other.getCroppedFoothold());
      this.wiggleTransformInWorld.set(other.getWiggleTransformInWorld());
//      this.snappedFootstepTransform.set(other.snappedFootstepTransform);
      this.regionIndex = other.getRegionIndex();
      this.achievedInsideDelta = other.getAchievedInsideDelta();
      this.snappedFootstepTransformIncludesWiggleTransform = false;

      this.heightMapSnapArea = other.getHeightMapArea();
      this.rmsErrorHeightMap = other.getRMSErrorHeightMap();
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
      rmsErrorHeightMap = Double.NaN;
      snappedToPlanarRegions = false;
      snappedToHeightMap = false;
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
      return identityData(0.0);
   }

   public static FootstepSnapData identityData(double flatGroundHeight)
   {
      FootstepSnapData snapData = new FootstepSnapData();
      snapData.getSnapTransform().setIdentity();
      snapData.getSnapTransform().getTranslation().setZ(flatGroundHeight);
      snapData.getWiggleTransformInWorld().setIdentity();
      snapData.getCroppedFoothold().clearAndUpdate();
      return snapData;
   }
}
