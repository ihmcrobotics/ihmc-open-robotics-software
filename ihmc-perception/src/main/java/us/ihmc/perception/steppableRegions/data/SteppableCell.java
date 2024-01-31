package us.ihmc.perception.steppableRegions.data;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class SteppableCell
{
   private final int x;
   private final int y;
   private final double z;
   private final Vector3D normal;

   private final int hashCode;
   private final boolean isBorderCell;

   private SteppableRegionDataHolder region;
   private boolean cellHasBeenExpanded = false;
   private SteppableBorderRing borderRing = null;

   private final List<SteppableCell> validNeighbors = new ArrayList<>();

   public SteppableCell(int x, int y, double z, Vector3DReadOnly normal, int cellsPerSide, boolean isBorderCell)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.normal = new Vector3D(normal);
      this.isBorderCell = isBorderCell;

      hashCode = x * cellsPerSide + y;
   }

   public void setCellHasBeenExpanded(boolean cellHasBeenExpanded)
   {
      this.cellHasBeenExpanded = cellHasBeenExpanded;
   }

   public boolean isBorderCell()
   {
      return isBorderCell;
   }

   public int getXIndex()
   {
      return x;
   }

   public int getYIndex()
   {
      return y;
   }

   public double getZ()
   {
      return z;
   }

   public Vector3DReadOnly getNormal()
   {
      return normal;
   }

   public boolean hasRegion()
   {
      return getRegion() != null;
   }

   public SteppableRegionDataHolder getRegion()
   {
      return region;
   }

   public SteppableBorderRing getBorderRing()
   {
      return borderRing;
   }

   public void setRegion(SteppableRegionDataHolder region)
   {
      this.region = region;
   }

   public void setBorderRing(SteppableBorderRing borderRing)
   {
      this.borderRing = borderRing;
   }

   public boolean cellHasBeenAssigned()
   {
      if (isBorderCell && region != null && borderRing == null)
         throw new RuntimeException("Shouldn't have gotten here!");

      return region != null;
   }

   public boolean cellHasBeenExpanded()
   {
      return cellHasBeenExpanded;
   }

   public List<SteppableCell> getValidNeighbors()
   {
      return validNeighbors;
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj instanceof SteppableCell other)
      {
         return hashCode == other.hashCode();
      }

      return false;
   }
}
