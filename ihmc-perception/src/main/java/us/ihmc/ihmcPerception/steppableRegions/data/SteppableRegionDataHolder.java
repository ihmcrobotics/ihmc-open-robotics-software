package us.ihmc.ihmcPerception.steppableRegions.data;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class SteppableRegionDataHolder
{
   public int regionNumber;
   private boolean isCentroidUpToDate = false;
   private boolean isNormalUpToDate = false;

   private int lastRingNumber = -1;

   private final List<SteppableCell> memberCells = new ArrayList<>();
   private final List<SteppableBorderRing> borderCellRings = new ArrayList<>();

   private final Point3D regionCentroidTotal = new Point3D();
   private final Point3D regionCentroid = new Point3D();
   private final Vector3D regionNormal = new Vector3D();

   private int minX = Integer.MAX_VALUE;
   private int maxX = Integer.MIN_VALUE;
   private int minY = Integer.MAX_VALUE;
   private int maxY = Integer.MIN_VALUE;

   public SteppableRegionDataHolder(int regionNumber)
   {
      this.regionNumber = regionNumber;
   }

   public SteppableBorderRing createNewBorderRing()
   {
      lastRingNumber++;
      SteppableBorderRing borderRing = new SteppableBorderRing(10000 * regionNumber + lastRingNumber);
      borderCellRings.add(borderRing);
      return borderRing;
   }

   public List<SteppableBorderRing> getBorderRings()
   {
      return borderCellRings;
   }

   public void removeBorderRing(SteppableBorderRing ring)
   {
      ring.clear();
      borderCellRings.remove(ring);
   }

   public void addCell(SteppableCell cell)
   {
      markChanged();
      memberCells.add(cell);
      cell.setRegion(this);

      minX = Math.min(minX, cell.getX());
      maxX = Math.max(maxX, cell.getX());
      minY = Math.min(minY, cell.getY());
      maxY = Math.max(maxY, cell.getY());
   }

   public boolean mergeRegion(SteppableRegionDataHolder other)
   {
      if (this == other)
         return false;

      for (SteppableCell otherCell : other.memberCells)
      {
         addCell(otherCell);
      }
      this.borderCellRings.addAll(other.borderCellRings);

      return true;
   }

   public Point3DReadOnly getCentroidInWorld()
   {
      if (!isCentroidUpToDate)
         updateCentroid();
      return regionCentroid;
   }

   public Vector3DReadOnly getNormalInWorld()
   {
      if (!isNormalUpToDate)
         updateNormal();
      return regionNormal;
   }

   public Collection<SteppableCell> getCells()
   {
      return memberCells;
   }

   private void markChanged()
   {
      isCentroidUpToDate = false;
      isNormalUpToDate = false;
   }

   private void updateCentroid()
   {
      regionCentroid.setAndScale(1.0 / memberCells.size(), regionCentroidTotal);
      isCentroidUpToDate = true;
   }

   private void updateNormal()
   {
      // TODO run a least squares fit here
      regionNormal.set(0.0, 0.0, 1.0);
      isNormalUpToDate = false;
   }

   public int getMinX()
   {
      return minX;
   }

   public int getMaxX()
   {
      return maxX;
   }

   public int getMinY()
   {
      return minY;
   }

   public int getMaxY()
   {
      return maxY;
   }

   public List<SteppableCell> getMemberCells()
   {
      return memberCells;
   }

   public void clear()
   {
      regionNumber = -1;
      memberCells.clear();
      borderCellRings.clear();
   }

   @Override
   public String toString()
   {
      return "Region : " + regionNumber + " with " + memberCells.size() + " cells and " + borderCellRings.size() + " rings.";
   }
}
