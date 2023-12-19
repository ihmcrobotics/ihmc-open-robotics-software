package us.ihmc.perception.steppableRegions.data;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.perception.steppableRegions.SteppableRegionsCalculator;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.sensorProcessing.heightMap.HeightGridContainer;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class SteppableRegionDataHolder
{
   public int regionNumber;
   private boolean isPlaneUpToDate = false;

   private int lastRingNumber = -1;

   private final List<SteppableCell> memberCells = new ArrayList<>();
   private final List<Point3DReadOnly> memberPoints = new ArrayList<>();
   private final List<SteppableBorderRing> borderCellRings = new ArrayList<>();

   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   private final Point3D regionCentroid = new Point3D();
   private final Vector3D regionNormal = new Vector3D();

   private final Plane3D plane = new Plane3D();

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

   public void addCell(SteppableCell cell, double gridCenterX, double gridCenterY, double resolutionXY, int centerIndex, HeightGridContainer heightMapData)
   {
      addCell(cell, SteppableRegionsCalculator.convertCellToPoint(cell, gridCenterX, gridCenterY, resolutionXY, centerIndex, heightMapData));
   }

   public void addCell(SteppableCell cell, Point3DReadOnly point)
   {
      markChanged();
      memberCells.add(cell);
      memberPoints.add(point);

      cell.setRegion(this);

      minX = Math.min(minX, cell.getXIndex());
      maxX = Math.max(maxX, cell.getXIndex());
      minY = Math.min(minY, cell.getYIndex());
      maxY = Math.max(maxY, cell.getYIndex());
   }

   public boolean mergeRegion(SteppableRegionDataHolder other)
   {
      if (this == other)
         return false;

      for (int i = 0; i < other.memberCells.size(); i++)
      {
         addCell(other.memberCells.get(i), other.memberPoints.get(i));
      }

      this.borderCellRings.addAll(other.borderCellRings);

      return true;
   }

   public Point3DReadOnly getCentroidInWorld()
   {
      if (!isPlaneUpToDate)
         updatePlane();
      return regionCentroid;
   }

   public Vector3DReadOnly getNormalInWorld()
   {
      if (!isPlaneUpToDate)
         updatePlane();
      return regionNormal;
   }

   public Collection<SteppableCell> getCells()
   {
      return memberCells;
   }

   private void markChanged()
   {
      isPlaneUpToDate = false;
   }

   private void updatePlane()
   {
      planeFitter.fitPlaneToPoints(memberPoints, plane);

      regionCentroid.set(plane.getPoint());
      regionNormal.set(plane.getNormal());

      isPlaneUpToDate = true;
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

   public List<Point3DReadOnly> getMemberPoints()
   {
      return memberPoints;
   }

   public void clear()
   {
      regionNumber = -1;
      memberCells.clear();
      borderCellRings.clear();
      memberPoints.clear();
   }

   @Override
   public String toString()
   {
      return "Region : " + regionNumber + " with " + memberCells.size() + " cells and " + borderCellRings.size() + " rings.";
   }
}
