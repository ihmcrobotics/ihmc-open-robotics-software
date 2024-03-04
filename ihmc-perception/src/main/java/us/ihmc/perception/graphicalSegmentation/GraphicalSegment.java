package us.ihmc.perception.graphicalSegmentation;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple2D.Point2D;

public class GraphicalSegment
{
   protected final RecyclingArrayList<Point2D> regionIndices = new RecyclingArrayList<>(Point2D::new);
   protected final RecyclingArrayList<Point2D> borderIndices = new RecyclingArrayList<>(Point2D::new);
   protected int numberOfPatches;
   protected int id;

   public void reset(int id)
   {
      this.id = id;
      regionIndices.clear();
      borderIndices.clear();
      numberOfPatches = 0;
   }

   public void addRegionPatch(int row, int column)
   {
      regionIndices.add().set(column, row);
      numberOfPatches++;
   }

   public void addBorderPatch(int row, int column)
   {
      borderIndices.add().set(column, row);
   }

   public RecyclingArrayList<Point2D> getRegionIndices()
   {
      return regionIndices;
   }

   public RecyclingArrayList<Point2D> getBorderIndices()
   {
      return borderIndices;
   }

   public int getId()
   {
      return id;
   }

   public int getNumberOfPatches()
   {
      return numberOfPatches;
   }
}
