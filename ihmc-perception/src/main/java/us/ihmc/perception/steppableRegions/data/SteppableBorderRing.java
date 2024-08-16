package us.ihmc.perception.steppableRegions.data;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class SteppableBorderRing implements Iterable<SteppableCell>
{
   private final List<SteppableCell> ringCells = new ArrayList<>();
   private int ringNumber;

   public SteppableBorderRing(int ringNumber)
   {
      this.ringNumber = ringNumber;
   }

   public void addCell(SteppableCell cell)
   {
      cell.setBorderRing(this);
      ringCells.add(cell);
   }

   public boolean mergeRing(SteppableBorderRing other)
   {
      if (this == other)
         return false;

      other.ringCells.forEach(cell ->
                              {
                                 cell.setBorderRing(this);
                                 ringCells.add(cell);
                              });

      other.clear();

      return true;
   }

   public int size()
   {
      return ringCells.size();
   }

   public void clear()
   {
      ringNumber = -1;
      ringCells.clear();
   }

   @Override
   public Iterator<SteppableCell> iterator()
   {
      return ringCells.iterator();
   }

   public int getRingNumber()
   {
      return ringNumber;
   }

   @Override
   public String toString()
   {
      return "Ring : " + getRingNumber() + " with " + size() + " cells.";
   }
}
