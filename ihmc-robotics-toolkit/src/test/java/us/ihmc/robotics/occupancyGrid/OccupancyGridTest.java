package us.ihmc.robotics.occupancyGrid;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.robotics.Assert.*;

public class OccupancyGridTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testDuplicatePoints()
   {
      OccupancyGrid occupancyGrid = new OccupancyGrid("", worldFrame, new YoRegistry("test"));
      assertEquals(1, occupancyGrid.registerPoint(new FramePoint2D()));
      assertEquals(1, occupancyGrid.registerPoint(new FramePoint2D()));

      assertTrue(occupancyGrid.occupancyCellMap.containsKey(OccupancyGridCell.computeHashCode(0, 0)));
      assertEquals(1, occupancyGrid.allCellsPool.size());
      assertTrue(occupancyGrid.isCellOccupied(0, 0));

      for (int i = 0; i < 10; i++)
         occupancyGrid.update();

      assertEquals(1, occupancyGrid.getNumberOfOccupiedCells());

      double decayRate = 0.1;
      int ticksToBarelyOccupied = (int) Math.floor(Math.log(0.5) / Math.log((1.0 - decayRate)));
      occupancyGrid.setOccupancyDecayRate(decayRate);
      for (int i = 0; i < ticksToBarelyOccupied; i++)
         occupancyGrid.update();
      assertTrue(occupancyGrid.isCellOccupied(0, 0));
      occupancyGrid.update();
      assertFalse(occupancyGrid.isCellOccupied(0, 0));
      occupancyGrid.registerPoint(new FramePoint2D());
      assertTrue(occupancyGrid.isCellOccupied(0, 0));
   }

   @Test
   public void testRegisteringFourPoints()
   {
      OccupancyGrid occupancyGrid = new OccupancyGrid("", worldFrame, new YoRegistry("test"));
      occupancyGrid.registerPoint(new FramePoint2D(worldFrame, 1.0, 1.0));
      occupancyGrid.registerPoint(new FramePoint2D(worldFrame, -1.0, 1.0));
      occupancyGrid.registerPoint(new FramePoint2D(worldFrame, -1.0, -1.0));
      occupancyGrid.registerPoint(new FramePoint2D(worldFrame, 1.0, -1.0));

      assertEquals(4, occupancyGrid.getAllActiveCells().size());

      assertEquals(4, occupancyGrid.allCellsPool.size());
      assertEquals(4, occupancyGrid.getNumberOfOccupiedCells());

      // check the points
      assertTrue(occupancyGrid.isCellOccupied(1.0, 1.0));
      assertTrue(occupancyGrid.isCellOccupied(-1.0, 1.0));
      assertTrue(occupancyGrid.isCellOccupied(-1.0, -1.0));
      assertTrue(occupancyGrid.isCellOccupied(1.0, -1.0));

      // check the off axis things
      assertFalse(occupancyGrid.isCellOccupied(0.0, -1));
      assertFalse(occupancyGrid.isCellOccupied(0.0, 1));
      assertFalse(occupancyGrid.isCellOccupied(1.0, 0.0));
      assertFalse(occupancyGrid.isCellOccupied(-1.0, 0.0));

      // CLEAR
      occupancyGrid.reset();

      // check the points
      assertEquals(0, occupancyGrid.getAllActiveCells().size());

      assertFalse(occupancyGrid.isCellOccupied(1.0, 1.0));
      assertFalse(occupancyGrid.isCellOccupied(-1.0, 1.0));
      assertFalse(occupancyGrid.isCellOccupied(-1.0, -1.0));
      assertFalse(occupancyGrid.isCellOccupied(1.0, -1.0));

      assertEquals(0, occupancyGrid.getAllActiveCells().size());
      assertEquals(0, occupancyGrid.getNumberOfOccupiedCells());
   }

   @Test
   public void testHashMapUniqueness()
   {
      int hashMap0 = OccupancyGridCell.computeHashCode(100, 100);
      int hashMap1 = OccupancyGridCell.computeHashCode(-100, 100);
      int hashMap2 = OccupancyGridCell.computeHashCode(-100, -100);
      int hashMap3 = OccupancyGridCell.computeHashCode(100, -100);

      assertFalse(hashMap0 == hashMap1);
      assertFalse(hashMap0 == hashMap2);
      assertFalse(hashMap0 == hashMap3);
      assertFalse(hashMap1 == hashMap2);
      assertFalse(hashMap1 == hashMap3);
      assertFalse(hashMap2 == hashMap3);
   }
}
