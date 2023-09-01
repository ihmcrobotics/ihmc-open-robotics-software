package us.ihmc.avatar.reachabilityMap;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DKey;

public class Voxel3DGridTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testVoxel3DKey()
   {
      Random random = new Random(83957);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(Voxel3DGrid.MAX_GRID_SIZE_VOXELS - 1) + 1;
         int x = random.nextInt(size);
         int y = random.nextInt(size);
         int z = random.nextInt(size);
         int arrayIndex = Voxel3DKey.toArrayIndex(x, y, z, size);

         assertEquals(x, Voxel3DKey.toXindex(arrayIndex, size));
         assertEquals(y, Voxel3DKey.toYindex(arrayIndex, size));
         assertEquals(z, Voxel3DKey.toZindex(arrayIndex, size));
      }
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(Voxel3DGrid.MAX_GRID_SIZE_VOXELS - 1) + 1;
         int x = random.nextInt(size);
         int y = random.nextInt(size);
         int z = random.nextInt(size);
         Voxel3DKey keyA = new Voxel3DKey(x, y, z, size);
         Voxel3DKey keyB = new Voxel3DKey(keyA.getIndex(), size);
         
         assertEquals(keyA, keyB);
      }
   }
}
