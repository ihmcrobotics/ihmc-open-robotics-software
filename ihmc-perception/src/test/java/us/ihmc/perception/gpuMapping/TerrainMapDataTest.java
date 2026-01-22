package us.ihmc.perception.gpuMapping;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.perception.gpuMapping.TerrainMapData.*;

public class TerrainMapDataTest
{
   @Test
   public void testDataConversion()
   {
      // First compute an equivalent for x and y
      float initialData = 0.5f;
      byte byteVal = TerrainMapData.packFloatAsByte(initialData, -NORMAL_MIN_MAX_XY, NORMAL_MIN_MAX_XY);
      float returnedFloat = TerrainMapData.unpackByteAsFloat(byteVal, -NORMAL_MIN_MAX_XY, NORMAL_MIN_MAX_XY);
      assertEquals(initialData, returnedFloat, 2e-3);

      // Now compute an equivalent for z
      byteVal = TerrainMapData.packFloatAsByte(initialData, NORMAL_MIN_Z, NORMAL_MAX_Z);
      returnedFloat = TerrainMapData.unpackByteAsFloat(byteVal, NORMAL_MIN_Z, NORMAL_MAX_Z);
      assertEquals(initialData, returnedFloat, 2e-3);
   }

}
