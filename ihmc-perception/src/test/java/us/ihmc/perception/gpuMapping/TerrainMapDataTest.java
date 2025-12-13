package us.ihmc.perception.gpuMapping;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TerrainMapDataTest
{
   @Test
   public void testDataConversion()
   {
      // First compute an equivalent for x and y
      float initialData = 0.5f;
      byte byteVal = TerrainMapData.packFloatAsByte(initialData, -1.0f, 1.0f);
      float returnedFloat = TerrainMapData.unpackByteAsFloat(byteVal, -1.0f, 1.0f);
      assertEquals(initialData, returnedFloat, 2e-3);

      // Now compute an equivalent for z
      byteVal = TerrainMapData.packFloatAsByte(initialData, 0.0f, 1.0f);
      returnedFloat = TerrainMapData.unpackByteAsFloat(byteVal, 0.0f, 1.0f);
      assertEquals(initialData, returnedFloat, 2e-3);
   }

}
