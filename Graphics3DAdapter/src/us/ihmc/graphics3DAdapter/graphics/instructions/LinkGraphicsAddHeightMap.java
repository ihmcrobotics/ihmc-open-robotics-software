package us.ihmc.graphics3DAdapter.graphics.instructions;

import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceDefinition;

public class LinkGraphicsAddHeightMap extends LinkGraphicsInstruction
{
   private final HeightMap heightMap;
   
   public LinkGraphicsAddHeightMap(HeightMap heightMap, YoAppearanceDefinition appearance)
   {
      this.heightMap = heightMap;
      this.appearance = appearance;
   }

   public HeightMap getHeightMap()
   {
      return heightMap;
   }
   
}
