package us.ihmc.graphics3DAdapter.graphics.instructions;

import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;

public class Graphics3DAddHeightMapInstruction extends Graphics3DInstruction
{
   private final HeightMap heightMap;
   private final int xPointsPerSide, yPointsPerSide;
   
   public Graphics3DAddHeightMapInstruction(HeightMap heightMap, int xPointsPerSide, int yPointsPerSide, AppearanceDefinition appearance)
   {
      this.heightMap = heightMap;
      this.xPointsPerSide = xPointsPerSide;
      this.yPointsPerSide = yPointsPerSide;
      
      setAppearance(appearance);
   }

   public HeightMap getHeightMap()
   {
      return heightMap;
   }
   
   public int getXPointsPerSide()
   {
      return xPointsPerSide;
   }
   
   public int getYPointsPerSide()
   {
      return yPointsPerSide;
   }
   
}
