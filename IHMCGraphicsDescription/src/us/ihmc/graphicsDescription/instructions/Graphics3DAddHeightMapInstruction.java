package us.ihmc.graphicsDescription.instructions;

import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;

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
