package us.ihmc.graphics3DAdapter.graphics.instructions;

import javax.media.j3d.Transform3D;

import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceDefinition;

public class LinkGraphicsAddHeightMap extends LinkGraphicsInstruction
{
   private final HeightMap heightMap;
   private final int xPointsPerSide, yPointsPerSide;
   private final Transform3D transform;
   
   public LinkGraphicsAddHeightMap(HeightMap heightMap, int xPointsPerSide, int yPointsPerSide, YoAppearanceDefinition appearance, Transform3D transform)
   {
      this.heightMap = heightMap;
      this.xPointsPerSide = xPointsPerSide;
      this.yPointsPerSide = yPointsPerSide;
      
      this.appearance = appearance;
      
      this.transform = transform;
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

   public Transform3D getTransform()
   {
      return transform;
   }
   
}
