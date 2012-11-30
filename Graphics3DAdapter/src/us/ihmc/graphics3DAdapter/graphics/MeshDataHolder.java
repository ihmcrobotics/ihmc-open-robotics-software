package us.ihmc.graphics3DAdapter.graphics;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;

public class MeshDataHolder
{
   private final Point3f[] vertices;
   private final TexCoord2f[] texturePoints;
   private final int[] polygonIndices;
   private final int[] polygonStripCounts;
   
   public MeshDataHolder(Point3f[] vertices, TexCoord2f[] textPoints, int[] polygonIndices, int[] polygonStripCounts)
   {
      this.vertices = vertices;
      this.texturePoints = textPoints;
      this.polygonIndices = polygonIndices;
      this.polygonStripCounts = polygonStripCounts;
   }
   
   public Point3f[] getVertices()
   {
      return vertices;
   }
   
   public TexCoord2f[] getTexturePoints()
   {
      return texturePoints;
   }
   
   public int[] getPolygonIndices()
   {
      return polygonIndices;
   }

   public int[] getPolygonStripCounts()
   {
      return polygonStripCounts;
   }
}
