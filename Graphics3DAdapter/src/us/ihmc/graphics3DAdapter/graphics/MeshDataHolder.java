package us.ihmc.graphics3DAdapter.graphics;

import javax.vecmath.Point3d;
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
   
   public static MeshDataHolder createFromVerticesAndStripCounts(Point3d[] vertices, int[] polygonStripCounts)
   {
      Point3f[] verticesWithFloats = new Point3f[vertices.length];
      
      for (int i=0; i<vertices.length; i++)
      {
         verticesWithFloats[i] = new Point3f(vertices[i]);
      }
      
      return createFromVerticesAndStripCounts(verticesWithFloats, polygonStripCounts);
   }
   
   public static MeshDataHolder createFromVerticesAndStripCounts(Point3f[] vertices, int[] polygonStripCounts)
   {
      int[] polygonIndices = new int[vertices.length];

      for (int i=0; i<vertices.length; i++)
      {
         polygonIndices[i] = i;
      }

      TexCoord2f[] textPoints = new TexCoord2f[vertices.length];
      for (int i=0; i<vertices.length; i++)
      {
         textPoints[i] = new TexCoord2f();
      }

      return new MeshDataHolder(vertices, textPoints, polygonIndices, polygonStripCounts);
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
