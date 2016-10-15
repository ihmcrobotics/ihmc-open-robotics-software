package us.ihmc.graphics3DAdapter.graphics;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3f;

public class MeshDataHolder
{
   private final Point3f[] vertices;
   private final TexCoord2f[] texturePoints;
   private final int[] triangleIndices;
   private final Vector3f[] vertexNormals;

   public MeshDataHolder(Point3f[] vertices, TexCoord2f[] textPoints, int[] triangleIndices, Vector3f[] polygonNormals)
   {
      this.vertices = vertices;
      this.texturePoints = textPoints;
      this.triangleIndices = triangleIndices;
      this.vertexNormals = polygonNormals;
   }

   public Point3f[] getVertices()
   {
      return vertices;
   }

   public TexCoord2f[] getTexturePoints()
   {
      return texturePoints;
   }

   public int[] getTriangleIndices()
   {
      return triangleIndices;
   }

   public Vector3f[] getVertexNormals()
   {
      return vertexNormals;
   }
}
