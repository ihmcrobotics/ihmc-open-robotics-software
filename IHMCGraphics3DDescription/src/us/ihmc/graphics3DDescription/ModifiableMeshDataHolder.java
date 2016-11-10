package us.ihmc.graphics3DDescription;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Vector3f;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class ModifiableMeshDataHolder
{
   private final RecyclingArrayList<Point3f> vertices = new RecyclingArrayList<>(Point3f.class);
   private final RecyclingArrayList<TexCoord2f> texturePoints = new RecyclingArrayList<>(TexCoord2f.class);
   private final TIntArrayList triangleIndices = new TIntArrayList();
   private final RecyclingArrayList<Vector3f> vertexNormals = new RecyclingArrayList<>(Vector3f.class);

   public ModifiableMeshDataHolder()
   {
   }

   public void clear()
   {
      vertices.clear();
      texturePoints.clear();
      triangleIndices.reset();
      vertexNormals.clear();
   }

   public MeshDataHolder createMeshDataHolder()
   {
      Point3f[] vertexArray = (Point3f[]) vertices.toArray(new Point3f[0]);
      TexCoord2f[] texturePointArray = (TexCoord2f[]) texturePoints.toArray(new TexCoord2f[0]);
      int[] triangleIndexArray = triangleIndices.toArray();
      Vector3f[] vertexNormalArray = (Vector3f[]) vertexNormals.toArray(new Vector3f[0]);
      return new MeshDataHolder(vertexArray, texturePointArray, triangleIndexArray, vertexNormalArray);
   }

   public void add(MeshDataHolder meshDataHolder, boolean updateTriangleIndices)
   {
      if (meshDataHolder == null)
         return;
      Point3f[] otherVertices = meshDataHolder.getVertices();
      if (otherVertices == null || otherVertices.length < 3)
         return;
      int[] otherTriangleIndices = meshDataHolder.getTriangleIndices();
      if (otherTriangleIndices == null || otherTriangleIndices.length < 3)
         return;

      if (updateTriangleIndices)
      {
         int shift = vertices.size();
         for (int triangleIndex : otherTriangleIndices)
            triangleIndices.add(triangleIndex + shift);
      }
      else
      {
         triangleIndices.add(otherTriangleIndices);
      }
      for (Point3f vertex : otherVertices)
         vertices.add().set(vertex);
      for (TexCoord2f texturePoint : meshDataHolder.getTexturePoints())
         texturePoints.add().set(texturePoint);
      for (Vector3f normal : meshDataHolder.getVertexNormals())
         vertexNormals.add().set(normal);
   }

   public void add(ModifiableMeshDataHolder other, boolean updateTriangleIndices)
   {
      if (updateTriangleIndices)
      {
         int shift = vertices.size();
         for (int i = 0; i < other.triangleIndices.size(); i++)
            triangleIndices.add(other.triangleIndices.get(i) + shift);
      }
      else
      {
         triangleIndices.addAll(other.triangleIndices);
      }
      for (int i = 0; i < other.vertices.size(); i++)
         vertices.add().set(other.vertices.get(i));
      for (int i = 0; i < other.texturePoints.size(); i++)
         texturePoints.add().set(other.texturePoints.get(i));
      for (int i = 0; i < other.vertexNormals.size(); i++)
         vertexNormals.add().set(other.vertexNormals.get(i));
   }
}
