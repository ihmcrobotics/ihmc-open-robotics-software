package us.ihmc.graphicsDescription;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.robotics.lists.RecyclingArrayList;

/**
 * This class provides an mutable data structure for 3D graphic mesh that is independent from the graphics engine to be used.
 * It contains all the data necessary to create a mesh.
 * Using the corresponding mesh data interpreter, a {@link ModifiableMeshDataHolder} can be translated into a specific mesh data type usable by a specific graphics engine such as JME or JavaFX.
 */
public class ModifiableMeshDataHolder
{
   private final RecyclingArrayList<Point3D32> vertices = new RecyclingArrayList<>(Point3D32.class);
   private final RecyclingArrayList<TexCoord2f> texturePoints = new RecyclingArrayList<>(TexCoord2f.class);
   private final TIntArrayList triangleIndices = new TIntArrayList();
   private final RecyclingArrayList<Vector3D32> vertexNormals = new RecyclingArrayList<>(Vector3D32.class);

   /**
    * Creates an empty mesh.
    */
   public ModifiableMeshDataHolder()
   {
   }

   /**
    * Clears this mesh data holder. After calling this method, this is an empty mesh.
    */
   public void clear()
   {
      vertices.clear();
      texturePoints.clear();
      triangleIndices.reset();
      vertexNormals.clear();
   }

   /**
    * Creates an immutable mesh data holder that can be used by a mesh data interpreter.
    * The data contained in this is copied in the immutable mesh data holder.
    * @return the immutable mesh data holder.
    */
   public MeshDataHolder createMeshDataHolder()
   {
      Point3D32[] vertexArray = (Point3D32[]) vertices.toArray(new Point3D32[0]);
      TexCoord2f[] texturePointArray = (TexCoord2f[]) texturePoints.toArray(new TexCoord2f[0]);
      int[] triangleIndexArray = triangleIndices.toArray();
      Vector3D32[] vertexNormalArray = (Vector3D32[]) vertexNormals.toArray(new Vector3D32[0]);
      return new MeshDataHolder(vertexArray, texturePointArray, triangleIndexArray, vertexNormalArray);
   }

   /**
    * Append a mesh to this.
    * @param meshDataHolder the mesh to append. Not modified.
    * @param updateTriangleIndices whether the indices of the given mesh should be updated when appended. Highly recommended, set it to false only if you what you are doing.
    */
   public void add(MeshDataHolder meshDataHolder, boolean updateTriangleIndices)
   {
      if (meshDataHolder == null)
         return;
      Point3D32[] otherVertices = meshDataHolder.getVertices();
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
      for (Point3D32 vertex : otherVertices)
         vertices.add().set(vertex);
      for (TexCoord2f texturePoint : meshDataHolder.getTexturePoints())
         texturePoints.add().set(texturePoint);
      for (Vector3D32 normal : meshDataHolder.getVertexNormals())
         vertexNormals.add().set(normal);
   }

   /**
    * Append a mesh to this.
    * @param other the mesh to append. Not modified.
    * @param updateTriangleIndices whether the indices of the given mesh should be updated when appended. Highly recommended, set it to false only if you what you are doing.
    */
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
