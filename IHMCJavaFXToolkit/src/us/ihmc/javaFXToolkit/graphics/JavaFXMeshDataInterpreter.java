package us.ihmc.javaFXToolkit.graphics;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Tuple2f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import org.apache.commons.lang3.tuple.Triple;

import gnu.trove.list.array.TIntArrayList;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;

public class JavaFXMeshDataInterpreter
{
   public static TriangleMesh interpretMeshData(MeshDataHolder meshData)
   {
      if (meshData == null)
         return null;

      Point3f[] vertices = meshData.getVertices();
      TexCoord2f[] texturePoints = meshData.getTexturePoints();
      int[] polygonIndices = meshData.getPolygonIndices();
      int[] pointsPerPolygonCount = meshData.getPolygonStripCounts();

      TIntArrayList facesVertexOnlyIndices = new TIntArrayList();
      TIntArrayList facesIndices = new TIntArrayList();

      int polygonIndicesStart = 0;
      for (int index = 0; index < pointsPerPolygonCount.length; index++)
      {
         int pointsForThisPolygon = pointsPerPolygonCount[index];
         int[] polygon = new int[pointsForThisPolygon];

         for (int i = 0; i < pointsForThisPolygon; i++)
         {
            polygon[i] = polygonIndices[polygonIndicesStart + i];
         }

         Triple<int[], Point3f[], TexCoord2f[]> newPolygonData = splitPolygonIntoTriangles(polygon, vertices, texturePoints);
         int[] splitIntoTriangles = newPolygonData.getLeft();
         vertices = newPolygonData.getMiddle();
         texturePoints = newPolygonData.getRight();

         for (int i : splitIntoTriangles)
         {
            facesVertexOnlyIndices.add(i);
            facesIndices.add(i); // vertex index
            facesIndices.add(i); // normal index
            facesIndices.add(i); // texture index
         }

         polygonIndicesStart += pointsForThisPolygon;
      }

      int[] indices = facesIndices.toArray();
      float[] normals = findNormalsPerVertex(facesVertexOnlyIndices.toArray(), vertices);

      TriangleMesh triangleMesh = new TriangleMesh(VertexFormat.POINT_NORMAL_TEXCOORD);
      triangleMesh.getPoints().addAll(convertToFloatArray(vertices));
      triangleMesh.getTexCoords().addAll(convertToFloatArray(texturePoints));
      triangleMesh.getFaces().addAll(indices);
      triangleMesh.getFaceSmoothingGroups().addAll(new int[indices.length / triangleMesh.getFaceElementSize()]);
      triangleMesh.getNormals().addAll(normals);

      return triangleMesh;
   }

   private static float[] findNormalsPerVertex(int[] indices, Point3f[] vertices)
   {
      Map<Integer, Set<Integer>> participatingFacesPerVertex = new LinkedHashMap<Integer, Set<Integer>>();

      Set<Integer> vertexFacesSet;
      for (int i = 0; i < indices.length; i++)
      {
         if (participatingFacesPerVertex.get(indices[i]) == null)
         {
            vertexFacesSet = new LinkedHashSet<Integer>();
            participatingFacesPerVertex.put(indices[i], vertexFacesSet);
         }
         else
         {
            vertexFacesSet = participatingFacesPerVertex.get(indices[i]);
         }

         vertexFacesSet.add(i / 3); // Abuse integer division.
      }

      Vector3f[] normalsPerFace = findNormalsPerFace(indices, vertices);

      int pos = 0;
      float[] normals = new float[3 * vertices.length];
      Vector3f vertexNormal, faceNormal;
      for (int vertexIndex = 0; vertexIndex < vertices.length; vertexIndex++)
      {
         Set<Integer> participatingFaceIndices = participatingFacesPerVertex.get(vertexIndex);
         if (participatingFaceIndices == null)
            continue;
         vertexNormal = new Vector3f();
         for (Integer face : participatingFaceIndices)
         {
            faceNormal = normalsPerFace[face];
            vertexNormal.add(faceNormal);
         }
         vertexNormal.negate();
         float faces = (float) participatingFaceIndices.size();
         normals[pos++] = vertexNormal.x / faces;
         normals[pos++] = vertexNormal.y / faces;
         normals[pos++] = vertexNormal.z / faces;
      }

      return normals;
   }

   private static Vector3f[] findNormalsPerFace(int[] indices, Point3f[] vertices)
   {
      Vector3f[] normalsPerFace = new Vector3f[indices.length / 3]; // Abuse integer division.

      Vector3f firstVector = new Vector3f();
      Vector3f secondVector = new Vector3f();
      Point3f[] faceVertices = new Point3f[3];

      for (int face = 0; face < normalsPerFace.length; face++)
      {
         normalsPerFace[face] = new Vector3f();

         for (int i = 0; i < faceVertices.length; i++)
         {
            faceVertices[i] = vertices[indices[face * 3 + i]];
         }

         firstVector.set(faceVertices[2]);
         firstVector.sub(faceVertices[1]);

         secondVector.set(faceVertices[2]);
         secondVector.sub(faceVertices[0]);

         normalsPerFace[face].cross(firstVector, secondVector);
         normalsPerFace[face].normalize();
      }

      return normalsPerFace;
   }

   private static Triple<int[], Point3f[], TexCoord2f[]> splitPolygonIntoTriangles(int[] polygonIndices, Point3f[] vertices, TexCoord2f[] textCoords)
   {
      if (polygonIndices.length <= 3)
         return Triple.of(polygonIndices, vertices, textCoords);

      // Do a naive way of splitting a polygon into triangles. Assumes convexity and ccw ordering.
      int[] triangleIndices = new int[3 * (polygonIndices.length + 1)];
      Point3f[] newVertices = new Point3f[vertices.length + 1];
      System.arraycopy(vertices, 0, newVertices, 0, vertices.length);
      Point3f averageVertex = newVertices[vertices.length] = new Point3f();
      TexCoord2f[] newTextCoords = new TexCoord2f[textCoords.length + 1];
      System.arraycopy(textCoords, 0, newTextCoords, 0, textCoords.length);
      TexCoord2f averageTextCoord = newTextCoords[textCoords.length] = new TexCoord2f();

      int i = 0;

      averageVertex.add(vertices[polygonIndices[0]]);
      averageTextCoord.add(textCoords[polygonIndices[0]]);

      for (int j = 0; j < polygonIndices.length; j++)
      {
         triangleIndices[i++] = vertices.length;
         triangleIndices[i++] = polygonIndices[j];
         triangleIndices[i++] = polygonIndices[(j + 1) % polygonIndices.length];

         averageVertex.add(vertices[polygonIndices[j]]);
         averageTextCoord.add(textCoords[polygonIndices[j]]);
      }
      averageVertex.scale(1.0f / polygonIndices.length);
      averageTextCoord.scale(1.0f / polygonIndices.length);

      return Triple.of(triangleIndices, newVertices, newTextCoords);
   }

   private static float[] convertToFloatArray(Tuple3f[] tuple3fs)
   {
      float[] array = new float[3 * tuple3fs.length];
      int index = 0;
      for (Tuple3f tuple : tuple3fs)
      {
         array[index++] = tuple.getX();
         array[index++] = tuple.getY();
         array[index++] = tuple.getZ();
      }
      return array;
   }

   private static float[] convertToFloatArray(Tuple2f[] tuple2fs)
   {
      float[] array = new float[2 * tuple2fs.length];
      int index = 0;
      for (Tuple2f tuple : tuple2fs)
      {
         array[index++] = tuple.getX();
         array[index++] = tuple.getY();
      }
      return array;
   }
}
