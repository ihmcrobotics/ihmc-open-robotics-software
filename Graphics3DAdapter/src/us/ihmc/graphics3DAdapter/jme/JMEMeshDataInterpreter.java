package us.ihmc.graphics3DAdapter.jme;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;
import us.ihmc.graphics3DAdapter.jme.util.JMEDataTypeUtils;

import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;

public class JMEMeshDataInterpreter
{

   public static Mesh interpretMeshData(MeshDataHolder meshData)
   {
      Vector3f[] vertices = JMEDataTypeUtils.vecMathTuple3fArrayToJMEVector3fArray(meshData.getVertices());
      Vector2f[] textureCoords = JMEDataTypeUtils.texCoord2fArrayToJMEVector2fArray(meshData.getTexturePoints());
      int[] polygonIndices = meshData.getPolygonIndices();
      int[] pointsPerPolygonCount = meshData.getPolygonStripCounts();
            
      ArrayList<Integer> triangleIndices = new ArrayList<Integer>();
      
      int polygonIndicesStart = 0;
      for (int pointsForThisPolygon : pointsPerPolygonCount)
      {
         int[] polygon = new int[pointsForThisPolygon];
         
         for(int i = 0; i < pointsForThisPolygon; i++)
         {
            polygon[i] = polygonIndices[polygonIndicesStart + i];
         }
         
         int[] splitIntoTriangles = splitPolygonIntoTriangles(polygon);
         
         for (int i : splitIntoTriangles)
         {
            triangleIndices.add(i);
         }
         
         polygonIndicesStart += pointsForThisPolygon;
      }
      
      int[] indices = new int[triangleIndices.size()];
      for(int i = 0; i < indices.length; i++)
      {
         indices[i] = triangleIndices.get(i);
      }
      
      float[] normals = findNormalsPerVertex(indices, vertices);
      
      Mesh mesh = new Mesh();
      mesh.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
      if(textureCoords != null)
         mesh.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(textureCoords));
      mesh.setBuffer(Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
      mesh.setBuffer(Type.Index, 3, BufferUtils.createIntBuffer(indices));
      mesh.updateBound();
      
      return mesh;
   }
   
   private static float[] findNormalsPerVertex(int[] indices, Vector3f[] vertices)
   {
      Map<Integer,Set<Integer>> participatingFacesPerVertex = new LinkedHashMap<Integer,Set<Integer>>();
      
      Set<Integer> vertexFacesSet;
      for (int i = 0; i < indices.length; i++)
      {
         if(participatingFacesPerVertex.get(indices[i]) == null)
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
         vertexNormal = new Vector3f();
         for (Integer face : participatingFaceIndices)
         {
            faceNormal = normalsPerFace[face];
            vertexNormal.addLocal(faceNormal);
         }
                  
         float faces = (float) participatingFaceIndices.size();
         normals[pos++] = vertexNormal.x / faces;
         normals[pos++] = vertexNormal.y / faces;
         normals[pos++] = vertexNormal.z / faces;
      }
      
      return normals;
   }

   private static Vector3f[] findNormalsPerFace(int[] indices, Vector3f[] vertices)
   {
      Vector3f[] normalsPerFace = new Vector3f[indices.length / 3]; // Abuse integer division. 
      
      Vector3f firstVector = new Vector3f();
      Vector3f secondVector = new Vector3f();
      Vector3f[] faceVertices = new Vector3f[3];
      for(int face = 0; face < normalsPerFace.length; face++)
      {
         normalsPerFace[face] = new Vector3f();
         
         for(int i = 0; i < faceVertices.length; i++)
         {
            faceVertices[i] = vertices[indices[face * 3 + i]];
         }

         firstVector.set(faceVertices[2]);
         firstVector.subtractLocal(faceVertices[1]);        
         
         secondVector.set(faceVertices[2]);
         secondVector.subtractLocal(faceVertices[0]);    
         
         normalsPerFace[face] = secondVector.cross(firstVector, normalsPerFace[face]);
         
         normalsPerFace[face].normalizeLocal();
      }
      
      return normalsPerFace;
   }

   private static int[] splitPolygonIntoTriangles(int[] polygonIndices)
   {
      if(polygonIndices.length <= 3)
         return polygonIndices;
   
      // Do a naive way of splitting a polygon into triangles. Assumes convexity and ccw ordering.
      int[] ret = new int[3 * (polygonIndices.length - 2)];
      int i = 0;
      for(int j = 2; j < polygonIndices.length; j++)
      {
         ret[i++] = polygonIndices[0];
         ret[i++] = polygonIndices[j-1];
         ret[i++] = polygonIndices[j];
      }
      
      return ret;
   }

   
}
