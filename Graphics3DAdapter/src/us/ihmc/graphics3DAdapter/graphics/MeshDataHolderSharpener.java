package us.ihmc.graphics3DAdapter.graphics;

import java.util.ArrayList;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;

/***
 * Sharpens the edges by generating redundant vertices so that they can have surface normals aligned with their face,
 * rather than averaged with several faces which then gets rid of the sharp edge.
 *
 */

public class MeshDataHolderSharpener
{
   ArrayList<Point3f> vertices = new ArrayList<Point3f>();
   ArrayList<TexCoord2f> textPoints = new ArrayList<TexCoord2f>();
   ArrayList<Integer> polygonIndices = new ArrayList<Integer>();
   ArrayList<Integer> polygonStripCounts = new ArrayList<Integer>();
   
   public MeshDataHolderSharpener()
   {
      
   }
   
   public MeshDataHolder sharpenMeshData(MeshDataHolder inputMesh)
   {
      Point3f[] inputVertices = inputMesh.getVertices();
      TexCoord2f[] inputTextPoints = inputMesh.getTexturePoints();
      int[] inputPolygonIndices = inputMesh.getPolygonIndices();
      int[] inputPolygonStripCounts = inputMesh.getPolygonStripCounts();
      
      
      vertices.clear();
      textPoints.clear();
      polygonIndices.clear();
      polygonStripCounts.clear();
      
      for (int polygonIndicesIndex = 0; polygonIndicesIndex < inputPolygonIndices.length; polygonIndicesIndex++)
      {
         int index = inputPolygonIndices[polygonIndicesIndex];
         Point3f vertex = inputVertices[index];
         TexCoord2f textPoint = inputTextPoints[index];
         
         int newIndex = vertices.size();
         vertices.add(new Point3f(vertex));
         
         textPoints.add(new TexCoord2f(textPoint));
         polygonIndices.add(newIndex);
      }    
      
      Point3f[] outputVertices = new Point3f[vertices.size()];
      vertices.toArray(outputVertices);
      
      TexCoord2f[] outputTextPoints = new TexCoord2f[textPoints.size()];
      textPoints.toArray(outputTextPoints);
            
      int[] outputPolygonIndices = new int[polygonIndices.size()];
      for (int i=0; i<polygonIndices.size(); i++)
      {
         outputPolygonIndices[i] = polygonIndices.get(i);
      }
      
      int[] outputPolygonStripCounts = inputPolygonStripCounts;
      
      
      MeshDataHolder outputMesh = new MeshDataHolder(outputVertices, outputTextPoints, outputPolygonIndices, outputPolygonStripCounts);
      
      return outputMesh;
   }
}
