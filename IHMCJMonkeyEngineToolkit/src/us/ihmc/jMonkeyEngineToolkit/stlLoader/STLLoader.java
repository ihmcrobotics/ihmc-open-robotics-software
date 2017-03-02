/*
 *   Copyright 2014 Florida Institute for Human and Machine Cognition (IHMC)
 *    
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    
 *    http://www.apache.org/licenses/LICENSE-2.0
 *    
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *    
 *    Written by Jesper Smith with assistance from IHMC team members
 */
package us.ihmc.jMonkeyEngineToolkit.stlLoader;

import java.io.IOException;
import java.util.List;

import com.jme3.asset.AssetInfo;
import com.jme3.asset.AssetLoader;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;

/**
 * JMonkeyEngine AssetLoader for the STL file format. Able to load both binary and ASCII format.
 * 
 * Normals are calculated based on the right hand rule when not defined in the STL file itself.
 * 
 * @usage  assetManager.registerLoader(STLLoader.class, "stl");
 * @usage  assetManager.loadModel("[filename].stl");
 * 
 * @author Jesper Smith
 *
 */
public class STLLoader implements AssetLoader
{

   public Object load(AssetInfo assetInfo) throws IOException
   {
      STLReader reader = STLReaderFactory.create(assetInfo);
      List<Triangle> triangles = reader.getTriangles();            
      
      Mesh mesh = new Mesh();
      Vector3f[] vertices = new Vector3f[triangles.size() * 3];
      Vector3f[] normals = new Vector3f[triangles.size() * 3];
      int[] indices = new int[triangles.size() * 3];

      for(int i = 0; i < triangles.size(); i++)
      {
         Triangle triangle = triangles.get(i);
         float[] normal = triangle.getNormal();
         
         if(NormalCalculator.isZeroNormal(normal, 1e-5f))
         {
            NormalCalculator.calculateNormal(normal, triangle.getVertices());
         }
         float ni = normal[0];
         float nj = normal[1];
         float nk = normal[2];
         Vector3f normal3f = new Vector3f(ni, nj, nk);
         
         for(int v = 0; v < 3; v++)
         {
            int index = i * 3 + v;
            float[] vertex = triangle.getVertex(v);
            float vx = vertex[0];
            float vy = vertex[1];
            float vz = vertex[2];
            vertices[index] = new Vector3f(vx, vy, vz);
            indices[index] = index;
            normals[index] = normal3f;
         }
      }
      
      mesh.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
      mesh.setBuffer(Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
      mesh.setBuffer(Type.Index, 3, BufferUtils.createIntBuffer(indices));
      mesh.updateBound();
      
      Geometry geom = new Geometry(reader.getName(), mesh);
      
      return geom;
   }
}
