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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Data storage class for STL data, consisting of a surface normal and vertices
 * 
 * @author Jesper Smith
 *
 */
public class Triangle
{
   private final float[] normal = new float[3];
   private final ArrayList<float[]> vertices = new ArrayList<float[]>();

   void setNormal(float ni, float nj, float nk)
   {
      normal[0] = ni;
      normal[1] = nj;
      normal[2] = nk;
   }

   void addVertex(float vx, float vy, float vz)
   {
      float[] vertex = { vx, vy, vz };
      vertices.add(vertex);
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("Triangle [normal=").append(Arrays.toString(normal)).append(", vertices=");
      for (float[] vertex : vertices)
      {
         builder.append(Arrays.toString(vertex));
      }
      return builder.toString();
   }

   /**
    * Get the surface normal for this triangle, as defined in the STL file
    * 
    * @return normal vector (x, y, z)
    */
   public float[] getNormal()
   {
      return normal;
   }
   
   /**
    * Get the specified vertex for this triangle
    * 
    * @param vertex Get the numbered vertex (0, 1, 2)
    * @return Vertex coordinates (x,y,z)
    */
   public float[] getVertex(int vertex)
   {
      return vertices.get(vertex);
   }

   /**
    * Get all 3 vertices for this triangle
    * 
    * @return vertices
    */
   public List<float[]> getVertices()
   {
      return vertices;
   }
   
}
