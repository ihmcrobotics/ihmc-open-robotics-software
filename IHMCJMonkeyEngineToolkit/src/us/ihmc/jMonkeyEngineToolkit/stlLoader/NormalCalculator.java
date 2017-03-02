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

import java.util.List;

/**
 * Calculate surface normals based on http://www.opengl.org/wiki/Calculating_a_Surface_Normal
 * 
 * Apparently, STL files do not always have normal data.
 * 
 * Using primitive math for maximum portability.
 * 
 * @author Jesper Smith
 *
 */
public class NormalCalculator
{
   /**
    * Calculate a normal vector based on three vertex coordinates that make up a triangle using the right hand rule.
    * 
    * @param normalToPack Set to the calculated normal
    * @param vertices Vertices that make up the triangle
    */
   public static void calculateNormal(float[] normalToPack, List<float[]> vertices)
   {
      float[] v1 = vertices.get(0);
      float[] v2 = vertices.get(1);
      float[] v3 = vertices.get(2);

      // Calculate U x H = (v2 - v1) x (v3 - v1)

      float Ux = v2[0] - v1[0];
      float Uy = v2[1] - v1[1];
      float Uz = v2[2] - v1[2];

      float Vx = v3[0] - v1[0];
      float Vy = v3[1] - v1[1];
      float Vz = v3[2] - v1[2];

      float Nx = Uy * Vz - Uz * Vy;
      float Ny = Uz * Vx - Ux * Vz;
      float Nz = Ux * Vy - Uy * Vx;

      normalToPack[0] = Nx;
      normalToPack[1] = Ny;
      normalToPack[2] = Nz;
      
   }

   /**
    * Compare if two normals are within epsilon
    * 
    * Normalizes both vectors in place
    * 
    * @param a normal vector
    * @param b normal vector
    * @param epsilon
    * @return true if normals are equal
    */
   public static boolean compareNormal(float[] a, float[] b, float epsilon)
   {
      normalize(a);
      normalize(b);
      return Math.abs(a[0] - b[0]) < epsilon && Math.abs(a[1] - b[1]) < epsilon  && Math.abs(a[2] - b[2]) < epsilon;
   }
   
   /**
    * Checks if normal vector is zero
    * 
    * @param normal to check
    * @param epsilon
    * @return true if normal is zero
    */
   public static boolean isZeroNormal(float[] normal, float epsilon)
   {
      return Math.abs(normal[0]) < epsilon && Math.abs(normal[1]) < epsilon  && Math.abs(normal[2]) < epsilon ;
   }
   
   /**
    * Normalize normal vector
    * 
    * @param normal Vector normalized in place
    */
   public static void normalize(float[] normal)
   {
      float length = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
      if (length != 1f && length != 0f){
          length = 1.0f / (float)Math.sqrt(length);
          normal[0] *= length;
          normal[1] *= length;
          normal[2] *= length;
      }
   }
}
