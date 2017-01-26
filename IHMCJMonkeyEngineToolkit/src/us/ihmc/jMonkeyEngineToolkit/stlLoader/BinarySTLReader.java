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

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/**
 * Reader for the binary STL format
 * 
 * @see STLReaderFactory
 * 
 * @author Jesper Smith
 *
 */
public class BinarySTLReader implements STLReader
{
   private final String name;
   private final ArrayList<Triangle> triangles = new ArrayList<Triangle>();

   public BinarySTLReader(InputStream stream) throws IOException
   {
      DataInputStream dataInputStream = new DataInputStream(stream);
      
      this.name = getHeader(stream);
      
      byte[] backingArray = new byte[4];
      dataInputStream.readFully(backingArray);
      ByteBuffer elementsBuffer = ByteBuffer.wrap(backingArray);
      elementsBuffer.order(ByteOrder.LITTLE_ENDIAN);
      long numberOfTriangles = elementsBuffer.getInt() & 0xFFFFFFFF;
      if(numberOfTriangles > Integer.MAX_VALUE)
      {
         throw new IOException("Cannot handle more than " + Integer.MAX_VALUE + " elements, got " + numberOfTriangles + " elements");
      }
      
      for(int i = 0; i < numberOfTriangles; i++)
      {
         triangles.add(readTriangle(dataInputStream));
      }
      
      dataInputStream.close();
   }

   private Triangle readTriangle(DataInputStream dataInputStream) throws IOException
   {
      byte[] backingArray = new byte[50];
      dataInputStream.readFully(backingArray);
      ByteBuffer data = ByteBuffer.wrap(backingArray);
      data.order(ByteOrder.LITTLE_ENDIAN);

      
      Triangle triangle = new Triangle();
      float ni = data.getFloat();
      float nj = data.getFloat();
      float nk = data.getFloat();
      triangle.setNormal(ni, nj, nk);
      
      for(int i = 0; i < 3; i++)
      {
         float vx = data.getFloat();
         float vy = data.getFloat();
         float vz = data.getFloat();
         triangle.addVertex(vx, vy, vz);
      }
      
      return triangle;
   }

   private String getHeader(InputStream stream) throws IOException
   {
      byte[] header = new byte[80];
      stream.read(header);
      String name = new String(header, 0, 80, "US-ASCII");
      
      return name.trim();
   }

   public String getName()
   {
      return name;
   }

   public List<Triangle> getTriangles()
   {
      return triangles;
   }

}
