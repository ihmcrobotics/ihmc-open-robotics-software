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

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;


/**
 * Reader for the ASCII STL format
 * 
 * @see STLReaderFactory
 * 
 * @author Jesper Smith
 *
 */
public class ASCIISTLReader implements STLReader
{
   private enum State
   {
      FACET, OUTER_LOOP, VERTICES, END_FACET
   }

   private final String name;
   private final ArrayList<Triangle> triangles = new ArrayList<Triangle>();

   public ASCIISTLReader(InputStream stream) throws IOException
   {
      BufferedReader reader = new BufferedReader(new InputStreamReader(stream, Charset.forName("US-ASCII")));

      this.name = getModelName(reader);

      Triangle triangle;
      while ((triangle = getTriangle(reader)) != null)
      {
         triangles.add(triangle);
      }
      
      stream.close();
   }

   private String getModelName(BufferedReader reader) throws IOException
   {
      String header;
      do
      {
         header = reader.readLine();
      }
      while (header != null && !header.matches("^\\s*solid.*"));

      if (header == null)
      {
         throw new IOException("File is not a valid STL file");
      }

      return header.replaceFirst("^\\s*solid", "").trim();
   }

   private Triangle getTriangle(BufferedReader reader) throws IOException
   {
      String line;
      Triangle result = new Triangle();

      State state = State.FACET;
      while ((line = reader.readLine()) != null)
      {
         StringTokenizer tokenizer = new StringTokenizer(line);
         if (tokenizer.countTokens() == 0)
         {
            continue;
         }

         String firstToken = tokenizer.nextToken();
         if (firstToken.equals("endsolid")) // reached end of file
         {
            if (state != State.FACET)
            {
               throw new RuntimeException("Cannot pare ASCII STL file: Unexpected end of file");
            }
            return null;
         }
         switch (state)
         {
         case FACET:
            if (!"facet".equals(firstToken) || !"normal".equals(tokenizer.nextToken()))
            {
               throw new IOException("Expected \"facet normal\", got " + line);
            }

            float ni = Float.parseFloat(tokenizer.nextToken());
            float nj = Float.parseFloat(tokenizer.nextToken());
            float nk = Float.parseFloat(tokenizer.nextToken());
            result.setNormal(ni, nj, nk);
            state = State.OUTER_LOOP;
            break;
         case OUTER_LOOP:
            if (!"outer".equals(firstToken) || !"loop".equals(tokenizer.nextToken()))
            {
               throw new IOException("Expected \"outer loop\", got " + line);
            }
            state = State.VERTICES;
            break;
         case VERTICES:
            if ("vertex".equals(firstToken))
            {
               float vx = Float.parseFloat(tokenizer.nextToken());
               float vy = Float.parseFloat(tokenizer.nextToken());
               float vz = Float.parseFloat(tokenizer.nextToken());
               result.addVertex(vx, vy, vz);
            }
            else if ("endloop".equals(firstToken))
            {
               state = State.END_FACET;
            }
            else if ("end".equals(firstToken) && "loop".equals(tokenizer.nextToken())) // handle broken STL files
            {
               state = State.END_FACET;
            }
            else
            {
               throw new IOException("Expected \"vertex\" or \"endloop\", got " + line);
            }
            break;
         case END_FACET:
            if (!"endfacet".equals(firstToken))
            {
               throw new IOException("Expected \"endfacet\", got " + line);
            }
            return result;
         }

      }

      throw new RuntimeException("Cannot pare ASCII STL file: Unexpected end of file");
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
