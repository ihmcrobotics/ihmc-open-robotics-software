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
package us.ihmc.loaders.jme;

import java.io.IOException;
import java.io.InputStream;
import java.util.regex.Pattern;

public class STLReaderFactory
{
   /**
    * Create an STLReader based on the inputstream. 
    * Assumes the STL file is ASCII if the first non-whitespace characters are "solid", and the next line contains either facet or endso else assumes it is binary data.
    * 
    * @see STLReader
    * 
    * @param stream InputStream pointing to a valid STL file. Must support mark() and reset()
    * @return Correct STL reader (binary or ASCII) based on loading heurstics 
    * @throws IOException An error occured during parsing of this STL file
    */
   public static STLReader create(InputStream stream) throws IOException
   {
      stream.mark(80);
      byte[] header = new byte[80];
      byte[] nextLine = new byte[80];
      stream.read(header);
      stream.read(nextLine);
      stream.reset();


      String type = new String(header, 0, 80, "US-ASCII");
      String proof = new String(nextLine, 0, 80, "US-ASCII");
      Pattern asciiSTL = Pattern.compile("^\\s*solid.*", Pattern.DOTALL);
      Pattern asciiSTLNextLineMatcher = Pattern.compile(".*(facet.*|endso.*)", Pattern.DOTALL);

      if (asciiSTL.matcher(type).matches() && (asciiSTL.matcher(proof).matches() || asciiSTLNextLineMatcher.matcher(proof).matches()))
      {
         return new ASCIISTLReader(stream);
      }
      else
      {
         return new BinarySTLReader(stream);
      }
   }
}
