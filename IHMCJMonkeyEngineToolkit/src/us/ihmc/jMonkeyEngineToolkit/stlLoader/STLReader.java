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
 * Interface to the raw STL data. Allows access to both binary and ASCII formats.
 * 
 * @see STLReaderFactory
 * 
 * @author Jesper Smith
 *
 */
public interface STLReader
{
   /**
    * Name of the model as defined in the STL file.
    * 
    * For ASCII format: Description following the solid keyword
    * For binary format: The first 80 bytes as ASCII, trimmed from whitespace
    * 
    * @return model name 
    */
   public String getName();

   /**
    * List of triangles (facets) as defined in the STL file
    * 
    * @return List<Triangle> All triangles in the STL file
    */
   public List<Triangle> getTriangles();
}
