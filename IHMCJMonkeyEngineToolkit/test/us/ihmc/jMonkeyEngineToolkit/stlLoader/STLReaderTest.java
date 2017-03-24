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

import static org.junit.Assert.*;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

/**
 * Test loading of STL files based on a hand crafted STL files with three triangles and known values.
 * Test both binary and ASCII versions of the same model.
 * 
 * The model in the test files is not a valid scene, and only created for easy testing of the reading code.
 * 
 * @author Jesper Smith
 *
 */

@ContinuousIntegrationPlan(categories = IntegrationCategory.IN_DEVELOPMENT)
public class STLReaderTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testASCIILoad() throws IOException
   {
      InputStream stream = getClass().getClassLoader().getResourceAsStream("testASCIISTL.STL");
      
      STLReader reader = STLReaderFactory.create(stream);
      assertEquals(reader.getClass(), ASCIISTLReader.class);
      
      checkData(reader);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testBinaryLoad() throws IOException
   {
      InputStream stream = getClass().getClassLoader().getResourceAsStream("testBinarySTL.STL");
      
      STLReader reader = STLReaderFactory.create(stream);
      assertEquals(reader.getClass(), BinarySTLReader.class);
      
      checkData(reader);
   }

   private void checkData(STLReader reader)
   {
      assertEquals("TEST_CASE", reader.getName());
      
      List<Triangle> triangles = reader.getTriangles();
      assertEquals(3, triangles.size());
      Triangle triangle = triangles.get(0);
      
      assertEquals(1e3, triangle.getNormal()[0], 1e-7);
      assertEquals(0, triangle.getNormal()[1], 1e-7);
      assertEquals(0, triangle.getNormal()[2], 1e-7);
      assertEquals(1e-2, triangle.getVertex(0)[0], 1e-7);
      assertEquals(0, triangle.getVertex(0)[1], 1e-7);
      assertEquals(2e3, triangle.getVertex(0)[2], 1e-7);
      assertEquals(0, triangle.getVertex(1)[0], 1e-7);
      assertEquals(0, triangle.getVertex(1)[1], 1e-7);
      assertEquals(0, triangle.getVertex(1)[2], 1e-7);
      assertEquals(2e-5, triangle.getVertex(2)[0], 1e-7);
      assertEquals(9, triangle.getVertex(2)[1], 1e-7);
      assertEquals(2, triangle.getVertex(2)[2], 1e-7);
      
      triangle = triangles.get(1);
      assertEquals(0, triangle.getNormal()[0], 1e-7);
      assertEquals(1e2, triangle.getNormal()[1], 1e-7);
      assertEquals(0, triangle.getNormal()[2], 1e-7);
      assertEquals(1e-2, triangle.getVertex(0)[0], 1e-7);
      assertEquals(0, triangle.getVertex(0)[1], 1e-7);
      assertEquals(2e3, triangle.getVertex(0)[2], 1e-7);
      assertEquals(1, triangle.getVertex(1)[0], 1e-7);
      assertEquals(2, triangle.getVertex(1)[1], 1e-7);
      assertEquals(3, triangle.getVertex(1)[2], 1e-7);
      assertEquals(3, triangle.getVertex(2)[0], 1e-7);
      assertEquals(4, triangle.getVertex(2)[1], 1e-7);
      assertEquals(5, triangle.getVertex(2)[2], 1e-7);
      
      triangle = triangles.get(2);
      assertEquals(0, triangle.getNormal()[0], 1e-7);
      assertEquals(1e3, triangle.getNormal()[1], 1e-7);
      assertEquals(2, triangle.getNormal()[2], 1e-7);
      assertEquals(1e-2, triangle.getVertex(0)[0], 1e-7);
      assertEquals(0, triangle.getVertex(0)[1], 1e-7);
      assertEquals(2e3, triangle.getVertex(0)[2], 1e-7);
      assertEquals(4, triangle.getVertex(1)[0], 1e-7);
      assertEquals(6, triangle.getVertex(1)[1], 1e-7);
      assertEquals(7, triangle.getVertex(1)[2], 1e-7);
      assertEquals(0, triangle.getVertex(2)[0], 1e-7);
      assertEquals(2, triangle.getVertex(2)[1], 1e-7);
      assertEquals(3, triangle.getVertex(2)[2], 1e-7);
   }
}
