package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;

public class IcoSphereCreatorTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIcosphereCreator()
   {
      IcoSphereCreator icoSphereCreator = new IcoSphereCreator();

      int recursionLevel = 0;
      SimpleTriangleMesh icoSphere = icoSphereCreator.createIcoSphere(recursionLevel);

      ArrayList<Point3D> positions = icoSphere.positions;
      ArrayList<Integer> triangleIndices = icoSphere.triangleIndices;

      assertEquals(12, positions.size());
      assertEquals(60, triangleIndices.size());

      recursionLevel = 1;
      icoSphere = icoSphereCreator.createIcoSphere(recursionLevel);

      positions = icoSphere.positions;
      triangleIndices = icoSphere.triangleIndices;

      assertEquals(42, positions.size());
      assertEquals(240, triangleIndices.size());

      recursionLevel = 4;
      icoSphere = icoSphereCreator.createIcoSphere(recursionLevel);

      positions = icoSphere.positions;
      triangleIndices = icoSphere.triangleIndices;

      assertEquals(2562, positions.size());
      assertEquals(15360, triangleIndices.size());
   }

}
