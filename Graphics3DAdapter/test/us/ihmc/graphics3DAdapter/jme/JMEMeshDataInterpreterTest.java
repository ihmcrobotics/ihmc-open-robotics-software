package us.ihmc.graphics3DAdapter.jme;

import static org.junit.Assert.assertEquals;

import javax.vecmath.Point3f;

import org.junit.Test;

import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.UI})
public class JMEMeshDataInterpreterTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testForASingleTriangle()
   {
      Point3f[] vertices =
         {
               new Point3f(0.1f, -1.0f, 0.2f),
               new Point3f(0.8f, -0.3f, 0.2f),
               new Point3f(-0.4f, 0.6f, 0.2f)
         };
      MeshDataHolder meshData = MeshDataGenerator.Polygon(vertices);
      Mesh interpretMeshData = JMEMeshDataInterpreter.interpretMeshData(meshData);

      assertEquals(1, interpretMeshData.getTriangleCount());
      Triangle triangle = new Triangle();
      interpretMeshData.getTriangle(0, triangle);

      Vector3f normal = triangle.getNormal();
      Vector3f expectedNormal = new Vector3f(0.0f, 0.0f, 1.0f);
      assertJMEVectorsEqual(expectedNormal, normal);

      assertJMEVectorEqualsPoint(triangle.get1(), vertices[0]);
      assertJMEVectorEqualsPoint(triangle.get2(), vertices[1]);
      assertJMEVectorEqualsPoint(triangle.get3(), vertices[2]);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testForASimpleCube()
   {
      MeshDataHolder meshData = MeshDataGenerator.Cube(1.0f, 1.0f, 1.0f, true, null);
      Mesh interpretMeshData = JMEMeshDataInterpreter.interpretMeshData(meshData);

      assertEquals(12, interpretMeshData.getTriangleCount());
      assertEquals(24, interpretMeshData.getVertexCount());

      for (int i = 0; i < interpretMeshData.getTriangleCount(); i++)
      {
         Triangle triangle = new Triangle();
         interpretMeshData.getTriangle(i, triangle);
         Vector3f normal = triangle.getNormal();
         System.out.println("normal = " + normal);

         Vector3f trianglePoint1 = triangle.get1();
         Vector3f trianglePoint2 = triangle.get2();
         Vector3f trianglePoint3 = triangle.get3();

         System.out.println("trianglePoint1 = " + trianglePoint1);
         System.out.println("trianglePoint2 = " + trianglePoint2);
         System.out.println("trianglePoint3 = " + trianglePoint3);

      }

   }

   private void assertJMEVectorEqualsPoint(Vector3f vector, Point3f point)
   {
      assertJMEVectorsEqual(vector, new Vector3f(point.getX(), point.getY(), point.getZ()));
   }

   private void assertJMEVectorsEqual(Vector3f vectorOne, Vector3f vectorTwo)
   {
      assertEquals(0.0, (double) vectorOne.distance(vectorTwo), 1e-7);
   }

}
