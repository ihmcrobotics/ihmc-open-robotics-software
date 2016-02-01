package us.ihmc.graphics3DAdapter.jme;

import static org.junit.Assert.assertEquals;

import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.MeshDataHolder;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import com.jme3.math.Triangle;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;

@DeployableTestClass(targets={TestPlanTarget.UI})
public class JMEMeshDataInterpreterTest
{

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testForASingleTriangle()
   {
      Point3f[] vertices = new Point3f[]{new Point3f(0.0f, 0.0f, 0.0f), new Point3f(1.0f, 0.0f, 0.0f), new Point3f(1.0f, 1.0f, 0.0f)};
      
      TexCoord2f[] textPoints = new TexCoord2f[]{new TexCoord2f(0.0f, 0.0f), new TexCoord2f(1.0f, 0.0f), new TexCoord2f(0.0f, 1.0f)};
      int[] polygonIndices = new int[]{0, 1, 2};
      int[] polygonStripCounts = new int[]{3};
      MeshDataHolder meshData = new MeshDataHolder(vertices, textPoints, polygonIndices, polygonStripCounts);
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

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testForASimpleCube()
   {
      Point3f[] vertices = new Point3f[]{new Point3f(0.0f, 0.0f, 0.0f), new Point3f(1.0f, 0.0f, 0.0f), new Point3f(1.0f, 1.0f, 0.0f), new Point3f(0.0f, 1.0f, 0.0f), 
            new Point3f(0.0f, 0.0f, 1.0f), new Point3f(1.0f, 0.0f, 1.0f), new Point3f(1.0f, 1.0f, 1.0f), new Point3f(0.0f, 1.0f, 1.0f)};
      
      TexCoord2f[] textPoints = null;
      int[] polygonIndices = new int[]{0, 3, 2, 1, 4, 5, 6, 7, 1, 2, 6, 5, 0, 4, 7, 3, 0, 1, 5, 4, 2, 3, 7, 6};
      int[] polygonStripCounts = new int[]{4, 4, 4, 4, 4, 4};
      MeshDataHolder meshData = new MeshDataHolder(vertices, textPoints, polygonIndices, polygonStripCounts);
      Mesh interpretMeshData = JMEMeshDataInterpreter.interpretMeshData(meshData);
      
      assertEquals(12, interpretMeshData.getTriangleCount());
      assertEquals(8, interpretMeshData.getVertexCount());
      
      for (int i=0; i<interpretMeshData.getTriangleCount(); i++)
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
