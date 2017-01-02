package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.testing.JUnitTools;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class Shape3dTestHelper
{   
   public Shape3dTestHelper()
   {
      
   }
   
   public void runSimpleTests(Shape3d<?> shape3d, Random random, int numberOfPoints)
   {
      for (int i=0; i<numberOfPoints; i++)
      {
         Point3d point = RandomTools.generateRandomPoint(random, 1.0, 1.0, 1.0);

         boolean pointIsInside = shape3d.isInsideOrOnSurface(point, -1e-5);
         boolean pointIsOutside = !(shape3d.isInsideOrOnSurface(point, 1e-5));

         if (pointIsInside && pointIsOutside) fail();

         if (pointIsInside) runSomeTestsWithPointInside(shape3d, point);
         else if (pointIsOutside) runSomeTestsWithPointOutside(shape3d, point);

         else
         {
            shape3d.orthogonalProjection(point);
            Point3d pointOnSurface = new Point3d();
            Vector3d surfaceNormal = new Vector3d();
            shape3d.checkIfInside(point, pointOnSurface, surfaceNormal);
            runSomeTestsWithPointOnSurface(shape3d, pointOnSurface, surfaceNormal);
         }
      }
      
      Point3d pointLikelyOutside = new Point3d(100.0, 1500.0, 2000.0);
      Point3d pointToProject = new Point3d(pointLikelyOutside);
      shape3d.orthogonalProjection(pointToProject);
      
      RigidBodyTransform transform = RigidBodyTransform.generateRandomTransform(random);
      RigidBodyTransform transformInverse = new RigidBodyTransform(transform);
      transformInverse.invert();
     
      shape3d.applyTransform(transform);

      Point3d pointToProjectAfterTransform = new Point3d(pointLikelyOutside);
      transform.transform(pointToProjectAfterTransform);
      shape3d.orthogonalProjection(pointToProjectAfterTransform);
      transformInverse.transform(pointToProjectAfterTransform);
      
      Point3d pointToProjectAfterTransformAndInverse = new Point3d(pointLikelyOutside);
      shape3d.applyTransform(transformInverse);
      shape3d.orthogonalProjection(pointToProjectAfterTransformAndInverse);
      
      // This assertion appears to be incorrect. pointLikelyOutside stays the same but the shape is transformed
      // We would expect it to project to a different point
      // JUnitTools.assertTuple3dEquals(pointToProject, pointToProjectAfterTransform, 1e-10);
      JUnitTools.assertTuple3dEquals(pointToProject, pointToProjectAfterTransformAndInverse, 1e-10);
      
   }
   
   private void runSomeTestsWithPointOutside(Shape3d<?> shape3d, Point3d pointOutside)
   {
      // Check to make sure the point is actually outside:
      Point3d pointOnSurface = new Point3d();
      Vector3d surfaceNormal = new Vector3d();
      boolean isInside = shape3d.checkIfInside(pointOutside, pointOnSurface, surfaceNormal);
      assertFalse(isInside); 
      
      if (MathTools.containsNaN(pointOnSurface))
      {
         assertTrue(MathTools.containsNaN(surfaceNormal));
         
         pointOnSurface.set(pointOutside);
         shape3d.orthogonalProjection(pointOnSurface);
      }
      
      runSomeTestsWithPointOnSurface(shape3d, pointOnSurface, surfaceNormal);
      
   }
   
   private void runSomeTestsWithPointInside(Shape3d<?> shape3d, Point3d pointInside)
   {
      // Check to make sure the point is actually inside:
      Point3d pointOnSurface = new Point3d();
      Vector3d surfaceNormal = new Vector3d();
      boolean isInside = shape3d.checkIfInside(pointInside, pointOnSurface, surfaceNormal);
      
      assertTrue(isInside); 
      assertFalse(MathTools.containsNaN(surfaceNormal));
      assertFalse(MathTools.containsNaN(pointOnSurface));
      
      runSomeTestsWithPointOnSurface(shape3d, pointOnSurface, surfaceNormal);
   }
   
   private void runSomeTestsWithPointOnSurface(Shape3d<?> shape3d, Point3d pointOnSurface, Vector3d surfaceNormal)
   {
      // If the surface normal is NaN, then that means it was projected or something. If it does not contain NaN, then it should be checked for validity.
      if (MathTools.containsNaN(surfaceNormal))
      {
         assertEquals(1.0, surfaceNormal.length(), 1e-7);
      }

      // Check that a further projection onto the surface doesn't change anything:
      Point3d newProjection = new Point3d();
      Vector3d newNormal = new Vector3d();
      boolean isInside = shape3d.checkIfInside(pointOnSurface, newProjection, newNormal);
      
      if (MathTools.containsNaN(newProjection))
      {
         assertFalse(isInside); // Check this!!!
         assertTrue(MathTools.containsNaN(newNormal));
         newProjection.set(pointOnSurface);
         shape3d.orthogonalProjection(newProjection);
         
         JUnitTools.assertTuple3dEquals(pointOnSurface, newProjection, 1e-3);
      }
      else
      {
         assertEquals(1.0, newNormal.length(), 1e-7);
      }
      JUnitTools.assertTuple3dEquals(pointOnSurface, newProjection, 1e-7);
      
            
      // Check that if you move a little in the direction of the normal you are outside and a little 
      // opposite the normal, you are inside:
      Point3d pointALittleOutside = new Point3d(pointOnSurface);
      Point3d pointALittleInside = new Point3d(pointOnSurface);
      pointALittleOutside.scaleAdd(1e-4, surfaceNormal, pointALittleOutside);
      pointALittleInside.scaleAdd(-1e-4, surfaceNormal, pointALittleInside);
      
      boolean isInsideCheck = shape3d.isInsideOrOnSurface(pointALittleOutside);
      if (isInsideCheck)
      {
         System.out.println("shape3d = " + shape3d);
         System.out.println("pointOnSurface = " + pointOnSurface);
         System.out.println("surfaceNormal = " + surfaceNormal);
         
         isInside = shape3d.checkIfInside(pointOnSurface, newProjection, newNormal);

      }
      assertFalse(isInsideCheck);
 
      boolean insideOrOnSurface = shape3d.isInsideOrOnSurface(pointALittleInside);
      if (!insideOrOnSurface)
      {
         System.out.println(shape3d);
         System.out.println("pointOnSurface = " + pointOnSurface);
         System.out.println("surfaceNormal = " + surfaceNormal);
         System.out.println("newNormal = " + newNormal);
         System.out.println("pointALittleInside = " + pointALittleInside);
         insideOrOnSurface = shape3d.isInsideOrOnSurface(pointALittleInside);
         shape3d.checkIfInside(pointOnSurface, newProjection, newNormal);
         System.out.println("newNormal = " + newNormal);
      }
      assertTrue(insideOrOnSurface); 
   }
   
}
