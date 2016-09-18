package us.ihmc.geometry.polytope;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class GilbertJohnsonKeerthiCollisionDetectorTest
{

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeSupportPointOnMinkowskiDifference()
   {
      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructUnitCube();
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructUnitCube();

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(-1.0, -1.0, -1.0);
      cubeTwo.applyTransform(transform);

      GilbertJohnsonKeerthiCollisionDetector detector = new GilbertJohnsonKeerthiCollisionDetector();

      Vector3d directionVector = new Vector3d(1.0, 1.0, 1.0);
      Point3d supportPoint = new Point3d();
      detector.computeSupportPointOnMinkowskiDifference(cubeOne, cubeTwo, directionVector, supportPoint);
      JUnitTools.assertPoint3dEquals("", new Point3d(2.0, 2.0, 2.0), supportPoint, 1e-7);

      directionVector.set(-1.0, -1.0, -1.0);
      detector.computeSupportPointOnMinkowskiDifference(cubeOne, cubeTwo, directionVector, supportPoint);
      JUnitTools.assertPoint3dEquals("", new Point3d(0.0, 0.0, 0.0), supportPoint, 1e-7);

      directionVector.set(1.0, 1.0, -1.0);
      detector.computeSupportPointOnMinkowskiDifference(cubeOne, cubeTwo, directionVector, supportPoint);
      JUnitTools.assertPoint3dEquals("", new Point3d(2.0, 2.0, 0.0), supportPoint, 1e-7);
   }


}
