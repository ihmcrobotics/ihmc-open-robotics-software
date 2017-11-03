package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedConvexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection.HybridGJKEPACollisionDetector;

public class HybridGJKEPACollisionDetectorTest
{
   @Test(timeout = 1000)
   public void testSupportVectorDirectionNegative()
   {
      HybridGJKEPACollisionDetector collisionDetector = new HybridGJKEPACollisionDetector();
      collisionDetector.setSupportVectorDirection(new Vector3D(1, -1, 10));
      Vector3D vectorToGet = new Vector3D();
      collisionDetector.getSupportVectorDirectionNegative(vectorToGet);
      assertTrue(vectorToGet.getX() == -1);
      assertTrue(vectorToGet.getY() == 1);
      assertTrue(vectorToGet.getZ() == -10);
   }  
   
   @Test(timeout = 1000)
   public void testCollisionDetection()
   {
      ExtendedConvexPolytope polytope1 = ConvexPolytopeConstructor.constructExtendedBox(new Point3D(), new Quaternion(), 5, 5, 4);
   }
}
