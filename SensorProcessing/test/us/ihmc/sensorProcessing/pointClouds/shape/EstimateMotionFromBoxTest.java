package us.ihmc.sensorProcessing.pointClouds.shape;

import static junit.framework.Assert.assertEquals;
import georegression.geometry.GeometryMath_F64;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

import java.util.Random;

import org.junit.Test;

/**
 * @author Peter Abeles
 */
public class EstimateMotionFromBoxTest
{

   Random rand = new Random(324234);

   // planes in canonical frame
   PlaneNormal3D_F64 left = new PlaneNormal3D_F64(0,0,0,1,0,0);
   PlaneNormal3D_F64 right = new PlaneNormal3D_F64(0,0,0,0,1,0);
   PlaneNormal3D_F64 top = new PlaneNormal3D_F64(0,0,1.5,0,0,1);

   Point3D_F64 testP = new Point3D_F64(3,-5,6);
   Point3D_F64 foundP = new Point3D_F64();
   Point3D_F64 expectedP = new Point3D_F64();

   @Test(timeout=300000)
   public void translation() {
      Se3_F64 canonicalToA =  new Se3_F64();
      Se3_F64 canonicalToB =  new Se3_F64();

      canonicalToA.T.set(1,-0.5,-0.23);
      canonicalToB.T.set(2,0.05,-0.4);

      checkCase(new EstimateMotionFromBox(), canonicalToA, canonicalToB);
   }

   @Test(timeout=300000)
   public void rotation() {
      Se3_F64 canonicalToA =  new Se3_F64();
      Se3_F64 canonicalToB =  new Se3_F64();

      RotationMatrixGenerator.eulerXYZ(0.1,-0.5,0.05,canonicalToA.getR());
      RotationMatrixGenerator.eulerXYZ(0.3,-0.7,-0.12,canonicalToB.getR());

      checkCase(new EstimateMotionFromBox(), canonicalToA, canonicalToB);
   }

   @Test(timeout=300000)
   public void both() {
      Se3_F64 canonicalToA =  new Se3_F64();
      Se3_F64 canonicalToB =  new Se3_F64();

      canonicalToA.T.set(1,-0.5,-0.23);
      canonicalToB.T.set(2,0.05,-0.4);
      RotationMatrixGenerator.eulerXYZ(0.1,-0.5,0.05,canonicalToA.getR());
      RotationMatrixGenerator.eulerXYZ(0.3,-0.7,-0.12,canonicalToB.getR());

      checkCase(new EstimateMotionFromBox(), canonicalToA, canonicalToB);
   }

   @Test(timeout=300000)
   public void randomTransforms()
   {
      EstimateMotionFromBox alg = new EstimateMotionFromBox();

      for( int i = 0; i < 100; i++ ) {
         Se3_F64 canonicalToA =  randomTran();
         Se3_F64 canonicalToB =  randomTran();

         checkCase(alg, canonicalToA, canonicalToB);
      }
   }

   private void checkCase(EstimateMotionFromBox alg, Se3_F64 canonicalToA, Se3_F64 canonicalToB)
   {
      Se3_F64 aToB = canonicalToA.invert(null).concat(canonicalToB,null);
      Se3_F64 found;

      PlaneNormal3D_F64 leftA = create(left,canonicalToA);
      PlaneNormal3D_F64 rightA = create(right,canonicalToA);
      PlaneNormal3D_F64 topA = create(top,canonicalToA);

      PlaneNormal3D_F64 leftB = create(left,canonicalToB);
      PlaneNormal3D_F64 rightB = create(right,canonicalToB);
      PlaneNormal3D_F64 topB = create(top,canonicalToB);

      alg.setSrc(leftA,rightA,topA);
      alg.computeMotion(leftB,rightB,topB);

      found = alg.getMotionSrcToDst();

      SePointOps_F64.transform(aToB, testP, expectedP);
      SePointOps_F64.transform(found,testP,foundP);

      assertEquals(aToB.T.x,found.T.x,1e-8);
      assertEquals(aToB.T.y,found.T.y,1e-8);
      assertEquals(aToB.T.z,found.T.z,1e-8);

      assertEquals(expectedP.x,foundP.x,1e-8);
      assertEquals(expectedP.y,foundP.y,1e-8);
      assertEquals(expectedP.z,foundP.z,1e-8);
   }

   private PlaneNormal3D_F64 create( PlaneNormal3D_F64 plane , Se3_F64 tran ) {
      PlaneNormal3D_F64 b = new PlaneNormal3D_F64();
      SePointOps_F64.transform(tran,plane.p,b.p);
      GeometryMath_F64.mult(tran.R,plane.n,b.n);

      return b;
   }

   private Se3_F64 randomTran() {
      Se3_F64 a = new Se3_F64();

      a.T.x = rand.nextGaussian()*2;
      a.T.y = rand.nextGaussian()*2;
      a.T.z = rand.nextGaussian()*2;

      RotationMatrixGenerator.eulerXYZ(rand.nextDouble(),rand.nextDouble(),rand.nextDouble(),a.R);

      return a;
   }

}
