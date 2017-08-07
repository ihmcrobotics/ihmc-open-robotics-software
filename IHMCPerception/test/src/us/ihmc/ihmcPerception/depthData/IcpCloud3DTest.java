package us.ihmc.ihmcPerception.depthData;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ddogleg.nn.FactoryNearestNeighbor;
import org.ddogleg.nn.NearestNeighbor;
import org.junit.Test;

import georegression.fitting.MotionTransformPoint;
import georegression.fitting.se.MotionSe3PointSVD_F64;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * @author Peter Abeles
 */
public class IcpCloud3DTest {

   Random rand = new Random(234);

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void perfect_nomotion() {

      List<Point3D_F64> ref = new ArrayList<Point3D_F64>();
      List<Point3D_F64> curr = new ArrayList<Point3D_F64>();

      for( int i = 0; i < 100; i++ ) {
         Point3D_F64 p = new Point3D_F64();
         p.x = rand.nextGaussian()*2;
         p.y = rand.nextGaussian()*2;
         p.z = rand.nextGaussian()*2;

         ref.add(p);
         curr.add(p);
      }

      IcpCloud3D alg = createIcp(10);

      alg.setReference(ref);
      assertTrue(alg.setCurrent(curr));

      Se3_F64 found = alg.getReferenceToCurrent();
      double euler[] = ConvertRotation3D_F64.matrixToEuler(found.getR(), EulerType.XYZ, null);
      assertEquals(0,euler[0],1e-8);
      assertEquals(0,euler[1],1e-8);
      assertEquals(0,euler[2],1e-8);

      assertTrue(found.getT().normSq()<=1e-8);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void perfect_translation() {

      List<Point3D_F64> ref = new ArrayList<Point3D_F64>();
      List<Point3D_F64> curr = new ArrayList<Point3D_F64>();

      Se3_F64 expected = new Se3_F64();
      expected.getT().set(0.5,-0.3,0.2);

      for( int i = 0; i < 100; i++ ) {
         Point3D_F64 p = new Point3D_F64();
         p.x = rand.nextGaussian()*4;
         p.y = rand.nextGaussian()*4;
         p.z = rand.nextGaussian()*4;

         ref.add(p);
         curr.add(SePointOps_F64.transform(expected,p,null));
      }

      IcpCloud3D alg = createIcp(10);

      alg.setReference(ref);
      assertTrue(alg.setCurrent(curr));

      Se3_F64 found = alg.getReferenceToCurrent();
      double euler[] = ConvertRotation3D_F64.matrixToEuler(found.getR(), EulerType.XYZ, null);
      assertEquals(0,euler[0],1e-8);
      assertEquals(0,euler[1],1e-8);
      assertEquals(0,euler[2],1e-8);

      assertEquals(expected.getT().x,found.getT().x,1e-8);
      assertEquals(expected.getT().y,found.getT().y,1e-8);
      assertEquals(expected.getT().z,found.getT().z,1e-8);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void perfect_tran_rot() {

      List<Point3D_F64> ref = new ArrayList<Point3D_F64>();
      List<Point3D_F64> curr = new ArrayList<Point3D_F64>();

      Se3_F64 expected = new Se3_F64();
      expected.getT().set(0.5,-0.3,0.2);
      ConvertRotation3D_F64.eulerToMatrix(EulerType.XYZ, -0.1, 0.05, 0.07, expected.getR());

      for( int i = 0; i < 100; i++ ) {
         Point3D_F64 p = new Point3D_F64();
         p.x = rand.nextGaussian()*4;
         p.y = rand.nextGaussian()*4;
         p.z = rand.nextGaussian()*4;

         ref.add(p);
         curr.add(SePointOps_F64.transform(expected,p,null));
      }

      IcpCloud3D alg = createIcp(10);

      alg.setReference(ref);
      assertTrue(alg.setCurrent(curr));

      Se3_F64 found = alg.getReferenceToCurrent();
      double euler[] = ConvertRotation3D_F64.matrixToEuler(found.getR(), EulerType.XYZ, null);
      assertEquals(-0.1,euler[0],1e-8);
      assertEquals(0.05,euler[1],1e-8);
      assertEquals(0.07,euler[2],1e-8);

      assertEquals(expected.getT().x,found.getT().x,1e-7);
      assertEquals(expected.getT().y,found.getT().y,1e-7);
      assertEquals(expected.getT().z,found.getT().z,1e-7);
   }

   private IcpCloud3D createIcp(double maxDist) {

      MotionTransformPoint<Se3_F64, Point3D_F64> motionAlg = new MotionSe3PointSVD_F64();
      NearestNeighbor nn = FactoryNearestNeighbor.kdtree();

      return new IcpCloud3D(maxDist,100,1e-12,motionAlg,nn);
   }

}
