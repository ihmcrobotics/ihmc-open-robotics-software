package us.ihmc.sensorProcessing.pointClouds.testbed;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.ejml.ops.MatrixFeatures;
import org.junit.Ignore;
import org.junit.Test;

import com.thoughtworks.xstream.XStream;

import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

/**
 * @author Peter Abeles
 */
public class TestbedAutomaticAlignmentTest {

   String directory = "/home/pja/voo/b/";
   Se3_F64 estimatedToModel = (Se3_F64) new XStream().fromXML(this.getClass().
           getResourceAsStream("/testbed/estimatedToModel.xml"));

   /**
    * Need the file savedTestbedCloud00_scans.csv to run this, which is huge. So manual test really.
    */
   @Ignore
	@DeployableTestMethod(quarantined = true)
	@Test(timeout=300000)
   public void expectedSolution() {

      TestbedAutomaticAlignment alg = new TestbedAutomaticAlignment(3, estimatedToModel);

      List<List<Point3D_F64>> scans = loadScanLines(directory + "savedTestbedCloud00_scans.csv");
      for (int j = 0; j < scans.size(); j++) {
         alg.addScan(scans.get(j));
      }
      assertTrue(alg.process());
      Se3_F64 found = alg.getEstimatedToWorld();

      Se3_F64 expected = new Se3_F64();
      expected.getT().set(0.9125441345977366,-1.4171253460622035,-0.22046243685370556);
      expected.getR().set(3,3,true,-0.090, -0.996,  0.020,
      0.996 , -0.091 , -0.025  ,
      0.027 ,  0.018 ,  0.999);

      assertTrue(found.getT().distance(expected.getT()) < 0.01 );
      assertTrue(MatrixFeatures.isIdentical(expected.getR(),found.getR(),0.01));
   }

   /**
    * Should produce the approximately the solution when run multiple times
    */

   /**
    * Need the file savedTestbedCloud00_scans.csv to run this, which is huge. So manual test really.
    */
	@Ignore
	@DeployableTestMethod(quarantined = true)
	@Test(timeout=300000)
   public void multipleRuns() {

      TestbedAutomaticAlignment alg = new TestbedAutomaticAlignment(3,estimatedToModel);

      List<List<Point3D_F64>> scans = loadScanLines(directory+"savedTestbedCloud00_scans.csv");
      for (int j = 0; j < scans.size(); j++) {
         alg.addScan(scans.get(j));
      }
      assertTrue(alg.process());
      Se3_F64 first = alg.getEstimatedToWorld().copy();
      int cloudSize1 = alg.getCloud1().size();

      // run again
      alg.reset();
      for (int j = 0; j < scans.size(); j++) {
         alg.addScan(scans.get(j));
      }
      assertTrue(alg.process());
      Se3_F64 second = alg.getEstimatedToWorld().copy();
      int cloudSize2 = alg.getCloud1().size();

      // see if it's the same
//      first.print();
//      second.print();
//      System.out.println(" distance = "+first.getT().distance(second.getT()));

      assertEquals(cloudSize1,cloudSize2);
      assertTrue(first.getT().distance(second.getT())<1e-3);
      assertTrue(MatrixFeatures.isIdentical(first.getR(), second.getR(), 0.01));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void findLineLocation() {

      LineParametric3D_F64 line = new LineParametric3D_F64();
      List<Point3D_F64> points = new ArrayList<>();

      line.p.set(1,2,3);
      line.slope.set(-0.2,1.5,0.9);

      double start = -.15;
      double end = 9.5;

      for (int i = 0; i < 100; i++) {
         double t = (end-start)*i/99.0 + start;
         Point3D_F64 p = line.getPointOnLine(t);
         points.add(p);
      }

      Collections.shuffle(points);

      double found[] = TestbedAutomaticAlignment.findLineLocation(line,points);
      assertEquals(found[0],start,1e-8);
      assertEquals(found[1],end,1e-8);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void adjustSign() {

      Vector3D_F64 v = new Vector3D_F64(1,0,0);
      Point3D_F64 p = new Point3D_F64(1,2,3);
      List<Point3D_F64> cloud = new ArrayList<>();
      for (int i = 0; i < 20; i++) {
         cloud.add(new Point3D_F64(5,-1+i*0.1,3));
      }

      TestbedAutomaticAlignment.adjustSign(v,p,cloud);
      assertEquals(1,v.x,1e-8);

      v.x = -1;
      TestbedAutomaticAlignment.adjustSign(v,p,cloud);
      assertEquals(1,v.x,1e-8);
   }
}
