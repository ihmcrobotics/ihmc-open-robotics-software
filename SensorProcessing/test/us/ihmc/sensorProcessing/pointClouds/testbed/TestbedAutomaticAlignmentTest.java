package us.ihmc.sensorProcessing.pointClouds.testbed;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

import java.util.List;

import org.ejml.ops.MatrixFeatures;
import org.junit.Test;

import us.ihmc.sensorProcessing.pointClouds.testbed.TestbedAutomaticAlignment;

import com.thoughtworks.xstream.XStream;

/**
 * @author Peter Abeles
 */
public class TestbedAutomaticAlignmentTest {

   String directory = "/home/pja/voo/b/";
   Se3_F64 estimatedToModel = (Se3_F64) new XStream().fromXML(this.getClass().
           getResourceAsStream("/testbed/estimatedToModel.xml"));

   @Test
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
   @Test
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

   @Test
   public void findLineLocation() {
      fail("Implement");
   }

   @Test
   public void adjustSign() {
      fail("Implement");
   }
}