package us.ihmc.sensorProcessing.pointClouds.testbed;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.List;

import com.thoughtworks.xstream.XStream;

/**
 * Processes a set of point clouds and saves the found location of the testbed.
 *
 * @author Peter Abeles
 */
public class ComputeEstimatedToWorldBatch {

   public static void main(String[] args) {
      String directory = "../SensorProcessing/data/testbed/2014-08-01/";

//      TestbedAutomaticAlignment alg = new TestbedAutomaticAlignment(); // TODO get this to work with recycling the data structure

      for (int i = 0; i < 16; i++) {
         String name = String.format("cloud%02d_scans.txt", i);
         System.out.println("Processing "+name);
         List<List<Point3D_F64>> scans = loadScanLines(directory+name);
         Se3_F64 estimatedToModel = (Se3_F64) new XStream().fromXML(directory.getClass().
                 getResourceAsStream("/testbed/estimatedToModel.xml"));
         TestbedAutomaticAlignment alg = new TestbedAutomaticAlignment(3,estimatedToModel);
         alg.reset();
         for (int j = 0; j < scans.size(); j++) {
            alg.addScan(scans.get(j));
         }
         if( alg.process() ) {
            Se3_F64 estimatedToWorld = alg.getEstimatedToWorld();
            try {
               new XStream().toXML(estimatedToWorld, new FileOutputStream(directory+String.format("estimatedTestbedToWorld%02d.xml",i)));
            } catch (FileNotFoundException e) {
               throw new RuntimeException(e);
            }
         } else {
            System.out.println("  FAILED!!!!!  "+name);
         }
      }
   }
}
