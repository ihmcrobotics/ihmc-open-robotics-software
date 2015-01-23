package us.ihmc.sensorProcessing.pointClouds.testbed;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadLocation;
import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

import java.awt.Dimension;
import java.util.List;

import javax.swing.JPanel;

import boofcv.gui.image.ShowImages;

import com.thoughtworks.xstream.XStream;

/**
 * @author Peter Abeles
 */
public class AutomaticAlignTestbedToPlanes {


   public static void main(String[] args) {

      String directory = "../SensorProcessing/data/testbed/2014-08-27/";

      Se3_F64 estimatedToModel = (Se3_F64) new XStream().fromXML(directory.getClass().
              getResourceAsStream("/testbed/estimatedToModel.xml"));
      TestbedAutomaticAlignment alg = new TestbedAutomaticAlignment(3,estimatedToModel);

      int which = 3;

      System.out.println("Loading and filtering point clouds");
      Point3D_F64 p = loadLocation(directory+String.format("headLocation%02d.csv",which));
      List<List<Point3D_F64>> scans0 = loadScanLines(directory+String.format("cloud%02d_scans.txt",which));
      for (int i = 0; i < scans0.size(); i++) {
         alg.addScan(scans0.get(i));
      }

      System.out.println("Detecting the testbed");
      System.out.println("  head "+p);
      ManualAlignTestbedToCloud display = new ManualAlignTestbedToCloud();
      alg.setheadLocation(p.x,p.y,p.z);

      long before = System.currentTimeMillis();
      if( alg.process() ) {
         Se3_F64 modelToWorld = alg.getModelToWorld();

         System.out.println("Rendering results");

         display.addTestBedModel();
         display.setTestbedToWorld(modelToWorld);
      } else {
         System.out.println("COULDN'T FIND TESTBED!");
      }
      long after = System.currentTimeMillis();
      System.out.println("Elapsed Time: "+(after-before));

      display.addPoints(alg.getCloud1(),0xFF0000,3);

      JPanel gui = new JPanel();
      gui.add( display.getCanvas() );
      gui.setPreferredSize( new Dimension(800,800));

      ShowImages.showWindow(gui, "Automatic Alignment");
      gui.requestFocus();
   }
}
