package us.ihmc.sensorProcessing.pointClouds.testbed;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import static us.ihmc.sensorProcessing.pointClouds.testbed.CreateCloudFromFilteredScanApp.filter;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.sensorProcessing.pointClouds.GeometryOps;
import boofcv.gui.image.ShowImages;
import bubo.gui.FactoryVisualization3D;
import bubo.gui.UtilDisplayBubo;
import bubo.gui.d3.PointCloudPanel;

/**
 * @author Peter Abeles
 */
public class DisplaySavedResultsApp {


   public static void main(String[] args) {

      List<List<Point3D_F64>> scans0 = loadScanLines("../SensorProcessing/data/testbed/2014-07-10/cloud01_scans.txt");
      List<List<Point3D_F64>> scans1 = loadScanLines("../SensorProcessing/data/testbed/2014-07-10/cloud02_scans.txt");
      Se3_F64 transform = GeometryOps.loadCsvSe3("../SensorProcessing/data/testbed/2014-07-10/pcl_01_to_02.txt");


      List<Point3D_F64> cloud0 = filter(scans0,25.0);
      List<Point3D_F64> cloud1 = filter(scans1,25.0);

      List<Point3D_F64> original = new ArrayList<>();
      for( Point3D_F64 p : cloud0 ) {
         original.add(p.copy());
      }

      SePointOps_F64.transform(transform,cloud0);

      FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();
      PointCloudPanel gui =  factory.displayPointCloud();

      gui.addPoints(cloud0,0xFF0000,1);
      gui.addPoints(cloud1,0x00FF00,1);
//      gui.addPoints(original,0x0000ff,1);

      ShowImages.showWindow(gui, "Point Cloud");
   }
}
