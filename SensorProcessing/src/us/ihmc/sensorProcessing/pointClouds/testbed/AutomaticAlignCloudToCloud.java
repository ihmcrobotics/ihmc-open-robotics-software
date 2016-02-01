package us.ihmc.sensorProcessing.pointClouds.testbed;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import static us.ihmc.sensorProcessing.pointClouds.testbed.CreateCloudFromFilteredScanApp.filter;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

import java.util.List;

import us.ihmc.sensorProcessing.pointClouds.GeometryOps;
import boofcv.gui.image.ShowImages;
import bubo.clouds.FactoryFitting;
import bubo.clouds.fit.MatchCloudToCloud;
import bubo.gui.FactoryVisualization3D;
import bubo.gui.UtilDisplayBubo;
import bubo.gui.d3.PointCloudPanel;
import bubo.struct.StoppingCondition;

/**
 * @author Peter Abeles
 */
public class AutomaticAlignCloudToCloud {
   public static void main(String[] args) {
      List<List<Point3D_F64>> scans0 = loadScanLines("../SensorProcessing/data/testbed/2014-07-10/cloud01_scans.txt");
      List<List<Point3D_F64>> scans1 = loadScanLines("../SensorProcessing/data/testbed/2014-07-10/cloud02_scans.txt");

      List<Point3D_F64> cloud0 = filter(scans0,3.0);
      List<Point3D_F64> cloud1 = filter(scans1,3.0);

      System.out.print("Fitting ");
      long start = System.currentTimeMillis();
      MatchCloudToCloud<Se3_F64,Point3D_F64> fit = FactoryFitting.cloudIcp3D(0.5, new StoppingCondition(100, 1e-4, 1e-6));
      fit.setSource(cloud0);
      fit.setDestination(cloud1);
      if( !fit.compute() )
         throw new RuntimeException("Matching failed!");
      System.out.println("total = "+(System.currentTimeMillis()-start)/1000.0);

      GeometryOps.saveCsv(fit.getSourceToDestination(),"bubo_01_to_02.txt");


      Se3_F64 srcToDst = fit.getSourceToDestination();
      SePointOps_F64.transform(srcToDst,cloud0);

      FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();
      PointCloudPanel gui =  factory.displayPointCloud();

      gui.addPoints(cloud0,0xFF0000,1);
      gui.addPoints(cloud1,0x00FF00,1);

      ShowImages.showWindow(gui, "Point Cloud");
   }
}
