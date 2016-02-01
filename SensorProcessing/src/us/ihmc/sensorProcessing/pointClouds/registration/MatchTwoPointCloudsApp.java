package us.ihmc.sensorProcessing.pointClouds.registration;

import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

import java.util.ArrayList;
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
public class MatchTwoPointCloudsApp {

   public static double MAX_DISTANCE = 2;

   public static List<Point3D_F64> filterPointCloud( String filename ) {
      List<Point3D_F64> input = GeometryOps.loadCloud(filename);
      List<Point3D_F64> output = new ArrayList<Point3D_F64>();

      for ( Point3D_F64 p : input ) {
         if( p.norm() < MAX_DISTANCE ) {
            output.add(p);
         }
      }
      System.out.println("Total Points "+output.size());
      return output;
   }

   public static void main(String[] args) {
      List<Point3D_F64> cloudA = filterPointCloud("../SensorProcessing/data/testbed/2014-07-10/cloud01.txt");
      List<Point3D_F64> cloudB = filterPointCloud("../SensorProcessing/data/testbed/2014-07-10/cloud02.txt");

      MatchCloudToCloud<Se3_F64,Point3D_F64> fit = FactoryFitting.cloudIcp3D(1, new StoppingCondition(100, 1e-4, 1e-6));

      List<Point3D_F64> originA = new ArrayList<Point3D_F64>();
      for (int i = 0; i < cloudA.size(); i++) {
         originA.add(cloudA.get(i).copy());
      }

      long time0 = System.currentTimeMillis();
      fit.setSource(cloudA);
      fit.setDestination(cloudB);
      long time1 = System.currentTimeMillis();
      System.out.println("Set cloud time: "+(time1-time0)/1000.0);

      System.out.println("Start fitting");
      if( !fit.compute() )
         throw new RuntimeException("Matching failed!");

      Se3_F64 srcToDst = fit.getSourceToDestination();
      long time2 = System.currentTimeMillis();
      System.out.println("Register time: "+(time2-time1)/1000.0);

      System.out.println(srcToDst);


      for (int i = 0; i < originA.size(); i++) {
         Point3D_F64 a = originA.get(i);
         SePointOps_F64.transform(srcToDst, a, cloudA.get(i));
      }
      System.out.println("Done fitting");

      FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();

      PointCloudPanel gui =  factory.displayPointCloud();

      gui.addPoints(cloudA,0xFF0000,1);
      gui.addPoints(cloudB,0x00FF00,1);
//      gui.addPoints(originA,0x0000FF,1);

      ShowImages.showWindow(gui, "Two point clouds");
   }
}
