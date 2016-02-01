package us.ihmc.sensorProcessing.pointClouds.testbed;

import georegression.struct.point.Point3D_F64;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.sensorProcessing.pointClouds.GeometryOps;
import boofcv.gui.image.ShowImages;
import bubo.gui.FactoryVisualization3D;
import bubo.gui.UtilDisplayBubo;
import bubo.gui.d3.PointCloudPanel;
import bubo.io.pcl.PointCloudLibraryPcdWriter;

/**
 * @author Peter Abeles
 */
public class CreateCloudFromFilteredScanApp {

   public static List<Point3D_F64> filter( List<List<Point3D_F64>> scans , double maxRange ) {

      List<Point3D_F64> cloud = new ArrayList<>();
      for (int i = 0; i < scans.size(); i++) {
         List<Point3D_F64> scan = scans.get(i);

         for (int j = 0; j < scan.size(); j++) {
            Point3D_F64 p = scan.get(j);

            if( p.normSq() <= maxRange*maxRange && countNeighbors(p,0.08,scan) > 6 ) {
               cloud.add(p);
            }
         }
      }

      return cloud;
   }

   public static List<Point3D_F64> filterScan( List<Point3D_F64> scan , double maxRange ) {

      List<Point3D_F64> cloud = new ArrayList<>();

      for (int j = 0; j < scan.size(); j++) {
         Point3D_F64 p = scan.get(j);

         if( p.normSq() <= maxRange*maxRange && countNeighbors(p,0.08,scan) > 6 ) {
            cloud.add(p);
         }
      }

      return cloud;
   }

   public static List<Point3D_F64> loadFilteredScans( String fileName , double maxRange ) {
      List<List<Point3D_F64>> scans = GeometryOps.loadScanLines(fileName);

      return filter(scans,maxRange);
   }

   public static int countNeighbors( Point3D_F64 target , double radius , List<Point3D_F64> scan ) {

      double r = radius*radius;

      int total = 0;
      for (int i = 0; i < scan.size(); i++) {
         if( scan.get(i).distance2(target) <= r ) {
            total++;
         }
      }
      return total;
   }


   public static void main(String[] args) throws FileNotFoundException {
      List<List<Point3D_F64>> scans = GeometryOps.loadScanLines("../SensorProcessing/data/testbed/2014-07-10/cloud01_scans.txt");

      FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();
      PointCloudPanel gui =  factory.displayPointCloud();

//      Random rand = new Random(234);
//      for( List<Point3D_F64> l : scans ) {
//         int color = rand.nextInt();
//         gui.addPoints(l, color, 1);
//      }

      double filterR = 25.0;

      PointCloudLibraryPcdWriter.save(filter(scans,filterR),"cloud.pcd");

      gui.addPoints(filter(scans,filterR),0xFF0000,1);

      ShowImages.showWindow(gui, "Point Cloud");
   }


}
