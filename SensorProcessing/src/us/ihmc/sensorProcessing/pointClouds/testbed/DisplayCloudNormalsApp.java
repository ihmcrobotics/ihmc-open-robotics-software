package us.ihmc.sensorProcessing.pointClouds.testbed;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import static us.ihmc.sensorProcessing.pointClouds.testbed.CreateCloudFromFilteredScanApp.filter;
import georegression.geometry.UtilPlane3D_F64;
import georegression.geometry.UtilPoint2D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Rectangle2D_F64;
import georegression.transform.se.SePointOps_F64;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ddogleg.struct.FastQueue;

import boofcv.gui.image.ShowImages;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.alg.ApproximateSurfaceNormals;
import bubo.clouds.detect.alg.PointVectorNN;
import bubo.gui.FactoryVisualization3D;
import bubo.gui.UtilDisplayBubo;
import bubo.gui.d3.PointCloudPanel;

/**
 * @author Peter Abeles
 */
public class DisplayCloudNormalsApp {

   private static void visualizePlane( PointCloudShapeFinder.Shape shape , PointCloudPanel gui ) {
      PlaneGeneral3D_F64 plane = (PlaneGeneral3D_F64)shape.parameters;
      Se3_F64 planeToWorld = UtilPlane3D_F64.planeToWorld(plane,null);

      List<Point2D_F64> points2D = new ArrayList<>();

      Point3D_F64 planeP = new Point3D_F64();
      for( Point3D_F64 p : shape.points ) {
         SePointOps_F64.transformReverse(planeToWorld, p, planeP);

         points2D.add(new Point2D_F64(planeP.x,planeP.y));
      }

      Rectangle2D_F64 r = UtilPoint2D_F64.bounding(points2D,(Rectangle2D_F64)null);

      List<Point2D_F64> polygon = new ArrayList<>();
      polygon.add( r.p0.copy());
      polygon.add( new Point2D_F64(r.p1.x,r.p0.y));
      polygon.add( r.p1.copy());
      polygon.add( new Point2D_F64(r.p0.x,r.p1.y));

      gui.addMesh2D(planeToWorld,polygon,0xA0FF0000);
   }

   public static void main(String[] args) {
      List<List<Point3D_F64>> scans0 = loadScanLines("../SensorProcessing/data/testbed/2014-07-10/cloud01_scans.txt");
      List<Point3D_F64> cloud0 = filter(scans0,2.5);


      ApproximateSurfaceNormals surfaceNormals = new ApproximateSurfaceNormals(40,0.05);

      FastQueue<PointVectorNN> pointNormList = new FastQueue<PointVectorNN>(PointVectorNN.class, false);

      surfaceNormals.process(cloud0,pointNormList);

      double scales = 0.10;
      List<Point3D_F64> points = new ArrayList<>();
      List<Vector3D_F64> normals = new ArrayList<>();


      Random rand = new Random(234);
      int totalZero = 0;
      int totalOne = 0;
      for (int i = 0; i < pointNormList.size(); i++) {
         PointVectorNN pnn = pointNormList.get(i);

         if( pnn.normal.norm() < 0.95 )
            totalZero++;
         if( pnn.normal.norm() == 1.0 )
            totalOne++;

         pnn.normal.scale(scales);


         if( rand.nextDouble() < 0.05 ) {
            points.add(pnn.p);
            normals.add(pnn.normal);
         }
      }
      System.out.println("zero = "+totalZero);
      System.out.println("one = "+totalOne);

      FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();
      PointCloudPanel gui = factory.displayPointCloud();


      gui.addPoints(cloud0,0xFFFF0000,1);
//      gui.addPoints(points,0xFF0000FF,1);
      gui.addVectors(points,normals,0xFF00FF00);


      ShowImages.showWindow(gui,"Surface Normal");
   }
}
