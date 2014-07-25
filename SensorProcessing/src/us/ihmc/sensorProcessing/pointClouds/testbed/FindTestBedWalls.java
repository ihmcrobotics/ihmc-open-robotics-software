package us.ihmc.sensorProcessing.pointClouds.testbed;

import boofcv.gui.image.ShowImages;
import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.alg.ApproximateSurfaceNormals;
import bubo.clouds.detect.alg.ConfigSchnabel2007;
import bubo.clouds.detect.alg.PointVectorNN;
import bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import bubo.clouds.detect.wrapper.ConfigRemoveFalseShapes;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import bubo.gui.FactoryVisualization3D;
import bubo.gui.UtilDisplayBubo;
import bubo.gui.d3.PointCloudPanel;
import georegression.geometry.UtilPlane3D_F64;
import georegression.geometry.UtilPoint2D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Rectangle2D_F64;
import georegression.transform.se.SePointOps_F64;
import org.ddogleg.struct.FastQueue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import static us.ihmc.sensorProcessing.pointClouds.testbed.CreateCloudFromFilteredScanApp.filter;

/**
 * @author Peter Abeles
 */
public class FindTestBedWalls {

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

      System.out.println("Plane "+plane);
      System.out.println("Points on cloud "+shape.points.size());
      System.out.println("Bounding rectangle "+r.getWidth()+" "+r.getHeight());


      int color = (new Random().nextInt() & 0x00FFFFFF ) | 0xFF000000;

      gui.addMesh2D(planeToWorld,polygon,(color&0x00FFFFFF )|0xA0000000); // 0xA0FF0000
      gui.addPoints(shape.points,color,1);
   }

   public static void main(String[] args) {
      List<List<Point3D_F64>> scans0 = loadScanLines("../SensorProcessing/data/testbed/2014-07-10/cloud01_scans.txt");
      List<Point3D_F64> cloud0 = filter(scans0,3);

      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac.createDefault(500,1.2,0.025, CloudShapeTypes.PLANE);
      configRansac.minimumPoints = 15000;
//      ConfigSchnabel2007 configSchnabel = ConfigSchnabel2007.createDefault(20000,0.6,0.15,CloudShapeTypes.PLANE);

      PointCloudShapeFinder finder = FactoryPointCloudShape.ransacSingleAll(
              new ConfigSurfaceNormals(100, 100, 0.15), configRansac);

      finder.process(cloud0,null);

      FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();
      PointCloudPanel gui = factory.displayPointCloud();

      System.out.println("Total found "+finder.getFound().size());

//      gui.addPoints(cloud0,0xFF00FF00,1);
      for (int i = 0; i < finder.getFound().size(); i++) {
         PointCloudShapeFinder.Shape shape = finder.getFound().get(i);
         visualizePlane(shape,gui);
      }

      // TODO find bounding polygon in 2D
      //      prune islands?

      // TODO render

      // TODO find second plane.
      //      must be close to perpendicular
      //      intersection must be close to part of the other plane


      ShowImages.showWindow(gui,"FooBar");
   }
}
