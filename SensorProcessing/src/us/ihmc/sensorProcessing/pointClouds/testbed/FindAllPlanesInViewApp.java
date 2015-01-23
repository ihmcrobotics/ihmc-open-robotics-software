package us.ihmc.sensorProcessing.pointClouds.testbed;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import static us.ihmc.sensorProcessing.pointClouds.testbed.CreateCloudFromFilteredScanApp.filter;
import georegression.geometry.GeometryMath_F64;
import georegression.geometry.UtilPlane3D_F64;
import georegression.geometry.UtilPoint2D_F64;
import georegression.metric.Intersection3D_F64;
import georegression.metric.UtilAngle;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.line.LineSegment3D_F64;
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

import boofcv.gui.image.ShowImages;
import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import bubo.clouds.filter.UniformDensityCloudOctree;
import bubo.gui.FactoryVisualization3D;
import bubo.gui.UtilDisplayBubo;
import bubo.gui.d3.PointCloudPanel;

/**
 * @author Peter Abeles
 */
public class FindAllPlanesInViewApp {

   public static Random rand = new Random(23423);

   private static void visualizeIntersections( List<PointCloudShapeFinder.Shape> shapes , PointCloudPanel gui ) {
      List<Vector3D_F64> normals = new ArrayList<>();

      // compute the orientation of each plane
      for (int i = 0; i < shapes.size(); i++) {
         PlaneGeneral3D_F64 plane = (PlaneGeneral3D_F64)shapes.get(i).parameters;
         Se3_F64 planeToWorld = UtilPlane3D_F64.planeToWorld(plane, null);
         Vector3D_F64 normal = new Vector3D_F64(0,0,1);
         GeometryMath_F64.mult(planeToWorld.getR(), normal, normal);
         normals.add(normal);
      }



      int total = 0;
      for (int i = 0; i < normals.size(); i++) {
         PlaneGeneral3D_F64 planeA = (PlaneGeneral3D_F64) shapes.get(i).parameters;
         List<Point3D_F64> pointsA = shapes.get(i).points;
         Vector3D_F64 a = normals.get(i);
         for (int j = i + 1; j < normals.size(); j++) {
            Vector3D_F64 b = normals.get(j);
            PlaneGeneral3D_F64 planeB = (PlaneGeneral3D_F64) shapes.get(j).parameters;
            List<Point3D_F64> pointsB = shapes.get(j).points;

            double angle = a.acute(b);
            angle = Math.min(Math.PI - angle, angle);

            // see if they are perpendicular
            if (UtilAngle.dist(angle, Math.PI / 2) < 0.1) {
               LineParametric3D_F64 intersection = new LineParametric3D_F64();
               Intersection3D_F64.intersect(planeA, planeB, intersection);

               System.out.println("planeA = "+planeA);
               System.out.println("planeB = "+planeB);
               System.out.println("intersection = "+intersection);
               double whereA[] = TestbedAutomaticAlignment.findLineLocation(intersection, pointsA);
               double whereB[] = TestbedAutomaticAlignment.findLineLocation(intersection, pointsB);

               if( whereA == null || whereB == null )
                  continue;

               total++;
               double min = Math.min(whereA[0],whereB[0]);
               double max = Math.max(whereA[1],whereB[1]);

               Point3D_F64 p0 = intersection.getPointOnLine(min);
               Point3D_F64 p1 = intersection.getPointOnLine(max);

               List<LineSegment3D_F64> lines = new ArrayList<>();
               lines.add( new LineSegment3D_F64(p0,p1));
               int red =  rand.nextInt(200)+56;
               int green =  rand.nextInt(200)+56;
               int blue =  rand.nextInt(200)+56;

               int color = (red << 16 | green << 8 | blue) | 0xFF000000;

               gui.addLines(lines, 10, color);
            }
         }
      }

      System.out.println("Total = "+total);

   }

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

//      System.out.println("Plane "+plane);
//      System.out.println("Points in shape "+shape.points.size());
//      System.out.println("Bounding rectangle "+r.getWidth()+" "+r.getHeight());

      // select the color so that it won't be nearly black
      int red =  rand.nextInt(200)+56;
      int green =  rand.nextInt(200)+56;
      int blue =  rand.nextInt(200)+56;

      int color = (red << 16 | green << 8 | blue) | 0xFF000000;

      gui.addMesh2D(planeToWorld,polygon,(color&0x00FFFFFF )|0xA0000000); // 0xA0FF0000
      gui.addPoints(shape.points,color,1);
   }

   public static void main(String[] args) {
      List<List<Point3D_F64>> scans0 = loadScanLines("../SensorProcessing/data/testbed/2014-08-27/cloud05_scans.txt");
      List<Point3D_F64> cloud0 = filter(scans0,3);
//      List<Point3D_F64> cloud0 = loadCloud("../SensorProcessing/data/testbed/2014-08-01/cloud02.txt");
      List<Point3D_F64> cloud1 = new ArrayList<>();

      UniformDensityCloudOctree uniform = new UniformDensityCloudOctree(20,0.05,234234);
      uniform.process(cloud0,cloud1);

      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac.createDefault(250,1.2,0.025, CloudShapeTypes.PLANE);
      configRansac.minimumPoints = 2500;
//      ConfigSchnabel2007 configSchnabel = ConfigSchnabel2007.createDefault(20000,0.6,0.15,CloudShapeTypes.PLANE);

      PointCloudShapeFinder finder = FactoryPointCloudShape.ransacSingleAll(
              new ConfigSurfaceNormals(100, 0.15), configRansac);

      finder.process(cloud1,null);

      FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();
      PointCloudPanel gui = factory.displayPointCloud();

      List<PointCloudShapeFinder.Shape> found = finder.getFound();
      System.out.println("Total found "+found.size());

//      gui.addPoints(cloud0,0xFF00FF00,1);
      List<PointCloudShapeFinder.Shape> foo = new ArrayList<>();
      foo.addAll(found);
      found = foo;
//      found.remove(4);
//      found.remove(3);
//      found.remove(1);

      for (int i = 0; i < found.size(); i++) {
         PointCloudShapeFinder.Shape shape = found.get(i);
         visualizePlane(shape,gui);
         PlaneGeneral3D_F64 plane = (PlaneGeneral3D_F64)shape.parameters;
         System.out.println("Plane "+plane);
      }

      visualizeIntersections(found,gui);



      // TODO find bounding polygon in 2D
      //      prune islands?

      // TODO render

      // TODO find second plane.
      //      must be close to perpendicular
      //      intersection must be close to part of the other plane


      ShowImages.showWindow(gui,"FooBar");
   }
}
