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
import com.thoughtworks.xstream.XStream;
import georegression.geometry.*;
import georegression.metric.*;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Rectangle2D_F64;
import georegression.struct.so.Rodrigues_F64;
import georegression.transform.se.SePointOps_F64;
import org.ddogleg.sorting.QuickSelect;
import org.ddogleg.sorting.QuickSelectArray;
import org.ddogleg.struct.FastQueue;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;
import static us.ihmc.sensorProcessing.pointClouds.testbed.CreateCloudFromFilteredScanApp.filter;

/**
 * @author Peter Abeles
 */
public class DetectTestbedSaveTrasform {

   public static Random rand = new Random(23423);

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
      System.out.println("Points in shape "+shape.points.size());
      System.out.println("Bounding rectangle "+r.getWidth()+" "+r.getHeight());

      // select the color so that it won't be nearly black
      int red =  rand.nextInt(200)+56;
      int green =  rand.nextInt(200)+56;
      int blue =  rand.nextInt(200)+56;

      int color = (red << 16 | green << 8 | blue) | 0xFF000000;

      gui.addMesh2D(planeToWorld,polygon,(color&0x00FFFFFF )|0xA0000000); // 0xA0FF0000
      gui.addPoints(shape.points,color,1);
   }

   public static Se3_F64 findTestbed( List<PointCloudShapeFinder.Shape> shapes ) {

      List<Vector3D_F64> normals = new ArrayList<>();

      // compute the orientation of each plane
      for (int i = 0; i < shapes.size(); i++) {
         PlaneGeneral3D_F64 plane = (PlaneGeneral3D_F64)shapes.get(i).parameters;
         Se3_F64 planeToWorld = UtilPlane3D_F64.planeToWorld(plane, null);
         Vector3D_F64 normal = new Vector3D_F64(0,0,1);
         GeometryMath_F64.mult(planeToWorld.getR(),normal,normal);
         normals.add(normal);
      }

      double bestDistance = Double.MAX_VALUE;
      int bestA = -1;
      int bestB = -1;

      double sumParallel[] = new double[ shapes.size() ];

      for (int i = 0; i < normals.size(); i++) {
         PlaneGeneral3D_F64 planeA = (PlaneGeneral3D_F64)shapes.get(i).parameters;
         List<Point3D_F64> pointsA = shapes.get(i).points;
         Vector3D_F64 a = normals.get(i);
         for (int j = i+1; j < normals.size(); j++) {
            Vector3D_F64 b = normals.get(j);
            PlaneGeneral3D_F64 planeB = (PlaneGeneral3D_F64)shapes.get(j).parameters;
            List<Point3D_F64> pointsB = shapes.get(j).points;

            double angle = a.acute(b);
            angle = Math.min(Math.PI-angle,angle);

            sumParallel[i] += Math.cos(angle)*pointsB.size();
            sumParallel[j] += Math.cos(angle)*pointsA.size();

            System.out.println("Angle "+i+" "+j+"  =  "+angle);

            if( UtilAngle.dist(angle,Math.PI/2) < 0.1 ) {
               double distanceA = distanceFromPlane(planeA,pointsB);
               double distanceB = distanceFromPlane(planeB,pointsA);

               if( distanceA+distanceB < bestDistance ) {
                  bestA = i;
                  bestB = j;
                  bestDistance = distanceA+distanceB;
               }
               System.out.println("  distances "+distanceA+"  "+distanceB);
            }
         }
      }

      if( bestDistance < 0.05 ) {
         System.out.println("Found Testbed!");

         // assume the floor has more parallel points than the wall will have
         System.out.println("Total Parallel:  "+sumParallel[bestA]+"  "+sumParallel[bestB]);
         int indexWall = sumParallel[bestA] < sumParallel[bestB] ?  bestA : bestB;
         int indexFloor = sumParallel[bestA] < sumParallel[bestB] ?  bestB : bestA;


         PlaneGeneral3D_F64 planeWall = (PlaneGeneral3D_F64)shapes.get(indexWall).parameters;
         PlaneGeneral3D_F64 planeFloor = (PlaneGeneral3D_F64)shapes.get(indexFloor).parameters;
         List<Point3D_F64> pointsWall = shapes.get(indexWall).points;
         List<Point3D_F64> pointsFloor = shapes.get(indexFloor).points;

         double locs[] = new double[ pointsWall.size() + pointsFloor.size() ];

         // find line of intersection
         LineParametric3D_F64 intersection = new LineParametric3D_F64();
         Intersection3D_F64.intersect(planeWall, planeFloor, intersection);

         for (int i = 0; i < pointsWall.size(); i++) {
            locs[i] = ClosestPoint3D_F64.closestPoint(intersection,pointsWall.get(i));
         }

         // can use median since that's influcence by the scan's density
         int N = pointsWall.size();
         for (int i = 0; i < pointsFloor.size(); i++) {
            locs[i+N] = ClosestPoint3D_F64.closestPoint(intersection, pointsWall.get(i));
         }

         double min = Double.MAX_VALUE;
         double max = -Double.MAX_VALUE;

         for (int i = 0; i < locs.length; i++) {
            double d = locs[i];

            if( d < min )
               min = d;
            if( d > max )
               max = d;
         }

         double middle = (max+min)/2.0;

         Point3D_F64 center = new Point3D_F64();
         center.x = intersection.p.x + middle*intersection.slope.x;
         center.y = intersection.p.y + middle*intersection.slope.y;
         center.z = intersection.p.z + middle*intersection.slope.z;

         Vector3D_F64 axisX = normals.get(indexWall);
         Vector3D_F64 axisZ = normals.get(indexFloor);

         // resolve sign ambiguity
         adjustSign(axisX,center,pointsFloor);
         adjustSign(axisZ,center,pointsWall);

         Vector3D_F64 axisY = axisZ.cross(axisX);
         axisY.normalize();
         axisZ.normalize();
         axisX = axisY.cross(axisZ);

         Se3_F64 testbedToWorld = new Se3_F64();
         UtilVector3D_F64.createMatrix(testbedToWorld.R,axisX,axisY,axisZ);
         testbedToWorld.getT().set(center.x,center.y,center.z);

         return testbedToWorld;
      }
      return  null;
   }

   private static void adjustSign( Vector3D_F64 v , Point3D_F64 start , List<Point3D_F64> cloud ) {
      Point3D_F64 centroid = UtilPoint3D_F64.mean(cloud,cloud.size(),null);

      Vector3D_F64 pointing = new Vector3D_F64(start,centroid);

      Vector3D_F64 v2 = v.copy();
      v2.scale(-1);
      double angle0 = v.acute(pointing);
      double angle1 = v2.acute(pointing);
      if( angle1 < angle0 )
         v.set(v2);
      System.out.println("--- adjustSign");
      System.out.println("  "+angle0+"  "+angle1);
   }

   private static double distanceFromPlane( PlaneGeneral3D_F64 plane , List<Point3D_F64> points ) {
      double distances[] = new double[points.size()];

      for (int i = 0; i < points.size(); i++) {
         distances[i] = Math.abs(Distance3D_F64.distance(plane,points.get(i)));
      }

      int k = Math.min(50,distances.length-1);

      return QuickSelectArray.select(distances,k,distances.length);
   }

   public static void main(String[] args) {
      List<List<Point3D_F64>> scans0 = loadScanLines("../SensorProcessing/data/testbed/2014-07-10/cloud04_scans.txt");
      List<Point3D_F64> cloud0 = filter(scans0,3);

      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac.createDefault(500,1.2,0.025, CloudShapeTypes.PLANE);
      configRansac.minimumPoints = 5000;
//      ConfigSchnabel2007 configSchnabel = ConfigSchnabel2007.createDefault(20000,0.6,0.15,CloudShapeTypes.PLANE);

      PointCloudShapeFinder finder = FactoryPointCloudShape.ransacSingleAll(
              new ConfigSurfaceNormals(100, 0.15), configRansac);

      finder.process(cloud0,null);

      FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();
      PointCloudPanel gui = factory.displayPointCloud();

      System.out.println("Total found "+finder.getFound().size());

//      gui.addPoints(cloud0,0xFF00FF00,1);
      for (int i = 0; i < finder.getFound().size(); i++) {
         PointCloudShapeFinder.Shape shape = finder.getFound().get(i);
         visualizePlane(shape,gui);
      }

      System.out.println();
      System.out.println("--------- Detecting Testbed");
      System.out.println();

      Se3_F64 testbedToWorld = findTestbed(finder.getFound());

      gui.addAxis(testbedToWorld,0.5,0.02);

      try {
         new XStream().toXML(testbedToWorld, new FileOutputStream("estimatedTestbedToWorld.xml"));
      } catch (FileNotFoundException e) {
         throw new RuntimeException(e);
      }

      ShowImages.showWindow(gui,"FooBar");
   }
}
