package us.ihmc.sensorProcessing.pointClouds.testbed;

import boofcv.gui.image.ShowImages;
import bubo.clouds.detect.PointCloudShapeFinder;
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
import georegression.struct.shapes.Box3D_F64;
import georegression.struct.shapes.Rectangle2D_F64;
import georegression.transform.se.SePointOps_F64;
import org.ddogleg.sorting.QuickSelectArray;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.sensorProcessing.pointClouds.GeometryOps.loadScanLines;

/**
 * @author Peter Abeles
 */
public class DetectTestbedSaveTransform {

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
      int red   =  rand.nextInt(200)+56;
      int green =  rand.nextInt(200)+56;
      int blue  =  rand.nextInt(200)+56;

      int color = (red << 16 | green << 8 | blue) | 0xFF000000;

      gui.addMesh2D(planeToWorld,polygon,(color&0x00FFFFFF )|0xA0000000); // 0xA0FF0000
      gui.addPoints(shape.points,color,1);
   }

   public static Se3_F64 findTestbed( List<PointCloudShapeFinder.Shape> shapes , List<Point3D_F64> cloud ) {

      List<Vector3D_F64> normals = new ArrayList<>();

      // compute the orientation of each plane
      for (int i = 0; i < shapes.size(); i++) {
         PlaneGeneral3D_F64 plane = (PlaneGeneral3D_F64)shapes.get(i).parameters;
         Se3_F64 planeToWorld = UtilPlane3D_F64.planeToWorld(plane, null);
         Vector3D_F64 normal = new Vector3D_F64(0,0,1);
         GeometryMath_F64.mult(planeToWorld.getR(),normal,normal);
         normals.add(normal);
//         System.out.println(i+"  plane "+plane);
      }

      List<Candidate> candidates = new ArrayList<>();

      double sumParallel[] = new double[ shapes.size() ];
      double inlierDistance = 0.25;

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

//            System.out.println("Angle "+i+" "+j+"  =  "+angle);

            if( UtilAngle.dist(angle,Math.PI/2) < 0.1 ) {
               double distanceA = distanceFromPlane(planeA,pointsB);
               double distanceB = distanceFromPlane(planeB,pointsA);

               if( distanceA+distanceB < inlierDistance ) {
                  candidates.add( new Candidate(planeA,planeB,pointsA,pointsB,i,j));
               }
            }
         }
      }

      Candidate best = null;
      int bestCount = 0;
      for( Candidate c : candidates ) {
         if( computeCoorindateSystem(c,sumParallel,normals) ) {
            int count = findValve(c, cloud);

            System.out.println("count = " + count);
            if (count > bestCount) {
               best = c;
               bestCount = count;
            }
         }
      }

      return best.testbedToWorld;
   }

   private static int findValve( Candidate c , List<Point3D_F64> cloud ) {

      Point3D_F64 p = new Point3D_F64(0.25,0.55,1.0);
      double r = 0.15;

      // translate center of box into world
      SePointOps_F64.transform(c.testbedToWorld,p,p);

      Box3D_F64 box = new Box3D_F64(p.x-r,p.y-r,p.z-r,p.x+r,p.y+r,p.z+r);

      int count = 0;
      for (int i = 0; i < cloud.size(); i++) {
         if( Intersection3D_F64.contained(box,cloud.get(i)) ) {
            count++;
         }
      }

      return count;
   }

   private static boolean computeCoorindateSystem( Candidate c , double sumParallel[] , List<Vector3D_F64> normals ) {
//      System.out.println("Total Parallel:  "+sumParallel[c.indexFloor]+"  "+sumParallel[c.indexWall]);

      if( sumParallel[c.indexFloor] < sumParallel[c.indexWall]) {
         c.swap();
      }

      // find line of intersection
      LineParametric3D_F64 intersection = new LineParametric3D_F64();
      Intersection3D_F64.intersect(c.planeWall, c.planeFloor, intersection);

      double wallR[] = findLineLocation(intersection,c.pointsWall);
      double floorR[] = findLineLocation(intersection,c.pointsFloor);

      double min = Math.min(wallR[0],floorR[0]);
      double max = Math.max(wallR[1], floorR[1]);

      double overlap = (Math.min(wallR[1], floorR[1]) - Math.max(wallR[0], floorR[0]))/(max-min);

      System.out.println("overlap "+overlap);
      // reject if the two planes are too far away
      if( overlap < 0.7)
         return false;

      double middle = (max+min)/2.0;

      Point3D_F64 centerPt = intersection.getPointOnLine(middle);
      Point3D_F64 minPt = intersection.getPointOnLine(min);
      Point3D_F64 maxPt = intersection.getPointOnLine(max);

      Vector3D_F64 axisX = normals.get(c.indexWall);
      Vector3D_F64 axisZ = normals.get(c.indexFloor);

      // resolve sign ambiguity
      adjustSign(axisX,centerPt,c.pointsFloor);
      adjustSign(axisZ,centerPt,c.pointsWall);

      Vector3D_F64 axisY = axisZ.cross(axisX);
      axisY.normalize();
      axisZ.normalize();
      axisX = axisY.cross(axisZ);

      // get the point in the corner
      double angleMin = axisY.acute(new Vector3D_F64(centerPt,minPt));
      double angleMax = axisY.acute(new Vector3D_F64(centerPt,maxPt));

//      System.out.println("min = "+angleMin+"  max = "+angleMax);

      Point3D_F64 selected = angleMin > angleMax ? minPt : maxPt;

      UtilVector3D_F64.createMatrix(c.testbedToWorld.R,axisX,axisY,axisZ);
      c.testbedToWorld.getT().set(selected.x,selected.y,selected.z);

      return true;
   }

   private static double[] findLineLocation( LineParametric3D_F64 line , List<Point3D_F64> points ) {
      double min = Double.MAX_VALUE;
      double max = -Double.MAX_VALUE;

      for (int i = 0; i < points.size(); i++) {
         double t = ClosestPoint3D_F64.closestPoint(line,points.get(i));
         if( t < min )
            min = t;
         if( t > max )
            max = t;
      }

      return new double[]{min,max};
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
//      System.out.println("--- adjustSign");
//      System.out.println("  "+angle0+"  "+angle1);
   }

   private static double distanceFromPlane( PlaneGeneral3D_F64 plane , List<Point3D_F64> points ) {
      double distances[] = new double[points.size()];

      for (int i = 0; i < points.size(); i++) {
         distances[i] = Math.abs(Distance3D_F64.distance(plane,points.get(i)));
      }

      int k = Math.min(50,distances.length-1);

      return QuickSelectArray.select(distances,k,distances.length);
   }

   private static double distanceFromSensor(  List<Point3D_F64> points ) {
      double min = Double.MAX_VALUE;

      for (int i = 0; i < points.size(); i++) {
         double d = points.get(i).normSq();
         if( d < min )
            min = d;
      }

      return Math.sqrt(min);
   }

   private static class Candidate
   {
      public PlaneGeneral3D_F64 planeFloor;
      public PlaneGeneral3D_F64 planeWall;
      public List<Point3D_F64> pointsFloor;
      public List<Point3D_F64> pointsWall;
      public int indexFloor;
      public int indexWall;

      public Se3_F64 testbedToWorld = new Se3_F64();

      public Candidate(PlaneGeneral3D_F64 floor, PlaneGeneral3D_F64 wall, List<Point3D_F64> pointsFloor, List<Point3D_F64> pointsWall,
                       int indexFloor , int indexWall ) {
         this.planeFloor = floor;
         this.planeWall = wall;
         this.pointsFloor = pointsFloor;
         this.pointsWall = pointsWall;
         this.indexFloor = indexFloor;
         this.indexWall = indexWall;
      }

      public void swap() {
         PlaneGeneral3D_F64 tmp = planeFloor;
         planeFloor = planeWall;
         planeWall = tmp;

         List<Point3D_F64> tmo = pointsFloor;
         pointsFloor = pointsWall;
         pointsWall = tmo;

         int tmi = indexFloor;
         indexFloor = indexWall;
         indexWall = tmi;
      }
   }

   public static void main(String[] args) {

      String directory = "../SensorProcessing/data/testbed/2014-08-01/";

      Se3_F64 estimatedToModel = (Se3_F64) new XStream().fromXML(directory.getClass().
              getResourceAsStream("/testbed/estimatedToModel.xml"));
      TestbedAutomaticAlignment alg = new TestbedAutomaticAlignment(3,estimatedToModel);

      System.out.println("Loading and filtering point clouds");
      List<List<Point3D_F64>> scans0 = loadScanLines(directory+"cloud12_scans.txt");
      for (int i = 0; i < scans0.size(); i++) {
         alg.addScan(scans0.get(i));
      }

      if( alg.process() ) {

         FactoryVisualization3D factory = UtilDisplayBubo.createVisualize3D();
         PointCloudPanel gui = factory.displayPointCloud();

         gui.addPoints(alg.getCloud1(),0xFFFF0000,1);

         System.out.println();
         System.out.println("--------- Detecting Testbed");
         System.out.println();

         Se3_F64 testbedToWorld = alg.getEstimatedToWorld();
         System.out.println(testbedToWorld);

         gui.addAxis(testbedToWorld, 0.5, 0.02);

         try {
            new XStream().toXML(testbedToWorld, new FileOutputStream("estimatedTestbedToWorld.xml"));
         } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
         }

         ShowImages.showWindow(gui, "FooBar");
      }
   }
}
