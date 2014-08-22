package us.ihmc.sensorProcessing.pointClouds.testbed;

import boofcv.gui.image.ShowImages;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.gui.FactoryVisualization3D;
import bubo.gui.UtilDisplayBubo;
import bubo.gui.d3.PointCloudPanel;
import com.thoughtworks.xstream.XStream;
import georegression.geometry.*;
import georegression.metric.ClosestPoint3D_F64;
import georegression.metric.Intersection3D_F64;
import georegression.metric.UtilAngle;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Box3D_F64;
import georegression.struct.shapes.Rectangle2D_F64;
import georegression.transform.se.SePointOps_F64;

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

      Se3_F64 best = new Se3_F64();
      int bestCount = 0;

      Se3_F64 found = new Se3_F64();
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

            // see if they are perpendicular
            if( UtilAngle.dist(angle,Math.PI/2) < 0.1 ) {
               // see how much they overlap
//               int count = computeTestbed(planeA,planeB,pointsA,pointsB,a,b,cloud,found);
               int count = computeTestbed(planeA,planeB,pointsA,pointsB,a,b,cloud,found);
//               System.out.println("  candidate "+count);
               if( count > bestCount ) {
                  bestCount = count;
                  best.set(found);
               }
            }
         }
      }

      return bestCount == 0 ? null : best;
   }

   private static int findValve( Se3_F64 testbedToWorld , List<Point3D_F64> cloud ) {

      Point3D_F64 p = new Point3D_F64(0.25,0.55,1.0);
      double r = 0.15;

      // translate center of box into world
      SePointOps_F64.transform(testbedToWorld,p,p);

      Box3D_F64 box = new Box3D_F64(p.x-r,p.y-r,p.z-r,p.x+r,p.y+r,p.z+r);

      int count = 0;
      for (int i = 0; i < cloud.size(); i++) {
         if( Intersection3D_F64.contained(box,cloud.get(i)) ) {
            count++;
         }
      }

      return count;
   }

   private static int computeTestbed( PlaneGeneral3D_F64 planeA , PlaneGeneral3D_F64 planeB ,
                                      List<Point3D_F64> pointsA , List<Point3D_F64> pointsB ,
                                      Vector3D_F64 normalA , Vector3D_F64 normalB ,
                                      List<Point3D_F64> cloud , Se3_F64 testbedToWorld ) {
      LineParametric3D_F64 intersection = new LineParametric3D_F64();
      Intersection3D_F64.intersect(planeA, planeB, intersection);

      double whereA[] = findLineLocation(intersection,pointsA);
      double whereB[] = findLineLocation(intersection,pointsB);

      double min = Math.min(whereA[0],whereB[0]);
      double max = Math.max(whereA[1], whereB[1]);

      double overlap = (Math.min(whereA[1], whereB[1]) - Math.max(whereA[0], whereB[0]))/(max-min);

//      System.out.println("Overlap = "+overlap);
      if( overlap < 0.7 )
         return 0;

      Point3D_F64 centerPt = intersection.getPointOnLine((max+min)/2.0);
      Point3D_F64 minPt = intersection.getPointOnLine(min);
      Point3D_F64 maxPt = intersection.getPointOnLine(max);

      Vector3D_F64 axisX = normalB;
      Vector3D_F64 axisZ = normalA;

      // resolve sign ambiguity
      adjustSign(axisX,centerPt,pointsA);
      adjustSign(axisZ,centerPt,pointsB);

      Vector3D_F64 axisY = axisZ.cross(axisX);
      axisY.normalize();
      axisZ.normalize();
      axisX = axisY.cross(axisZ);

      // get the point in the corner
      double angleMin = axisY.acute(new Vector3D_F64(centerPt,minPt));
      double angleMax = axisY.acute(new Vector3D_F64(centerPt,maxPt));

//      System.out.println("min = "+angleMin+"  max = "+angleMax);

      Point3D_F64 selected0 = angleMin > angleMax ? minPt : maxPt;
      Point3D_F64 selected1 = angleMin <= angleMax ? minPt : maxPt;

      Se3_F64 testbedToWorld0 = new Se3_F64();
      Se3_F64 testbedToWorld1 = new Se3_F64();

      UtilVector3D_F64.createMatrix(testbedToWorld0.R,axisX,axisY,axisZ);
      testbedToWorld0.getT().set(selected0.x, selected0.y, selected0.z);

      UtilVector3D_F64.createMatrix(testbedToWorld1.R,axisZ,axisY.times(-1),axisX);
      testbedToWorld1.getT().set(selected1.x,selected1.y,selected1.z);

      int count0 = findValve(testbedToWorld0, cloud);
      int count1 = findValve(testbedToWorld1, cloud);

      if( count0 > count1 ) {
         testbedToWorld.set(testbedToWorld0);
         return count0;
      } else {
         testbedToWorld.set(testbedToWorld1);
         return count1;
      }
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
