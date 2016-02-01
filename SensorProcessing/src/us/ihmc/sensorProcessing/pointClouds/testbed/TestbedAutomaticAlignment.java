package us.ihmc.sensorProcessing.pointClouds.testbed;

import georegression.geometry.GeometryMath_F64;
import georegression.geometry.UtilPlane3D_F64;
import georegression.geometry.UtilPoint3D_F64;
import georegression.geometry.UtilVector3D_F64;
import georegression.metric.ClosestPoint3D_F64;
import georegression.metric.Intersection3D_F64;
import georegression.metric.UtilAngle;
import georegression.struct.line.LineParametric3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.shapes.Box3D_F64;
import georegression.transform.se.SePointOps_F64;

import java.util.ArrayList;
import java.util.List;

import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import bubo.clouds.filter.UniformDensityCloudOctree;

/**
 * Automatic algorithm for detecting the testbed from LIDAR scans.  Uses RANSAC to find all the larger planes then
 * looks for perpendicular planes which are next to each other and have a blob of points near the valve.
 *
 * A transform is needed to go from found testbed reference frame to the testbed model reference frame.  The transform
 * is found using manually aligned data so that it can emulate what a user would selects.
 *
 * 1) Collect point cloud data
 * 2) Manually align data using TestBedManualAlignmentApplication.  It will save the results in the same directory
 * 3) Specify directory in ComputeEstimatedToWorldBatch to the same dataset
 * 4) Run ComputeEstimatedToModel on that directory and it will create a estimatedToModel transform and save it
 *
 * @author Peter Abeles
 */
public class TestbedAutomaticAlignment {

   // Maximum Euclidean distance of a point from the sensor head
   double maxDistancePoint;

   // original cloud before uniform density filtering
   List<Point3D_F64> cloud0 = new ArrayList<>();
   // cloud after filtering
   List<Point3D_F64> cloud1 = new ArrayList<>();

   // Code for detecting the planes
   PointCloudShapeFinder findPlanes;

   // Ensures that the point cloud has a uniform density
   UniformDensityCloudOctree uniform = new UniformDensityCloudOctree(20,0.05,234234);

   // transforms
   Se3_F64 modelToWorld = new Se3_F64();
   Se3_F64 modelToEstimated = new Se3_F64();
   Se3_F64 estimatedToWorld;

   Point3D_F64 headLocation = new Point3D_F64();

   /**
    *
    * @param maxDistancePoint Points greater than this distance from the sensor head are filtered
    * @param estimatedToModel Known transform from the found testbed to the testbed 3d model
    */
   public TestbedAutomaticAlignment(double maxDistancePoint , Se3_F64 estimatedToModel ) {
      this.maxDistancePoint = maxDistancePoint;
      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac.createDefault(250,1.2,0.025, CloudShapeTypes.PLANE);
      configRansac.minimumPoints = 2500;
//      ConfigSchnabel2007 configSchnabel = ConfigSchnabel2007.createDefault(20000,0.6,0.15,CloudShapeTypes.PLANE);

      findPlanes = FactoryPointCloudShape.ransacSingleAll(new ConfigSurfaceNormals(100, 0.15), configRansac);

      estimatedToModel.invert(modelToEstimated);
   }

   /**
    * Clears the point clouds.  Must be called before a new set of data is processed.
    */
   public void reset() {
      cloud0.clear();
      cloud1.clear();
   }

   /**
    * Sets the location of the sensor head in the world frame
    */
   public void setheadLocation( double x ,double y , double z ) {
      headLocation.set(x,y,z);
   }

   /**
    * Adds a new scan to the point cloud.  Filters out noise and points far away from the robot
    */
   public void addScan( List<Point3D_F64> scan ) {

      double r = maxDistancePoint*maxDistancePoint;

      for (int j = 0; j < scan.size(); j++) {
         Point3D_F64 p = scan.get(j);

         double d = headLocation.distance2(p);

         if( d <= r && countNeighbors(p,0.08,scan) > 6 ) {
            cloud0.add(p);
         }
      }
   }

   /**
    * Adds a whole point cloud
    */
   public void addCloud( List<Point3D_F64> cloud ) {
      cloud0.addAll(cloud);
   }

   /**
    * Used to filter out LIDAR shadow.  Looks to see how many neighbors there are near a point
    */
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

   /**
    * Processes the point cloud and detects the testbed
    *
    * @return true if successful and false if not
    */
   public boolean process() {
      uniform.process(cloud0,cloud1);
      findPlanes.process(cloud1, null);

      estimatedToWorld =  findTestbed(findPlanes.getFound(),cloud1);

      if( estimatedToWorld != null ) {
         modelToEstimated.concat(estimatedToWorld, modelToWorld);
         return true;
      } else {
         return false;
      }
   }

   /**
    * Given the found planes search for the testbed and compute its transform
    */
   public static Se3_F64 findTestbed( List<PointCloudShapeFinder.Shape> shapes , List<Point3D_F64> cloud ) {

      List<Vector3D_F64> normals = new ArrayList<>();

      // compute the orientation of each plane
      for (int i = 0; i < shapes.size(); i++) {
         PlaneGeneral3D_F64 plane = (PlaneGeneral3D_F64)shapes.get(i).parameters;
         Se3_F64 planeToWorld = UtilPlane3D_F64.planeToWorld(plane, null);
         Vector3D_F64 normal = new Vector3D_F64(0,0,1);
         GeometryMath_F64.mult(planeToWorld.getR(), normal, normal);
         normals.add(normal);
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

            System.out.println("Angle "+angle);

            // see if they are perpendicular
            if( UtilAngle.dist(angle, Math.PI / 2) < 0.1 ) {
               // see how much they overlap
               int count = computeTestbed(planeA,planeB,pointsA,pointsB,a,b,cloud,found);
               if( count > bestCount ) {
                  bestCount = count;
                  best.set(found);
               }
            }
         }
      }

      return bestCount == 0 ? null : best;
   }

   /**
    * Counts how many points are in the region where the valve should be
    */
   private static int findValve( Se3_F64 testbedToWorld , List<Point3D_F64> cloud ) {

      Point3D_F64 p = new Point3D_F64(0.25,0.55,1.0);
      double r = 0.15;

      // translate center of box into world
      SePointOps_F64.transform(testbedToWorld, p, p);

      Box3D_F64 box = new Box3D_F64(p.x-r,p.y-r,p.z-r,p.x+r,p.y+r,p.z+r);

      int count = 0;
      for (int i = 0; i < cloud.size(); i++) {
         if( Intersection3D_F64.contained(box, cloud.get(i)) ) {
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

      if( whereA == null || whereB == null )
         return 0;

      double min = Math.min(whereA[0],whereB[0]);
      double max = Math.max(whereA[1],whereB[1]);

      double overlap = (Math.min(whereA[1], whereB[1]) - Math.max(whereA[0], whereB[0]))/(max-min);

      // the two planes should overlap
      System.out.println("Overlap = "+overlap+"  "+pointsA.size()+" "+pointsB.size());
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

      UtilVector3D_F64.createMatrix(testbedToWorld0.R, axisX, axisY, axisZ);
      testbedToWorld0.getT().set(selected0.x, selected0.y, selected0.z);

      UtilVector3D_F64.createMatrix(testbedToWorld1.R,axisZ,axisY.times(-1),axisX);
      testbedToWorld1.getT().set(selected1.x,selected1.y,selected1.z);

      int count0 = findValve(testbedToWorld0, cloud);
      int count1 = findValve(testbedToWorld1, cloud);

      System.out.println(count0+"   "+count1);

      if( count0 > count1 ) {
         testbedToWorld.set(testbedToWorld0);
         return count0;
      } else {
         testbedToWorld.set(testbedToWorld1);
         return count1;
      }
   }

   /**
    * Finds the points at the beginning and end of the line.  coordinate specified in 1-D
    */
   public static double[] findLineLocation( LineParametric3D_F64 line , List<Point3D_F64> points ) {
      double min = Double.MAX_VALUE;
      double max = -Double.MAX_VALUE;

      Point3D_F64 p = new Point3D_F64();

      double closest = Double.MAX_VALUE;
      for (int i = 0; i < points.size(); i++) {
         double t = ClosestPoint3D_F64.closestPoint(line, points.get(i));
         p.x = line.p.x + t*line.slope.x;
         p.y = line.p.y + t*line.slope.y;
         p.z = line.p.z + t*line.slope.z;

         double d = p.distance2(points.get(i));
         if( d < closest )
            closest = d;

         // only consider points within a meter so that when it is close it's not as skewed
         if( d <= 1.0 ) {
            if (t < min)
               min = t;
            if (t > max)
               max = t;
         }
      }

      if( closest > 0.3*0.3 || min == Double.MAX_VALUE )
         return null;

      return new double[]{min,max};
   }

   /**
    * Adjusts the sign such that the vector is pointing towards the center of the provided cloud
    */
   public static void adjustSign( Vector3D_F64 v , Point3D_F64 start , List<Point3D_F64> cloud ) {
      Point3D_F64 centroid = UtilPoint3D_F64.mean(cloud, cloud.size(), null);

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

   public List<Point3D_F64> getCloud1() {
      return cloud1;
   }

   public Se3_F64 getEstimatedToWorld() {
      return estimatedToWorld;
   }

   public Se3_F64 getModelToWorld() {
      return modelToWorld;
   }
}
