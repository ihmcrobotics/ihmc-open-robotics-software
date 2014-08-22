package us.ihmc.sensorProcessing.pointClouds.testbed;

import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import bubo.clouds.filter.UniformDensityCloudOctree;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Peter Abeles
 */
public class TestbedAutomaticAlignment {

   double maxDistancePoint;

   List<Point3D_F64> cloud0 = new ArrayList<>();
   List<Point3D_F64> cloud1 = new ArrayList<>();

   PointCloudShapeFinder findPlanes;

   UniformDensityCloudOctree uniform = new UniformDensityCloudOctree(20,0.05,234234);

   Se3_F64 modelToWorld = new Se3_F64();
   Se3_F64 modelToEstimated = new Se3_F64();
   Se3_F64 estimatedToWorld;

   Point3D_F64 headLocation = new Point3D_F64();

   public TestbedAutomaticAlignment(double maxDistancePoint , Se3_F64 estimatedToModel ) {
      this.maxDistancePoint = maxDistancePoint;
      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac.createDefault(250,1.2,0.025, CloudShapeTypes.PLANE);
      configRansac.minimumPoints = 2500;
//      ConfigSchnabel2007 configSchnabel = ConfigSchnabel2007.createDefault(20000,0.6,0.15,CloudShapeTypes.PLANE);

      findPlanes = FactoryPointCloudShape.ransacSingleAll(new ConfigSurfaceNormals(100, 0.15), configRansac);

      estimatedToModel.invert(modelToEstimated);
   }

   public void reset() {
      cloud0.clear();
      cloud1.clear();
   }

   public void setheadLocation( double x ,double y , double z ) {
      headLocation.set(x,y,z);
   }

   public void addScan( List<Point3D_F64> scan ) {

      double r = maxDistancePoint*maxDistancePoint;

      for (int j = 0; j < scan.size(); j++) {
         Point3D_F64 p = scan.get(j);

         double d = headLocation.distance(p);

         if( d <= r && countNeighbors(p,0.08,scan) > 6 ) {
            cloud0.add(p);
         }
      }
   }

   public void addCloud( List<Point3D_F64> cloud ) {
      cloud0.addAll(cloud);
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

   public boolean process() {
      uniform.process(cloud0,cloud1);
      findPlanes.process(cloud1, null);

      estimatedToWorld =  DetectTestbedSaveTransform.findTestbed(findPlanes.getFound(),cloud1);

      if( estimatedToWorld != null ) {
         modelToEstimated.concat(estimatedToWorld, modelToWorld);
         return true;
      } else {
         return false;
      }
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
