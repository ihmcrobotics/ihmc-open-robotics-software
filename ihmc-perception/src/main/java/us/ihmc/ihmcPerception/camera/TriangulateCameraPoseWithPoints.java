package us.ihmc.ihmcPerception.camera;

import java.util.ArrayList;
import java.util.List;

import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.fitting.modelset.ransac.Ransac;

import boofcv.abst.geo.Estimate1ofPnP;
import boofcv.alg.geo.DistanceModelMonoPixels;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.pose.PnPDistanceReprojectionSq;
import boofcv.factory.geo.EnumPNP;
import boofcv.factory.geo.EstimatorToGenerator;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.geo.Point2D3D;
import georegression.fitting.se.ModelManagerSe3_F64;
import georegression.struct.GeoTuple3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

/**
 * Given a set of known points, triangulate the camera pose in the world.
 *
 * @author Peter Abeles
 */
public class TriangulateCameraPoseWithPoints
{
   /**
    * Obvservations are hand selected inside a scaled 600x600 image.  Orignal image size is 800x800
    */
   public static List<Point2D3D> createObservations() {
      List<Point2D_F64> pixels = new ArrayList<Point2D_F64>();
      List<Point3D_F64> locations = new ArrayList<Point3D_F64>();

      // center of box
      Point3D_F64 box = new Point3D_F64(5.5,0.5,0.5);
      double r = 0.5;

      pixels.add(new Point2D_F64(159,106));
      locations.add( new Point3D_F64(box.x+r,box.y+r,box.z+r));

      pixels.add(new Point2D_F64(299,106));
      locations.add( new Point3D_F64(box.x+r,box.y-r,box.z+r));

      pixels.add(new Point2D_F64(299,150));
      locations.add( new Point3D_F64(box.x-r,box.y-r,box.z+r));

      pixels.add(new Point2D_F64(298,302));
      locations.add( new Point3D_F64(box.x-r,box.y-r,box.z-r));

      pixels.add(new Point2D_F64(151,302));
      locations.add( new Point3D_F64(box.x-r,box.y+r,box.z-r));

      pixels.add(new Point2D_F64(99,147));
      locations.add( new Point3D_F64(box.x-r,box.y+r,box.z+r));

      List<Point2D3D> ret = new ArrayList<>();
      for( int i = 0; i < pixels.size(); i++ )
         ret.add( new Point2D3D(pixels.get(i),locations.get(i)));

      return ret;
   }

   public static void convertToCameraSystem( List<Point2D3D> obs ) {
      for( Point2D3D o : obs ) {
         workToCamera(o.location);
      }
   }

   public static void workToCamera(Point3D_F64 p) {
      double x = p.x;
      double y = p.y;
      double z = p.z;

      p.x = -y;
      p.y = -z;
      p.z =  x;
   }

   public static void cameraToWorld(GeoTuple3D_F64 p) {
      double x = p.x;
      double y = p.y;
      double z = p.z;

      p.x =  z;
      p.y = -x;
      p.z = -y;
   }

   public static void convertToCalibrate( List<Point2D3D> obs , IntrinsicParameters intrinsic ) {
      for( Point2D3D o : obs ) {
         PerspectiveOps.convertPixelToNorm(intrinsic,o.observation,o.observation);
      }
   }

   public static void main(String[] args)
   {
      double inlierPixelTol = 1.5;

      IntrinsicParameters intrinsic = new IntrinsicParameters(476,476,0,400.5,400.5,800,800);
      PerspectiveOps.scaleIntrinsic(intrinsic,600.0/800.0);

      List<Point2D3D> obs = createObservations();
      convertToCameraSystem(obs);
      convertToCalibrate(obs,intrinsic);

      Estimate1ofPnP estimator = FactoryMultiView.computePnP_1(EnumPNP.P3P_FINSTERWALDER, -1, 1);
      final DistanceModelMonoPixels<Se3_F64,Point2D3D> distance = new PnPDistanceReprojectionSq();
      distance.setIntrinsic(intrinsic.fx,intrinsic.fy,intrinsic.skew);

      ModelManagerSe3_F64 manager = new ModelManagerSe3_F64();
      EstimatorToGenerator<Se3_F64,Point2D3D> generator = new EstimatorToGenerator<Se3_F64,Point2D3D>(estimator);

      // 1/2 a pixel tolerance for RANSAC inliers
      double ransacTOL = inlierPixelTol * inlierPixelTol;

      ModelMatcher<Se3_F64, Point2D3D> motion = new Ransac<Se3_F64, Point2D3D>(2323, manager, generator, distance, 200, ransacTOL);

      if( !motion.process(obs) )
         throw new RuntimeException("Failed ransac");

      System.out.println("inlier size "+motion.getMatchSet().size());

      Se3_F64 camToWorld = motion.getModelParameters();
      Se3_F64 worldToCam = motion.getModelParameters().invert(null);

      System.out.println("camera coordinate:  cam to world");
      camToWorld.print();
      System.out.println("camera coordinate:  world to cam");
      worldToCam.print();

      System.out.println("world coordinate: world to cam ");
      cameraToWorld(worldToCam.getT());
      System.out.println("Location in world " + worldToCam.getT());

      System.out.println("head at world: 3.2  0   1.47");

      // cam pos:(0.17361246918263024, 0.025942661928142104, 0.4475182500676753)quat: (-2.7001006509502116E-5, 0.3332867475694028, 4.1077389840779185E-5, 0.9428255095605989)
   }
}
