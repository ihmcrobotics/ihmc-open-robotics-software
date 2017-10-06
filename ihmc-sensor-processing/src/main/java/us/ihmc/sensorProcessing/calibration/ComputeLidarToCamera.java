package us.ihmc.sensorProcessing.calibration;

import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.GeoTuple3D_F64;
import georegression.struct.plane.PlaneNormal3D_F64;
import georegression.struct.so.Quaternion_F64;
import us.ihmc.sensorProcessing.pointClouds.shape.EstimateMotionFromBox;
import us.ihmc.sensorProcessing.pointClouds.shape.IhmcPointCloudOps;

/**
 *
 * From Alex: Plane's LIDAR
 *
 * P( 0.5247640194038867 -0.39945635873663504 0.38590603161631976 ) V( -0.6867638250614645 0.5227724591002402 -0.505039012941807 )
 * P( 0.6126300640898954 0.3540583053918326 0.459725832957747 ) V( 0.7260256918861299 0.41959322796541115 0.5448194359297699 )
 * P( 0.9267633912726818 -0.2607428654010602 -0.45680965716854194 ) V( -0.8696916526697748 0.24468585581600652 0.4286785056894092 )
 *
 * @author Peter Abeles
 */
public class ComputeLidarToCamera
{

   public static void cameraToWorld(GeoTuple3D_F64 p) {
      double x = p.x;
      double y = p.y;
      double z = p.z;

      p.x =  z;
      p.y = -x;
      p.z = -y;
   }

   public static void main( String args[] ) {
      PlaneNormal3D_F64 camera0 = new PlaneNormal3D_F64(-0.2442424772296845,0.10157254619123487,1.5003267003526426,0.44682101275127867,-0.4128457418664506,0.7936683035038388);
      PlaneNormal3D_F64 camera1 = new PlaneNormal3D_F64(0.14894700258459206,0.13854568531017736,1.3003909885204032,-0.5255056708112398,-0.5118114040742201,0.6796270128569084);
      PlaneNormal3D_F64 camera2 = new PlaneNormal3D_F64(-0.1550633436315736,-0.25647845567954375,1.3157878496097721,0.12087719133464096,0.4789184317698088,0.86949746424368358);

      cameraToWorld(camera0.p);cameraToWorld(camera0.n);
      cameraToWorld(camera1.p);cameraToWorld(camera1.n);
      cameraToWorld(camera2.p);cameraToWorld(camera2.n);

      GeometryMath_F64.changeSign(camera2.getN());

      IhmcPointCloudOps.adjustBoxNormals(camera0,camera1,camera2);

      PlaneNormal3D_F64 lidar0 = new PlaneNormal3D_F64(0.5247640194038867,-0.39945635873663504,0.38590603161631976, -0.6867638250614645,0.5227724591002402,-0.505039012941807);
      PlaneNormal3D_F64 lidar1 = new PlaneNormal3D_F64(0.6126300640898954,0.3540583053918326,0.459725832957747, 0.7260256918861299,0.41959322796541115,0.5448194359297699);
      PlaneNormal3D_F64 lidar2 = new PlaneNormal3D_F64(0.9267633912726818,-0.2607428654010602,-0.45680965716854194, -0.8696916526697748,0.24468585581600652,0.4286785056894092);

      GeometryMath_F64.changeSign(lidar1.getN());
      GeometryMath_F64.changeSign(lidar2.getN());
//      IhmcPointCloudOps.adjustBoxNormals(lidar0,lidar1,lidar2);

      camera0.p.print();
      camera0.n.print();
      System.out.println("------------");
      lidar0.p.print();
      lidar0.n.print();
      System.out.println();
      camera1.p.print();
      camera1.n.print();
      System.out.println("------------");
      lidar1.p.print();
      lidar1.n.print();
      System.out.println();
      camera2.p.print();
      camera2.n.print();
      System.out.println("------------");
      lidar2.p.print();
      lidar2.n.print();

      EstimateMotionFromBox alg = new EstimateMotionFromBox();

      alg.setSrc(camera0,camera1,camera2);
      alg.computeMotion(lidar0,lidar1,lidar2);

      alg.getMotionSrcToDst().print();
      alg.getMotionSrcToDst().invert(null).print();
      Quaternion_F64 q = ConvertRotation3D_F64.matrixToQuaternion(alg.getMotionSrcToDst().getR(), null);
      System.out.println(q);

   }
}
