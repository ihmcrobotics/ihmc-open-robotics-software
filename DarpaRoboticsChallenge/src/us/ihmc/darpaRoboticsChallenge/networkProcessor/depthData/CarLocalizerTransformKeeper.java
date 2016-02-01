package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import georegression.struct.point.Point3D_F64;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class CarLocalizerTransformKeeper
{
   private static final double BOUNDING_INNER_WHEEL_RADIUS = 0.051;
   public static final double BOUNDING_OUTER_WHEEL_RADIUS = 0.135;
   private static final double BOUNDING_WHEEL_FAR_X = 0.07600000000000001;
   private static final double BOUNDING_WHEEL_NEAR_X = 0.00;
   static final double WHEEL_FRAME_X = 0.389364477992432;
   static final double WHEEL_FRAME_XX = -0.05;
   static final double WHEEL_FRAME_Y = 0.044410360039198536;
   static final double WHEEL_FRAME_YY = .72;
   static final double WHEEL_FRAME_Z = 0.31863173661042754;
   static final double WHEEL_FRAME_ZZ = 0.31;

   private static final double OLD_PELVIS_X = -0.018;
   private static final double OLD_PELVIS_Y = -0.005;
   private static final double OLD_PELVIS_Z = -0.115;
   private static final double OLD_PELVIS_XX = -0.031;
   private static final double OLD_PELVIS_YY = 0.073;
   private static final double OLD_PELVIS_ZZ = -0.22;


   // (-0.062481913473283565, 0.34247790931370775, 0.18649702326635595, 0.9187076534097331) and translation (0.07878049463644286, 0.032262334981346175, 0.8466124223441878)
   private static final double CAMERA_QX = -0.062481913473283565;
   private static final double CAMERA_QY = 0.34247790931370775;
   private static final double CAMERA_QZ = 0.18649702326635595;
   private static final double CAMERA_QW = 0.9187076534097331;
   private static final double CAMERA_X = 0.07878049463644286;
   private static final double CAMERA_Y = 0.032262334981346175;
   private static final double CAMERA_Z = 0.8466124223441878;

   private final RigidBodyTransform transformToDesiredPelvis = new RigidBodyTransform(new double[]
   {
      0.9841102754435644, -0.04105097667157578, -0.17274774406834234, -0.06307616753352636, 0.04096992980892717, 0.9991522275780105, -0.004036208288863233,
      0.35943507901629285, 0.17276698358726802, -0.0031053888982297056, 0.9849578295246693, 1.139612394219399, 0.0, 0.0, 0.0, 1.0
   });
   private final RigidBodyTransform inverseTransformToDesiredPelvis = new RigidBodyTransform();

   {
      inverseTransformToDesiredPelvis.invert(transformToDesiredPelvis);
   }

   private final RigidBodyTransform transformFromOldBasePelvisToCameraStandard = new RigidBodyTransform();
   private final RigidBodyTransform inverseTransformFromOldBasePelvisToCameraStandard = new RigidBodyTransform();

   {
      Quat4d tempQuat = new Quat4d(CAMERA_QX, CAMERA_QY, CAMERA_QZ, CAMERA_QW);
      Vector3d tempVector = new Vector3d(CAMERA_X, CAMERA_Y, CAMERA_Z);
      transformFromOldBasePelvisToCameraStandard.set(tempQuat, tempVector);
      inverseTransformFromOldBasePelvisToCameraStandard.invert(transformFromOldBasePelvisToCameraStandard);
   }


   private static final double OLD_PELVIS_QUATERNION_W = Math.sqrt(1 - OLD_PELVIS_XX * OLD_PELVIS_XX - OLD_PELVIS_YY * OLD_PELVIS_YY
                                                            - OLD_PELVIS_ZZ * OLD_PELVIS_ZZ);
   private final RigidBodyTransform transformFromNewPelvisToOldPelvis = new RigidBodyTransform();
   private final RigidBodyTransform inverseTransformFromNewPelvisToOldPelvis = new RigidBodyTransform();
   private final RigidBodyTransform transformFromOldBasePelvisToWheel = new RigidBodyTransform();

   {
      Quat4d rot = new Quat4d(OLD_PELVIS_XX, OLD_PELVIS_YY, OLD_PELVIS_ZZ, OLD_PELVIS_QUATERNION_W);
      Vector3d translation = new Vector3d(OLD_PELVIS_X, OLD_PELVIS_Y, OLD_PELVIS_Z);
      transformFromNewPelvisToOldPelvis.set(rot, translation);
      inverseTransformFromNewPelvisToOldPelvis.invert(transformFromNewPelvisToOldPelvis);
   }

   private final RigidBodyTransform inverseTransformToWheel = new RigidBodyTransform();
   private final Point3d tempPoint = new Point3d();
   private final Vector3d tempVector = new Vector3d();
   private final Vector3d tempVector2 = new Vector3d();
   private final Vector3d tempVector3 = new Vector3d();
   private final RigidBodyTransform transformToWheel = new RigidBodyTransform();

   public void packTransformFromBasePelvisToWheel(RigidBodyTransform transformToPack)
   {
      transformFromOldBasePelvisToWheel.setEuler(WHEEL_FRAME_XX, WHEEL_FRAME_YY, WHEEL_FRAME_ZZ);
      transformFromOldBasePelvisToWheel.setTranslation(new Vector3d(WHEEL_FRAME_X, WHEEL_FRAME_Y, WHEEL_FRAME_Z));
      transformToPack.multiply(transformFromNewPelvisToOldPelvis, transformFromOldBasePelvisToWheel);
   }

   public void packInverseTransformFromBasePelvisToWheel(RigidBodyTransform transformToPack)
   {
      packTransformFromBasePelvisToWheel(transformToPack);
      transformToPack.invert();
   }

   public void packTransformFromOldBasePelvisToNewBasePelvis(RigidBodyTransform transformToPack)
   {
      transformToPack.set(inverseTransformFromNewPelvisToOldPelvis);
   }



   public boolean isPointInPelvisFrameNearWheel(double x, double y, double z)
   {
      packInverseTransformFromBasePelvisToWheel(inverseTransformToWheel);

      return isPointNearWheel(x, y, z, inverseTransformToWheel);
   }

   public void applyTransformFromBasePelvisToWheel(List<Point3D_F64> pointsOriginallyInWheelFrame)
   {
      packTransformFromBasePelvisToWheel(transformToWheel);
      applyTransformToPoints(pointsOriginallyInWheelFrame, transformToWheel);
   }

   public void applyInverseTransformFromBasePelvisToWheel(List<Point3D_F64> pointsOriginallyInBasePelvisFrame)
   {
      packInverseTransformFromBasePelvisToWheel(inverseTransformToWheel);
      applyTransformToPoints(pointsOriginallyInBasePelvisFrame, inverseTransformToWheel);
   }

   public void applyTransformToPoints(List<Point3D_F64> points, RigidBodyTransform transformToUse)
   {
      for (int i = 0; i < points.size(); i++)
      {
         tempPoint.set(points.get(i).x, points.get(i).y, points.get(i).z);
         transformToUse.transform(tempPoint);
         points.get(i).set(tempPoint.getX(), tempPoint.getY(), tempPoint.getZ());
      }
   }

   boolean isPointNearWheel(double x, double y, double z, RigidBodyTransform inverseTransformFromSomeframeToWheel)
   {
      return isPointConservitivelyNearWheel(x, y, z, inverseTransformFromSomeframeToWheel, 0.0);
   }

   public boolean isPointConservitivelyNearWheel(double x, double y, double z, RigidBodyTransform inverseTransformFromSomeframeToWheel, double fudgeFactor)
   {
      tempPoint.set(x, y, z);
      inverseTransformFromSomeframeToWheel.transform(tempPoint);
      x = tempPoint.x;
      y = tempPoint.y;
      z = tempPoint.z;

      return isPointNearWheel(x, y, z, fudgeFactor);
   }

   public boolean isPointNearWheel(double x, double y, double z, double fudgeFactor)
   {
      tempVector.set(x, y, z);
      tempVector3.set(1.0, 0.0, 0.0);
      double dotProduct = tempVector.dot(tempVector3);
      tempVector2.set(tempVector3);
      tempVector2.scale(dotProduct);
      tempVector.sub(tempVector2);
      double radius = tempVector.length();

      return (dotProduct > -BOUNDING_WHEEL_NEAR_X) && (dotProduct < BOUNDING_WHEEL_FAR_X) && (radius < BOUNDING_OUTER_WHEEL_RADIUS - fudgeFactor)
             && (radius > BOUNDING_INNER_WHEEL_RADIUS + fudgeFactor);
   }

   public boolean isInValidCarRegion(double x, double y, double z)
   {
      tempPoint.set(x, y, z);
      transformToDesiredPelvis.transform(tempPoint);

//    inverseTransformFromNewPelvisToOldPelvis.transform(tempPoint);
      x = tempPoint.x;
      y = tempPoint.y;
      z = tempPoint.z;

      return isPointInCarFrameWithinTemplateBounds(x, y, z);
   }

   public boolean isPointInCarFrameWithinTemplateBounds(double x, double y, double z)
   {
      double zClipPlane = 1.1;
      boolean isAboveLowerClipPlane = z>zClipPlane;
      boolean isBelowRollBar = z<1.85;
      boolean isBehindRobotZoneXBound = x<0.5;
      boolean isRightOfLeftWall=y<0.45;
      boolean isLeftOfRightWall = y>-0.2;
      boolean isInTheRobotZone = isBelowRollBar&&isBehindRobotZoneXBound&&isRightOfLeftWall&&isRightOfLeftWall&&isLeftOfRightWall;
      
      boolean isInLeftArmZone = isBelowRollBar&&x<0.4;
      boolean isInFrontOfSlantedBack = x>0.0;
      return isAboveLowerClipPlane&&isInFrontOfSlantedBack&&!isInTheRobotZone&&!isInLeftArmZone;
   }

   public void packInverseTransformFromOldBasePelvisToStandardCamera(RigidBodyTransform inverseTransformFromOldBasePelvisToStandardCameraToPack)
   {
      inverseTransformFromOldBasePelvisToStandardCameraToPack.set(this.inverseTransformFromOldBasePelvisToCameraStandard);
   }
   
   public void packTransformFromCarToDesiredPelvis(RigidBodyTransform transformFromCarToDesiredPelvisToPack)
   {
      transformFromCarToDesiredPelvisToPack.set(this.transformToDesiredPelvis);
   }

}
