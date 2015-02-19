package us.ihmc.ihmcPerception.depthData;

import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;

public class RobotDepthDataFilter extends DepthDataFilter
{
   private final RobotBoundingBoxes robotBoundingBoxes;
   private final SDFFullRobotModel fullRobotModel;

   public RobotDepthDataFilter(RobotBoundingBoxes robotBoundingBoxes, SDFFullRobotModel fullRobotModel)
   {
      super((fullRobotModel.getHead() == null) ? ReferenceFrame.getWorldFrame() : fullRobotModel.getHead().getBodyFixedFrame());
      this.robotBoundingBoxes = robotBoundingBoxes;
      this.fullRobotModel = fullRobotModel;
   }

   @Override
   public boolean isValidNearScan(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = super.isValidNearScan(point, lidarOrigin);
      valid &= point.z > getMidFootPoint().z + parameters.nearScanZMinAboveFeet;
      valid &= parameters.nearScanCollisions || robotBoundingBoxes.isValidPoint(lidarOrigin, point);
      valid &= Math.abs(getAngleToPelvis(point, lidarOrigin)) < parameters.nearScanRadians;

      return valid;
   }

   // TODO: isAheadOfPelvis must be commented out when debugging val currently
   @Override
   public boolean isValidOctree(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = super.isValidOctree(point, lidarOrigin);
      valid &= robotBoundingBoxes.isValidPoint(lidarOrigin, point);
      valid &= isAheadOfPelvis(point);

      return valid;
   }

   @Override
   public boolean isPossibleGround(Point3d point, Point3d lidarOrigin)
   {
      Point3d footAvg = getMidFootPoint();

      double footZ = footAvg.z;
      footAvg.setZ(point.z);

      double maxHeight = parameters.quadTreeZAboveFeet + point.distance(footAvg) * parameters.quadTreeZSlope;
      if (maxHeight > parameters.quadTreeZMax)
      {
         maxHeight = parameters.quadTreeZMax;
      }

      return (point.z - footZ) < maxHeight;
   }

   public Point3d getMidFootPoint()
   {
      RigidBodyTransform temp = new RigidBodyTransform();
      Point3d left = new Point3d();
      Point3d avg = new Point3d();

      fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame().getTransformToDesiredFrame(temp, ReferenceFrame.getWorldFrame());
      temp.transform(left);
      fullRobotModel.getFoot(RobotSide.RIGHT).getBodyFixedFrame().getTransformToDesiredFrame(temp, ReferenceFrame.getWorldFrame());
      temp.transform(avg);

      avg.add(left);
      avg.scale(0.5);

      return avg;
   }

   private boolean isAheadOfPelvis(Point3d point)
   {
      RigidBodyTransform tf = new RigidBodyTransform();
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tf, fullRobotModel.getPelvis().getBodyFixedFrame());
      Point3d tfPoint = new Point3d(point);
      tf.transform(tfPoint);

      return tfPoint.x > parameters.xCutoffPelvis;
   }

   public double getAngleToPelvis(Point3d point, Point3d lidarOrigin)
   {
      RigidBodyTransform tf = new RigidBodyTransform();
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tf, fullRobotModel.getPelvis().getBodyFixedFrame());
      Point3d tfPoint = new Point3d(point);
      tf.transform(tfPoint);

      return Math.atan2(tfPoint.y, tfPoint.x);
   }

   @Override
   public void setParameters(DepthDataFilterParameters parameters)
   {
      super.setParameters(parameters);
      robotBoundingBoxes.setScale(parameters.boundingBoxScale);

   }
}
