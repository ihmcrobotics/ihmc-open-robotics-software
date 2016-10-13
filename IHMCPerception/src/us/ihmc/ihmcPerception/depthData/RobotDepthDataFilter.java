package us.ihmc.ihmcPerception.depthData;

import javax.vecmath.Point3d;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class RobotDepthDataFilter extends DepthDataFilter
{
   private final FullHumanoidRobotModel fullRobotModel;


   public RobotDepthDataFilter(FullHumanoidRobotModel fullRobotModel)
   {
      super();
      this.fullRobotModel = fullRobotModel;
   }

   
   @Override
   public boolean isValidNearScan(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = super.isValidNearScan(point, lidarOrigin);
      valid &= point.getZ() > getMidFootPoint().getZ() + parameters.nearScanZMinAboveFeet;
//      valid &= parameters.nearScanCollisions || robotBoundingBoxes.isValidPoint(lidarOrigin, point);
      valid &= Math.abs(getAngleToPelvis(point, lidarOrigin)) < parameters.nearScanRadians;

      return valid;
   }

   // TODO: isAheadOfPelvis must be commented out when debugging val currently
   @Override
   public boolean isValidPoint(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = super.isValidPoint(point, lidarOrigin);
//      valid &= robotBoundingBoxes.isValidPoint(lidarOrigin, point);
      valid &= isAheadOfPelvis(point);

      return valid;
   }

   @Override
   public boolean isPossibleGround(Point3d point, Point3d lidarOrigin)
   {
      Point3d footAvg = getMidFootPoint();

      double footZ = footAvg.getZ();
      footAvg.setZ(point.getZ());

      double maxHeight = parameters.quadTreeZAboveFeet + point.distance(footAvg) * parameters.quadTreeZSlope;
      if (maxHeight > parameters.quadTreeZMax)
      {
         maxHeight = parameters.quadTreeZMax;
      }

      return (point.getZ() - footZ) < maxHeight;
   }

   private Point3d getMidFootPoint()
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

      return tfPoint.getX() > parameters.xCutoffPelvis;
   }

   private double getAngleToPelvis(Point3d point, Point3d lidarOrigin)
   {
      RigidBodyTransform tf = new RigidBodyTransform();
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tf, fullRobotModel.getPelvis().getBodyFixedFrame());
      Point3d tfPoint = new Point3d(point);
      tf.transform(tfPoint);

      return Math.atan2(tfPoint.getY(), tfPoint.getX());
   }

   @Override
   public void setParameters(DepthDataFilterParameters parameters)
   {
      super.setParameters(parameters);
//      robotBoundingBoxes.setScale(parameters.boundingBoxScale);

   }
}
