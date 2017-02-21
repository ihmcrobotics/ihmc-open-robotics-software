package us.ihmc.ihmcPerception.depthData;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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
   public boolean isValidNearScan(Point3D point, Point3D lidarOrigin)
   {
      boolean valid = super.isValidNearScan(point, lidarOrigin);
      valid &= point.getZ() > getMidFootPoint().getZ() + parameters.nearScanZMinAboveFeet;
//      valid &= parameters.nearScanCollisions || robotBoundingBoxes.isValidPoint(lidarOrigin, point);
      valid &= Math.abs(getAngleToPelvis(point, lidarOrigin)) < parameters.nearScanRadians;

      return valid;
   }

   // TODO: isAheadOfPelvis must be commented out when debugging val currently
   @Override
   public boolean isValidPoint(Point3D point, Point3D lidarOrigin)
   {
      boolean valid = super.isValidPoint(point, lidarOrigin);
//      valid &= robotBoundingBoxes.isValidPoint(lidarOrigin, point);
      valid &= isAheadOfPelvis(point);

      return valid;
   }

   @Override
   public boolean isPossibleGround(Point3D point, Point3D lidarOrigin)
   {
      Point3D footAvg = getMidFootPoint();

      double footZ = footAvg.getZ();
      footAvg.setZ(point.getZ());

      double maxHeight = parameters.quadTreeZAboveFeet + point.distance(footAvg) * parameters.quadTreeZSlope;
      if (maxHeight > parameters.quadTreeZMax)
      {
         maxHeight = parameters.quadTreeZMax;
      }

      return (point.getZ() - footZ) < maxHeight;
   }

   private Point3D getMidFootPoint()
   {
      RigidBodyTransform temp = new RigidBodyTransform();
      Point3D left = new Point3D();
      Point3D avg = new Point3D();

      fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame().getTransformToDesiredFrame(temp, ReferenceFrame.getWorldFrame());
      temp.transform(left);
      fullRobotModel.getFoot(RobotSide.RIGHT).getBodyFixedFrame().getTransformToDesiredFrame(temp, ReferenceFrame.getWorldFrame());
      temp.transform(avg);

      avg.add(left);
      avg.scale(0.5);

      return avg;
   }

   private boolean isAheadOfPelvis(Point3D point)
   {
      RigidBodyTransform tf = new RigidBodyTransform();
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tf, fullRobotModel.getPelvis().getBodyFixedFrame());
      Point3D tfPoint = new Point3D(point);
      tf.transform(tfPoint);

      return tfPoint.getX() > parameters.xCutoffPelvis;
   }

   private double getAngleToPelvis(Point3D point, Point3D lidarOrigin)
   {
      RigidBodyTransform tf = new RigidBodyTransform();
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tf, fullRobotModel.getPelvis().getBodyFixedFrame());
      Point3D tfPoint = new Point3D(point);
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
