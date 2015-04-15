package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCHandType;

public class RobotDepthDataFilter extends DepthDataFilter
{
   private final FullRobotModel fullRobotModel;
   private final SideDependentList<ArrayList<Point2d>> contactPoints;


   public RobotDepthDataFilter(DRCHandType drcHandType, SDFFullRobotModel fullRobotModel, SideDependentList<ArrayList<Point2d>> contactPoints)
   {
      super();
      this.fullRobotModel = fullRobotModel;
      this.contactPoints = contactPoints;
   }

   
   @Override
   public boolean isValidNearScan(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = super.isValidNearScan(point, lidarOrigin);
      valid &= point.z > getMidFootPoint().z + parameters.nearScanZMinAboveFeet;
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

      double footZ = footAvg.z;
      footAvg.setZ(point.z);

      double maxHeight = parameters.quadTreeZAboveFeet + point.distance(footAvg) * parameters.quadTreeZSlope;
      if (maxHeight > parameters.quadTreeZMax)
      {
         maxHeight = parameters.quadTreeZMax;
      }

      return (point.z - footZ) < maxHeight;
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

      return tfPoint.x > parameters.xCutoffPelvis;
   }

   private double getAngleToPelvis(Point3d point, Point3d lidarOrigin)
   {
      RigidBodyTransform tf = new RigidBodyTransform();
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tf, fullRobotModel.getPelvis().getBodyFixedFrame());
      Point3d tfPoint = new Point3d(point);
      tf.transform(tfPoint);

      return Math.atan2(tfPoint.y, tfPoint.x);
   }

   public void clearLidarData(DepthDataTree lidarTree)
   {
      super.clearLidarData(lidarTree);
      getQuadTree().clearTree(getMidFootPoint().z);
      addPointsUnderFeet();
   }

   private void addPointsUnderFeet()
   {
      final double QuadTreePointUnderFeetScaling = 1.1;
      for (RobotSide side : RobotSide.values)
      {
         ReferenceFrame soleFrame = fullRobotModel.getFoot(side).getBodyFixedFrame();
         for (Point2d point : contactPoints.get(side))
         {
            FramePoint footContactPoint = new FramePoint(soleFrame, point.getX(), point.getY(), 0.0);
            footContactPoint.scale(QuadTreePointUnderFeetScaling);
            footContactPoint.changeFrame(ReferenceFrame.getWorldFrame());
            getQuadTree().addPoint(footContactPoint.getX(), footContactPoint.getY(), footContactPoint.getZ());
         }

         FramePoint footCenter = new FramePoint(soleFrame, 0.0, 0.0, 0.0);
         footCenter.changeFrame(ReferenceFrame.getWorldFrame());
            getQuadTree().addPoint(footCenter.getX(), footCenter.getY(), footCenter.getZ());

      }

   }


   @Override
   public void setParameters(DepthDataFilterParameters parameters)
   {
      super.setParameters(parameters);
//      robotBoundingBoxes.setScale(parameters.boundingBoxScale);

   }
}
