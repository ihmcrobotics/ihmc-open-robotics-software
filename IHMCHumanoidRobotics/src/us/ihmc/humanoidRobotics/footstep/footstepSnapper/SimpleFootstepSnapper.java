package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.InsufficientDataException;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.geometry.PlaneFitter;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * Created by agrabertilton on 1/14/15.
 */
public class SimpleFootstepSnapper implements FootstepSnapper
{
   // Generates Footsteps using grid without a mask
   private List<Point3d> pointList = new ArrayList<Point3d>();
   private final PlaneFitter planeFitter = new LeastSquaresZPlaneFitter();
   private double searchWidth = 0.15;
   private double searchLength = 0.15;
   private BasicFootstepMask footstepMask = null;
   private boolean useMask = false;
   private double maskBuffer = 0.0;

   @Override
   public Footstep generateSnappedFootstep(double soleX, double soleY, double yaw, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide,
           HeightMapWithPoints heightMap)
           throws InsufficientDataException
   {
      FramePose2d footPose2d = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(soleX, soleY), yaw);

      Footstep footstep = generateFootstepUsingHeightMap(footPose2d, foot, soleFrame, robotSide, heightMap);
      double z = footstep.getZ();
      if (Double.isInfinite(z) || Double.isNaN(z))
      {
         System.out.println("Houston, we have a problem in the SimpleFootstepSnapper");
      }

      return footstep;
   }

   @Override
   public void setMask(List<Point2d> footShape)
   {
      footstepMask = new BasicFootstepMask(footShape, maskBuffer);
   }

   public void setMask(BasicFootstepMask footstepMask)
   {
      this.footstepMask = footstepMask;
   }

   @Override
   public Footstep.FootstepType snapFootstep(Footstep footstep, HeightMapWithPoints heightMap){
      FootstepDataMessage originalFootstep = new FootstepDataMessage(footstep);

      //set to the sole pose
      Vector3d position = new Vector3d();
      Quat4d orientation = new Quat4d();
      RigidBodyTransform solePose = new RigidBodyTransform();
      footstep.getSolePose(solePose);
      solePose.get(orientation, position);
      originalFootstep.setLocation(new Point3d(position));
      originalFootstep.setOrientation(orientation);

      //get the footstep
      Footstep.FootstepType type = snapFootstep(originalFootstep, heightMap);
      footstep.setPredictedContactPointsFromPoint2ds(originalFootstep.getPredictedContactPoints());
      footstep.setFootstepType(type);
      FramePose solePoseInWorld = new FramePose(ReferenceFrame.getWorldFrame(), originalFootstep.getLocation(), originalFootstep.getOrientation());
      footstep.setSolePose(solePoseInWorld);

      footstep.setSwingHeight(originalFootstep.getSwingHeight());
      footstep.setTrajectoryType(originalFootstep.getTrajectoryType());

      return type;
   }

   @Override
   public Footstep.FootstepType snapFootstep(FootstepDataMessage footstep, HeightMapWithPoints heightMap)
   {
      Quat4d orientation = footstep.getOrientation();
      Point3d position = footstep.getLocation();
      double yaw = RotationTools.computeYaw(orientation);

      if (!useMask)
      {
         pointList = heightMap.getAllPointsWithinArea(position.getX(), position.getY(), searchWidth, searchLength);
      }
      else
      {
         if (footstepMask == null)
         {
            throw new RuntimeException("Footstep Snapper does not have a mask");
         }

         footstepMask.setPositionAndYaw(position.x, position.y, yaw);
         pointList = heightMap.getAllPointsWithinArea(position.x, position.y, searchWidth, searchLength, footstepMask);
      }

      Plane3d fittedPlane = new Plane3d();

      planeFitter.fitPlaneToPoints(new Point2d(position.getX(), position.getY()), pointList, fittedPlane);
      double height = fittedPlane.getZOnPlane(position.getX(), position.getY());

      Vector3d surfaceNormal = fittedPlane.getNormalCopy();
      if (MathTools.containsNaN(surfaceNormal))
      {
         surfaceNormal.set(0.0, 0.0, 1.0);
         height = heightMap.getHeightAtPoint(position.getX(), position.getY());
         if (Double.isInfinite(height))
            height = 0.0;
      }

      adjustFootstepWithoutHeightmap(footstep, height, surfaceNormal);
      return Footstep.FootstepType.FULL_FOOTSTEP;
   }

   @Override
   public Footstep generateFootstepUsingHeightMap(FramePose2d footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide,
           HeightMapWithPoints heightMap)
           throws InsufficientDataException
   {
      Point2d position = new Point2d(footPose2d.getX(), footPose2d.getY());
      if (!useMask)
      {
         pointList = heightMap.getAllPointsWithinArea(footPose2d.getX(), footPose2d.getY(), searchWidth, searchLength);
      }
      else
      {
         if (footstepMask == null)
         {
            throw new RuntimeException("Footstep Snapper does not have a mask");
         }

         footstepMask.setPositionAndYaw(position, footPose2d.getYaw());
         pointList = heightMap.getAllPointsWithinArea(position.x, position.y, searchWidth, searchLength, footstepMask);
      }

      Plane3d fittedPlane = new Plane3d();

      planeFitter.fitPlaneToPoints(new Point2d(footPose2d.getX(), footPose2d.getY()), pointList, fittedPlane);
      double height = fittedPlane.getZOnPlane(footPose2d.getX(), footPose2d.getY());

      Vector3d surfaceNormal = fittedPlane.getNormalCopy();
      if (MathTools.containsNaN(surfaceNormal))
      {
         surfaceNormal.set(0.0, 0.0, 1.0);
         height = heightMap.getHeightAtPoint(footPose2d.getX(), footPose2d.getY());
         if (Double.isInfinite(height))
            height = 0.0;
      }

      return generateFootstepWithoutHeightMap(footPose2d, foot, soleFrame, robotSide, height, surfaceNormal);
   }

   @Override
   public void adjustFootstepWithoutHeightmap(FootstepDataMessage footstep, double height, Vector3d planeNormal)
   {
      Point3d position = footstep.getLocation();
      Quat4d orientation = footstep.getOrientation();
      RigidBodyTransform solePose = new RigidBodyTransform();
      double yaw = RotationTools.computeYaw(orientation);

      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);
      position.setZ(height);
   }

   @Override
   public void adjustFootstepWithoutHeightmap(Footstep footstep, double height, Vector3d planeNormal)
   {
      Vector3d position = new Vector3d();
      Quat4d orientation = new Quat4d();
      RigidBodyTransform solePose = new RigidBodyTransform();
      footstep.getSolePose(solePose);
      solePose.get(orientation, position);
      double yaw = RotationTools.computeYaw(orientation);

      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);
      position.setZ(height);

      footstep.setSolePose(new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(position), orientation));
   }

   @Override
   public Footstep generateFootstepWithoutHeightMap(FramePose2d footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide, double height,
           Vector3d planeNormal)
   {
      double yaw = footPose2d.getYaw();
      Point3d position = new Point3d(footPose2d.getX(), footPose2d.getY(), height);
      Quat4d orientation = new Quat4d();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);

      Footstep footstep = new Footstep(foot, robotSide, soleFrame);
      footstep.setSolePose(new FramePose(ReferenceFrame.getWorldFrame(), position, orientation));

      return footstep;
   }

   @Override
   public void setUseMask(boolean useMask, double maskSafetyBuffer, double boundingBoxDimension)
   {
      this.useMask = useMask;
      maskBuffer = maskSafetyBuffer;

      if (footstepMask != null)
      {
         footstepMask.setSafetyBuffer(maskBuffer);
      }

      searchWidth = boundingBoxDimension;
      searchLength = boundingBoxDimension;
   }

   public List<Point3d> getPointList()
   {
      return pointList;
   }
}
