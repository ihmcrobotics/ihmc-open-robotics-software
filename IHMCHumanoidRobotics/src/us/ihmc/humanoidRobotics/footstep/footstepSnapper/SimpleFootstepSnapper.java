package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.InsufficientDataException;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.geometry.PlaneFitter;
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
   private List<Point3D> pointList = new ArrayList<Point3D>();
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
      FramePose2d footPose2d = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(soleX, soleY), yaw);

      Footstep footstep = generateFootstepUsingHeightMap(footPose2d, foot, soleFrame, robotSide, heightMap);
      double z = footstep.getZ();
      if (Double.isInfinite(z) || Double.isNaN(z))
      {
         System.out.println("Houston, we have a problem in the SimpleFootstepSnapper");
      }

      return footstep;
   }

   @Override
   public void setMask(List<Point2D> footShape)
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
      Vector3D position = new Vector3D();
      Quaternion orientation = new Quaternion();
      RigidBodyTransform solePose = new RigidBodyTransform();
      footstep.getSolePose(solePose);
      solePose.get(orientation, position);
      originalFootstep.setLocation(new Point3D(position));
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
      Quaternion orientation = footstep.getOrientation();
      Point3D position = footstep.getLocation();
      double yaw = orientation.getYaw();

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

         footstepMask.setPositionAndYaw(position.getX(), position.getY(), yaw);
         pointList = heightMap.getAllPointsWithinArea(position.getX(), position.getY(), searchWidth, searchLength, footstepMask);
      }

      Plane3d fittedPlane = new Plane3d();

      planeFitter.fitPlaneToPoints(new Point2D(position.getX(), position.getY()), pointList, fittedPlane);
      double height = fittedPlane.getZOnPlane(position.getX(), position.getY());

      Vector3D surfaceNormal = fittedPlane.getNormalCopy();
      if (surfaceNormal.containsNaN())
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
      Point2D position = new Point2D(footPose2d.getX(), footPose2d.getY());
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
         pointList = heightMap.getAllPointsWithinArea(position.getX(), position.getY(), searchWidth, searchLength, footstepMask);
      }

      Plane3d fittedPlane = new Plane3d();

      planeFitter.fitPlaneToPoints(new Point2D(footPose2d.getX(), footPose2d.getY()), pointList, fittedPlane);
      double height = fittedPlane.getZOnPlane(footPose2d.getX(), footPose2d.getY());

      Vector3D surfaceNormal = fittedPlane.getNormalCopy();
      if (surfaceNormal.containsNaN())
      {
         surfaceNormal.set(0.0, 0.0, 1.0);
         height = heightMap.getHeightAtPoint(footPose2d.getX(), footPose2d.getY());
         if (Double.isInfinite(height))
            height = 0.0;
      }

      return generateFootstepWithoutHeightMap(footPose2d, foot, soleFrame, robotSide, height, surfaceNormal);
   }

   @Override
   public void adjustFootstepWithoutHeightmap(FootstepDataMessage footstep, double height, Vector3D planeNormal)
   {
      Point3D position = footstep.getLocation();
      Quaternion orientation = footstep.getOrientation();
      RigidBodyTransform solePose = new RigidBodyTransform();
      double yaw = orientation.getYaw();

      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);
      position.setZ(height);
   }

   @Override
   public void adjustFootstepWithoutHeightmap(Footstep footstep, double height, Vector3D planeNormal)
   {
      Vector3D position = new Vector3D();
      Quaternion orientation = new Quaternion();
      RigidBodyTransform solePose = new RigidBodyTransform();
      footstep.getSolePose(solePose);
      solePose.get(orientation, position);
      double yaw = orientation.getYaw();

      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);
      position.setZ(height);

      footstep.setSolePose(new FramePose(ReferenceFrame.getWorldFrame(), new Point3D(position), orientation));
   }

   @Override
   public Footstep generateFootstepWithoutHeightMap(FramePose2d footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide, double height,
           Vector3D planeNormal)
   {
      double yaw = footPose2d.getYaw();
      Point3D position = new Point3D(footPose2d.getX(), footPose2d.getY(), height);
      Quaternion orientation = new Quaternion();
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

   public List<Point3D> getPointList()
   {
      return pointList;
   }
}
