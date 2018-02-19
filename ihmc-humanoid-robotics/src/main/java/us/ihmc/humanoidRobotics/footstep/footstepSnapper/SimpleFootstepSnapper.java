package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.InsufficientDataException;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.geometry.PlaneFitter;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.TrajectoryType;

/**
 * Created by agrabertilton on 1/14/15.
 */
public class SimpleFootstepSnapper implements QuadTreeFootstepSnapper
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
      FramePose2D footPose2d = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(soleX, soleY), yaw);

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
   public Footstep.FootstepType snapFootstep(Footstep footstep, HeightMapWithPoints heightMap)
   {
      // can only snap footsteps in world frame
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      FootstepDataMessage originalFootstep = HumanoidMessageTools.createFootstepDataMessage(footstep);

      //set to the sole pose
      FramePoint3D position = new FramePoint3D();
      FrameQuaternion orientation = new FrameQuaternion();
      footstep.getPose(position, orientation);
      originalFootstep.setLocation(position);
      originalFootstep.setOrientation(orientation);

      //get the footstep
      Footstep.FootstepType type = snapFootstep(originalFootstep, heightMap);
      footstep.setPredictedContactPoints(originalFootstep.getPredictedContactPoints().toArray());
      footstep.setFootstepType(type);
      FramePose3D solePoseInWorld = new FramePose3D(ReferenceFrame.getWorldFrame(), originalFootstep.getLocation(), originalFootstep.getOrientation());
      footstep.setPose(solePoseInWorld);

      footstep.setSwingHeight(originalFootstep.getSwingHeight());
      footstep.setTrajectoryType(TrajectoryType.fromByte(originalFootstep.getTrajectoryType()));

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

      Plane3D fittedPlane = new Plane3D();

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
   public Footstep generateFootstepUsingHeightMap(FramePose2D footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide,
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

      Plane3D fittedPlane = new Plane3D();

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
      // can only snap footsteps in world frame
      footstep.getFootstepPose().checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      FramePose3D footstepPose = new FramePose3D();
      footstep.getPose(footstepPose);

      Point3D position = new Point3D(footstepPose.getPosition());
      double yaw = footstep.getFootstepPose().getYaw();

      Quaternion orientation = new Quaternion();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);
      position.setZ(height);

      footstepPose.set(new Point3D(position), orientation);
      footstep.setPose(footstepPose);
   }

   @Override
   public Footstep generateFootstepWithoutHeightMap(FramePose2D footPose2d, RigidBody foot, ReferenceFrame soleFrame, RobotSide robotSide, double height,
           Vector3D planeNormal)
   {
      double yaw = footPose2d.getYaw();
      Point3D position = new Point3D(footPose2d.getX(), footPose2d.getY(), height);
      Quaternion orientation = new Quaternion();
      RotationTools.computeQuaternionFromYawAndZNormal(yaw, planeNormal, orientation);

      FramePose3D solePose = new FramePose3D(ReferenceFrame.getWorldFrame(), position, orientation);
      Footstep footstep = new Footstep(robotSide, solePose);

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

   @Override
   public List<Point3D> getPointList()
   {
      return pointList;
   }
}
