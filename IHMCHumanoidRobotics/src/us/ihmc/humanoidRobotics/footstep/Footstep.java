package us.ihmc.humanoidRobotics.footstep;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.robotics.ArrayTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;

public class Footstep
{
   public static enum FootstepType {FULL_FOOTSTEP, PARTIAL_FOOTSTEP, BAD_FOOTSTEP}

   ;

   private static int counter = 0;
   private final String id;
   private final RigidBody endEffector;
   private RobotSide robotSide;
   private FootstepType footstepType = FootstepType.FULL_FOOTSTEP;
   private final PoseReferenceFrame ankleReferenceFrame;
   private final ReferenceFrame soleReferenceFrame;

   private List<Point2d> predictedContactPoints = new ArrayList<>();

   private final boolean trustHeight;
   private boolean scriptedFootstep;

   private final RigidBodyTransform transformFromAnkleToSoleFrame = new RigidBodyTransform();

   // fot trajectory generation
   public TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   public double swingHeight = 0;

   public Footstep(RigidBody endEffector, RobotSide robotSide, ReferenceFrame soleFrame, PoseReferenceFrame poseReferenceFrame, boolean trustHeight)
   {
      this(createAutomaticID(endEffector), endEffector, robotSide, soleFrame, poseReferenceFrame, trustHeight);
   }

   public Footstep(String id, RigidBody endEffector, RobotSide robotSide, ReferenceFrame soleFrame, PoseReferenceFrame poseReferenceFrame, boolean trustHeight)
   {
      this(id, endEffector, robotSide, soleFrame, poseReferenceFrame, trustHeight, null);
   }

   public Footstep(RigidBody endEffector, RobotSide robotSide, ReferenceFrame soleFrame)
   {
      this(createAutomaticID(endEffector), endEffector, robotSide, soleFrame, new PoseReferenceFrame(endEffector.getName(), ReferenceFrame.getWorldFrame()),
           true);
   }

   public Footstep(RigidBody endEffector, RobotSide robotSide, ReferenceFrame soleFrame, PoseReferenceFrame poseReferenceFrame)
   {
      this(createAutomaticID(endEffector), endEffector, robotSide, soleFrame, poseReferenceFrame, true);
   }

   public Footstep(Footstep footstep)
   {
      this(footstep.endEffector, footstep.robotSide, footstep.soleReferenceFrame, footstep.ankleReferenceFrame, footstep.trustHeight);
      this.trajectoryType = footstep.trajectoryType;
      this.swingHeight = footstep.swingHeight;
   }

   public Footstep(RigidBody endEffector, RobotSide robotSide, ReferenceFrame soleFrame, PoseReferenceFrame poseReferenceFrame, boolean trustHeight,
                   List<Point2d> predictedContactPoints)
   {
      this(createAutomaticID(endEffector), endEffector, robotSide, soleFrame, poseReferenceFrame, trustHeight, predictedContactPoints, TrajectoryType.DEFAULT,
           0.0);
   }

   public Footstep(String id, RigidBody endEffector, RobotSide robotSide, ReferenceFrame soleFrame, PoseReferenceFrame poseReferenceFrame, boolean trustHeight,
                   List<Point2d> predictedContactPoints)
   {
      this(id, endEffector, robotSide, soleFrame, poseReferenceFrame, trustHeight, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public Footstep(String id, RigidBody endEffector, RobotSide robotSide, ReferenceFrame soleFrame, PoseReferenceFrame poseReferenceFrame, boolean trustHeight,
                   List<Point2d> predictedContactPoints, TrajectoryType trajectoryType, double swingHeight)
   {
      poseReferenceFrame.getParent().checkIsWorldFrame();

      this.id = id;
      this.endEffector = endEffector;
      this.robotSide = robotSide;
      this.ankleReferenceFrame = poseReferenceFrame;
      endEffector.getParentJoint().getFrameAfterJoint().getTransformToDesiredFrame(transformFromAnkleToSoleFrame, soleFrame);

      this.soleReferenceFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("soleFrame", poseReferenceFrame, transformFromAnkleToSoleFrame);

      this.trustHeight = trustHeight;
      setPredictedContactPointsFromPoint2ds(predictedContactPoints);
      this.trajectoryType = trajectoryType;
      this.swingHeight = swingHeight;
   }

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setPredictedContactPointsFromPoint2ds(List<Point2d> contactPointList)
   {
      efficientlyResizeContactPointList(contactPointList);

      if ((contactPointList == null) || contactPointList.isEmpty())
      {
         if ((footstepType != FootstepType.FULL_FOOTSTEP) && (footstepType != FootstepType.BAD_FOOTSTEP))
         {
            footstepType = FootstepType.FULL_FOOTSTEP;
         }

         predictedContactPoints.clear();

         return;
      }

      footstepType = FootstepType.PARTIAL_FOOTSTEP;

      for (int i = 0; i < contactPointList.size(); i++)
      {
         Point2d point = contactPointList.get(i);
         this.predictedContactPoints.get(i).set(point.getX(), point.getY());
      }
   }

   public void setPredictedContactPointsFromFramePoint2ds(List<FramePoint2d> contactPointList)
   {
      efficientlyResizeContactPointList(contactPointList);

      if ((contactPointList == null) || contactPointList.isEmpty())
      {
         if ((footstepType != FootstepType.FULL_FOOTSTEP) && (footstepType != FootstepType.BAD_FOOTSTEP))
         {
            footstepType = FootstepType.FULL_FOOTSTEP;
         }

         predictedContactPoints.clear();

         return;
      }

      footstepType = FootstepType.PARTIAL_FOOTSTEP;

      for (int i = 0; i < contactPointList.size(); i++)
      {
         FramePoint2d point = contactPointList.get(i);
         this.predictedContactPoints.get(i).set(point.getX(), point.getY());
      }

   }

   private void efficientlyResizeContactPointList(List<?> contactPointList)
   {
      if ((contactPointList == null) || contactPointList.isEmpty())
      {
         this.predictedContactPoints.clear();

         return;
      }

      footstepType = FootstepType.PARTIAL_FOOTSTEP;

      int newSize = contactPointList.size();
      int currentSize = this.predictedContactPoints.size();

      if (currentSize > newSize)
      {
         for (int i = 0; i < currentSize - newSize; i++)
         {
            this.predictedContactPoints.remove(0);
         }

         currentSize = this.predictedContactPoints.size();
      }

      if (currentSize < newSize)
      {
         for (int i = 0; i < newSize - currentSize; i++)
         {
            this.predictedContactPoints.add(new Point2d());
         }
      }
   }

   private static String createAutomaticID(RigidBody endEffector)
   {
      return endEffector.getName() + "_" + counter++;
   }

   public ReferenceFrame getPoseReferenceFrame()
   {
      return ankleReferenceFrame;
   }

   public ReferenceFrame getSoleReferenceFrame()
   {
      return soleReferenceFrame;
   }

   public void getSolePose(FramePose soleFrameToPack)
   {
      soleFrameToPack.setToZero(soleReferenceFrame);
      soleFrameToPack.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public void getSolePose(RigidBodyTransform transformToPack)
   {
      transformToPack.set(soleReferenceFrame.getTransformToWorldFrame());
   }

   public List<Point2d> getPredictedContactPoints()
   {
      if (predictedContactPoints.isEmpty())
      {
         return null;
      }
      return predictedContactPoints;
   }

   public ReferenceFrame getParentFrame()
   {
      return ankleReferenceFrame.getParent();
   }

   public void setX(double x)
   {
      ankleReferenceFrame.setX(x);
      ankleReferenceFrame.update();
   }

   public void setY(double y)
   {
      ankleReferenceFrame.setY(y);
      ankleReferenceFrame.update();
   }

   public void setZ(double z)
   {
      ankleReferenceFrame.setZ(z);
      ankleReferenceFrame.update();
   }

   public void setPose(RigidBodyTransform transformFromAnkleToWorldFrame)
   {
      this.ankleReferenceFrame.setPoseAndUpdate(transformFromAnkleToWorldFrame);
   }

   public void setPose(Footstep newFootstep)
   {
      ankleReferenceFrame.setPoseAndUpdate(newFootstep.ankleReferenceFrame);
   }

   public void setPose(FramePose newFootstepPose)
   {
      ankleReferenceFrame.setPoseAndUpdate(newFootstepPose);
   }

   public void setPose(FramePoint newPosition, FrameOrientation newOrientation)
   {
      ankleReferenceFrame.setPoseAndUpdate(newPosition, newOrientation);
   }

   public void setSolePose(RigidBodyTransform transform)
   {
      RigidBodyTransform transformFromAnkleToWorld = new RigidBodyTransform();
      transformFromAnkleToWorld.multiply(transform, transformFromAnkleToSoleFrame);

      this.ankleReferenceFrame.setPoseAndUpdate(transformFromAnkleToWorld);
   }

   public void setSolePose(FramePose newSolePoseInWorldFrame)
   {
      newSolePoseInWorldFrame.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      RigidBodyTransform transformFromSoleToWorld = new RigidBodyTransform();
      newSolePoseInWorldFrame.getPose(transformFromSoleToWorld);

      RigidBodyTransform transformFromAnkleToWorld = new RigidBodyTransform();
      transformFromAnkleToWorld.multiply(transformFromSoleToWorld, transformFromAnkleToSoleFrame);

      this.ankleReferenceFrame.setPoseAndUpdate(transformFromAnkleToWorld);
   }

   public void setPositionChangeOnlyXY(FramePoint2d position2d)
   {
      ankleReferenceFrame.getParent().checkReferenceFrameMatch(position2d);
      setX(position2d.getX());
      setY(position2d.getY());
   }

   public String getId()
   {
      return id;
   }

   public boolean getTrustHeight()
   {
      return trustHeight;
   }

   public double getX()
   {
      return ankleReferenceFrame.getX();
   }

   public double getY()
   {
      return ankleReferenceFrame.getY();
   }

   public double getZ()
   {
      return ankleReferenceFrame.getZ();
   }

   public double getYaw()
   {
      return ankleReferenceFrame.getYaw();
   }

   public double getPitch()
   {
      return ankleReferenceFrame.getPitch();
   }

   public double getRoll()
   {
      return ankleReferenceFrame.getRoll();
   }

   public void getPose(Point3d pointToPack, Quat4d quaternionToPack)
   {
      ankleReferenceFrame.getPose(pointToPack, quaternionToPack);
   }

   public void getPose(RigidBodyTransform transformToPack)
   {
      ankleReferenceFrame.getPose(transformToPack);
   }

   public void getPose(FramePoint positionToPack, FrameOrientation orientationToPack)
   {
      ankleReferenceFrame.getPoseIncludingFrame(positionToPack, orientationToPack);
   }

   public void getPose(FramePose poseToPack)
   {
      ankleReferenceFrame.getPoseIncludingFrame(poseToPack);
   }

   public void getPoseReferenceFrameAndUpdate(PoseReferenceFrame poseReferenceFrameToPackAndUpdate)
   {
      poseReferenceFrameToPackAndUpdate.setPoseAndUpdate(ankleReferenceFrame);
   }

   public RigidBody getBody()
   {
      return endEffector;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void getPosition(Point3d pointToPack)
   {
      ankleReferenceFrame.getPosition(pointToPack);
   }

   public void getPositionInWorldFrame(Point3d location)
   {
      FramePoint anklePosition = new FramePoint(ReferenceFrame.getWorldFrame(), location);
      ankleReferenceFrame.getPositionIncludingFrame(anklePosition);
      anklePosition.changeFrame(ReferenceFrame.getWorldFrame());
      location.set(anklePosition.getPoint());
   }

   public void getOrientationInWorldFrame(Quat4d orientation)
   {
      FrameOrientation ankleOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation);
      ankleReferenceFrame.getOrientationIncludingFrame(ankleOrientation);
      ankleOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      ankleOrientation.getQuaternion(orientation);
   }

   public void getPositionIncludingFrame(FramePoint framePointToPack)
   {
      ankleReferenceFrame.getPositionIncludingFrame(framePointToPack);
   }

   public void getOrientation(Quat4d quaternionToPack)
   {
      ankleReferenceFrame.getOrientation(quaternionToPack);
   }

   public void getOrientation(Matrix3d matrixToPack)
   {
      ankleReferenceFrame.getOrientation(matrixToPack);
   }

   public void getOrientationIncludingFrame(FrameOrientation frameOrientationToPack)
   {
      ankleReferenceFrame.getOrientationIncludingFrame(frameOrientationToPack);
   }

   public void getPose2d(FramePose2d framePose2dToPack)
   {
      ankleReferenceFrame.getPose2dIncludingFrame(framePose2dToPack);
   }

   public void getPosition2d(FramePoint2d framePoint2dToPack)
   {
      ankleReferenceFrame.getPosition2dIncludingFrame(framePoint2dToPack);
   }

   public boolean epsilonEquals(Footstep otherFootstep, double epsilon)
   {
      boolean arePosesEqual = ankleReferenceFrame.epsilonEquals(otherFootstep.ankleReferenceFrame, epsilon);
      boolean bodiesHaveTheSameName = endEffector.getName().equals(otherFootstep.endEffector.getName());
      boolean sameRobotSide = robotSide == otherFootstep.robotSide;
      boolean isTrustHeightTheSame = trustHeight == otherFootstep.trustHeight;

      return arePosesEqual && bodiesHaveTheSameName && sameRobotSide && isTrustHeightTheSame;
   }

   public void setFootstepType(FootstepType footstepType)
   {
      this.footstepType = footstepType;
   }

   public FootstepType getFootstepType()
   {
      return footstepType;
   }

   public String toString()
   {
      FrameOrientation frameOrientation = new FrameOrientation(ankleReferenceFrame);
      frameOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      double[] ypr = frameOrientation.getYawPitchRoll();
      String yawPitchRoll = "YawPitchRoll = " + ArrayTools.arrayToString(ypr);

      return "id: " + id + " - pose: " + ankleReferenceFrame + " - trustHeight = " + trustHeight + "\n\tYawPitchRoll= {" + yawPitchRoll + "}";
   }

   public boolean isScriptedFootstep()
   {
      return scriptedFootstep;
   }

   public void setScriptedFootstep(boolean scriptedFootstep)
   {
      this.scriptedFootstep = scriptedFootstep;
   }

}
