package us.ihmc.humanoidRobotics.footstep;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class Footstep
{
   public static enum FootstepType {FULL_FOOTSTEP, PARTIAL_FOOTSTEP, BAD_FOOTSTEP}

   public static final int maxNumberOfSwingWaypoints = 10;

   private static int counter = 0;
   private final String id;
   private final RigidBody endEffector;
   private RobotSide robotSide;
   private FootstepType footstepType = FootstepType.FULL_FOOTSTEP;

   private final FramePose footstepPose = new FramePose();
   private final FramePose expectedInitialPose = new FramePose();

   private final FramePose tempPose = new FramePose();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final PoseReferenceFrame footstepSoleFrame;

   private final List<Point2D> predictedContactPoints = new ArrayList<>();

   private final RecyclingArrayList<FramePoint> customPositionWaypoints = new RecyclingArrayList<>(2, FramePoint.class);
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingTrajectory = new RecyclingArrayList<>(maxNumberOfSwingWaypoints, FrameSE3TrajectoryPoint.class);
   private double swingTrajectoryBlendDuration = 0.0;

   private final boolean trustHeight;
   private boolean scriptedFootstep;

   // foot trajectory generation
   public TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   public double swingHeight = 0;

   public Footstep(RigidBody endEffector, RobotSide robotSide, FramePose footstepPose, boolean trustHeight)
   {
      this(createAutomaticID(endEffector), endEffector, robotSide, footstepPose, trustHeight);
   }

   public Footstep(String id, RigidBody endEffector, RobotSide robotSide, FramePose footstepPose, boolean trustHeight)
   {
      this(id, endEffector, robotSide, footstepPose, trustHeight, null);
   }

   public Footstep(RigidBody endEffector, RobotSide robotSide)
   {
      this(endEffector, robotSide, new FramePose());
   }

   public Footstep(RigidBody endEffector, RobotSide robotSide, FramePose footstepPose)
   {
      this(createAutomaticID(endEffector), endEffector, robotSide, footstepPose, true);
   }

   public Footstep(Footstep footstep)
   {
      this(footstep.endEffector, footstep.robotSide, footstep.footstepPose, footstep.trustHeight);
      this.trajectoryType = footstep.trajectoryType;
      this.swingHeight = footstep.swingHeight;
      this.swingTrajectoryBlendDuration = footstep.swingTrajectoryBlendDuration;
      this.expectedInitialPose.setIncludingFrame(footstep.expectedInitialPose);
   }

   public Footstep(RigidBody endEffector, RobotSide robotSide, FramePose footstepPose, boolean trustHeight, List<Point2D> predictedContactPoints)
   {
      this(createAutomaticID(endEffector), endEffector, robotSide, footstepPose, trustHeight, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public Footstep(String id, RigidBody endEffector, RobotSide robotSide, FramePose footstepPose, boolean trustHeight, List<Point2D> predictedContactPoints)
   {
      this(id, endEffector, robotSide, footstepPose, trustHeight, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public Footstep(String id, RigidBody endEffector, RobotSide robotSide, FramePose footstepPose, boolean trustHeight, List<Point2D> predictedContactPoints,
                   TrajectoryType trajectoryType, double swingHeight)
   {
      this.id = id;
      this.endEffector = endEffector;
      this.robotSide = robotSide;
      this.trustHeight = trustHeight;
      this.footstepPose.setIncludingFrame(footstepPose);
      setPredictedContactPointsFromPoint2ds(predictedContactPoints);
      this.expectedInitialPose.setToNaN();
      this.trajectoryType = trajectoryType;
      this.swingHeight = swingHeight;
      footstepSoleFrame = new PoseReferenceFrame(id + "_FootstepSoleFrame", ReferenceFrame.getWorldFrame());
   }

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public List<FramePoint> getCustomPositionWaypoints()
   {
      return customPositionWaypoints;
   }

   public void setCustomPositionWaypoints(RecyclingArrayList<FramePoint> customPositionWaypoints)
   {
      this.customPositionWaypoints.clear();
      for (int i = 0; i < customPositionWaypoints.size(); i++)
         this.customPositionWaypoints.add().set(customPositionWaypoints.get(i));
   }

   public List<FrameSE3TrajectoryPoint> getSwingTrajectory()
   {
      return swingTrajectory;
   }

   public void setSwingTrajectory(RecyclingArrayList<FrameSE3TrajectoryPoint> swingTrajectory)
   {
      this.swingTrajectory.clear();
      for (int i = 0; i < swingTrajectory.size(); i++)
         this.swingTrajectory.add().set(swingTrajectory.get(i));
   }

   public void setSwingTrajectoryBlendDuration(double swingTrajectoryBlendDuration)
   {
      this.swingTrajectoryBlendDuration = swingTrajectoryBlendDuration;
   }

   public double getSwingTrajectoryBlendDuration()
   {
      return swingTrajectoryBlendDuration;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setPredictedContactPointsFromPoint2ds(List<Point2D> contactPointList)
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
         Point2D point = contactPointList.get(i);
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
            this.predictedContactPoints.add(new Point2D());
         }
      }
   }

   private static String createAutomaticID(RigidBody endEffector)
   {
      return endEffector.getName() + "_" + counter++;
   }

   public List<Point2D> getPredictedContactPoints()
   {
      if (predictedContactPoints.isEmpty())
      {
         return null;
      }
      return predictedContactPoints;
   }

   public void setX(double x)
   {
      footstepPose.setX(x);
   }

   public void setY(double y)
   {
      footstepPose.setY(y);
   }

   public void setZ(double z)
   {
      footstepPose.setZ(z);
   }

   public double getX()
   {
      return footstepPose.getX();
   }

   public double getY()
   {
      return footstepPose.getY();
   }

   public double getZ()
   {
      return footstepPose.getZ();
   }

   public void setPose(RigidBodyTransform transformFromAnkleToWorldFrame)
   {
      footstepPose.setToNaN(ReferenceFrame.getWorldFrame());
      footstepPose.setPose(transformFromAnkleToWorldFrame);
   }

   public void setPose(Footstep newFootstep)
   {
      footstepPose.setIncludingFrame(newFootstep.getFootstepPose());
   }

   public void setPose(FramePose footstepPose)
   {
      this.footstepPose.setIncludingFrame(footstepPose);
   }

   public void setPose(FramePoint position, FrameOrientation orientation)
   {
      footstepPose.setPoseIncludingFrame(position, orientation);
   }

   public void setPositionChangeOnlyXY(FramePoint2d position2d)
   {
      position2d.checkReferenceFrameMatch(footstepPose);
      setX(position2d.getX());
      setY(position2d.getY());
   }

   public void setExpectedInitialPose(FramePose expectedInitialPose)
   {
      this.expectedInitialPose.setIncludingFrame(expectedInitialPose);
   }

   public void setExpectedInitialPose(FramePoint expectedInitialPosition, FrameOrientation expectedInitialOrientation)
   {
      this.expectedInitialPose.setPoseIncludingFrame(expectedInitialPosition, expectedInitialOrientation);
   }

   public String getId()
   {
      return id;
   }

   public boolean getTrustHeight()
   {
      return trustHeight;
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

   public FramePose getFootstepPose()
   {
      return footstepPose;
   }

   public void getPose(FramePose poseToPack)
   {
      poseToPack.setIncludingFrame(footstepPose);
   }

   public void getPose(FramePoint positionToPack, FrameOrientation orientationToPack)
   {
      footstepPose.getPoseIncludingFrame(positionToPack, orientationToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      footstepPose.getPositionIncludingFrame(positionToPack);
   }

   public void getOrientation(FrameOrientation orientationToPack)
   {
      footstepPose.getOrientationIncludingFrame(orientationToPack);
   }

   public void getExpectedInitialPose(FramePose poseToPack)
   {
      poseToPack.setIncludingFrame(expectedInitialPose);
   }

   public ReferenceFrame getTrajectoryFrame()
   {
      return footstepPose.getReferenceFrame();
   }

   public void setFootstepType(FootstepType footstepType)
   {
      this.footstepType = footstepType;
   }

   public FootstepType getFootstepType()
   {
      return footstepType;
   }

   public boolean isScriptedFootstep()
   {
      return scriptedFootstep;
   }

   public void setScriptedFootstep(boolean scriptedFootstep)
   {
      this.scriptedFootstep = scriptedFootstep;
   }

   public boolean epsilonEquals(Footstep otherFootstep, double epsilon)
   {
      boolean arePosesEqual = footstepPose.epsilonEquals(otherFootstep.footstepPose, epsilon);
      boolean bodiesHaveTheSameName = endEffector.getName().equals(otherFootstep.endEffector.getName());
      boolean sameRobotSide = robotSide == otherFootstep.robotSide;
      boolean isTrustHeightTheSame = trustHeight == otherFootstep.trustHeight;

      boolean sameWaypoints = customPositionWaypoints.size() == otherFootstep.customPositionWaypoints.size();
      if (sameWaypoints)
      {
         for (int i = 0; i < customPositionWaypoints.size(); i++)
         {
            FramePoint waypoint = customPositionWaypoints.get(i);
            FramePoint otherWaypoint = otherFootstep.customPositionWaypoints.get(i);
            sameWaypoints = sameWaypoints && waypoint.epsilonEquals(otherWaypoint, epsilon);
         }
      }

      boolean sameExpectedInitialPose = true;
      if (expectedInitialPose.containsNaN() && !otherFootstep.expectedInitialPose.containsNaN())
         sameExpectedInitialPose = false;
      else if (!expectedInitialPose.containsNaN() && otherFootstep.expectedInitialPose.containsNaN())
         sameExpectedInitialPose = false;
      else if (!expectedInitialPose.containsNaN())
         sameExpectedInitialPose = expectedInitialPose.epsilonEquals(otherFootstep.expectedInitialPose, epsilon);
      boolean sameBlendDuration = MathTools.epsilonEquals(swingTrajectoryBlendDuration, otherFootstep.swingTrajectoryBlendDuration, epsilon);

      return arePosesEqual && bodiesHaveTheSameName && sameRobotSide && isTrustHeightTheSame && sameWaypoints && sameExpectedInitialPose && sameBlendDuration;
   }

   @Override
   public String toString()
   {
      return "id: " + id + " - pose: " + footstepPose + " - trustHeight = " + trustHeight;
   }

   public ReferenceFrame getSoleReferenceFrame()
   {
      tempPose.setIncludingFrame(footstepPose);
      tempPose.changeFrame(footstepSoleFrame.getParent());
      footstepSoleFrame.setPoseAndUpdate(tempPose);
      return footstepSoleFrame;
   }

   public void getAnklePose(FramePose poseToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      poseToPack.setPoseIncludingFrame(footstepPose.getReferenceFrame(), tempTransform);
   }

   public void getAnklePosition(FramePoint positionToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      positionToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform.getTranslationVector());
   }

   public void getAnklePosition2d(FramePoint2d position2dToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      double x = tempTransform.getTranslationVector().getX();
      double y = tempTransform.getTranslationVector().getY();
      position2dToPack.setIncludingFrame(footstepPose.getReferenceFrame(), x, y);
   }

   public void getAnkleOrientation(FrameOrientation orientationToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      orientationToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform.getRotationMatrix());
   }

   public void setFromAnklePose(FramePose anklePose, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(anklePose.getOrientation());
      tempTransform.setTranslation(anklePose.getPosition());
      tempTransform.multiplyInvertOther(transformFromAnkleToSole);
      footstepPose.setPoseIncludingFrame(anklePose.getReferenceFrame(), tempTransform);
   }

}
