package us.ihmc.humanoidRobotics.footstep;

import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class Footstep implements Settable<Footstep>
{
   public static final int maxNumberOfSwingWaypoints = 10;

   // --- TODO: GW nuke this:
   public static enum FootstepType
   {
      FULL_FOOTSTEP, PARTIAL_FOOTSTEP, BAD_FOOTSTEP
   }

   private FootstepType footstepType = FootstepType.FULL_FOOTSTEP;
   private static int counter = 0;
   private final PoseReferenceFrame footstepSoleFrame = new PoseReferenceFrame(counter++ + "_FootstepSoleFrame", ReferenceFrame.getWorldFrame());
   private boolean scriptedFootstep = false;
   // ---

   private RobotSide robotSide;
   private final FramePose3D footstepPose = new FramePose3D();

   private final RecyclingArrayList<Point2D> predictedContactPoints = new RecyclingArrayList<>(6, Point2D.class);
   private final RecyclingArrayList<MutableDouble> customWaypointProportions = new RecyclingArrayList<>(2, MutableDouble.class);
   private final RecyclingArrayList<FramePoint3D> customPositionWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingTrajectory = new RecyclingArrayList<>(maxNumberOfSwingWaypoints,
                                                                                                        FrameSE3TrajectoryPoint.class);

   private TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   private double swingHeight = 0.0;
   private double swingTrajectoryBlendDuration = 0.0;
   private boolean trustHeight = true;

   private double transferSplitFraction = Double.NaN;
   private boolean isAdjustable = false;

   private final FramePose3D tempPose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public Footstep()
   {
      predictedContactPoints.clear();
      customPositionWaypoints.clear();
      swingTrajectory.clear();
      customWaypointProportions.clear();
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight)
   {
      this(robotSide, footstepPose, trustHeight, false, null);
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight, boolean isAdjustable)
   {
      this(robotSide, footstepPose, trustHeight, isAdjustable, null);
   }

   public Footstep(RobotSide robotSide)
   {
      this(robotSide, new FramePose3D());
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose)
   {
      this(robotSide, footstepPose, true);
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight, List<Point2D> predictedContactPoints)
   {
      this(robotSide, footstepPose, trustHeight, false, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight, boolean isAdjustable, List<Point2D> predictedContactPoints)
   {
      this(robotSide, footstepPose, trustHeight, isAdjustable, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight, boolean isAdjustable, TrajectoryType trajectoryType, double swingHeight)
   {
      this(robotSide, footstepPose, trustHeight, isAdjustable, null, trajectoryType, swingHeight);
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight, boolean isAdjustable, List<Point2D> predictedContactPoints,
                   TrajectoryType trajectoryType, double swingHeight)
   {
      this.robotSide = robotSide;
      this.trustHeight = trustHeight;
      this.isAdjustable = isAdjustable;
      this.footstepPose.setIncludingFrame(footstepPose);
      setPredictedContactPoints(predictedContactPoints);
      this.trajectoryType = trajectoryType;
      this.swingHeight = swingHeight;
   }

   @Override
   public void set(Footstep other)
   {
      this.robotSide = other.robotSide;
      this.footstepType = other.footstepType;
      this.swingTrajectoryBlendDuration = other.swingTrajectoryBlendDuration;
      this.transferSplitFraction = other.getTransferSplitFraction();
      this.trustHeight = other.trustHeight;
      this.isAdjustable = other.isAdjustable;
      this.scriptedFootstep = other.scriptedFootstep;
      this.trajectoryType = other.trajectoryType;
      this.swingHeight = other.swingHeight;

      this.footstepPose.setIncludingFrame(other.footstepPose);

      this.predictedContactPoints.clear();
      for (int i = 0; i < other.predictedContactPoints.size(); i++)
      {
         this.predictedContactPoints.add().set(other.predictedContactPoints.get(i));
      }

      this.customWaypointProportions.clear();
      for (int i = 0; i < other.customWaypointProportions.size(); i++)
      {
         this.customWaypointProportions.add().setValue(other.customWaypointProportions.get(i));
      }

      this.customPositionWaypoints.clear();
      for (int i = 0; i < other.customPositionWaypoints.size(); i++)
      {
         this.customPositionWaypoints.add().set(other.customPositionWaypoints.get(i));
      }

      this.swingTrajectory.clear();
      for (int i = 0; i < other.swingTrajectory.size(); i++)
      {
         this.swingTrajectory.add().set(other.swingTrajectory.get(i));
      }
   }

   /**
    * Sets all properties of the footstep to the values provided.
    */
   public void set(FootstepDataCommand command, boolean trustHeight, boolean isAdjustable)
   {
      this.robotSide = command.getRobotSide();
      this.swingTrajectoryBlendDuration = command.getSwingTrajectoryBlendDuration();
      this.transferSplitFraction = command.getTransferSplitFraction();
      this.trajectoryType = command.getTrajectoryType();
      this.swingHeight = command.getSwingHeight();
      this.isAdjustable = isAdjustable;
      this.trustHeight = trustHeight;

      this.footstepPose.setIncludingFrame(command.getPosition(), command.getOrientation());

      this.predictedContactPoints.clear();
      RecyclingArrayList<Point2D> commandPredictedContactPoints = command.getPredictedContactPoints();
      if (commandPredictedContactPoints != null)
      {
         for (int i = 0; i < commandPredictedContactPoints.size(); i++)
         {
            this.predictedContactPoints.add().set(commandPredictedContactPoints.get(i));
         }
      }

      this.customWaypointProportions.clear();
      RecyclingArrayList<MutableDouble> commandWaypointProportions = command.getCustomWaypointProportions();
      if(commandWaypointProportions != null && commandWaypointProportions.size() == 2)
      {
         for (int i = 0; i < commandWaypointProportions.size(); i++)
         {
            this.customWaypointProportions.add().setValue(commandWaypointProportions.get(i));
         }
      }

      this.customPositionWaypoints.clear();
      RecyclingArrayList<FramePoint3D> commandCustomPositionWaypoints = command.getCustomPositionWaypoints();
      if (commandCustomPositionWaypoints != null)
      {
         for (int i = 0; i < commandCustomPositionWaypoints.size(); i++)
         {
            this.customPositionWaypoints.add().set(commandCustomPositionWaypoints.get(i));
         }
      }

      this.swingTrajectory.clear();
      RecyclingArrayList<FrameSE3TrajectoryPoint> commandSwingTrajectory = command.getSwingTrajectory();
      if (commandSwingTrajectory != null)
      {
         for (int i = 0; i < commandSwingTrajectory.size(); i++)
         {
            this.swingTrajectory.add().set(commandSwingTrajectory.get(i));
         }
      }
   }

   /**
    * Resets this footstep to the default values and clears all data contained in it.
    */
   public void clear()
   {
      robotSide = null;
      footstepType = FootstepType.FULL_FOOTSTEP;
      footstepPose.setToZero(ReferenceFrame.getWorldFrame());
      predictedContactPoints.clear();
      customWaypointProportions.clear();
      customPositionWaypoints.clear();
      swingTrajectory.clear();
      swingTrajectoryBlendDuration = 0.0;
      transferSplitFraction = Double.NaN;
      trustHeight = true;
      isAdjustable = false;
      scriptedFootstep = false;
      trajectoryType = TrajectoryType.DEFAULT;
      swingHeight = 0.0;
   }

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public RecyclingArrayList<MutableDouble> getCustomWaypointProportions()
   {
      return customWaypointProportions;
   }

   public List<FramePoint3D> getCustomPositionWaypoints()
   {
      return customPositionWaypoints;
   }

   public void setCustomPositionWaypoints(RecyclingArrayList<FramePoint3D> customPositionWaypoints)
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

   public void setTransferSplitFraction(double transferSplitFraction)
   {
      this.transferSplitFraction = transferSplitFraction;
   }

   public double getTransferSplitFraction()
   {
      return transferSplitFraction;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setTrustHeight(boolean trustHeight)
   {
      this.trustHeight = trustHeight;
   }

   public void setIsAdjustable(boolean isAdjustable)
   {
      this.isAdjustable = isAdjustable;
   }

   public void setPredictedContactPoints(Point2DReadOnly[] contactPointArray)
   {
      predictedContactPoints.clear();

      if (contactPointArray == null)
      {
         return;
      }

      for (int i = 0; i < contactPointArray.length; i++)
      {
         Point2DReadOnly point = contactPointArray[i];
         this.predictedContactPoints.add().set(point);
      }
   }

   public void setPredictedContactPoints(List<? extends Point2DReadOnly> contactPointList)
   {
      predictedContactPoints.clear();

      if (contactPointList == null)
      {
         return;
      }

      for (int i = 0; i < contactPointList.size(); i++)
      {
         Point2DReadOnly point = contactPointList.get(i);
         this.predictedContactPoints.add().set(point);
      }
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

   public void setPose(RigidBodyTransform transformFromSoleToWorldFrame)
   {
      footstepPose.setToNaN(ReferenceFrame.getWorldFrame());
      footstepPose.set(transformFromSoleToWorldFrame);
   }

   public void setPose(FramePose3DReadOnly footstepPose)
   {
      this.footstepPose.setIncludingFrame(footstepPose);
   }

   public void setPose(FrameTuple3DReadOnly position, FrameQuaternionReadOnly orientation)
   {
      footstepPose.setIncludingFrame(position, orientation);
   }

   public void setPositionChangeOnlyXY(FramePoint2DReadOnly position2d)
   {
      position2d.checkReferenceFrameMatch(footstepPose);
      setX(position2d.getX());
      setY(position2d.getY());
   }

   public boolean getTrustHeight()
   {
      return trustHeight;
   }

   public boolean getIsAdjustable()
   {
      return isAdjustable;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public FramePose3D getFootstepPose()
   {
      return footstepPose;
   }

   public void getPose(FramePose3D poseToPack)
   {
      poseToPack.setIncludingFrame(footstepPose);
   }

   public void getPose(FramePoint3D positionToPack, FrameQuaternion orientationToPack)
   {
      footstepPose.get(positionToPack, orientationToPack);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(footstepPose.getPosition());
   }

   public void getPosition2d(FramePoint2D positionToPack)
   {
      positionToPack.setIncludingFrame(footstepPose.getPosition());
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(footstepPose.getOrientation());
   }

   public void setFootstepType(FootstepType footstepType)
   {
      this.footstepType = footstepType;
   }

   public FootstepType getFootstepType()
   {
      if (predictedContactPoints.isEmpty())
      {
         return FootstepType.FULL_FOOTSTEP;
      }
      else
      {
         return FootstepType.PARTIAL_FOOTSTEP;
      }
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
      boolean sameRobotSide = robotSide == otherFootstep.robotSide;
      boolean isTrustHeightTheSame = trustHeight == otherFootstep.trustHeight;
      boolean isAdjustableTheSame = isAdjustable == otherFootstep.isAdjustable;

      boolean sameWaypoints = customPositionWaypoints.size() == otherFootstep.customPositionWaypoints.size();
      if (sameWaypoints)
      {
         for (int i = 0; i < customPositionWaypoints.size(); i++)
         {
            FramePoint3D waypoint = customPositionWaypoints.get(i);
            FramePoint3D otherWaypoint = otherFootstep.customPositionWaypoints.get(i);
            sameWaypoints = sameWaypoints && waypoint.epsilonEquals(otherWaypoint, epsilon);
         }
      }

      boolean sameBlendDuration = MathTools.epsilonEquals(swingTrajectoryBlendDuration, otherFootstep.swingTrajectoryBlendDuration, epsilon);

      return arePosesEqual && sameRobotSide && isTrustHeightTheSame && isAdjustableTheSame && sameWaypoints && sameBlendDuration;
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("Footstep:\n");
      builder.append(" Side: " + robotSide + "\n");
      builder.append(" Position: " + footstepPose.getPosition() + "\n");
      builder.append(" Orientation: " + footstepPose.getOrientation() + "\n");
      builder.append(" Trust Height: " + trustHeight + "\n");
      builder.append(" Adjustable: " + isAdjustable + "\n");
      return builder.toString();
   }

   // Going to be removed soon. Is duplicate information.
   @Deprecated
   public ReferenceFrame getSoleReferenceFrame()
   {
      tempPose.setIncludingFrame(footstepPose);
      tempPose.changeFrame(footstepSoleFrame.getParent());
      footstepSoleFrame.setPoseAndUpdate(tempPose);
      return footstepSoleFrame;
   }

   public void getAnklePose(FramePose3D poseToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      poseToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform);
   }

   public void getAnklePosition(FramePoint3D positionToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      positionToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform.getTranslationVector());
   }

   public void getAnklePosition2d(FramePoint2D position2dToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      double x = tempTransform.getTranslationVector().getX();
      double y = tempTransform.getTranslationVector().getY();
      position2dToPack.setIncludingFrame(footstepPose.getReferenceFrame(), x, y);
   }

   public void getAnkleOrientation(FrameQuaternion orientationToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(footstepPose.getOrientation());
      tempTransform.setTranslation(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      orientationToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform.getRotationMatrix());
   }

   public void setFromAnklePose(FramePose3DReadOnly anklePose, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.setRotation(anklePose.getOrientation());
      tempTransform.setTranslation(anklePose.getPosition());
      tempTransform.multiplyInvertOther(transformFromAnkleToSole);
      footstepPose.setIncludingFrame(anklePose.getReferenceFrame(), tempTransform);
   }

   public void addOffset(FrameVector3DReadOnly offset)
   {
      footstepPose.prependTranslation(offset);

      for (int pointIdx = 0; pointIdx < customPositionWaypoints.size(); pointIdx++)
      {
         customPositionWaypoints.get(pointIdx).add(offset);
      }

      for (int pointIdx = 0; pointIdx < swingTrajectory.size(); pointIdx++)
      {
         FrameSE3TrajectoryPoint trajectoryPoint = swingTrajectory.get(pointIdx);
         trajectoryPoint.getPoseIncludingFrame(tempPose);
         tempPose.prependTranslation(offset);
         trajectoryPoint.setPosition(tempPose.getPosition());
      }
   }

   public static Footstep[] createFootsteps(int numberOfSteps)
   {
      Footstep[] footsteps = new Footstep[numberOfSteps];
      for (int i = 0; i < numberOfSteps; i++)
      {
         footsteps[i] = new Footstep();
      }
      return footsteps;
   }

}
