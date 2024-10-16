package us.ihmc.humanoidRobotics.footstep;

import java.util.List;
import java.util.function.IntToDoubleFunction;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.commons.referenceFrames.PoseReferenceFrame;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class Footstep implements Settable<Footstep>
{
   public static final int maxNumberOfSwingWaypoints = 12;

   private final PoseReferenceFrame footstepSoleFrame = new PoseReferenceFrame("FootstepSoleFrame", ReferenceFrame.getWorldFrame());
   private final FramePose3D footstepPose = new FramePose3D();
   private long sequenceID = -1;
   private RobotSide robotSide;

   private final RecyclingArrayList<Point2D> predictedContactPoints = new RecyclingArrayList<>(6, Point2D.class);
   private final RecyclingArrayList<MutableDouble> customWaypointProportions = new RecyclingArrayList<>(2, MutableDouble.class);
   private final RecyclingArrayList<FramePoint3D> customPositionWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingTrajectory = new RecyclingArrayList<>(maxNumberOfSwingWaypoints,
                                                                                                        FrameSE3TrajectoryPoint.class);

   private TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   private double swingHeight = 0.0;
   private double swingTrajectoryBlendDuration = 0.0;
   private boolean trustHeight = true;

   private boolean isAdjustable = false;
   private boolean shouldCheckForReachability = false;

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
      this(robotSide, footstepPose, trustHeight, false, false, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight, boolean isAdjustable, List<Point2D> predictedContactPoints)
   {
      this(robotSide, footstepPose, trustHeight, isAdjustable, false, predictedContactPoints, TrajectoryType.DEFAULT, 0.0);
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight, boolean isAdjustable, TrajectoryType trajectoryType, double swingHeight)
   {
      this(robotSide, footstepPose, trustHeight, isAdjustable, false, null, trajectoryType, swingHeight);
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight, boolean isAdjustable, boolean shouldCheckForReachability, TrajectoryType trajectoryType, double swingHeight)
   {
      this(robotSide, footstepPose, true, isAdjustable, shouldCheckForReachability, null, trajectoryType, swingHeight);
   }

   public Footstep(RobotSide robotSide, FramePose3D footstepPose, boolean trustHeight, boolean isAdjustable, boolean shouldCheckForReachability,
                   List<Point2D> predictedContactPoints, TrajectoryType trajectoryType, double swingHeight)
   {
      this.robotSide = robotSide;
      this.trustHeight = trustHeight;
      this.isAdjustable = isAdjustable;
      this.shouldCheckForReachability = shouldCheckForReachability;
      this.footstepPose.setIncludingFrame(footstepPose);
      setPredictedContactPoints(predictedContactPoints);
      this.trajectoryType = trajectoryType;
      this.swingHeight = swingHeight;
   }

   @Override
   public void set(Footstep other)
   {
      this.sequenceID = other.sequenceID;
      this.robotSide = other.robotSide;
      this.swingTrajectoryBlendDuration = other.swingTrajectoryBlendDuration;
      this.trustHeight = other.trustHeight;
      this.isAdjustable = other.isAdjustable;
      this.shouldCheckForReachability = other.shouldCheckForReachability;
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
   public void set(FootstepDataCommand command, boolean trustHeight, boolean isAdjustable, boolean shouldCheckForReachability)
   {
      this.sequenceID = command.getSequenceId();
      this.robotSide = command.getRobotSide();
      this.swingTrajectoryBlendDuration = command.getSwingTrajectoryBlendDuration();
      this.trajectoryType = command.getTrajectoryType();
      this.swingHeight = command.getSwingHeight();
      this.isAdjustable = isAdjustable;
      this.shouldCheckForReachability = shouldCheckForReachability;
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
      sequenceID = -1;
      robotSide = null;
      footstepPose.setToZero(ReferenceFrame.getWorldFrame());
      predictedContactPoints.clear();
      customWaypointProportions.clear();
      customPositionWaypoints.clear();
      swingTrajectory.clear();
      swingTrajectoryBlendDuration = 0.0;
      trustHeight = true;
      isAdjustable = false;
      shouldCheckForReachability = false;
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

   public void setCustomPositionWaypoints(List<? extends Tuple3DReadOnly> customPositionWaypoints)
   {
      this.customPositionWaypoints.clear();
      for (int i = 0; i < customPositionWaypoints.size(); i++)
      {
         this.customPositionWaypoints.add().set(customPositionWaypoints.get(i));
      }
   }

   public void setCustomWaypointProportions(IntToDoubleFunction customWaypointProportions)
   {
      for (int i = 0; i < 2; i++)
      {
         this.customWaypointProportions.get(i).setValue(customWaypointProportions.applyAsDouble(i));
      }
   }

   public RecyclingArrayList<FrameSE3TrajectoryPoint> getSwingTrajectory()
   {
      return swingTrajectory;
   }

   public void setSwingTrajectory(List<FrameSE3TrajectoryPoint> swingTrajectory)
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

   public void setTrustHeight(boolean trustHeight)
   {
      this.trustHeight = trustHeight;
   }

   public void setIsAdjustable(boolean isAdjustable)
   {
      this.isAdjustable = isAdjustable;
   }

   public void setShouldCheckForReachability(boolean shouldCheckForReachability)
   {
      this.shouldCheckForReachability = shouldCheckForReachability;
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

   public boolean hasPredictedContactPoints()
   {
      return !predictedContactPoints.isEmpty();
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

   public boolean getShouldCheckForReachability()
   {
      return shouldCheckForReachability;
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

   public boolean epsilonEquals(Footstep otherFootstep, double epsilon)
   {
      boolean arePosesEqual = footstepPose.epsilonEquals(otherFootstep.footstepPose, epsilon);
      boolean sameRobotSide = robotSide == otherFootstep.robotSide;
      boolean isTrustHeightTheSame = trustHeight == otherFootstep.trustHeight;
      boolean isAdjustableTheSame = isAdjustable == otherFootstep.isAdjustable;
      boolean isCheckForReachabilityTheSame = shouldCheckForReachability == otherFootstep.shouldCheckForReachability;

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

      return arePosesEqual && sameRobotSide && isTrustHeightTheSame && isAdjustableTheSame && isCheckForReachabilityTheSame && sameWaypoints && sameBlendDuration;
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
      builder.append(" Check for Reacahbility: " + shouldCheckForReachability + "\n");
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
      tempTransform.getRotation().set(footstepPose.getOrientation());
      tempTransform.getTranslation().set(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      poseToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform);
   }

   public void getAnklePosition(FramePoint3D positionToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.getRotation().set(footstepPose.getOrientation());
      tempTransform.getTranslation().set(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      positionToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform.getTranslation());
   }

   public void getAnklePosition2d(FramePoint2D position2dToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.getRotation().set(footstepPose.getOrientation());
      tempTransform.getTranslation().set(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      double x = tempTransform.getTranslation().getX();
      double y = tempTransform.getTranslation().getY();
      position2dToPack.setIncludingFrame(footstepPose.getReferenceFrame(), x, y);
   }

   public void getAnkleOrientation(FrameQuaternion orientationToPack, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.getRotation().set(footstepPose.getOrientation());
      tempTransform.getTranslation().set(footstepPose.getPosition());
      tempTransform.multiply(transformFromAnkleToSole);
      orientationToPack.setIncludingFrame(footstepPose.getReferenceFrame(), tempTransform.getRotation());
   }

   public void setFromAnklePose(FramePose3DReadOnly anklePose, RigidBodyTransform transformFromAnkleToSole)
   {
      tempTransform.getRotation().set(anklePose.getOrientation());
      tempTransform.getTranslation().set(anklePose.getPosition());
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
         swingTrajectory.get(pointIdx).getPosition().add(offset);
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

   public long getSequenceID()
   {
      return sequenceID;
   }
}
