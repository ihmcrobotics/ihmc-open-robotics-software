package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class FootstepDataCommand implements Command<FootstepDataCommand, FootstepDataMessage>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private long sequenceId;
   private RobotSide robotSide;
   private TrajectoryType trajectoryType = TrajectoryType.DEFAULT;
   private double swingHeight = 0.0;
   private final FramePoint3D position = new FramePoint3D();
   private final FrameQuaternion orientation = new FrameQuaternion();

   private final RecyclingArrayList<Point2D> predictedContactPoints = new RecyclingArrayList<>(4, Point2D.class);

   private final RecyclingArrayList<MutableDouble> customWaypointProportions = new RecyclingArrayList<>(2, MutableDouble.class);
   private final RecyclingArrayList<FramePoint3D> customPositionWaypoints = new RecyclingArrayList<>(2, FramePoint3D.class);
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingTrajectory = new RecyclingArrayList<>(Footstep.maxNumberOfSwingWaypoints,
                                                                                                        FrameSE3TrajectoryPoint.class);

   private double swingTrajectoryBlendDuration = 0.0;
   private double swingDuration = Double.NaN;
   private double transferDuration = Double.NaN;

   private double transferSplitFraction = Double.NaN;

   private double liftoffDuration = Double.NaN;
   private double touchdownDuration = Double.NaN;

   private ReferenceFrame trajectoryFrame;

   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;
   /** the execution time. This number is set if the execution delay is non zero **/
   public double adjustedExecutionTime;

   public FootstepDataCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotSide = null;
      trajectoryType = TrajectoryType.DEFAULT;
      swingHeight = 0.0;
      position.set(0.0, 0.0, 0.0);
      orientation.set(0.0, 0.0, 0.0, 1.0);
      predictedContactPoints.clear();
      customWaypointProportions.clear();
      customPositionWaypoints.clear();
      swingTrajectory.clear();

      swingDuration = Double.NaN;
      transferDuration = Double.NaN;

      transferSplitFraction = Double.NaN;

      touchdownDuration = Double.NaN;
      liftoffDuration = Double.NaN;
   }

   @Override
   public void setFromMessage(FootstepDataMessage message)
   {
      sequenceId = message.getSequenceId();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      trajectoryType = TrajectoryType.fromByte(message.getTrajectoryType());
      swingHeight = message.getSwingHeight();
      swingTrajectoryBlendDuration = message.getSwingTrajectoryBlendDuration();
      position.setIncludingFrame(worldFrame, message.getLocation());
      orientation.setIncludingFrame(worldFrame, message.getOrientation());

      us.ihmc.idl.IDLSequence.Double messageWaypointProportions = message.getCustomWaypointProportions();
      customWaypointProportions.clear();
      if (messageWaypointProportions != null && messageWaypointProportions.size() == 2)
      {
         for (int i = 0; i < messageWaypointProportions.size(); i++)
         {
            customWaypointProportions.add().setValue(messageWaypointProportions.get(i));
         }
      }

      List<Point3D> originalPositionWaypointList = message.getCustomPositionWaypoints();
      customPositionWaypoints.clear();
      if (originalPositionWaypointList != null)
      {
         for (int i = 0; i < originalPositionWaypointList.size(); i++)
            customPositionWaypoints.add().setIncludingFrame(trajectoryFrame, originalPositionWaypointList.get(i));
      }

      List<SE3TrajectoryPointMessage> messageSwingTrajectory = message.getSwingTrajectory();
      swingTrajectory.clear();
      if (messageSwingTrajectory != null)
      {
         for (int i = 0; i < messageSwingTrajectory.size(); i++)
         {
            FrameSE3TrajectoryPoint point = swingTrajectory.add();
            point.setToZero(trajectoryFrame);
            SE3TrajectoryPointMessage trajectoryPoint = messageSwingTrajectory.get(i);
            point.set(trajectoryPoint.getTime(), trajectoryPoint.getPosition(), trajectoryPoint.getOrientation(), trajectoryPoint.getLinearVelocity(),
                      trajectoryPoint.getAngularVelocity());
         }
      }

      List<Point3D> originalPredictedContactPoints = message.getPredictedContactPoints2d();
      predictedContactPoints.clear();
      if (originalPredictedContactPoints != null)
      {
         for (int i = 0; i < originalPredictedContactPoints.size(); i++)
            predictedContactPoints.add().set(originalPredictedContactPoints.get(i));
      }

      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();

      transferSplitFraction = message.getTransferSplitFraction();

      touchdownDuration = message.getTouchdownDuration();
      liftoffDuration = message.getLiftoffDuration();

      this.executionDelayTime = message.getExecutionDelayTime();
   }

   @Override
   public void set(FootstepDataCommand other)
   {
      sequenceId = other.sequenceId;
      robotSide = other.robotSide;
      trajectoryType = other.trajectoryType;
      swingHeight = other.swingHeight;
      swingTrajectoryBlendDuration = other.swingTrajectoryBlendDuration;
      position.setIncludingFrame(other.position);
      orientation.setIncludingFrame(other.orientation);

      RecyclingArrayList<MutableDouble> otherWaypointProportions = other.customWaypointProportions;
      customWaypointProportions.clear();
      for (int i = 0; i < otherWaypointProportions.size(); i++)
         customWaypointProportions.add().setValue(otherWaypointProportions.get(i));

      RecyclingArrayList<FramePoint3D> otherWaypointList = other.customPositionWaypoints;
      customPositionWaypoints.clear();
      for (int i = 0; i < otherWaypointList.size(); i++)
         customPositionWaypoints.add().setIncludingFrame(otherWaypointList.get(i));

      RecyclingArrayList<FrameSE3TrajectoryPoint> otherSwingTrajectory = other.swingTrajectory;
      swingTrajectory.clear();
      for (int i = 0; i < otherSwingTrajectory.size(); i++)
         swingTrajectory.add().setIncludingFrame(otherSwingTrajectory.get(i));

      RecyclingArrayList<Point2D> otherPredictedContactPoints = other.predictedContactPoints;
      predictedContactPoints.clear();
      for (int i = 0; i < otherPredictedContactPoints.size(); i++)
         predictedContactPoints.add().set(otherPredictedContactPoints.get(i));

      swingDuration = other.swingDuration;
      transferDuration = other.transferDuration;

      transferSplitFraction = other.getTransferSplitFraction();

      touchdownDuration = other.touchdownDuration;
      liftoffDuration = other.liftoffDuration;
      this.executionDelayTime = other.executionDelayTime;
   }

   public void set(ReferenceFrame trajectoryFrame, FootstepDataMessage message)
   {
      this.trajectoryFrame = trajectoryFrame;
      setFromMessage(message);
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setPose(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      this.position.set(position);
      this.orientation.set(orientation);
   }

   public void setSwingTrajectoryBlendDuration(double swingTrajectoryBlendDuration)
   {
      this.swingTrajectoryBlendDuration = swingTrajectoryBlendDuration;
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public void setPredictedContactPoints(RecyclingArrayList<Point2D> predictedContactPoints)
   {
      this.predictedContactPoints.clear();
      for (int i = 0; i < predictedContactPoints.size(); i++)
         this.predictedContactPoints.add().set(predictedContactPoints.get(i));
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public ReferenceFrame getTrajectoryFrame()
   {
      return trajectoryFrame;
   }

   public RecyclingArrayList<MutableDouble> getCustomWaypointProportions()
   {
      return customWaypointProportions;
   }

   public RecyclingArrayList<FramePoint3D> getCustomPositionWaypoints()
   {
      return customPositionWaypoints;
   }

   public RecyclingArrayList<FrameSE3TrajectoryPoint> getSwingTrajectory()
   {
      return swingTrajectory;
   }

   public double getSwingTrajectoryBlendDuration()
   {
      return swingTrajectoryBlendDuration;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public FramePoint3D getPosition()
   {
      return position;
   }

   public FrameQuaternion getOrientation()
   {
      return orientation;
   }

   public RecyclingArrayList<Point2D> getPredictedContactPoints()
   {
      return predictedContactPoints;
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

   public double getTransferDuration()
   {
      return transferDuration;
   }

   public double getTransferSplitFraction()
   {
      return transferSplitFraction;
   }

   public double getTouchdownDuration()
   {
      return touchdownDuration;
   }

   public double getLiftoffDuration()
   {
      return liftoffDuration;
   }

   @Override
   public Class<FootstepDataMessage> getMessageClass()
   {
      return FootstepDataMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null;
   }

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * 
    * @return the time to delay this command in seconds
    */
   @Override
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * 
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }

   /**
    * returns the expected execution time of this command. The execution time will be computed when
    * the controller receives the command using the controllers time plus the execution delay time.
    * This is used when {@code getExecutionDelayTime} is non-zero
    */
   @Override
   public double getExecutionTime()
   {
      return adjustedExecutionTime;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is
    * received.
    */
   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      this.adjustedExecutionTime = adjustedExecutionTime;
   }

   /**
    * tells the controller if this command supports delayed execution (Spoiler alert: It does)
    * 
    * @return
    */
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
