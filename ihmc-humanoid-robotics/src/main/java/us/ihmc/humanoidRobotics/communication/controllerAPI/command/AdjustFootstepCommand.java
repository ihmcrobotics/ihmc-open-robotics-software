package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;

import controller_msgs.msg.dds.AdjustFootstepMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;

public class AdjustFootstepCommand implements Command<AdjustFootstepCommand, AdjustFootstepMessage>
{
   private long sequenceId;
   private RobotSide robotSide;
   private final FramePoint3D adjustedPosition = new FramePoint3D();
   private final FrameQuaternion adjustedOrientation = new FrameQuaternion();
   private final RecyclingArrayList<Point2D> predictedContactPoints = new RecyclingArrayList<>(4, Point2D.class);
  
   /** the time to delay this command on the controller side before being executed **/
   public double executionDelayTime;
   
   /** the execution time. This number is set if the execution delay is non zero**/
   public double adjustedExecutionTime;

   public AdjustFootstepCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotSide = null;
      adjustedPosition.set(0.0, 0.0, 0.0);
      adjustedOrientation.set(0.0, 0.0, 0.0, 1.0);
      predictedContactPoints.clear();
   }

   @Override
   public void setFromMessage(AdjustFootstepMessage message)
   {
      sequenceId = message.getSequenceId();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      adjustedPosition.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getLocation());
      adjustedOrientation.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getOrientation());
      List<Point3D> originalPredictedContactPoints = message.getPredictedContactPoints2d();
      predictedContactPoints.clear();
      executionDelayTime = message.getExecutionDelayTime();
      if (originalPredictedContactPoints != null)
      {
         for (int i = 0; i < originalPredictedContactPoints.size(); i++)
            predictedContactPoints.add().set(originalPredictedContactPoints.get(i));
      }
   }

   @Override
   public void set(AdjustFootstepCommand other)
   {
      sequenceId = other.sequenceId;
      robotSide = other.robotSide;
      adjustedPosition.set(other.adjustedPosition);
      adjustedOrientation.set(other.adjustedOrientation);
      executionDelayTime = other.executionDelayTime;
      RecyclingArrayList<Point2D> otherPredictedContactPoints = other.predictedContactPoints;
      predictedContactPoints.clear();
      for (int i = 0; i < otherPredictedContactPoints.size(); i++)
         predictedContactPoints.add().set(otherPredictedContactPoints.get(i));
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void setPose(Point3D position, Quaternion orientation)
   {
      adjustedPosition.set(position);
      adjustedOrientation.set(orientation);
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

   public FramePoint3D getPosition()
   {
      return adjustedPosition;
   }

   public FrameQuaternion getOrientation()
   {
      return adjustedOrientation;
   }

   public RecyclingArrayList<Point2D> getPredictedContactPoints()
   {
      return predictedContactPoints;
   }

   @Override
   public Class<AdjustFootstepMessage> getMessageClass()
   {
      return AdjustFootstepMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null;
   }

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   @Override
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }
   
   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }

   /**
    * returns the expected execution time of this command. The execution time will be computed when the controller 
    * receives the command using the controllers time plus the execution delay time.
    * This is used when {@code getExecutionDelayTime} is non-zero
    */
   @Override
   public double getExecutionTime()
   {
      return adjustedExecutionTime;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is received.
    */
   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      this.adjustedExecutionTime = adjustedExecutionTime;
   }

   /**
    * tells the controller if this command supports delayed execution
    * (Spoiler alert: It does)
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
