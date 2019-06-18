package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class PelvisHeightTrajectoryCommand implements Command<PelvisHeightTrajectoryCommand, PelvisHeightTrajectoryMessage>,
      FrameBasedCommand<PelvisHeightTrajectoryMessage>, EpsilonComparable<PelvisHeightTrajectoryCommand>
{
   private long sequenceId;
   /**
    * Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the
    * leg kinematics
    **/
   private boolean enableUserPelvisControl = false;

   /**
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking}
    * will keep the height manager in user mode while walking. If this is false the height manager
    * will switch to controller mode when walking
    **/
   private boolean enableUserPelvisControlDuringWalking = false;

   private Point3D tempPoint = new Point3D();
   private Vector3D tempVector = new Vector3D();

   private final EuclideanTrajectoryControllerCommand euclideanTrajectory;

   public PelvisHeightTrajectoryCommand()
   {
      euclideanTrajectory = new EuclideanTrajectoryControllerCommand();
   }

   /**
    * clears the points and sets the frame to the reference frame supplied. Sets
    * enableUserPelvisControl and enableUserPelvisControlDuringWalking to false;
    */
   public void clear()
   {
      sequenceId = 0;
      euclideanTrajectory.clear();
      enableUserPelvisControl = false;
      enableUserPelvisControlDuringWalking = false;
   }

   /**
    * clears the points and sets the frame to the reference frame supplied. Sets
    * enableUserPelvisControl and enableUserPelvisControlDuringWalking to false;
    * 
    * @param referenceFrame the reference frame the trajectory points will be expressed in
    */
   public void clear(ReferenceFrame referenceFrame)
   {
      sequenceId = 0;
      euclideanTrajectory.clear(referenceFrame);
      enableUserPelvisControl = false;
      enableUserPelvisControlDuringWalking = false;
   }

   /**
    * copy setter, copies all the data from the other command to this command
    * 
    * @param command the other command
    */
   @Override
   public void set(PelvisHeightTrajectoryCommand command)
   {
      sequenceId = command.sequenceId;
      euclideanTrajectory.set(command.euclideanTrajectory);
      enableUserPelvisControl = command.enableUserPelvisControl;
      enableUserPelvisControlDuringWalking = command.enableUserPelvisControlDuringWalking;
   }

   @Override
   public void setFromMessage(PelvisHeightTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, PelvisHeightTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      euclideanTrajectory.set(resolver, message.getEuclideanTrajectory());
      enableUserPelvisControl = message.getEnableUserPelvisControl();
      enableUserPelvisControlDuringWalking = message.getEnableUserPelvisControlDuringWalking();
   }

   /**
    * Set this command to the contents of the z height of the pelvis trajectory command Copies the z
    * points, velocities, and the linear z weight and frame
    * 
    * @param command the other command
    */
   public void set(PelvisTrajectoryCommand command)
   {
      clear(command.getSE3Trajectory().getDataFrame());
      euclideanTrajectory.setQueueableCommandVariables(command.getSE3Trajectory());

      WeightMatrix3D weightMatrix = command.getSE3Trajectory().getWeightMatrix().getLinearPart();
      double zAxisWeight = weightMatrix.getZAxisWeight();
      WeightMatrix3D currentWeightMatrix = euclideanTrajectory.getWeightMatrix();
      currentWeightMatrix.setZAxisWeight(zAxisWeight);
      currentWeightMatrix.setWeightFrame(weightMatrix.getWeightFrame());

      for (int i = 0; i < command.getSE3Trajectory().getNumberOfTrajectoryPoints(); i++)
      {
         FrameSE3TrajectoryPoint trajectoryPoint = command.getSE3Trajectory().getTrajectoryPoint(i);
         double time = trajectoryPoint.getTime();
         double position = trajectoryPoint.getPositionZ();
         double velocity = trajectoryPoint.getLinearVelocityZ();
         tempPoint.setToZero();
         tempPoint.setZ(position);
         tempVector.setToZero();
         tempVector.setZ(velocity);
         euclideanTrajectory.addTrajectoryPoint(time, tempPoint, tempVector);
      }

      enableUserPelvisControl = true;
      enableUserPelvisControlDuringWalking = command.isEnableUserPelvisControlDuringWalking();
   }

   /**
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking}
    * will keep the height manager in user mode while walking. If this is false the height manager
    * will switch to controller mode when walking
    * 
    * @return whether or not user mode is enabled while walking
    **/
   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   /**
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking}
    * will keep the height manager in user mode while walking. If this is false the height manager
    * will switch to controller mode when walking
    * 
    * @param enableUserPelvisControlDuringWalking sets whether or not user mode is enabled while
    *           walking
    **/
   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   /**
    * Returns whether or not user mode is enabled. If enabled the controller will execute the
    * trajectory in user mode. User mode will try to achieve the desireds regardless of the leg
    * kinematics
    * 
    * @return whether or not user mode is enabled.
    */
   public boolean isEnableUserPelvisControl()
   {
      return enableUserPelvisControl;
   }

   /**
    * If enabled the controller will execute the trajectory in user mode. User mode will try to
    * achieve the desireds regardless of the leg kinematics
    */
   public void setEnableUserPelvisControl(boolean enableUserPelvisControl)
   {
      this.enableUserPelvisControl = enableUserPelvisControl;
   }

   @Override
   public Class<PelvisHeightTrajectoryMessage> getMessageClass()
   {
      return PelvisHeightTrajectoryMessage.class;
   }

   /**
    * add a trajectory point to this message
    */
   public void addTrajectoryPoint(double time, double zHeight, double velocity)
   {
      tempPoint.setToZero();
      tempPoint.setZ(zHeight);
      tempVector.setToZero();
      tempVector.setZ(velocity);
      euclideanTrajectory.addTrajectoryPoint(time, tempPoint, tempVector);
   }

   public EuclideanTrajectoryControllerCommand getEuclideanTrajectory()
   {
      return euclideanTrajectory;
   }

   @Override
   public boolean isCommandValid()
   {
      return euclideanTrajectory.isCommandValid();
   }

   @Override
   public boolean epsilonEquals(PelvisHeightTrajectoryCommand other, double epsilon)
   {
      if (enableUserPelvisControl != other.enableUserPelvisControl)
         return false;
      if (enableUserPelvisControlDuringWalking != other.enableUserPelvisControlDuringWalking)
         return false;
      return euclideanTrajectory.epsilonEquals(other.euclideanTrajectory, epsilon);
   }

   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }

   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      euclideanTrajectory.setExecutionDelayTime(delayTime);
   }

   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      euclideanTrajectory.setExecutionTime(adjustedExecutionTime);
   }

   @Override
   public double getExecutionDelayTime()
   {
      return euclideanTrajectory.getExecutionDelayTime();
   }

   @Override
   public double getExecutionTime()
   {
      return euclideanTrajectory.getExecutionTime();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
