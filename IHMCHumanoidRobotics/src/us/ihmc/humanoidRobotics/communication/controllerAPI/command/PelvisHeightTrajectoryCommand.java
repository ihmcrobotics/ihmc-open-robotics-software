package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

public class PelvisHeightTrajectoryCommand extends EuclideanTrajectoryControllerCommand<PelvisHeightTrajectoryCommand, PelvisHeightTrajectoryMessage>
{
   private boolean enableUserPelvisControl = false;
   private boolean enableUserPelvisControlDuringWalking = false;
   
   private Point3D tempPoint = new Point3D();
   private Vector3D tempVector = new Vector3D();
   
   public PelvisHeightTrajectoryCommand()
   {
      super(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   @Override
   public void clear()
   {
      super.clear();
      enableUserPelvisControl = false;
      enableUserPelvisControlDuringWalking = false;
   }
   
   public void clear(ReferenceFrame referenceFrame)
   {
      super.clear(referenceFrame);
      enableUserPelvisControl = false;
      enableUserPelvisControlDuringWalking = false;
   }

   @Override
   public void set(PelvisHeightTrajectoryCommand command)
   {
      super.set(command);
      enableUserPelvisControl = command.enableUserPelvisControl;
      enableUserPelvisControlDuringWalking = command.enableUserPelvisControlDuringWalking;
   }
   
   @Override
   public void set(PelvisHeightTrajectoryMessage message)
   {
      super.set(message);
      enableUserPelvisControl = message.isEnableUserPelvisControl();
      enableUserPelvisControlDuringWalking = message.isEnableUserPelvisControlDuringWalking();
   }
   
   public void set(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame, PelvisHeightTrajectoryMessage message)
   {
      clear(dataFrame);
      set(message);
      enableUserPelvisControl = message.isEnableUserPelvisControl();
      enableUserPelvisControlDuringWalking = message.isEnableUserPelvisControlDuringWalking();
   }

   public void set(PelvisTrajectoryCommand command)
   {
      clear(command.getDataFrame());
      setQueueqableCommandVariables(command);
      
      WeightMatrix3D weightMatrix = command.getWeightMatrix().getLinearPart();
      double zAxisWeight = weightMatrix.getZAxisWeight();
      WeightMatrix3D currentWeightMatrix = getWeightMatrix();
      currentWeightMatrix.setZAxisWeight(zAxisWeight);
      currentWeightMatrix.setWeightFrame(weightMatrix.getWeightFrame());
      
      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         FrameSE3TrajectoryPoint trajectoryPoint = command.getTrajectoryPoint(i);
         double time = trajectoryPoint.getTime();
         double position = trajectoryPoint.getPositionZ();
         double velocity = trajectoryPoint.getLinearVelocityZ();
         tempPoint.setToZero();
         tempPoint.setZ(position);
         tempVector.setToZero();
         tempVector.setZ(velocity);
         addTrajectoryPoint(time, tempPoint, tempVector);
      }

      enableUserPelvisControl = true;
      enableUserPelvisControlDuringWalking = command.isEnableUserPelvisControlDuringWalking();
   }
   
   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   public boolean isEnableUserPelvisControl()
   {
      return enableUserPelvisControl;
   }

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
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public void addTrajectoryPoint(double time, double zHeight, double velocity)
   {
      tempPoint.setToZero();
      tempPoint.setZ(zHeight);
      tempVector.setToZero();
      tempVector.setZ(velocity);
      addTrajectoryPoint(time, tempPoint, tempVector);
   }
}
