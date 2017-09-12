package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

public class PelvisHeightTrajectoryCommand extends EuclideanTrajectoryControllerCommand<PelvisHeightTrajectoryCommand, PelvisHeightTrajectoryMessage>
{
   /** Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics **/
   private boolean enableUserPelvisControl = false;
   
   /** 
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking} will keep the height manager in user mode while walking. If this is false the height
    * manager will switch to controller mode when walking
    **/
   private boolean enableUserPelvisControlDuringWalking = false;
   
   private Point3D tempPoint = new Point3D();
   private Vector3D tempVector = new Vector3D();
   
   public PelvisHeightTrajectoryCommand()
   {
      super(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   /**
    * clears the points and sets the frame to the reference frame supplied. 
    * Sets enableUserPelvisControl and enableUserPelvisControlDuringWalking to false;
    */
   @Override
   public void clear()
   {
      super.clear();
      enableUserPelvisControl = false;
      enableUserPelvisControlDuringWalking = false;
   }
   
   /**
    * clears the points and sets the frame to the reference frame supplied. 
    * Sets enableUserPelvisControl and enableUserPelvisControlDuringWalking to false;
    * @param referenceFrame the reference frame the trajectory points will be expressed in
    */
   public void clear(ReferenceFrame referenceFrame)
   {
      super.clear(referenceFrame);
      enableUserPelvisControl = false;
      enableUserPelvisControlDuringWalking = false;
   }

   /**
    * copy setter, copies all the data from the other command to this command
    * @param command the other command
    */
   @Override
   public void set(PelvisHeightTrajectoryCommand command)
   {
      super.set(command);
      enableUserPelvisControl = command.enableUserPelvisControl;
      enableUserPelvisControlDuringWalking = command.enableUserPelvisControlDuringWalking;
   }
   
   /**
    * set this command to the contents of the message
    * @param message the message that has trajectory data
    */
   @Override
   public void set(PelvisHeightTrajectoryMessage message)
   {
      super.set(message);
      enableUserPelvisControl = message.isEnableUserPelvisControl();
      enableUserPelvisControlDuringWalking = message.isEnableUserPelvisControlDuringWalking();
   }
   
   /**
    * set this command to the contents of the message
    * @param dataFrame the frame the data is expressed in
    * @param trajectoryFrame the frame the trajectory will be executed in
    * @param message the message that has trajectory data
    */
   public void set(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame, PelvisHeightTrajectoryMessage message)
   {
      clear(dataFrame);
      set(message);
      enableUserPelvisControl = message.isEnableUserPelvisControl();
      enableUserPelvisControlDuringWalking = message.isEnableUserPelvisControlDuringWalking();
   }

   /**
    * Set this command to the contents of the z height of the pelvis trajectory command 
    * Copies the z points, velocities, and the linear z weight and frame
    * @param command the other command
    */
   public void set(PelvisTrajectoryCommand command)
   {
      clear(command.getDataFrame());
      setQueueableCommandVariables(command);
      
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
   
   /** 
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking} will keep the height manager in user mode while walking. If this is false the height
    * manager will switch to controller mode when walking
    * @return whether or not user mode is enabled while walking
    **/
   public boolean isEnableUserPelvisControlDuringWalking()
   {
      return enableUserPelvisControlDuringWalking;
   }

   /** 
    * If {@code enableUserPelvisControl} is true then {@code enableUserPelvisControlDuringWalking} will keep the height manager in user mode while walking. If this is false the height
    * manager will switch to controller mode when walking
    * @param enableUserPelvisControlDuringWalking sets whether or not user mode is enabled while walking
    **/
   public void setEnableUserPelvisControlDuringWalking(boolean enableUserPelvisControlDuringWalking)
   {
      this.enableUserPelvisControlDuringWalking = enableUserPelvisControlDuringWalking;
   }

   /**
    * Returns whether or not user mode is enabled.
    * If enabled the controller will execute the trajectory in user mode. 
    * User mode will try to achieve the desireds regardless of the leg kinematics
    * @return  whether or not user mode is enabled.
    */
   public boolean isEnableUserPelvisControl()
   {
      return enableUserPelvisControl;
   }

   /**
    * If enabled the controller will execute the trajectory in user mode. 
    * User mode will try to achieve the desireds regardless of the leg kinematics
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
      addTrajectoryPoint(time, tempPoint, tempVector);
   }
}
