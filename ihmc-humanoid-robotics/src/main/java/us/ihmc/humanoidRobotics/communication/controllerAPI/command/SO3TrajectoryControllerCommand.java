package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public final class SO3TrajectoryControllerCommand extends QueueableCommand<SO3TrajectoryControllerCommand, SO3TrajectoryMessage>
      implements FrameBasedCommand<SO3TrajectoryMessage>
{
   private long sequenceId;
   private final FrameSO3TrajectoryPointList trajectoryPointList = new FrameSO3TrajectoryPointList();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
   private final WeightMatrix3D weightMatrix = new WeightMatrix3D();
   private ReferenceFrame trajectoryFrame;

   private boolean useCustomControlFrame = false;
   private final RigidBodyTransform controlFramePoseInBodyFrame = new RigidBodyTransform();

   public SO3TrajectoryControllerCommand()
   {
      clear();
   }

   public SO3TrajectoryControllerCommand(Random random)
   {
      sequenceId = random.nextInt();
      int randomNumberOfPoints = random.nextInt(16) + 1;
      for (int i = 0; i < randomNumberOfPoints; i++)
      {
         trajectoryPointList.addTrajectoryPoint(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01), RandomGeometry.nextQuaternion(random),
                                                RandomGeometry.nextVector3D(random));
      }

      trajectoryFrame = EuclidFrameRandomTools.nextReferenceFrame("trajectoryFrame", random, ReferenceFrame.getWorldFrame());
      controlFramePoseInBodyFrame.set(RandomGeometry.nextQuaternion(random), RandomGeometry.nextVector3D(random));
      useCustomControlFrame = random.nextBoolean();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      clearQueuableCommandVariables();
      trajectoryPointList.clear();
      selectionMatrix.resetSelection();
      weightMatrix.clear();
   }

   public void clear(ReferenceFrame referenceFrame)
   {
      sequenceId = 0;
      clearQueuableCommandVariables();
      trajectoryPointList.clear(referenceFrame);
      selectionMatrix.resetSelection();
      weightMatrix.clear();
   }

   @Override
   public void set(SO3TrajectoryControllerCommand other)
   {
      trajectoryPointList.setIncludingFrame(other.getTrajectoryPointList());
      setPropertiesOnly(other);
      trajectoryFrame = other.getTrajectoryFrame();
      useCustomControlFrame = other.useCustomControlFrame();
      other.getControlFramePose(controlFramePoseInBodyFrame);
   }

   @Override
   public void setFromMessage(SO3TrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, SO3TrajectoryMessage message)
   {
      if (resolver != null)
      {
         FrameInformation frameInformation = message.getFrameInformation();
         long trajectoryFrameId = frameInformation.getTrajectoryReferenceFrameId();
         long dataFrameId = HumanoidMessageTools.getDataFrameIDConsideringDefault(frameInformation);
         this.trajectoryFrame = resolver.getReferenceFrame(trajectoryFrameId);
         ReferenceFrame dataFrame = resolver.getReferenceFrame(dataFrameId);
         
         clear(dataFrame);
      }
      else
      {
         clear();
      }

      sequenceId = message.getSequenceId();
      HumanoidMessageTools.checkIfDataFrameIdsMatch(message.getFrameInformation(), trajectoryPointList.getReferenceFrame());

      List<SO3TrajectoryPointMessage> trajectoryPointMessages = message.getTaskspaceTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.size();

      for (int i = 0; i < numberOfPoints; i++)
      {
         SO3TrajectoryPointMessage so3TrajectoryPointMessage = trajectoryPointMessages.get(i);
         trajectoryPointList.addTrajectoryPoint(so3TrajectoryPointMessage.getTime(), so3TrajectoryPointMessage.getOrientation(),
                                                so3TrajectoryPointMessage.getAngularVelocity());
      }
      setQueueableCommandVariables(message.getQueueingProperties());
      selectionMatrix.resetSelection();
      selectionMatrix.setAxisSelection(message.getSelectionMatrix().getXSelected(), message.getSelectionMatrix().getYSelected(), message.getSelectionMatrix().getZSelected());
      weightMatrix.clear();
      weightMatrix.setWeights(message.getWeightMatrix().getXWeight(), message.getWeightMatrix().getYWeight(), message.getWeightMatrix().getZWeight());
      useCustomControlFrame = message.getUseCustomControlFrame();
      message.getControlFramePose().get(controlFramePoseInBodyFrame);

      if (resolver != null)
      {
         ReferenceFrame selectionFrame = resolver.getReferenceFrame(message.getSelectionMatrix().getSelectionFrameId());
         selectionMatrix.setSelectionFrame(selectionFrame);
         ReferenceFrame weightSelectionFrame = resolver.getReferenceFrame(message.getWeightMatrix().getWeightFrameId());
         weightMatrix.setWeightFrame(weightSelectionFrame);
      }
   }

   /**
    * Same as {@link #set(SO3TrajectoryControllerCommand)} but does not change the trajectory
    * points.
    *
    * @param other
    */
   public void setPropertiesOnly(SO3TrajectoryControllerCommand other)
   {
      sequenceId = other.sequenceId;
      setQueueableCommandVariables(other);
      selectionMatrix.set(other.getSelectionMatrix());
      weightMatrix.set(other.getWeightMatrix());
   }

   public void setTrajectoryPointList(FrameSO3TrajectoryPointList trajectoryPointList)
   {
      this.trajectoryPointList.setIncludingFrame(trajectoryPointList);
   }

   public SelectionMatrix3D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   public WeightMatrix3D getWeightMatrix()
   {
      return weightMatrix;
   }

   public void setWeightMatrix(WeightMatrix3D weightMatrix)
   {
      this.weightMatrix.set(weightMatrix);
   }

   public void setUseCustomControlFrame(boolean useCustomControlFrame)
   {
      this.useCustomControlFrame = useCustomControlFrame;
   }

   public void setControlFramePose(RigidBodyTransform controlFramePose)
   {
      this.controlFramePoseInBodyFrame.set(controlFramePose);
   }

   public FrameSO3TrajectoryPointList getTrajectoryPointList()
   {
      return trajectoryPointList;
   }

   @Override
   public boolean isCommandValid()
   {
      return executionModeValid() && trajectoryPointList.getNumberOfTrajectoryPoints() > 0;
   }

   /** {@inheritDoc}} */
   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      trajectoryPointList.addTimeOffset(timeOffsetToAdd);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPointList.getNumberOfTrajectoryPoints();
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      trajectoryPointList.subtractTimeOffset(timeOffsetToSubtract);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public void addTrajectoryPoint(double time, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      trajectoryPointList.addTrajectoryPoint(time, orientation, angularVelocity);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public void addTrajectoryPoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      trajectoryPointList.addTrajectoryPoint(trajectoryPoint);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public FrameSO3TrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPointList.getTrajectoryPoint(trajectoryPointIndex);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public FrameSO3TrajectoryPoint getLastTrajectoryPoint()
   {
      return trajectoryPointList.getLastTrajectoryPoint();
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public void changeFrame(ReferenceFrame referenceFrame)
   {
      trajectoryPointList.changeFrame(referenceFrame);
   }

   public ReferenceFrame getDataFrame()
   {
      return trajectoryPointList.getReferenceFrame();
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public void checkReferenceFrameMatch(ReferenceFrame frame)
   {
      trajectoryPointList.checkReferenceFrameMatch(frame);
   }

   public ReferenceFrame getTrajectoryFrame()
   {
      return trajectoryFrame;
   }

   public void setTrajectoryFrame(ReferenceFrame trajectoryFrame)
   {
      this.trajectoryFrame = trajectoryFrame;
   }

   public RigidBodyTransform getControlFramePose()
   {
      return controlFramePoseInBodyFrame;
   }

   public void getControlFramePose(RigidBodyTransform transformToPack)
   {
      transformToPack.set(controlFramePoseInBodyFrame);
   }

   public boolean useCustomControlFrame()
   {
      return useCustomControlFrame;
   }

   @Override
   public Class<SO3TrajectoryMessage> getMessageClass()
   {
      return SO3TrajectoryMessage.class;
   }

   public void setSequenceId(long sequenceId)
   {
      this.sequenceId = sequenceId;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
