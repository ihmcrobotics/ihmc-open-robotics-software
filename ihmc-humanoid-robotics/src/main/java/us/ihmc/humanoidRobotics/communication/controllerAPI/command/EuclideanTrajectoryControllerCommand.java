package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.EuclideanTrajectoryMessage;
import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.FrameInformation;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public final class EuclideanTrajectoryControllerCommand extends QueueableCommand<EuclideanTrajectoryControllerCommand, EuclideanTrajectoryMessage>
      implements FrameBasedCommand<EuclideanTrajectoryMessage>
{
   private long sequenceId;
   private final FrameEuclideanTrajectoryPointList trajectoryPointList = new FrameEuclideanTrajectoryPointList();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
   private final WeightMatrix3D weightMatrix = new WeightMatrix3D();
   private ReferenceFrame trajectoryFrame;

   private boolean useCustomControlFrame = false;
   private final RigidBodyTransform controlFramePoseInBodyFrame = new RigidBodyTransform();

   public EuclideanTrajectoryControllerCommand()
   {
      this(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   public EuclideanTrajectoryControllerCommand(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame)
   {
      clear(dataFrame);
      this.trajectoryFrame = trajectoryFrame;
   }

   public EuclideanTrajectoryControllerCommand(Random random)
   {
      int randomNumberOfPoints = random.nextInt(16) + 1;
      for (int i = 0; i < randomNumberOfPoints; i++)
      {
         trajectoryPointList.addTrajectoryPoint(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01), RandomGeometry.nextPoint3D(random, -1.0, 1.0),
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
      trajectoryPointList.clear(referenceFrame);
      clearQueuableCommandVariables();
      selectionMatrix.resetSelection();
      weightMatrix.clear();
   }

   @Override
   public void set(EuclideanTrajectoryControllerCommand other)
   {
      trajectoryPointList.setIncludingFrame(other.getTrajectoryPointList());
      setPropertiesOnly(other);
      trajectoryFrame = other.getTrajectoryFrame();
      useCustomControlFrame = other.useCustomControlFrame();
      other.getControlFramePose(controlFramePoseInBodyFrame);
   }

   @Override
   public void setFromMessage(EuclideanTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, EuclideanTrajectoryMessage message)
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
      List<EuclideanTrajectoryPointMessage> trajectoryPointMessages = message.getTaskspaceTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.size();

      for (int i = 0; i < numberOfPoints; i++)
      {
         EuclideanTrajectoryPointMessage euclideanTrajectoryPointMessage = trajectoryPointMessages.get(i);
         trajectoryPointList.addTrajectoryPoint(euclideanTrajectoryPointMessage.getTime(), euclideanTrajectoryPointMessage.getPosition(),
                                                euclideanTrajectoryPointMessage.getLinearVelocity());
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
         ReferenceFrame linearSelectionFrame = resolver.getReferenceFrame(message.getSelectionMatrix().getSelectionFrameId());
         selectionMatrix.setSelectionFrame(linearSelectionFrame);
         
         ReferenceFrame linearWeightFrame = resolver.getReferenceFrame(message.getWeightMatrix().getWeightFrameId());
         weightMatrix.setWeightFrame(linearWeightFrame);
      }
   }

   /**
    * Same as {@link #set(EuclideanTrajectoryControllerCommand)} but does not change the trajectory
    * points.
    *
    * @param other
    */
   public void setPropertiesOnly(EuclideanTrajectoryControllerCommand other)
   {
      setQueueableCommandVariables(other);
      sequenceId = other.sequenceId;
      selectionMatrix.set(other.getSelectionMatrix());
      weightMatrix.set(other.getWeightMatrix());
      trajectoryFrame = other.getTrajectoryFrame();
   }

   public void setTrajectoryPointList(FrameEuclideanTrajectoryPointList trajectoryPointList)
   {
      this.trajectoryPointList.setIncludingFrame(trajectoryPointList);
   }

   public SelectionMatrix3D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public WeightMatrix3D getWeightMatrix()
   {
      return weightMatrix;
   }

   public void setWeightMatrix(WeightMatrix3D weightMatrix)
   {
      this.weightMatrix.set(weightMatrix);
   }

   public void setSelectionMatrix(SelectionMatrix3D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   public void setUseCustomControlFrame(boolean useCustomControlFrame)
   {
      this.useCustomControlFrame = useCustomControlFrame;
   }

   public void setControlFramePose(RigidBodyTransform controlFramePose)
   {
      this.controlFramePoseInBodyFrame.set(controlFramePose);
   }

   public FrameEuclideanTrajectoryPointList getTrajectoryPointList()
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
   public void addTrajectoryPoint(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      trajectoryPointList.addTrajectoryPoint(time, position, linearVelocity);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public void addTrajectoryPoint(FrameEuclideanTrajectoryPoint trajectoryPoint)
   {
      trajectoryPointList.addTrajectoryPoint(trajectoryPoint);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public FrameEuclideanTrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPointList.getTrajectoryPoint(trajectoryPointIndex);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public FrameEuclideanTrajectoryPoint getLastTrajectoryPoint()
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

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
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
   public Class<EuclideanTrajectoryMessage> getMessageClass()
   {
      return EuclideanTrajectoryMessage.class;
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
