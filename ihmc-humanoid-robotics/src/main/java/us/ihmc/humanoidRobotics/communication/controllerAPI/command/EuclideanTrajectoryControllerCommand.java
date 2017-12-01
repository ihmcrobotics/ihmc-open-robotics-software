package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.AbstractEuclideanTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.FrameInformation;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public abstract class EuclideanTrajectoryControllerCommand<T extends EuclideanTrajectoryControllerCommand<T, M>, M extends AbstractEuclideanTrajectoryMessage<M>>
      extends QueueableCommand<T, M> implements FrameBasedCommand<M>
{
   private final FrameEuclideanTrajectoryPointList trajectoryPointList = new FrameEuclideanTrajectoryPointList();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
   private final WeightMatrix3D weightMatrix = new WeightMatrix3D();
   private ReferenceFrame trajectoryFrame;

   private boolean useCustomControlFrame = false;
   private final RigidBodyTransform controlFramePoseInBodyFrame = new RigidBodyTransform();

   public EuclideanTrajectoryControllerCommand(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame)
   {
      clear(dataFrame);
      this.trajectoryFrame = trajectoryFrame;
   }

   @Override
   public void clear()
   {
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
   public void set(T other)
   {
      trajectoryPointList.setIncludingFrame(other.getTrajectoryPointList());
      setPropertiesOnly(other);
      trajectoryFrame = other.getTrajectoryFrame();
      useCustomControlFrame = other.useCustomControlFrame();
      other.getControlFramePose(controlFramePoseInBodyFrame);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, M message)
   {
      FrameInformation frameInformation = message.getFrameInformation();
      long trajectoryFrameId = frameInformation.getTrajectoryReferenceFrameId();
      long dataFrameId = FrameInformation.getDataFrameIDConsideringDefault(frameInformation);
      this.trajectoryFrame = resolver.getReferenceFrameFromNameBaseHashCode(trajectoryFrameId);
      ReferenceFrame dataFrame = resolver.getReferenceFrameFromNameBaseHashCode(dataFrameId);

      clear(dataFrame);
      set(message);

      ReferenceFrame linearSelectionFrame = resolver.getReferenceFrameFromNameBaseHashCode(message.getLinearSelectionFrameId());
      selectionMatrix.setSelectionFrame(linearSelectionFrame);

      ReferenceFrame linearWeightFrame = resolver.getReferenceFrameFromNameBaseHashCode(message.getLinearWeightMatrixFrameId());
      weightMatrix.setWeightFrame(linearWeightFrame);
   }

   @Override
   public void set(M message)
   {
      message.getTrajectoryPoints(trajectoryPointList);
      setQueueableCommandVariables(message);
      message.getSelectionMatrix(selectionMatrix);
      message.getWeightMatrix(weightMatrix);
      useCustomControlFrame = message.useCustomControlFrame();
      message.getControlFramePose(controlFramePoseInBodyFrame);
   }

   public void set(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame, M message)
   {
      this.trajectoryFrame = trajectoryFrame;
      clear(dataFrame);
      set(message);
   }

   /**
    * Same as {@link #set(T)} but does not change the trajectory points.
    *
    * @param other
    */
   public void setPropertiesOnly(T other)
   {
      setQueueableCommandVariables(other);
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

   public void getControlFramePose(RigidBodyTransform transformToPack)
   {
      transformToPack.set(controlFramePoseInBodyFrame);
   }

   public boolean useCustomControlFrame()
   {
      return useCustomControlFrame;
   }
}
