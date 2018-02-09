package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.EuclideanTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.FrameInformation;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public final class EuclideanTrajectoryControllerCommand extends QueueableCommand<EuclideanTrajectoryControllerCommand, EuclideanTrajectoryMessage>
      implements FrameBasedCommand<EuclideanTrajectoryMessage>
{
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
   public void set(ReferenceFrameHashCodeResolver resolver, EuclideanTrajectoryMessage message)
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
   public void set(EuclideanTrajectoryMessage message)
   {
      message.getTrajectoryPoints(trajectoryPointList);
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
      message.getSelectionMatrix(selectionMatrix);
      message.getWeightMatrix(weightMatrix);
      useCustomControlFrame = message.useCustomControlFrame();
      message.getControlFramePose(controlFramePoseInBodyFrame);
   }

   public void set(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame, EuclideanTrajectoryMessage message)
   {
      this.trajectoryFrame = trajectoryFrame;
      clear(dataFrame);
      set(message);
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

   @Override
   public Class<EuclideanTrajectoryMessage> getMessageClass()
   {
      return EuclideanTrajectoryMessage.class;
   }
}
