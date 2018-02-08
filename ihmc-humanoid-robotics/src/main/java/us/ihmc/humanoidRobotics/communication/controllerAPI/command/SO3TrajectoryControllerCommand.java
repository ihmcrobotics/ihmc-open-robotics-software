package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.SO3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.FrameInformation;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public final class SO3TrajectoryControllerCommand extends QueueableCommand<SO3TrajectoryControllerCommand, SO3TrajectoryMessage>
      implements FrameBasedCommand<SO3TrajectoryMessage>
{
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
      clearQueuableCommandVariables();
      trajectoryPointList.clear();
      selectionMatrix.resetSelection();
      weightMatrix.clear();
   }

   public void clear(ReferenceFrame referenceFrame)
   {
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

   public void set(SE3TrajectoryControllerCommand other)
   {
      trajectoryPointList.setIncludingFrame(other.getTrajectoryPointList());
      trajectoryFrame = other.getTrajectoryFrame();
      useCustomControlFrame = other.useCustomControlFrame();
      other.getControlFramePose(controlFramePoseInBodyFrame);
      setQueueableCommandVariables(other);
      selectionMatrix.set(other.getSelectionMatrix().getAngularPart());
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, SO3TrajectoryMessage message)
   {
      FrameInformation frameInformation = message.getFrameInformation();
      long trajectoryFrameId = frameInformation.getTrajectoryReferenceFrameId();
      long dataFrameId = FrameInformation.getDataFrameIDConsideringDefault(frameInformation);
      this.trajectoryFrame = resolver.getReferenceFrameFromNameBaseHashCode(trajectoryFrameId);
      ReferenceFrame dataFrame = resolver.getReferenceFrameFromNameBaseHashCode(dataFrameId);

      clear(dataFrame);
      set(message);

      ReferenceFrame selectionFrame = resolver.getReferenceFrameFromNameBaseHashCode(message.getSelectionFrameId());
      selectionMatrix.setSelectionFrame(selectionFrame);
      ReferenceFrame weightSelectionFrame = resolver.getReferenceFrameFromNameBaseHashCode(message.getWeightMatrixFrameId());
      weightMatrix.setWeightFrame(weightSelectionFrame);
   }

   @Override
   public void set(SO3TrajectoryMessage message)
   {
      message.getTrajectoryPoints(trajectoryPointList);
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
      message.getSelectionMatrix(selectionMatrix);
      message.getWeightMatrix(weightMatrix);
      useCustomControlFrame = message.useCustomControlFrame();
      message.getControlFramePose(controlFramePoseInBodyFrame);
   }

   /**
    * Same as {@link #set(SO3TrajectoryControllerCommand)} but does not change the trajectory
    * points.
    *
    * @param other
    */
   public void setPropertiesOnly(SO3TrajectoryControllerCommand other)
   {
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
}
