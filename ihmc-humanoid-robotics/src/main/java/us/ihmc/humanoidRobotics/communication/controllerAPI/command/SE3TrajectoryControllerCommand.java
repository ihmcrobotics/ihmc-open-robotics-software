package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.FrameInformation;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public final class SE3TrajectoryControllerCommand extends QueueableCommand<SE3TrajectoryControllerCommand, SE3TrajectoryMessage>
      implements FrameBasedCommand<SE3TrajectoryMessage>
{
   private final FrameSE3TrajectoryPointList trajectoryPointList = new FrameSE3TrajectoryPointList();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();
   private final WeightMatrix6D weightMatrix = new WeightMatrix6D();
   private ReferenceFrame trajectoryFrame;

   private boolean useCustomControlFrame = false;
   private final RigidBodyTransform controlFramePoseInBodyFrame = new RigidBodyTransform();

   public SE3TrajectoryControllerCommand()
   {
      this(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   public SE3TrajectoryControllerCommand(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame)
   {
      clear(dataFrame);
      this.trajectoryFrame = trajectoryFrame;
   }

   public SE3TrajectoryControllerCommand(Random random)
   {
      int randomNumberOfPoints = random.nextInt(16) + 1;
      for (int i = 0; i < randomNumberOfPoints; i++)
      {
         trajectoryPointList.addTrajectoryPoint(RandomNumbers.nextDoubleWithEdgeCases(random, 0.01), RandomGeometry.nextPoint3D(random, -1.0, 1.0),
                                                RandomGeometry.nextQuaternion(random), RandomGeometry.nextVector3D(random),
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
   public void set(SE3TrajectoryControllerCommand other)
   {
      trajectoryPointList.setIncludingFrame(other.getTrajectoryPointList());
      setPropertiesOnly(other);
      trajectoryFrame = other.getTrajectoryFrame();
      useCustomControlFrame = other.useCustomControlFrame();
      other.getControlFramePose(controlFramePoseInBodyFrame);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, SE3TrajectoryMessage message)
   {
      FrameInformation frameInformation = message.getFrameInformation();
      long trajectoryFrameId = frameInformation.getTrajectoryReferenceFrameId();
      long dataFrameId = FrameInformation.getDataFrameIDConsideringDefault(frameInformation);
      this.trajectoryFrame = resolver.getReferenceFrameFromNameBaseHashCode(trajectoryFrameId);
      ReferenceFrame dataFrame = resolver.getReferenceFrameFromNameBaseHashCode(dataFrameId);

      clear(dataFrame);
      set(message);

      ReferenceFrame angularSelectionFrame = resolver.getReferenceFrameFromNameBaseHashCode(message.getAngularSelectionFrameId());
      ReferenceFrame linearSelectionFrame = resolver.getReferenceFrameFromNameBaseHashCode(message.getLinearSelectionFrameId());
      selectionMatrix.setSelectionFrames(angularSelectionFrame, linearSelectionFrame);

      ReferenceFrame angularWeightFrame = resolver.getReferenceFrameFromNameBaseHashCode(message.getAngularWeightMatrixFrameId());
      ReferenceFrame linearWeightFrame = resolver.getReferenceFrameFromNameBaseHashCode(message.getLinearWeightMatrixFrameId());
      weightMatrix.setWeightFrames(angularWeightFrame, linearWeightFrame);
   }

   @Override
   public void set(SE3TrajectoryMessage message)
   {
      message.getTrajectoryPoints(trajectoryPointList);
      setQueueableCommandVariables(message.getUniqueId(), message.getQueueingProperties());
      message.getSelectionMatrix(selectionMatrix);
      message.getWeightMatrix(weightMatrix);
      useCustomControlFrame = message.useCustomControlFrame();
      message.getControlFramePose(controlFramePoseInBodyFrame);
   }

   public void set(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame, SE3TrajectoryMessage message)
   {
      this.trajectoryFrame = trajectoryFrame;
      clear(dataFrame);
      set(message);
   }

   /**
    * Same as {@link #set(SE3TrajectoryControllerCommand)} but does not change the trajectory
    * points.
    *
    * @param other
    */
   public void setPropertiesOnly(SE3TrajectoryControllerCommand other)
   {
      setQueueableCommandVariables(other);
      selectionMatrix.set(other.getSelectionMatrix());
      weightMatrix.set(other.getWeightMatrix());
      trajectoryFrame = other.getTrajectoryFrame();
   }

   public void setTrajectoryPointList(FrameSE3TrajectoryPointList trajectoryPointList)
   {
      this.trajectoryPointList.setIncludingFrame(trajectoryPointList);
   }

   public SelectionMatrix6D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public WeightMatrix6D getWeightMatrix()
   {
      return weightMatrix;
   }

   public void setWeightMatrix(WeightMatrix6D weightMatrix)
   {
      this.weightMatrix.set(weightMatrix);
   }

   public void setSelectionMatrix(SelectionMatrix6D selectionMatrix)
   {
      this.selectionMatrix.set(selectionMatrix);
   }

   public FrameSE3TrajectoryPointList getTrajectoryPointList()
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
   public void addTrajectoryPoint(double time, Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity,
                                  Vector3DReadOnly angularVelocity)
   {
      trajectoryPointList.addTrajectoryPoint(time, position, orientation, linearVelocity, angularVelocity);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public void addTrajectoryPoint(FrameSE3TrajectoryPoint trajectoryPoint)
   {
      trajectoryPointList.addTrajectoryPoint(trajectoryPoint);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public FrameSE3TrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPointList.getTrajectoryPoint(trajectoryPointIndex);
   }

   /**
    * Convenience method for accessing {@link #trajectoryPointList}. To get the list use
    * {@link #getTrajectoryPointList()}.
    */
   public FrameSE3TrajectoryPoint getLastTrajectoryPoint()
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
   public Class<SE3TrajectoryMessage> getMessageClass()
   {
      return SE3TrajectoryMessage.class;
   }
}
