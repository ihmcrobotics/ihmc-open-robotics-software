package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.WrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameBasedCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class WrenchTrajectoryControllerCommand extends QueueableCommand<WrenchTrajectoryControllerCommand, WrenchTrajectoryMessage>
      implements FrameBasedCommand<WrenchTrajectoryMessage>
{
   private long sequenceId;
   private ReferenceFrame dataFrame = ReferenceFrame.getWorldFrame();
   private final TDoubleArrayList trajectoryPointTimes = new TDoubleArrayList();
   private final RecyclingArrayList<SpatialVector> trajectoryPointList = new RecyclingArrayList<>(16, SpatialVector.class);
   private ReferenceFrame trajectoryFrame;

   private boolean useCustomControlFrame = false;
   private final RigidBodyTransform controlFramePoseInBodyFrame = new RigidBodyTransform();

   public WrenchTrajectoryControllerCommand()
   {
      this(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   public WrenchTrajectoryControllerCommand(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame)
   {
      clear(dataFrame);
      this.trajectoryFrame = trajectoryFrame;
   }

   public WrenchTrajectoryControllerCommand(Random random)
   {
      sequenceId = random.nextInt();

      dataFrame = ReferenceFrame.getWorldFrame();

      int randomNumberOfPoints = random.nextInt(16) + 1;
      double previousTime = 0.0;
      for (int i = 0; i < randomNumberOfPoints; i++)
      {
         trajectoryPointTimes.add(EuclidCoreRandomTools.nextDouble(random, previousTime, previousTime + 1.0));
         trajectoryPointList.add().set(MecanoRandomTools.nextSpatialVector(random, dataFrame));
      }

      trajectoryFrame = EuclidFrameRandomTools.nextReferenceFrame("trajectoryFrame", random, ReferenceFrame.getWorldFrame());
      controlFramePoseInBodyFrame.set(RandomGeometry.nextQuaternion(random), RandomGeometry.nextVector3D(random));
      useCustomControlFrame = random.nextBoolean();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      trajectoryPointTimes.reset();
      trajectoryPointList.clear();
      clearQueuableCommandVariables();
   }

   public void clear(ReferenceFrame dataFrame)
   {
      sequenceId = 0;
      this.dataFrame = dataFrame;
      trajectoryPointTimes.reset();
      trajectoryPointList.clear();
      clearQueuableCommandVariables();
   }

   @Override
   public void set(WrenchTrajectoryControllerCommand other)
   {
      dataFrame = other.getDataFrame();
      trajectoryPointTimes.reset();
      trajectoryPointList.clear();
      for (int i = 0; i < other.getNumberOfTrajectoryPoints(); i++)
      {
         trajectoryPointTimes.add(other.getTrajectoryPointTime(i));
         trajectoryPointList.add().setIncludingFrame(other.trajectoryPointList.get(i));
      }
      setPropertiesOnly(other);
      trajectoryFrame = other.getTrajectoryFrame();
      useCustomControlFrame = other.useCustomControlFrame();
      other.getControlFramePose(controlFramePoseInBodyFrame);
   }

   @Override
   public void setFromMessage(WrenchTrajectoryMessage message)
   {
      FrameBasedCommand.super.setFromMessage(message);
   }

   @Override
   public void set(ReferenceFrameHashCodeResolver resolver, WrenchTrajectoryMessage message)
   {
      if (message.getWrenchTrajectoryPoints().isEmpty())
      {
         clear();
         return;
      }

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
      HumanoidMessageTools.checkIfDataFrameIdsMatch(message.getFrameInformation(), dataFrame);
      List<WrenchTrajectoryPointMessage> trajectoryPointMessages = message.getWrenchTrajectoryPoints();
      int numberOfPoints = trajectoryPointMessages.size();

      for (int i = 0; i < numberOfPoints; i++)
      {
         WrenchTrajectoryPointMessage trajectoryPointMessage = trajectoryPointMessages.get(i);
         trajectoryPointTimes.add(trajectoryPointMessage.getTime());
         trajectoryPointList.add().setIncludingFrame(dataFrame, trajectoryPointMessage.getWrench().getTorque(), trajectoryPointMessage.getWrench().getForce());
      }
      setQueueableCommandVariables(message.getQueueingProperties());
      useCustomControlFrame = message.getUseCustomControlFrame();
      message.getControlFramePose().get(controlFramePoseInBodyFrame);
   }

   /**
    * Same as {@link #set(WrenchTrajectoryControllerCommand)} but does not change the trajectory
    * points.
    *
    * @param other
    */
   public void setPropertiesOnly(WrenchTrajectoryControllerCommand other)
   {
      sequenceId = other.sequenceId;
      setQueueableCommandVariables(other);
      trajectoryFrame = other.getTrajectoryFrame();
   }

   public TDoubleArrayList getTrajectoryPointTimes()
   {
      return trajectoryPointTimes;
   }

   public List<SpatialVector> getTrajectoryPointList()
   {
      return trajectoryPointList;
   }

   @Override
   public boolean isCommandValid()
   {
      return executionModeValid() && trajectoryPointList.size() > 0;
   }

   public void setControlFramePose(RigidBodyTransform controlFramePose)
   {
      this.controlFramePoseInBodyFrame.set(controlFramePose);
   }

   public void setUseCustomControlFrame(boolean useCustomControlFrame)
   {
      this.useCustomControlFrame = useCustomControlFrame;
   }

   /** {@inheritDoc}} */
   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      for (int i = 0; i < trajectoryPointTimes.size(); i++)
         trajectoryPointTimes.set(i, trajectoryPointTimes.get(i) + timeOffsetToAdd);
   }

   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPointList.size();
   }

   public void subtractTimeOffset(double timeOffsetToSubtract)
   {
      for (int i = 0; i < trajectoryPointTimes.size(); i++)
         trajectoryPointTimes.set(i, trajectoryPointTimes.get(i) - timeOffsetToSubtract);
   }

   public double getTrajectoryPointTime(int trajectoryPointIndex)
   {
      return trajectoryPointTimes.get(trajectoryPointIndex);
   }

   public SpatialVector getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPointList.get(trajectoryPointIndex);
   }

   public SpatialVector getLastTrajectoryPoint()
   {
      return trajectoryPointList.getLast();
   }

   public ReferenceFrame getDataFrame()
   {
      return dataFrame;
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
   public Class<WrenchTrajectoryMessage> getMessageClass()
   {
      return WrenchTrajectoryMessage.class;
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
