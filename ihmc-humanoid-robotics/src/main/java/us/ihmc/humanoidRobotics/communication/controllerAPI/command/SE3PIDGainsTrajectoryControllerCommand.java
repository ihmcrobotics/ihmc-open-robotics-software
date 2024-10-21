package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage;
import controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import controller_msgs.msg.dds.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3PIDGainsTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3PIDGainsTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.SE3PIDGainsTrajectoryPointList;

import java.util.List;

public class SE3PIDGainsTrajectoryControllerCommand extends QueueableCommand<SE3PIDGainsTrajectoryControllerCommand, SE3PIDGainsTrajectoryMessage>
      implements TrajectoryPointListBasics<SE3PIDGainsTrajectoryPoint>
{
   private long sequenceId;
   private final RecyclingArrayList<SE3PIDGainsTrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(200, SE3PIDGainsTrajectoryPoint.class);

   public SE3PIDGainsTrajectoryControllerCommand() {
   }

   @Override
   public void addTrajectoryPoint(SE3PIDGainsTrajectoryPoint trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void addTrajectoryPoint(SE3PIDGainsTrajectoryPointBasics trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void addTrajectoryPoint(double time, PID3DGains angularGains, PID3DGains linearGains)
   {
      us.ihmc.robotics.controllers.pidGains.PID3DGains newAngularGains = getNewPID3DGains(angularGains);
      us.ihmc.robotics.controllers.pidGains.PID3DGains newLinearGains = getNewPID3DGains(linearGains);

      trajectoryPoints.add().set(time, newAngularGains, newLinearGains);
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      trajectoryPoints.clear();
      clearQueuableCommandVariables();
   }

   @Override
   public void set(SE3PIDGainsTrajectoryControllerCommand other)
   {
      trajectoryPoints.clear();
      for (int i = 0; i < other.getNumberOfTrajectoryPoints(); i++)
      {
         trajectoryPoints.add().set(other.getTrajectoryPoint(i));
      }
      setPropertiesOnly(other);
   }

   @Override
   public void setFromMessage(SE3PIDGainsTrajectoryMessage message)
   {
      clear();

      sequenceId = message.getSequenceId();
      setQueueableCommandVariables(message.getQueueingProperties());
      SE3PIDGainsTrajectoryPointMessage pointMessage = new SE3PIDGainsTrajectoryPointMessage();

      for (int i = 0; i < message.getPidGainsTrajectoryPoints().size(); i++)
      {
         pointMessage = message.getPidGainsTrajectoryPoints().get(i);
         addTrajectoryPoint(pointMessage.getTime(), pointMessage.getAngularGains(), pointMessage.getLinearGains());
      }
   }

   /**
    * Same as {@link #set(SE3PIDGainsTrajectoryControllerCommand)} but does not change the trajectory
    * points.
    *
    * @param other
    */
   public void setPropertiesOnly(SE3PIDGainsTrajectoryControllerCommand other)
   {
      sequenceId = other.sequenceId;
      setQueueableCommandVariables(other);
   }

   public List<SE3PIDGainsTrajectoryPoint> getTrajectoryPoints()
   {
      return trajectoryPoints;
   }

   @Override
   public boolean isCommandValid()
   {
      return executionModeValid() && !trajectoryPoints.isEmpty();
   }

   /** {@inheritDoc}} */
   @Override
   public void addTimeOffset(double timeOffsetToAdd)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         getTrajectoryPoint(i).setTime(getTrajectoryPoint(i).getTime() + timeOffsetToAdd);
      }
   }

   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.size();
   }

   public SE3PIDGainsTrajectoryPoint getTrajectoryPoint(int trajectoryPointIndex)
   {
      return trajectoryPoints.get(trajectoryPointIndex);
   }

   public SE3PIDGainsTrajectoryPoint getLastTrajectoryPoint()
   {
      return trajectoryPoints.getLast();
   }

   @Override
   public Class<SE3PIDGainsTrajectoryMessage> getMessageClass()
   {
      return SE3PIDGainsTrajectoryMessage.class;
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

   public us.ihmc.robotics.controllers.pidGains.PID3DGains getNewPID3DGains(PID3DGains gains)
   {
      us.ihmc.robotics.controllers.pidGains.PID3DGains newGains = new DefaultPID3DGains();
      newGains.setProportionalGains(gains.getGainsX().getKp(), gains.getGainsY().getKp(), gains.getGainsZ().getKp());
      newGains.setDerivativeGains(gains.getGainsX().getKd(), gains.getGainsY().getKd(), gains.getGainsZ().getKd());
      newGains.setIntegralGains(gains.getGainsX().getKi(), gains.getGainsY().getKi(), gains.getGainsZ().getKi(), gains.getMaximumIntegralError());
      newGains.getDampingRatios()[0] = gains.getGainsX().getZeta();
      newGains.getDampingRatios()[1] = gains.getGainsY().getZeta();
      newGains.getDampingRatios()[2] = gains.getGainsZ().getZeta();
      newGains.setMaxFeedbackAndFeedbackRate(gains.getMaximumFeedback(), gains.getMaximumFeedbackRate());
      newGains.setMaxDerivativeError(gains.getMaximumDerivativeError());
      newGains.setMaxProportionalError(gains.getMaximumProportionalError());
      return newGains;
   }
}
