package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.JointOfflineMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class JointOfflineCommand extends QueueableCommand<JointOfflineCommand, JointOfflineMessage>
{
   private long sequenceId;

   /* Hash-code of the joints that are offline/dead */
   private final TIntArrayList jointOfflineHashCodes = new TIntArrayList();

   /* Nominal contact points of  */
   private final SideDependentList<RecyclingArrayList<Point2D>> nominalContactPoints = new SideDependentList<>(side -> new RecyclingArrayList<>(4, Point2D.class));

   private double executionDelayTime;
   public double adjustedExecutionTime;

   public JointOfflineCommand()
   {
      clear();
   }

   public int getNumberOfJointsOffline()
   {
      return jointOfflineHashCodes.size();
   }

   public int getJointOfflineHashCode(int index)
   {
      return jointOfflineHashCodes.get(index);
   }

   public void addJointOfflineHashCode(int jointOfflineHashCode)
   {
      this.jointOfflineHashCodes.add(jointOfflineHashCode);
   }

   @Override
   public void set(JointOfflineCommand other)
   {
      clear();
      sequenceId = other.sequenceId;
      for (int i = 0; i < other.jointOfflineHashCodes.size(); i++)
      {
         this.jointOfflineHashCodes.add(other.jointOfflineHashCodes.get(i));
      }
      for (RobotSide robotSide : RobotSide.values)
      {
         RecyclingArrayList<Point2D> otherNominalContactPoints = other.nominalContactPoints.get(robotSide);
         RecyclingArrayList<Point2D> thisNominalContactPoints = this.nominalContactPoints.get(robotSide);

         thisNominalContactPoints.clear();
         for (int i = 0; i < otherNominalContactPoints.size(); i++)
            thisNominalContactPoints.add().set(otherNominalContactPoints.get(i));
      }

      this.executionDelayTime = other.executionDelayTime;
   }

   @Override
   public void setFromMessage(JointOfflineMessage message)
   {
      sequenceId = message.getSequenceId();
      for (int i = 0; i < message.getJointOfflineHashCodes().size(); i++)
      {
         this.jointOfflineHashCodes.add(message.getJointOfflineHashCodes().get(i));
      }

      nominalContactPoints.get(RobotSide.LEFT).clear();
      for (int i = 0; i < message.getNominalLeftFootContactPoints2d().size(); i++)
         nominalContactPoints.get(RobotSide.LEFT).add().set(message.getNominalLeftFootContactPoints2d().get(i));

      nominalContactPoints.get(RobotSide.RIGHT).clear();
      for (int i = 0; i < message.getNominalRightFootContactPoints2d().size(); i++)
         nominalContactPoints.get(RobotSide.RIGHT).add().set(message.getNominalRightFootContactPoints2d().get(i));

      this.executionDelayTime = message.getExecutionDelayTime();
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      jointOfflineHashCodes.clear();

      for (RobotSide robotSide : RobotSide.values)
         nominalContactPoints.get(robotSide).clear();
   }

   public RecyclingArrayList<Point2D> getNominalFootContactPoints(RobotSide robotSide)
   {
      return nominalContactPoints.get(robotSide);
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public Class<JointOfflineMessage> getMessageClass()
   {
      return JointOfflineMessage.class;
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

   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   @Override
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }

   /**
    * sets the amount of time this command is delayed on the controller side before executing
    *
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }

   @Override
   public void addTimeOffset(double timeOffset)
   {
      throw new RuntimeException("This method should not be used with joint offline status.");
   }

   /**
    * returns the expected execution time of this command. The execution time will be computed when
    * the controller receives the command using the controllers time plus the execution delay time.
    * This is used when {@code getExecutionDelayTime} is non-zero
    */
   @Override
   public double getExecutionTime()
   {
      return adjustedExecutionTime;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is
    * received.
    */
   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      this.adjustedExecutionTime = adjustedExecutionTime;
   }
}
