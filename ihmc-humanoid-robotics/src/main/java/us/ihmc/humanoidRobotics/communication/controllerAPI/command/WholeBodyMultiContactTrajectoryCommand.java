package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.ContactStateChangeMessage;
import controller_msgs.msg.dds.WholeBodyMultiContactTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tools.EuclidCoreTools;

import java.util.List;

public class WholeBodyMultiContactTrajectoryCommand
      implements Command<WholeBodyMultiContactTrajectoryCommand, WholeBodyMultiContactTrajectoryMessage>, EpsilonComparable<WholeBodyMultiContactTrajectoryCommand>
{
   private long sequenceId;
   private double trajectoryDuration;
   private final TDoubleArrayList jointAngles = new TDoubleArrayList(50);
   private final Pose3D pelvisPose = new Pose3D();
   private final RecyclingArrayList<ContactStateChangeMessage> contactStateChangs = new RecyclingArrayList<>(10, ContactStateChangeMessage.class);

   @Override
   public void clear()
   {
      sequenceId = 0;
      trajectoryDuration = Double.NaN;
      jointAngles.reset();
      pelvisPose.setToNaN();
      contactStateChangs.clear();
   }

   @Override
   public void setFromMessage(WholeBodyMultiContactTrajectoryMessage message)
   {
      sequenceId = message.getSequenceId();
      trajectoryDuration = message.getTrajectoryDuration();
      pelvisPose.set(message.getPelvisPose());

      for (int i = 0; i < message.getJointAngles().size(); i++)
      {
         jointAngles.add(message.getJointAngles().get(i));
      }

      for (int i = 0; i < message.getContactStateChanges().size(); i++)
      {
         contactStateChangs.add().set(message.getContactStateChanges().get(i));
      }
   }

   @Override
   public Class<WholeBodyMultiContactTrajectoryMessage> getMessageClass()
   {
      return WholeBodyMultiContactTrajectoryMessage.class;
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public TDoubleArrayList getJointAngles()
   {
      return jointAngles;
   }

   public Pose3D getPelvisPose()
   {
      return pelvisPose;
   }

   public List<ContactStateChangeMessage> getContactStateChangs()
   {
      return contactStateChangs;
   }

   @Override
   public boolean isCommandValid()
   {
      return !Double.isNaN(trajectoryDuration) && !jointAngles.isEmpty() && !pelvisPose.containsNaN();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public boolean epsilonEquals(WholeBodyMultiContactTrajectoryCommand other, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(trajectoryDuration, other.trajectoryDuration, epsilon))
      {
         return false;
      }
      if (!pelvisPose.epsilonEquals(other.pelvisPose, epsilon))
      {
         return false;
      }
      if (jointAngles.size() != other.jointAngles.size())
      {
         return false;
      }

      for (int i = 0; i < jointAngles.size(); i++)
      {
         if (!EuclidCoreTools.epsilonEquals(jointAngles.get(i), other.jointAngles.get(i), epsilon))
         {
            return false;
         }
      }

      if (contactStateChangs.size() != other.contactStateChangs.size())
      {
         return false;
      }
      for (int i = 0; i < contactStateChangs.size(); i++)
      {
         if (contactStateChangs.get(i).epsilonEquals(other.contactStateChangs.get(i), epsilon))
         {
            return false;
         }
      }

      return false;
   }

   @Override
   public void set(WholeBodyMultiContactTrajectoryCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.trajectoryDuration = other.trajectoryDuration;
      this.pelvisPose.set(other.pelvisPose);

      this.jointAngles.reset();
      for (int i = 0; i < other.jointAngles.size(); i++)
      {
         this.jointAngles.add(other.jointAngles.get(i));
      }
   }
}
