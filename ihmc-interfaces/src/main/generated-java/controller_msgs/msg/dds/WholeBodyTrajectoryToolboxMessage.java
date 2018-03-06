package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Message that carries all the information required to configure the IHMC whole-body trajectory planner.
 * Main usage is the IHMC WholeBodyTrajectoryToolbox.
 */
public class WholeBodyTrajectoryToolboxMessage implements Settable<WholeBodyTrajectoryToolboxMessage>, EpsilonComparable<WholeBodyTrajectoryToolboxMessage>
{
   private controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage configuration_;
   private us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.WaypointBasedTrajectoryMessage> end_effector_trajectories_;
   private us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage> exploration_configurations_;
   private us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ReachingManifoldMessage> reaching_manifolds_;

   public WholeBodyTrajectoryToolboxMessage()
   {
      configuration_ = new controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage();
      end_effector_trajectories_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.WaypointBasedTrajectoryMessage>(10,
                                                                                                                              controller_msgs.msg.dds.WaypointBasedTrajectoryMessage.class,
                                                                                                                              new controller_msgs.msg.dds.WaypointBasedTrajectoryMessagePubSubType());

      exploration_configurations_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage>(10,
                                                                                                                                         controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage.class,
                                                                                                                                         new controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessagePubSubType());

      reaching_manifolds_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ReachingManifoldMessage>(10,
                                                                                                                controller_msgs.msg.dds.ReachingManifoldMessage.class,
                                                                                                                new controller_msgs.msg.dds.ReachingManifoldMessagePubSubType());
   }

   public WholeBodyTrajectoryToolboxMessage(WholeBodyTrajectoryToolboxMessage other)
   {
      set(other);
   }

   public void set(WholeBodyTrajectoryToolboxMessage other)
   {
      controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessagePubSubType.staticCopy(other.configuration_, configuration_);
      end_effector_trajectories_.set(other.end_effector_trajectories_);
      exploration_configurations_.set(other.exploration_configurations_);
      reaching_manifolds_.set(other.reaching_manifolds_);
   }

   public controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage getConfiguration()
   {
      return configuration_;
   }

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.WaypointBasedTrajectoryMessage> getEndEffectorTrajectories()
   {
      return end_effector_trajectories_;
   }

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage> getExplorationConfigurations()
   {
      return exploration_configurations_;
   }

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.ReachingManifoldMessage> getReachingManifolds()
   {
      return reaching_manifolds_;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.configuration_.epsilonEquals(other.configuration_, epsilon))
         return false;

      if (this.end_effector_trajectories_.isEnum())
      {
         if (!this.end_effector_trajectories_.equals(other.end_effector_trajectories_))
            return false;
      }
      else if (this.end_effector_trajectories_.size() == other.end_effector_trajectories_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.end_effector_trajectories_.size(); i++)
         {
            if (!this.end_effector_trajectories_.get(i).epsilonEquals(other.end_effector_trajectories_.get(i), epsilon))
               return false;
         }
      }

      if (this.exploration_configurations_.isEnum())
      {
         if (!this.exploration_configurations_.equals(other.exploration_configurations_))
            return false;
      }
      else if (this.exploration_configurations_.size() == other.exploration_configurations_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.exploration_configurations_.size(); i++)
         {
            if (!this.exploration_configurations_.get(i).epsilonEquals(other.exploration_configurations_.get(i), epsilon))
               return false;
         }
      }

      if (this.reaching_manifolds_.isEnum())
      {
         if (!this.reaching_manifolds_.equals(other.reaching_manifolds_))
            return false;
      }
      else if (this.reaching_manifolds_.size() == other.reaching_manifolds_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.reaching_manifolds_.size(); i++)
         {
            if (!this.reaching_manifolds_.get(i).epsilonEquals(other.reaching_manifolds_.get(i), epsilon))
               return false;
         }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof WholeBodyTrajectoryToolboxMessage))
         return false;

      WholeBodyTrajectoryToolboxMessage otherMyClass = (WholeBodyTrajectoryToolboxMessage) other;

      if (!this.configuration_.equals(otherMyClass.configuration_))
         return false;

      if (!this.end_effector_trajectories_.equals(otherMyClass.end_effector_trajectories_))
         return false;

      if (!this.exploration_configurations_.equals(otherMyClass.exploration_configurations_))
         return false;

      if (!this.reaching_manifolds_.equals(otherMyClass.reaching_manifolds_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyTrajectoryToolboxMessage {");
      builder.append("configuration=");
      builder.append(this.configuration_);

      builder.append(", ");
      builder.append("end_effector_trajectories=");
      builder.append(this.end_effector_trajectories_);

      builder.append(", ");
      builder.append("exploration_configurations=");
      builder.append(this.exploration_configurations_);

      builder.append(", ");
      builder.append("reaching_manifolds=");
      builder.append(this.reaching_manifolds_);

      builder.append("}");
      return builder.toString();
   }
}