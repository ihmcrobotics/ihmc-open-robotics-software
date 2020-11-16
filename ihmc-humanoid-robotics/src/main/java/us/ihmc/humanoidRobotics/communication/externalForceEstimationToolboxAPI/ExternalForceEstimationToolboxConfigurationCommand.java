package us.ihmc.humanoidRobotics.communication.externalForceEstimationToolboxAPI;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.List;

public class ExternalForceEstimationToolboxConfigurationCommand implements Command<ExternalForceEstimationToolboxConfigurationCommand, ExternalForceEstimationConfigurationMessage>
{
   private long sequenceId = 0;
   private double estimatorGain = 1.0;
   private double solverAlpha = 0.005;
   private boolean calculateRootJointWrench = false;
   private final TIntArrayList rigidBodyHashCodes = new TIntArrayList(10);
   private final RecyclingArrayList<Point3D> contactPointPositions = new RecyclingArrayList<>(10, Point3D::new);
   private boolean estimateContactLocation = false;

   @Override
   public void clear()
   {
      sequenceId = 0;
      estimatorGain = 1.0;
      solverAlpha = 0.005;
      calculateRootJointWrench = false;
      rigidBodyHashCodes.reset();
      contactPointPositions.clear();
      estimateContactLocation = false;
   }

   @Override
   public void setFromMessage(ExternalForceEstimationConfigurationMessage message)
   {
      this.sequenceId = message.getSequenceId();
      this.estimatorGain = message.getEstimatorGain();
      this.solverAlpha = message.getSolverAlpha();
      this.calculateRootJointWrench = message.getCalculateRootJointWrench();

      this.rigidBodyHashCodes.reset();
      this.contactPointPositions.clear();
      for (int i = 0; i < message.getRigidBodyHashCodes().size(); i++)
      {
         this.rigidBodyHashCodes.add(message.getRigidBodyHashCodes().get(i));
         this.contactPointPositions.add().set(message.getContactPointPositions().get(i));
      }

      this.estimateContactLocation = message.getEstimateContactLocation();
   }

   @Override
   public Class<ExternalForceEstimationConfigurationMessage> getMessageClass()
   {
      return ExternalForceEstimationConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return estimatorGain >= 0.0 && !rigidBodyHashCodes.isEmpty() && (rigidBodyHashCodes.size() == contactPointPositions.size());
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(ExternalForceEstimationToolboxConfigurationCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.estimatorGain = other.estimatorGain;
      this.solverAlpha = other.solverAlpha;
      this.calculateRootJointWrench = other.calculateRootJointWrench;

      this.rigidBodyHashCodes.reset();
      this.contactPointPositions.clear();
      for (int i = 0; i < other.getNumberOfContactPoints(); i++)
      {
         this.rigidBodyHashCodes.add(other.rigidBodyHashCodes.get(i));
         this.contactPointPositions.add().set(other.contactPointPositions.get(i));
      }
      this.estimateContactLocation = other.estimateContactLocation;
   }

   public double getEstimatorGain()
   {
      return estimatorGain;
   }

   public double getSolverAlpha()
   {
      return solverAlpha;
   }

   public boolean getCalculateRootJointWrench()
   {
      return calculateRootJointWrench;
   }

   public TIntArrayList getRigidBodyHashCodes()
   {
      return rigidBodyHashCodes;
   }

   public List<Point3D> getContactPointPositions()
   {
      return contactPointPositions;
   }

   public int getNumberOfContactPoints()
   {
      return rigidBodyHashCodes.size();
   }

   public boolean getEstimateContactLocation()
   {
      return estimateContactLocation;
   }
}
