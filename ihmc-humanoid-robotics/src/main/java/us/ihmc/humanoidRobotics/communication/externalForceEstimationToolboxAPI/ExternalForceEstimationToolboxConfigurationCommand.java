package us.ihmc.humanoidRobotics.communication.externalForceEstimationToolboxAPI;

import controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Point3D;

public class ExternalForceEstimationToolboxConfigurationCommand implements Command<ExternalForceEstimationToolboxConfigurationCommand, ExternalForceEstimationConfigurationMessage>
{
   private long sequenceId = 0;
   private double estimatorGain = 1.0;
   private double solverAlpha = 0.005;
   private int endEffectorHashCode;
   private final Point3D externalForcePosition = new Point3D();

   @Override
   public void clear()
   {
      sequenceId = 0;
      estimatorGain = 1.0;
      solverAlpha = 0.005;
      endEffectorHashCode = 0;
      externalForcePosition.setToNaN();
   }

   @Override
   public void setFromMessage(ExternalForceEstimationConfigurationMessage message)
   {
      this.sequenceId = message.getSequenceId();
      this.estimatorGain = message.getEstimatorGain();
      this.solverAlpha = message.getSolverAlpha();
      this.endEffectorHashCode = message.getEndEffectorHashCode();
      this.externalForcePosition.set(message.getExternalForcePosition());
   }

   @Override
   public Class<ExternalForceEstimationConfigurationMessage> getMessageClass()
   {
      return ExternalForceEstimationConfigurationMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return estimatorGain >= 0.0 && !externalForcePosition.containsNaN();
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
      this.endEffectorHashCode = other.endEffectorHashCode;
      this.externalForcePosition.set(other.externalForcePosition);
   }

   public double getEstimatorGain()
   {
      return estimatorGain;
   }

   public double getSolverAlpha()
   {
      return solverAlpha;
   }

   public int getEndEffectorHashCode()
   {
      return endEffectorHashCode;
   }

   public Point3D getExternalForcePosition()
   {
      return externalForcePosition;
   }
}
