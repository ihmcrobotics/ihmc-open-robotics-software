package us.ihmc.humanoidRobotics.communication.referenceSpreadingToolboxAPI;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.ReferenceSpreadingToolboxInputMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.List;

public class ReferenceSpreadingToolboxInputCommand implements Command<ReferenceSpreadingToolboxInputCommand, ReferenceSpreadingToolboxInputMessage>
{
   private long sequenceId;
   private int state;

   @Override
   public void clear()
   {
      sequenceId = 0;
      state = 0;
   }

   @Override
   public void set(ReferenceSpreadingToolboxInputCommand other)
   {
      sequenceId = other.sequenceId;
      state = other.state;
   }

   public void set(long sequenceId, int state)
   {
      this.sequenceId = sequenceId;
      this.state = state;
   }

   @Override
   public void setFromMessage(ReferenceSpreadingToolboxInputMessage message)
   {
      sequenceId = message.getSequenceId();
      state = message.getState();
   }

   public void setState(int state)
   {
      this.state = state;
   }

   public void setSequenceId(long sequenceId)
   {
      this.sequenceId = sequenceId;
   }


   @Override
   public Class<ReferenceSpreadingToolboxInputMessage> getMessageClass()
   {
      return ReferenceSpreadingToolboxInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      if (state < 0)
         return false;

      if (state > 2)
         return false;

      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }


   public int getState()
   {
      return state;
   }

   public static class COMMAND
   {
      public static final int START_RECORDING = 0;
      public static final int STOP_RECORDING = 1;
      public static final int START_PLAYBACK = 2;
   }
}
