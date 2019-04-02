package us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command;

import controller_msgs.msg.dds.HeightQuadTreeToolboxRequestMessage;
import us.ihmc.communication.controllerAPI.command.Command;

public class HeightQuadTreeToolboxRequestCommand implements Command<HeightQuadTreeToolboxRequestCommand, HeightQuadTreeToolboxRequestMessage>
{
   private long sequenceId;
   private boolean requestClearQuadTree;
   private boolean requestQuadTreeUpdate;

   @Override
   public void clear()
   {
      sequenceId = 0;
      requestClearQuadTree = false;
      requestQuadTreeUpdate = false;
   }

   @Override
   public void set(HeightQuadTreeToolboxRequestCommand other)
   {
      sequenceId = other.sequenceId;
      requestClearQuadTree = other.requestClearQuadTree;
      requestQuadTreeUpdate = other.requestQuadTreeUpdate;
   }

   @Override
   public void setFromMessage(HeightQuadTreeToolboxRequestMessage message)
   {
      sequenceId = message.getSequenceId();
      requestClearQuadTree = message.getRequestClearQuadTree();
      requestQuadTreeUpdate = message.getRequestQuadTreeUpdate();
   }

   public boolean isClearQuadTreeRequested()
   {
      return requestClearQuadTree;
   }

   public boolean isQuadTreeUpdateRequested()
   {
      return requestQuadTreeUpdate;
   }

   @Override
   public Class<HeightQuadTreeToolboxRequestMessage> getMessageClass()
   {
      return HeightQuadTreeToolboxRequestMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return requestClearQuadTree || requestQuadTreeUpdate;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": clear request = " + requestClearQuadTree + ", quadTree request = " + requestQuadTreeUpdate;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
