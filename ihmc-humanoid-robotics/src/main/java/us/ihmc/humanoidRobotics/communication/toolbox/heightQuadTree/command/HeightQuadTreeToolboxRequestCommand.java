package us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command;

import us.ihmc.communication.controllerAPI.command.Command;

public class HeightQuadTreeToolboxRequestCommand implements Command<HeightQuadTreeToolboxRequestCommand, HeightQuadTreeToolboxRequestMessage>
{
   private boolean requestClearQuadTree;
   private boolean requestQuadTreeUpdate;

   @Override
   public void clear()
   {
      requestClearQuadTree = false;
      requestQuadTreeUpdate = false;
   }

   @Override
   public void set(HeightQuadTreeToolboxRequestCommand other)
   {
      requestClearQuadTree = other.requestClearQuadTree;
      requestQuadTreeUpdate = other.requestQuadTreeUpdate;
   }

   @Override
   public void set(HeightQuadTreeToolboxRequestMessage message)
   {
      
      requestClearQuadTree = message.requestClearQuadTree;
      requestQuadTreeUpdate = message.requestQuadTreeUpdate;
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
}
