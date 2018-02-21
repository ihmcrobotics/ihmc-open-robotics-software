package us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command;

import us.ihmc.communication.packets.Packet;

public class HeightQuadTreeToolboxRequestMessage extends Packet<HeightQuadTreeToolboxRequestMessage>
{
   public boolean requestClearQuadTree;
   public boolean requestQuadTreeUpdate;

   public HeightQuadTreeToolboxRequestMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(HeightQuadTreeToolboxRequestMessage other)
   {
      requestClearQuadTree = other.requestClearQuadTree;
      requestQuadTreeUpdate = other.requestQuadTreeUpdate;
      setPacketInformation(other);
   }

   public boolean getRequestClearQuadTree()
   {
      return requestClearQuadTree;
   }

   public boolean getRequestQuadTreeUpdate()
   {
      return requestQuadTreeUpdate;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeToolboxRequestMessage other, double epsilon)
   {
      if (requestClearQuadTree != other.requestClearQuadTree)
         return false;
      if (requestQuadTreeUpdate != other.requestQuadTreeUpdate)
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": clear request = " + requestClearQuadTree + ", quadTree request = " + requestQuadTreeUpdate;
   }
}
