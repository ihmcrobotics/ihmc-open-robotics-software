package us.ihmc.communication.packets;

import javax.vecmath.Vector3f;

public class UIPositionCheckerPacket extends Packet<UIPositionCheckerPacket> implements VisualizablePacket
{
   public Vector3f position;

   public UIPositionCheckerPacket()
   {
   }

   public UIPositionCheckerPacket(Vector3f position)
   {
      this.position = position;
   }

   public void setPosition(Vector3f position)
   {
      this.position = position;
   }

   public Vector3f getPosition()
   {
      return position;
   }

   @Override
   public boolean epsilonEquals(UIPositionCheckerPacket other, double epsilon)
   {
      return position.epsilonEquals(other.getPosition(), (float)epsilon);
   }
}