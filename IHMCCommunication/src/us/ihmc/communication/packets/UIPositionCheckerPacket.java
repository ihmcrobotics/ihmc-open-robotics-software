package us.ihmc.communication.packets;

import javax.vecmath.Vector3f;

import us.ihmc.robotics.geometry.FramePose;

public class UIPositionCheckerPacket extends Packet<UIPositionCheckerPacket> implements VisualizablePacket
{

   private static final boolean DEBUG = true;

   Vector3f position;

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