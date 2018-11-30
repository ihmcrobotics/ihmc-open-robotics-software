package us.ihmc.robotEnvironmentAwareness.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D32;

public class LineSegment3DMessage extends Packet<LineSegment3DMessage>
{
   public Point3D32 start, end;

   public LineSegment3DMessage()
   {
   }

   @Override
   public void set(LineSegment3DMessage other)
   {
      start = new Point3D32(other.start);
      end = new Point3D32(other.end);
      setPacketInformation(other);
   }

   public Point3D32 getStart()
   {
      return start;
   }

   public Point3D32 getEnd()
   {
      return end;
   }

   public void setStart(Point3D32 start)
   {
      this.start = start;
   }

   public void setEnd(Point3D32 end)
   {
      this.end = end;
   }

   @Override
   public boolean epsilonEquals(LineSegment3DMessage other, double epsilon)
   {
      return start.epsilonEquals(other.start, (float) epsilon) && end.epsilonEquals(other.end, (float) epsilon);
   }
}
