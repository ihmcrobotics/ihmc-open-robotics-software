package us.ihmc.robotEnvironmentAwareness.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple3D.Point3D32;

public class LineSegment3DMessage extends Packet<LineSegment3DMessage>
{
   public Point3D32 start, end;

   public LineSegment3DMessage()
   {
   }

   public LineSegment3DMessage(Point3D32 start, Point3D32 end)
   {
      this.start = start;
      this.end = end;
   }

   public LineSegment3DMessage(LineSegment3D lineSegment3d)
   {
      start = new Point3D32(lineSegment3d.getFirstEndpoint());
      end = new Point3D32(lineSegment3d.getSecondEndpoint());
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
