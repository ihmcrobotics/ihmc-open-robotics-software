package us.ihmc.quadrupedRobotics.geometry.supportPolygon;

public class EmptySupportPolygonException extends RuntimeException
{
   private static final long serialVersionUID = 8705577923822473889L;

   public EmptySupportPolygonException()
   {
      super("Support polygon is empty.");
   }
   
   public EmptySupportPolygonException(String message)
   {
      super(message);
   }
}