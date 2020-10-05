package us.ihmc.robotics.geometry.concavePolygon2D;

public class ComplexPolygonException extends RuntimeException
{
   public ComplexPolygonException()
   {
      this("");
   }

   public ComplexPolygonException(String message)
   {
      super(message);
   }
}
