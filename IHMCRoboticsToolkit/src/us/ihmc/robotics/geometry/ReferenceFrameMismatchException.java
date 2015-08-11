package us.ihmc.robotics.geometry;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: IHMC</p>
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public class ReferenceFrameMismatchException extends RuntimeException
{
   /**
    *
    */
   private static final long serialVersionUID = -2379052247493923182L;

   /**
    * ReferenceFrameMismatchException
    */
   public ReferenceFrameMismatchException(String msg)
   {
      super(msg);
   }
}
