package us.ihmc.communication.producers;

import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Make a new enum when you need a new video source.
 * 
 * @author Duncan Calvert
 */
public enum VideoSource
{
   MULTISENSE_LEFT_EYE,
   MULTISENSE_RIGHT_EYE,
   FISHEYE_LEFT,
   FISHEYE_RIGHT,
   CV_THRESHOLD,
   IMAGE_PROCESSING_BEHAVIOR,
   AWARE_FACE_TRACKER,
   ;
   
   public static final VideoSource[] values = values();
   
   public static VideoSource getFisheyeSourceFromRobotSide(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
      {
         return FISHEYE_LEFT;
      }
      else
      {
         return FISHEYE_RIGHT;
      }
   }
   
   public static VideoSource getMultisenseSourceFromRobotSide(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
      {
         return MULTISENSE_LEFT_EYE;
      }
      else
      {
         return MULTISENSE_RIGHT_EYE;
      }
   }

   public RobotSide getRobotSide()
   {
      switch (this)
      {
      case MULTISENSE_LEFT_EYE:
      case FISHEYE_LEFT:
         return RobotSide.LEFT;
      case MULTISENSE_RIGHT_EYE:
      case FISHEYE_RIGHT:
         return RobotSide.RIGHT;
      default:
         throw new RuntimeException(this.name() + " is not associated with a side");
      }
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static VideoSource fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
