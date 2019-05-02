package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.awt.Color;

public enum ObjectType
{
   Door, DoorHandle, Cup, Human;
   Color getROIColor()
   {
      switch (this)
      {
      case Cup:
         return Color.WHITE;
      case Door:
         return Color.GREEN;
      case DoorHandle:
         return Color.BLUE;
      case Human:
         return Color.ORANGE;
      default:
         return null;
      }
   }
}