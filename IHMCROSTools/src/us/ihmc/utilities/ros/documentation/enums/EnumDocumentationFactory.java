package us.ihmc.utilities.ros.documentation.enums;

import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.tools.DocumentedEnum;


/**
 * Helper function to document Enums that do not depend on DocumentedEnum
 *
 */
public class EnumDocumentationFactory
{
   public static DocumentedEnum getDocumentation(Class<? extends Enum<?>> clazz)
   {
      
      if(clazz == RobotSide.class)
      {
         return new RobotSideDocumentation();
      }
      if (clazz == RobotEnd.class)
      {
         return new RobotEndDocumentation();
      }
      if (clazz == TrajectoryType.class)
      {
         return new TrajectoryTypeDocumentation();
      }
      
      // Catch all clause
      if (DocumentedEnum.class.isAssignableFrom(clazz))
      {
         Class<? extends DocumentedEnum> documentedEnum = (Class<? extends DocumentedEnum>) clazz;
         DocumentedEnum[] values = documentedEnum.getEnumConstants();
         if(values.length > 0)
         {
            return values[0];
         }
         else
         {
            return null;
         }
      }

      // Nothing found, return null
      return null;
   }
}
