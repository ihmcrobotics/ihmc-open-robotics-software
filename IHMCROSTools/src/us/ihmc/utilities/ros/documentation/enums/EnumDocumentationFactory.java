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
   public static Class<? extends DocumentedEnum> getDocumentation(Class<? extends Enum<?>> clazz)
   {
      if (DocumentedEnum.class.isAssignableFrom(clazz))
      {
         return (Class<? extends DocumentedEnum>) clazz;
      }
      
      if(clazz == RobotSide.class)
      {
         return RobotSideDocumentation.class;
      }
      else if (clazz == RobotEnd.class)
      {
         return RobotEndDocumentation.class;
      }
      else if (clazz == TrajectoryType.class)
      {
         return TrajectoryTypeDocumentation.class;
      }
      
      return null;
   }
}
