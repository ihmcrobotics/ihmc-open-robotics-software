package us.ihmc.utilities.ros.documentation.enums;

import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.tools.DocumentedEnum;

public class RobotEndDocumentation implements DocumentedEnum<RobotEnd>
{

   @Override
   public String getDocumentation(RobotEnd var)
   {
      return RobotEnd.FRONT.getDocumentation(var);
   }

   @Override
   public RobotEnd[] getDocumentedValues()
   {
      return RobotEnd.FRONT.getDocumentedValues();
   }

}
