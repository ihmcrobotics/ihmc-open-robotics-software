package us.ihmc.utilities.ros.documentation.enums;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.DocumentedEnum;

public class RobotSideDocumentation implements DocumentedEnum<RobotSide>
{

   @Override
   public String getDocumentation(RobotSide var)
   {
      return RobotSide.LEFT.getDocumentation(var);
   }

   @Override
   public RobotSide[] getDocumentedValues()
   {
      return RobotSide.LEFT.getDocumentedValues();
   }

}
