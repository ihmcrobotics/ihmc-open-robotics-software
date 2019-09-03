package us.ihmc.robotics.robotSide;

import java.util.EnumSet;

public interface RobotSegment<T extends Enum<T>>
{
   String getCamelCaseNameForStartOfExpression();

   T[] getValues();
   Class<T> getClassType();
   EnumSet<T> getEnumSet();
}
