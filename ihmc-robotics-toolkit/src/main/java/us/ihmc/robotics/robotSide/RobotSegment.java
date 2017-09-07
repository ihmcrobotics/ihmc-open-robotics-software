package us.ihmc.robotics.robotSide;

import java.util.EnumSet;

public interface RobotSegment<T extends Enum<T>>
{
   public T[] getValues();
   public EnumSet<T> getEnumSet();
}
