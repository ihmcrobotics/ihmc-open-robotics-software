package us.ihmc.tools.factories;

import java.lang.reflect.Field;

public interface FactoryFieldAccessor
{
   public Object accessFieldValue(Field field, Object object) throws IllegalArgumentException, IllegalAccessException;
}