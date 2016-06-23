package us.ihmc.tools.factories;

import java.lang.reflect.Field;

public interface Factory extends FactoryFieldAccessor
{
   @Override
   Object accessFieldValue(Field field, Object object) throws IllegalArgumentException, IllegalAccessException;
}
