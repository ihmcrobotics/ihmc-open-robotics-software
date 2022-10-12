package us.ihmc.tools.property;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.tools.property.StoredPropertySetTestParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface StoredPropertySetTestParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getTheFirstBooleanProperty()
   {
      return get(theFirstBooleanProperty);
   }

   default double getTheFirstDoubleProperty()
   {
      return get(theFirstDoubleProperty);
   }

   default int getTheFirstIntegerProperty()
   {
      return get(theFirstIntegerProperty);
   }
}
