package us.ihmc.tools.property;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface StoredPropertySetTestParametersBasics extends StoredPropertySetTestParametersReadOnly, StoredPropertySetBasics
{
   default void setTheFirstBooleanProperty(boolean theFirstBooleanProperty)
   {
      set(StoredPropertySetTestParameters.theFirstBooleanProperty, theFirstBooleanProperty);
   }

   default void setTheFirstDoubleProperty(double theFirstDoubleProperty)
   {
      set(StoredPropertySetTestParameters.theFirstDoubleProperty, theFirstDoubleProperty);
   }

   default void setTheFirstIntegerProperty(int theFirstIntegerProperty)
   {
      set(StoredPropertySetTestParameters.theFirstIntegerProperty, theFirstIntegerProperty);
   }
}
