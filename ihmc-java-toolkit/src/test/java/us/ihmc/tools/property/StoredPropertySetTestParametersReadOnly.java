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

   /**
    * The first boolean description.
    */
   default boolean getBooleanPropertyWithADescription()
   {
      return get(booleanPropertyWithADescription);
   }

   /**
    * The first integer description.
    */
   default double getDoublePropertyWithADescription()
   {
      return get(doublePropertyWithADescription);
   }

   /**
    * The first integer description.
    */
   default int getIntegerPropertyWithADescription()
   {
      return get(integerPropertyWithADescription);
   }

   /**
    * The double property with more stuff.
    */
   default double getDoublePropertyWithMoreStuff()
   {
      return get(doublePropertyWithMoreStuff);
   }

   /**
    * The integer property with more stuff.
    */
   default int getIntegerPropertyWithMoreStuff()
   {
      return get(integerPropertyWithMoreStuff);
   }

   /**
    * The integer property with discrete valid values.
    */
   default int getIntegerPropertyWithDiscreteValidValues()
   {
      return get(integerPropertyWithDiscreteValidValues);
   }
}
