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

   /**
    * The first boolean description.
    */
   default void setBooleanPropertyWithADescription(boolean booleanPropertyWithADescription)
   {
      set(StoredPropertySetTestParameters.booleanPropertyWithADescription, booleanPropertyWithADescription);
   }

   /**
    * The first integer description.
    */
   default void setDoublePropertyWithADescription(double doublePropertyWithADescription)
   {
      set(StoredPropertySetTestParameters.doublePropertyWithADescription, doublePropertyWithADescription);
   }

   /**
    * The first integer description.
    */
   default void setIntegerPropertyWithADescription(int integerPropertyWithADescription)
   {
      set(StoredPropertySetTestParameters.integerPropertyWithADescription, integerPropertyWithADescription);
   }

   /**
    * The double property with more stuff.
    */
   default void setDoublePropertyWithMoreStuff(double doublePropertyWithMoreStuff)
   {
      set(StoredPropertySetTestParameters.doublePropertyWithMoreStuff, doublePropertyWithMoreStuff);
   }

   /**
    * The integer property with more stuff.
    */
   default void setIntegerPropertyWithMoreStuff(int integerPropertyWithMoreStuff)
   {
      set(StoredPropertySetTestParameters.integerPropertyWithMoreStuff, integerPropertyWithMoreStuff);
   }

   /**
    * The integer property with discrete valid values.
    */
   default void setIntegerPropertyWithDiscreteValidValues(int integerPropertyWithDiscreteValidValues)
   {
      set(StoredPropertySetTestParameters.integerPropertyWithDiscreteValidValues, integerPropertyWithDiscreteValidValues);
   }
}
