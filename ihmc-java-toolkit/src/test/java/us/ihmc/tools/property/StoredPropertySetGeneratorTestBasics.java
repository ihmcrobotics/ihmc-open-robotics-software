package us.ihmc.tools.property;

import us.ihmc.tools.property.StoredPropertySetBasics;

public interface StoredPropertySetGeneratorTestBasics extends StoredPropertySetGeneratorTestReadOnly, StoredPropertySetBasics
{
   default void setTheFirstBooleanProperty(boolean theFirstBooleanProperty)
   {
      set(StoredPropertySetGeneratorTest.theFirstBooleanProperty, theFirstBooleanProperty);
   }

   default void setTheFirstDoubleProperty(double theFirstDoubleProperty)
   {
      set(StoredPropertySetGeneratorTest.theFirstDoubleProperty, theFirstDoubleProperty);
   }

   default void setTheFirstIntegerProperty(int theFirstIntegerProperty)
   {
      set(StoredPropertySetGeneratorTest.theFirstIntegerProperty, theFirstIntegerProperty);
   }
}
