package us.ihmc.tools.property;

public interface StoredPropertySetGeneratorTestBasics extends StoredPropertySetGeneratorTestReadOnly, StoredPropertySetBasics
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
