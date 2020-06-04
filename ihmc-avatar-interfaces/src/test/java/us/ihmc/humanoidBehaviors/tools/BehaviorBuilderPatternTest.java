package us.ihmc.humanoidBehaviors.tools;

import org.junit.jupiter.api.Test;

public class BehaviorBuilderPatternTest
{
   @Test
   public void testBuilder()
   {

   }

   class ExampleClass implements BehaviorBuilderPattern
   {
      BooleanField optionalBooleanField = optionalBoolean(false);
      BooleanField requiredBooleanField = requiredBoolean();
//      BooleanField optionalBooleanField = optionalInt(requiredBooleanFieldq)
   }
}
