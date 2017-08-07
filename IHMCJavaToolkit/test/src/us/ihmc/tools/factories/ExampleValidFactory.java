package us.ihmc.tools.factories;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ExampleValidFactory
{
   private final RequiredFactoryField<Double> requiredField1 = new RequiredFactoryField<>("requiredField1");
   private final RequiredFactoryField<Double> requiredField2 = new RequiredFactoryField<>("requiredField1");
   
   private final OptionalFactoryField<Double> optionalField1 = new OptionalFactoryField<>("optionalField1");
   private final OptionalFactoryField<Double> optionalField2 = new OptionalFactoryField<>("optionalField2");
   
   public Object createObject()
   {
      optionalField1.setDefaultValue(0.0);
      
      assertTrue(optionalField1.hasValue());
      assertFalse(optionalField2.hasValue());
      
      FactoryTools.checkAllFactoryFieldsAreSet(this);
      
      requiredField1.get();
      requiredField2.get();
      Object object = new Object();
      
      FactoryTools.disposeFactory(this);
      
      return object;
   }
   
   public void setRequiredField1(double value1)
   {
      requiredField1.set(value1);
   }
   
   public void setRequiredField2(double value2)
   {
      requiredField2.set(value2);
   }
   
   public void setOptionalField1(double value1)
   {
      optionalField1.set(value1);
   }
   
   public void setOptionalField2(double value2)
   {
      optionalField2.set(value2);
   }
}
