package us.ihmc.robotics.testing;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public abstract class YoVariableTestGoal implements VariableChangedListener
{
   private final YoVariable<?> yoVariable;
   private boolean hasMetGoal = false;
   
   private YoVariableTestGoal(YoVariable<?> yoVariable)
   {
      this.yoVariable = yoVariable;
      yoVariable.addVariableChangedListener(this);
   }
   
   @Override
   public void variableChanged(YoVariable<?> v)
   {
      if (!hasMetGoal && currentlyMeetsGoal())
      {
         hasMetGoal = true;
      }
   }
   
   public boolean hasMetGoal()
   {
      return hasMetGoal;
   }
   
   public void reset()
   {
      hasMetGoal = false;
   }
   
   public abstract boolean currentlyMeetsGoal();
   
   public YoVariable<?> getYoVariable()
   {
      return yoVariable;
   }

   public static YoVariableTestGoal doubleWithinEpsilon(final DoubleYoVariable doubleYoVariable, final double goalValue, final double epsilon)
   {
      return new YoVariableTestGoal(doubleYoVariable)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return MathTools.epsilonEquals(doubleYoVariable.getDoubleValue(), goalValue, epsilon);
         }
      };
   }
   
   public static YoVariableTestGoal doubleGreaterThan(final DoubleYoVariable doubleYoVariable, final double greaterThan)
   {
      return new YoVariableTestGoal(doubleYoVariable)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return doubleYoVariable.getDoubleValue() > greaterThan;
         }
      };
   }
   
   public static YoVariableTestGoal doubleLessThan(final DoubleYoVariable doubleYoVariable, final double lessThan)
   {
      return new YoVariableTestGoal(doubleYoVariable)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return doubleYoVariable.getDoubleValue() < lessThan;
         }
      };
   }
   
   public static <T extends Enum<T>> YoVariableTestGoal enumEquals(final EnumYoVariable<T> enumYoVariable, final Enum<T> enumValue)
   {
      return new YoVariableTestGoal(enumYoVariable)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return enumYoVariable.getEnumValue().equals(enumValue);
         }
      };
   }
   
   public static YoVariableTestGoal booleanEquals(final BooleanYoVariable booleanYoVariable, final boolean booleanValue)
   {
      return new YoVariableTestGoal(booleanYoVariable)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return booleanYoVariable.getBooleanValue() == booleanValue;
         }
      };
   }
}
