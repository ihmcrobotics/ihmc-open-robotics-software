package us.ihmc.robotics.testing;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.FormattingTools;

public abstract class YoVariableTestGoal implements VariableChangedListener
{
   private static final int SIGNIFICANT_FIGURES_FOR_PRINT_OUT = 3;

   private boolean hasMetGoal = false;

   private YoVariableTestGoal(YoVariable<?>... yoVariables)
   {
      for (YoVariable<?> yoVariable : yoVariables)
      {
         yoVariable.addVariableChangedListener(this);
      }
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

   @Override
   public abstract String toString();

   public static YoVariableTestGoal variablesEqual(final DoubleYoVariable variableOne, final DoubleYoVariable variableTwo, final double epsilon)
   {
      return new YoVariableTestGoal(variableOne)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return MathTools.epsilonEquals(variableOne.getDoubleValue(), variableTwo.getDoubleValue(), epsilon);
         }

         @Override
         public String toString()
         {
            String numberStringOne = getFormattedDoubleYoVariable(variableOne);
            String numberStringTwo = getFormattedDoubleYoVariable(variableTwo);
            String epsilonString = FormattingTools.getFormattedToSignificantFigures(epsilon, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            return numberStringOne + " (+/- " + epsilonString + ") == " + numberStringTwo;
         }
      };
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

         @Override
         public String toString()
         {
            String numberString = getFormattedDoubleYoVariable(doubleYoVariable);
            String epsilonString = FormattingTools.getFormattedToSignificantFigures(epsilon, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            String goalString = FormattingTools.getFormattedToSignificantFigures(goalValue, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            return numberString + " (+/- " + epsilonString + ") == " + goalString;
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

         @Override
         public String toString()
         {
            String numberString = getFormattedDoubleYoVariable(doubleYoVariable);
            String greaterThanString = FormattingTools.getFormattedToSignificantFigures(greaterThan, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            return numberString + " > " + greaterThanString;
         }
      };
   }

   public static YoVariableTestGoal deltaGreaterThan(final DoubleYoVariable minuend, final DoubleYoVariable subtrahend, final double difference)
   {
      return new YoVariableTestGoal(minuend, subtrahend)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return minuend.getDoubleValue() - subtrahend.getDoubleValue() > difference;
         }

         @Override
         public String toString()
         {
            String minuendString = getFormattedDoubleYoVariable(minuend);
            String subtrahendString = getFormattedDoubleYoVariable(subtrahend);
            String differenceString = FormattingTools.getFormattedToSignificantFigures(difference, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            return minuendString + " - " + subtrahendString + " > " + differenceString;
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

         @Override
         public String toString()
         {
            String numberString = getFormattedDoubleYoVariable(doubleYoVariable);
            String lessThanString = FormattingTools.getFormattedToSignificantFigures(lessThan, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            return numberString + " < " + lessThanString;
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

         @Override
         public String toString()
         {
            return getFormattedEnumYoVariable(enumYoVariable) + " == " + enumValue.name();
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

         @Override
         public String toString()
         {
            return getFormattedBooleanYoVariable(booleanYoVariable) + " == " + booleanValue;
         }

      };
   }

   public static YoVariableTestGoal or(YoVariableTestGoal goalOne, YoVariableTestGoal goalTwo)
   {
      return new YoVariableTestGoal()
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return goalOne.currentlyMeetsGoal() || goalTwo.currentlyMeetsGoal();
         }

         @Override
         public String toString()
         {
            return goalOne.toString() + " or " + goalTwo.toString();
         }

      };
   }

   public static YoVariableTestGoal and(YoVariableTestGoal goalOne, YoVariableTestGoal goalTwo)
   {
      return new YoVariableTestGoal()
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return goalOne.currentlyMeetsGoal() && goalTwo.currentlyMeetsGoal();
         }

         @Override
         public String toString()
         {
            return goalOne.toString() + " and " + goalTwo.toString();
         }

      };
   }

   public static YoVariableTestGoal not(YoVariableTestGoal goal)
   {
      return new YoVariableTestGoal()
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return !goal.currentlyMeetsGoal();
         }

         @Override
         public String toString()
         {
            return "not " + goal.toString();
         }
      };
   }
   
   public static YoVariableTestGoal timeInFuture(DoubleYoVariable timeYoVariable, double durationFromNow)
   {
      return doubleGreaterThan(timeYoVariable, timeYoVariable.getDoubleValue() + durationFromNow);
   }

   private static String getFormattedBooleanYoVariable(final BooleanYoVariable booleanYoVariable)
   {
      return booleanYoVariable.getName() + ":" + booleanYoVariable.getBooleanValue();
   }

   private static <T extends Enum<T>> String getFormattedEnumYoVariable(final EnumYoVariable<T> enumYoVariable)
   {
      return enumYoVariable.getName() + ":" + enumYoVariable.getEnumValue().name();
   }

   public static String getFormattedDoubleYoVariable(DoubleYoVariable doubleYoVariable)
   {
      return doubleYoVariable.getName() + ":"
            + FormattingTools.getFormattedToSignificantFigures(doubleYoVariable.getDoubleValue(), SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
   }
}
