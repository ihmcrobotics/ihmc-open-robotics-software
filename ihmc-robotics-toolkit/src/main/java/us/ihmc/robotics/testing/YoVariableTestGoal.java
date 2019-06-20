package us.ihmc.robotics.testing;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.commons.FormattingTools;

public abstract class YoVariableTestGoal extends GoalOrientedTestGoal
{
   private static final int SIGNIFICANT_FIGURES_FOR_PRINT_OUT = 3;

   protected YoVariableTestGoal(YoVariable<?>... yoVariables)
   {
      for (YoVariable<?> yoVariable : yoVariables)
      {
         yoVariable.addVariableChangedListener(this::notifyOfVariableChange);
      }
   }

   public void notifyOfVariableChange(YoVariable<?> v)
   {
      update();
   }

   public static YoVariableTestGoal variablesEqual(final YoDouble variableOne, final YoDouble variableTwo, final double epsilon)
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

   public static YoVariableTestGoal doubleWithinEpsilon(final YoDouble yoDouble, final double goalValue, final double epsilon)
   {
      return new YoVariableTestGoal(yoDouble)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return MathTools.epsilonEquals(yoDouble.getDoubleValue(), goalValue, epsilon);
         }

         @Override
         public String toString()
         {
            String numberString = getFormattedDoubleYoVariable(yoDouble);
            String epsilonString = FormattingTools.getFormattedToSignificantFigures(epsilon, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            String goalString = FormattingTools.getFormattedToSignificantFigures(goalValue, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            return numberString + " (+/- " + epsilonString + ") == " + goalString;
         }
      };
   }

   public static YoVariableTestGoal doubleGreaterThan(final YoDouble yoDouble, final double greaterThan)
   {
      return new YoVariableTestGoal(yoDouble)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return yoDouble.getDoubleValue() > greaterThan;
         }

         @Override
         public String toString()
         {
            String numberString = getFormattedDoubleYoVariable(yoDouble);
            String greaterThanString = FormattingTools.getFormattedToSignificantFigures(greaterThan, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            return numberString + " > " + greaterThanString;
         }
      };
   }

   public static YoVariableTestGoal deltaGreaterThan(final YoDouble minuend, final YoDouble subtrahend, final double difference)
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

   public static YoVariableTestGoal doubleLessThan(final YoDouble yoDouble, final double lessThan)
   {
      return new YoVariableTestGoal(yoDouble)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return yoDouble.getDoubleValue() < lessThan;
         }

         @Override
         public String toString()
         {
            String numberString = getFormattedDoubleYoVariable(yoDouble);
            String lessThanString = FormattingTools.getFormattedToSignificantFigures(lessThan, SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
            return numberString + " < " + lessThanString;
         }
      };
   }

   public static <T extends Enum<T>> YoVariableTestGoal enumEquals(final YoEnum<T> yoEnum, final Enum<T> enumValue)
   {
      return new YoVariableTestGoal(yoEnum)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return yoEnum.getEnumValue() == enumValue;
         }

         @Override
         public String toString()
         {
            return getFormattedEnumYoVariable(yoEnum) + " == " + enumValue.name();
         }
      };
   }

   public static YoVariableTestGoal booleanEquals(final YoBoolean yoBoolean, final boolean booleanValue)
   {
      return new YoVariableTestGoal(yoBoolean)
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return yoBoolean.getBooleanValue() == booleanValue;
         }

         @Override
         public String toString()
         {
            return getFormattedBooleanYoVariable(yoBoolean) + " == " + booleanValue;
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
   
   public static YoVariableTestGoal timeInFuture(YoDouble timeYoVariable, double durationFromNow)
   {
      return doubleGreaterThan(timeYoVariable, timeYoVariable.getDoubleValue() + durationFromNow);
   }

   private static String getFormattedBooleanYoVariable(final YoBoolean yoBoolean)
   {
      return yoBoolean.getName() + ": " + yoBoolean.getBooleanValue();
   }

   private static <T extends Enum<T>> String getFormattedEnumYoVariable(final YoEnum<T> yoEnum)
   {
      String enumValueName = yoEnum.getEnumValue() != null ? yoEnum.getEnumValue().name() : "null";
      return yoEnum.getName() + ": " + enumValueName;
   }

   public static String getFormattedDoubleYoVariable(YoDouble yoDouble)
   {
      return yoDouble.getName() + ": "
            + FormattingTools.getFormattedToSignificantFigures(yoDouble.getDoubleValue(), SIGNIFICANT_FIGURES_FOR_PRINT_OUT);
   }
}
