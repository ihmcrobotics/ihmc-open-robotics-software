package us.ihmc.ihmcPerception.depthData.collisionShapes;

import us.ihmc.robotics.MathTools;

public enum ConstraintType
{
   EQUALITY, INEQUALITY, STRICT_INEQUALITY;

   public String toString()
   {
      switch (this)
      {
      case EQUALITY:
         return "Equality";
      case INEQUALITY:
         return "Inequality";
      default:
         return "Strict Inequality";
      }
   }

   public String getMathOperator()
   {
      switch (this)
      {
      case EQUALITY:
         return " = ";
      case INEQUALITY:
         return " <= ";
      default:
         return " < ";
      }
   }

   public boolean applyMathOperator(double leftHandSide, double rightHandSide, double epsilon)
   {
      switch (this)
      {
      case EQUALITY:
         return MathTools.epsilonEquals(leftHandSide, rightHandSide, epsilon);
      case INEQUALITY:
         return MathTools.roundToPrecision(leftHandSide, epsilon) <= MathTools.roundToPrecision(rightHandSide, epsilon);
      default:
         return MathTools.roundToPrecision(leftHandSide, epsilon) < MathTools.roundToPrecision(rightHandSide, epsilon);
      }
   }

   public boolean applyMathOperator(double leftMinusRightSide, double epsilon)
   {
      return applyMathOperator(leftMinusRightSide, 0.0, epsilon);
   }

}
