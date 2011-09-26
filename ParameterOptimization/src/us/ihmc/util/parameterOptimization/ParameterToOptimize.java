package us.ihmc.util.parameterOptimization;

public interface ParameterToOptimize
{
   public abstract ParameterToOptimizeType getType();
   public abstract void setCurrentValueGivenZeroToOne(double zeroToOne);
}
