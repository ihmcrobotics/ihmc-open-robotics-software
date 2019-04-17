package us.ihmc.robotics.numericalMethods;

import gnu.trove.list.array.TDoubleArrayList;

public interface SingleQueryFunction
{
   public abstract double getQuery(TDoubleArrayList values);
}