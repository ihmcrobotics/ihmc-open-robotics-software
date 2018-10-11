package us.ihmc.manipulation.planning.gradientDescent;

import gnu.trove.list.array.TDoubleArrayList;

public interface SingleQueryFunction
{
   public abstract double getQuery(TDoubleArrayList values);
}