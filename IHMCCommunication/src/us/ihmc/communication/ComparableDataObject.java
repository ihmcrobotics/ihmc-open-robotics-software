package us.ihmc.communication;

public interface ComparableDataObject<T> 
{
   public boolean epsilonEquals(T other, double epsilon);
}
