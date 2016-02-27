package us.ihmc.robotics;

public interface ComparableDataObject<T> 
{
   public boolean epsilonEquals(T other, double epsilon);
}
