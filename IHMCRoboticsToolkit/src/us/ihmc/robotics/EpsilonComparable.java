package us.ihmc.robotics;

public interface EpsilonComparable<T>
{
   public abstract boolean epsilonEquals(T other, double epsilon);
}
