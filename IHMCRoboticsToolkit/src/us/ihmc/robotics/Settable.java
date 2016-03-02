package us.ihmc.robotics;

public interface Settable<T> extends EpsilonComparable<T>
{
   public abstract void set(T other);

   public abstract void setToZero();

   public abstract void setToNaN();

   public abstract boolean containsNaN();
}
