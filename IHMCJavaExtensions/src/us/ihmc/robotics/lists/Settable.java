package us.ihmc.robotics.lists;

/**
 * Base interface for any object that is that is settable with other objects of its own type.
 *
 * @param <T> the final type of the implementation of this interface.
 */
public interface Settable<T>
{
   /**
    * Copies the values from {@code other} into this object.
    *
    * @param other the other object to copy the values from. Not modified.
    */
   void set(T other);
}
