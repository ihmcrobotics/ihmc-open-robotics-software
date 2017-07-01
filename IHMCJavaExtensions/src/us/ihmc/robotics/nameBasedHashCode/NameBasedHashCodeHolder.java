package us.ihmc.robotics.nameBasedHashCode;

/**
 * {@code NameBasedHashCodeHolder} is the base interface for objects that are uniquely identifiable
 * by a hash code built from their name.
 * <p>
 * Please refer to {@link NameBasedHashCodeTools} for computing hash codes.
 * </p>
 * <p>
 * In certain context, using the standard hash code an be impractical for mapping two sets of
 * distinct instances of pairs of objects. Using a name based hash code can facilitate this mapping.
 * </p>
 * <p>
 * It can also be found useful for networking where the object can not be directly sent. This allows
 * to sent the reference to the object to be used.
 * </p>
 */
public interface NameBasedHashCodeHolder
{
   /**
    * Gets the value of this object's name based hash code.
    * <p>
    * This is a secondary unique hash code representing this object that is computed based on the
    * object unique name.
    * </p>
    * <p>
    * This hash code has the benefit of remaining the same when creating several instances of the
    * same object. It can be used to refer to a specific object instead of serializing and
    * deserializing it when communicating across a network.
    * </p>
    * 
    * @return this object's name based hash code.
    */
   public abstract long getNameBasedHashCode();
}
