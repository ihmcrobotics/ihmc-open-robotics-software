package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public interface FrameBasedCommand<M extends Settable<M>>
{
   /**
    * Sets this command from the given message.
    * <p>
    * The default implementation is a redirection to
    * {@link #set(ReferenceFrameHashCodeResolver, Settable)} with the resolver being {@code null}.
    * </p>
    * 
    * @param message the message to set this command with.
    */
   public default void setFromMessage(M message)
   {
      set(null, message);
   }

   /**
    * Sets this command from the given message using the resolver to retrieve frame information.
    * <p>
    * The resolver may be {@code null}.
    * </p>
    * 
    * @param resolver the map to use for retrieving the frames. May be {@code null}.
    * @param message the message to set this command with.
    */
   public void set(ReferenceFrameHashCodeResolver resolver, M message);
}
