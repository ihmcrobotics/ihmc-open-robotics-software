package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

/**
 * Implementations of this interface provides the API of the feedback control module of the
 * controller core.
 * <p>
 * <ul>
 * <li>See the feedback control module: {@link WholeBodyFeedbackController}.
 * <li>See the controller core: {@link WholeBodyControllerCore}.
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 * @param <T> the final type of the command implementing this interface.
 */
public interface FeedbackControlCommand<T extends FeedbackControlCommand<T>>
{
   /**
    * Sets this command to {@code other}.
    * 
    * @param other the other command to copy the data from. Not modified.
    */
   public abstract void set(T other);

   /**
    * Retrieves the type of this command.
    * <p>
    * This is used to identify of the command is be handled inside the controller core.
    * </p>
    * 
    * @return the type of this command.
    */
   public abstract ControllerCoreCommandType getCommandType();
}
