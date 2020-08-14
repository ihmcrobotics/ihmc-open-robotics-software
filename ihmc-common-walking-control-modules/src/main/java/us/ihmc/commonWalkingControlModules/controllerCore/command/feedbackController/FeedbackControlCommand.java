package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.interfaces.Settable;

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
 * @param <T> the final type of the command implementing this interface.
 */
public interface FeedbackControlCommand<T extends FeedbackControlCommand<T>> extends Settable<T>
{
   /**
    * Retrieves the type of this command.
    * <p>
    * This is used to identify the type of the command so it can be handled properly inside the
    * controller core.
    * </p>
    * 
    * @return the type of this command.
    */
   ControllerCoreCommandType getCommandType();

   /**
    * Sets an id for this command, it will be passed over when copied in the the controller core and
    * can be used to track control elements that are using this command.
    * 
    * @param id the new id for this command.
    */
   void setCommandId(int id);

   /**
    * Returns the id for this command.
    * 
    * @return this command's id.
    */
   int getCommandId();
}
