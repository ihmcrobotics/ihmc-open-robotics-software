package us.ihmc.commonWalkingControlModules.controllerCore.data;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.yoVariables.providers.BooleanProvider;

/**
 * Base interface for data types used by the feedback controllers.
 * <p>
 * Implementations use {@code ReferenceFrame} and {@code YoVariable} frameworks and provides extra
 * API directly related to their usage.
 * </p>
 */
public interface FeedbackControllerData
{
   /**
    * Registers an additional flag used to identify whether this data type is currently being used and
    * thus if the data it contains is actually relevant.
    * 
    * @param activeFlag used to indicate activity of this data type: {@code true} for active meaning
    *                   this data type contains relevant information, {@code false} for inactive and
    *                   thus this data type is not being updated and contains outdated information.
    */
   void addActiveFlag(BooleanProvider activeFlag);

   /**
    * Informs whether this data type is actively maintained and thus whether is contains relevant
    * information.
    * 
    * @return {@code true} if this data type contains information that is up-to-date, {@code false}
    *         otherwise.
    */
   boolean isActive();

   /**
    * Clears the internal data, i.e. typically by filling it with {@link Double#NaN}s, if this data
    * type is not active.
    * 
    * @return {@code true} if this has been cleared, {@code false} if it was not cleared.
    */
   boolean clearIfInactive();

   /**
    * Returns the command identifier that was used in the feedback controller that populated this data
    * type.
    * <p>
    * The command id is obtained from {@link FeedbackControlCommand#getCommandId()} and can be set
    * before submitting a command to the controller core.
    * </p>
    * 
    * @return the identifier of the command that was being processed when updating this data type.
    */
   int getCommandId();

   static String createNamePrefix(String namePrefix, Type type, SpaceData3D space)
   {
      return namePrefix + type.getName() + space.getName();
   }
}
