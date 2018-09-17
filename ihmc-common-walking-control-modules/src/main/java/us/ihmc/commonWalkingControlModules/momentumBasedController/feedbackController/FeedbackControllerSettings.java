package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.yoVariables.providers.DoubleProvider;

public interface FeedbackControllerSettings
{
   /**
    * Whether to setup the feedback controllers for computing a feedback term proportional to the
    * position/orientation error integrated. IMPORTANT: This cannot be changed at runtime.
    * 
    * @return {@code true} if the variables to create the integral term are to be created,
    *         {@code false} otherwise.
    */
   default boolean enableIntegralTerm()
   {
      return true;
   }

   /**
    * Gets a map from end-effector and/or joint name to break frequency that is to be used to setup
    * a low-pass filter on the velocity error computed for that end-effector or joint.
    * <p>
    * If there is no break frequency for a end-effector or a joint, no filter will be setup and this
    * cannot be changed later on. If the returned map is {@code null}, no filters at all will be
    * setup, this cannot be changed later on.
    * </p>
    * <p>
    * Note: for the center of mass feedback controller, use the following {@code String} for the
    * key: {@link FeedbackControllerToolbox#centerOfMassName}.
    * </p>
    * 
    * @return the break frequency providers.
    */
   default Map<String, DoubleProvider> getErrorVelocityFilterBreakFrequencies()
   {
      return null;
   }

   /**
    * Creates an instance of {@code FeedbackControllerSettings} for a default setup of the feedback
    * controllers.
    * 
    * @return the default settings.
    */
   public static FeedbackControllerSettings getDefault()
   {
      return new FeedbackControllerSettings()
      {
      };
   }
}
