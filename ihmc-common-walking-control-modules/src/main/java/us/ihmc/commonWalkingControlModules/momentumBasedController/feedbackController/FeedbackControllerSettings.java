package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface FeedbackControllerSettings
{
   /**
    * Whether to setup the feedback controllers for computing a feedback term proportional to the
    * position/orientation error integrated. IMPORTANT: This cannot be changed at runtime.
    *
    * @return {@code true} if the variables to create the integral term are to be created,
    *       {@code false} otherwise.
    */
   default boolean enableIntegralTerm()
   {
      return true;
   }

   /**
    * Gets the filter to apply to the error velocity of a 1-DoF joint.
    * <p>
    * Look at {@link FeedbackControllerFilterFactory} for default filter implementations.
    * </p>
    *
    * @param jointName the name of the 1-DoF joint to which the filter is to be applied.
    * @param dt        the time step of the controller.
    * @param registry  the registry to which the filter variables are to be added.
    * @return the filter to apply to the error velocity. If {@code null}, no filter will be applied.
    */
   default FilterDouble1D getVelocity1DErrorFilter(String jointName, double dt, YoRegistry registry)
   {
      return null;
   }

   /**
    * Gets the filter to apply to the error in angular velocity for an end-effector.
    * <p>
    * Look at {@link FeedbackControllerFilterFactory} for default filter implementations.
    * </p>
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    *                        Note that for the center of mass feedback controller, use the following {@code String}:
    *                        {@link FeedbackControllerToolbox#centerOfMassName}.
    * @param dt              the time step of the controller.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the filter to apply to the error velocity. If {@code null}, no filter will be applied.
    */
   default FilterVector3D getAngularVelocity3DErrorFilter(String endEffectorName, double dt, YoRegistry registry)
   {
      return null;
   }

   /**
    * Gets the filter to apply to the error in linear velocity for an end-effector.
    * <p>
    * Look at {@link FeedbackControllerFilterFactory} for default filter implementations.
    * </p>
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    *                        Note that for the center of mass feedback controller, use the following {@code String}:
    *                        {@link FeedbackControllerToolbox#centerOfMassName}.
    * @param dt              the time step of the controller.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the filter to apply to the error velocity. If {@code null}, no filter will be applied.
    */
   default FilterVector3D getLinearVelocity3DErrorFilter(String endEffectorName, double dt, YoRegistry registry)
   {
      return null;
   }

   /**
    * Creates an instance of {@code FeedbackControllerSettings} for a default setup of the feedback
    * controllers.
    *
    * @return the default settings.
    */
   static FeedbackControllerSettings getDefault()
   {
      return new FeedbackControllerSettings()
      {
      };
   }

   /**
    * Interface used to filter a 1D signal.
    */
   interface FilterDouble1D
   {
      /**
       * Reset the filter.
       */
      default void reset()
      {
      }

      /**
       * Apply the filter to the input.
       *
       * @param input1D the input to filter.
       * @return the filtered input.
       */
      double apply(double input1D);
   }

   /**
    * Interface used to filter a 3D vector.
    */
   interface FilterVector3D
   {
      /**
       * Reset the filter.
       */
      default void reset()
      {
      }

      /**
       * Apply the filter to the input vector.
       *
       * @param inputVector  the input to filter. Not modified.
       * @param outputVector the filtered input. Modified.
       */
      void apply(FrameVector3DReadOnly inputVector, FixedFrameVector3DBasics outputVector);
   }
}
