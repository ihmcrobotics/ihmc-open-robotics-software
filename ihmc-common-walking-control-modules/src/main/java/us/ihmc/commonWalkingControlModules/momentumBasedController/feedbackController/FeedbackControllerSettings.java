package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

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
    * Gets a list from end-effector and/or joint name to break frequency that is to be used to setup
    * a low-pass filter on the velocity error computed for that end-effector or joint and how to
    * regroup the parameters.
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
    * @return the list of grouped parameters to use for setting up break frequency providers.
    */
   default List<GroupParameter<Double>> getErrorVelocityFilterBreakFrequencies()
   {
      return null;
   }

   /**
    * Gets the filter to apply to the error velocity of a 1-DoF joint.
    *
    * @param joint    the joint to which the filter is to be applied.
    * @param registry the registry to which the filter variables are to be added.
    * @return the filter to apply to the error velocity. If {@code null}, no filter will be applied.
    */
   default FilterDouble1D getVelocity1DErrorFilter(OneDoFJointReadOnly joint, YoRegistry registry)
   {
      return null;
   }

   /**
    * Gets the filter to apply to the error in angular velocity for an end-effector.
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    *                        Note that for the center of mass feedback controller, use the following {@code String}:
    *                        {@link FeedbackControllerToolbox#centerOfMassName}.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the filter to apply to the error velocity. If {@code null}, no filter will be applied.
    */
   default FilterVector3D getAngularVelocity3DErrorFilter(String endEffectorName, YoRegistry registry)
   {
      return null;
   }

   /**
    * Gets the filter to apply to the error in linear velocity for an end-effector.
    *
    * @param endEffectorName the name of the end-effector to which the filter is to be applied.
    *                        Note that for the center of mass feedback controller, use the following {@code String}:
    *                        {@link FeedbackControllerToolbox#centerOfMassName}.
    * @param registry        the registry to which the filter variables are to be added.
    * @return the filter to apply to the error velocity. If {@code null}, no filter will be applied.
    */
   default FilterVector3D getLinearVelocity3DErrorFilter(String endEffectorName, YoRegistry registry)
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
