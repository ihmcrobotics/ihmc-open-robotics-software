package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * This is a parameterized implementation of the {@link JointAccelerationIntegrationParametersReadOnly} interface.
 * <p>
 * Each parameter will be represented by a {@link DoubleParameter}. Default values can be provided at construction time.
 * After that the only way to set these parameters is through a tuning application. If a parameter file is loaded that
 * defines the parameters created by this class the default values will be overwritten.
 * </p>
 */
public class TunableJointAccelerationIntegrationParameters implements JointAccelerationIntegrationParametersReadOnly
{
   private static final double SUGGESTED_MAXIMUM_POSITION_ERROR = 2.0 * Math.PI;
   private static final double SUGGESTED_MAXIMUM_VELOCITY = SUGGESTED_MAXIMUM_POSITION_ERROR / 0.1;

   private final DoubleParameter alphaPosition;
   private final DoubleParameter alphaVelocity;
   private final DoubleParameter maxPositionError;
   private final DoubleParameter maxVelocity;

   /**
    * Creates a new sets of parameters for acceleration integration.
    * <p>
    * If the parameters can not be loaded from file they will be initialized with the default values from the {@link JointAccelerationIntegrationCalculator}.
    * </p>
    *
    * @param namePrefix the {@code String} to be prepended to each {@code YoVariable} of this class.
    * @param registry the registry to which the {@code YoVariable}s of this class are registered to.
    */
   public TunableJointAccelerationIntegrationParameters(String namePrefix, YoVariableRegistry registry)
   {
      alphaPosition = new DoubleParameter(namePrefix + "AlphaPosition", registry, JointAccelerationIntegrationCalculator.DEFAULT_ALPHA_POSITION, 0.0, 1.0);
      alphaVelocity = new DoubleParameter(namePrefix + "AlphaVelocity", registry, JointAccelerationIntegrationCalculator.DEFAULT_ALPHA_VELOCITY, 0.0, 1.0);
      maxPositionError = new DoubleParameter(namePrefix + "MaxPositionError", registry, JointAccelerationIntegrationCalculator.DEFAULT_MAX_POSITION_ERROR, 0.0, SUGGESTED_MAXIMUM_POSITION_ERROR);
      maxVelocity = new DoubleParameter(namePrefix + "MaxVelocity", registry, JointAccelerationIntegrationCalculator.DEFAULT_MAX_VELOCITY, 0.0, SUGGESTED_MAXIMUM_VELOCITY);
   }

   /**
    * Creates a new sets of parameters for acceleration integration.
    * <p>
    * If the parameters can not be loaded from file they will be initialized with the default values from the provided {@link JointAccelerationIntegrationParametersReadOnly}.
    * </p>
    *
    * @param namePrefix the {@code String} to be prepended to each {@code YoVariable} of this class.
    * @param registry the registry to which the {@code YoVariable}s of this class are registered to.
    * @param defaults the default values to be used if the parameters can not be loaded from file.
    */
   public TunableJointAccelerationIntegrationParameters(String namePrefix, YoVariableRegistry registry, JointAccelerationIntegrationParametersReadOnly defaults)
   {
      alphaPosition = new DoubleParameter(namePrefix + "AlphaPosition", registry, defaults.getAlphaPosition(), 0.0, 1.0);
      alphaVelocity = new DoubleParameter(namePrefix + "AlphaVelocity", registry, defaults.getAlphaVelocity(), 0.0, 1.0);
      maxPositionError = new DoubleParameter(namePrefix + "MaxPositionError", registry, defaults.getMaxPositionError(), 0.0, SUGGESTED_MAXIMUM_POSITION_ERROR);
      maxVelocity = new DoubleParameter(namePrefix + "MaxVelocity", registry, defaults.getMaxVelocity(), 0.0, SUGGESTED_MAXIMUM_VELOCITY);
   }

   /**
    * Creates a new sets of parameters for acceleration integration.
    * <p>
    * If the parameters can not be loaded from file they will be initialized with the provided default values.
    * </p>
    *
    * @param namePrefix the {@code String} to be prepended to each {@code YoVariable} of this class.
    * @param alphaPosition the default leak ratio used to compute the desired position, see {@link #getAlphaPosition()}.
    * @param alphaVelocity the default leak ratio used to compute the desired velocity, see {@link #getAlphaVelocity()}.
    * @param maxPositionError the default maximum position error used to saturate the desired position, see {@link #getMaxPositionError()}.
    * @param maxVelocity the maximum default desired velocity, see {@link #getMaxVelocity()}.
    * @param registry the registry to which the {@code YoVariable}s of this class are registered to.
    */
   public TunableJointAccelerationIntegrationParameters(String namePrefix, double alphaPosition, double alphaVelocity, double maxPositionError, double maxVelocity,
                                                   YoVariableRegistry registry)
   {
      this.alphaPosition = new DoubleParameter(namePrefix + "AlphaPosition", registry, alphaPosition, 0.0, 1.0);
      this.alphaVelocity = new DoubleParameter(namePrefix + "AlphaVelocity", registry, alphaVelocity, 0.0, 1.0);
      this.maxPositionError = new DoubleParameter(namePrefix + "MaxPositionError", registry, maxPositionError, 0.0, SUGGESTED_MAXIMUM_POSITION_ERROR);
      this.maxVelocity = new DoubleParameter(namePrefix + "MaxVelocity", registry, maxVelocity, 0.0, SUGGESTED_MAXIMUM_VELOCITY);
   }

   /** {@inheritDoc} */
   @Override
   public double getAlphaPosition()
   {
      return alphaPosition.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getAlphaVelocity()
   {
      return alphaVelocity.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxPositionError()
   {
      return maxPositionError.getValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxVelocity()
   {
      return maxVelocity.getValue();
   }
}
