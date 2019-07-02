package us.ihmc.sensorProcessing.outputData;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Tunable implementation of the {@link JointDesiredBehaviorReadOnly} interface. This
 * implementation uses the {@link YoParameter} class for the parameters. Default
 * values for the parameters can be provided at construction time.
 */
public class TunableJointDesiredBehavior implements JointDesiredBehaviorReadOnly
{
   private static final String CONTROL_MODE_NAME = "ControlMode";
   private static final String STIFFNESS_NAME = "Stiffness";
   private static final String DAMPING_NAME = "Damping";
   private static final String MASTER_GAIN_NAME = "MasterGain";
   private static final String VELOCITY_SCALING_NAME = "VelocityScaling";
   private static final String POSITION_FEEDBACK_MAX_ERROR_NAME = "PositionFeedbackMaxError";
   private static final String VELOCITY_FEEDBACK_MAX_ERROR_NAME = "VelocityFeedbackMaxError";

   private static final double SUGGESTED_MAXIMUM_POSITION_ERROR = 2.0 * Math.PI;
   private static final double SUGGESTED_MAXIMUM_VELOCITY = SUGGESTED_MAXIMUM_POSITION_ERROR / 0.1;

   private final EnumParameter<JointDesiredControlMode> controlMode;
   private final DoubleParameter stiffness;
   private final DoubleParameter damping;
   private final DoubleParameter masterGain;
   private final DoubleParameter velocityScaling;
   private final DoubleParameter maxPositionError;
   private final DoubleParameter maxVelocityError;

   public TunableJointDesiredBehavior(String namePrefix, YoVariableRegistry registry)
   {
      controlMode = new EnumParameter<>(namePrefix + CONTROL_MODE_NAME, registry, JointDesiredControlMode.class, false);
      stiffness = new DoubleParameter(namePrefix + STIFFNESS_NAME, registry, 0.0, 10.0);
      damping = new DoubleParameter(namePrefix + DAMPING_NAME, registry, 0.0, 10.0);
      masterGain = new DoubleParameter(namePrefix + MASTER_GAIN_NAME, registry, 0.0, 1.0);
      velocityScaling = new DoubleParameter(namePrefix + VELOCITY_SCALING_NAME, registry, 0.0, 1.0);
      maxPositionError = new DoubleParameter(namePrefix + POSITION_FEEDBACK_MAX_ERROR_NAME, registry, SUGGESTED_MAXIMUM_POSITION_ERROR, 0.0, Double.POSITIVE_INFINITY);
      maxVelocityError = new DoubleParameter(namePrefix + VELOCITY_FEEDBACK_MAX_ERROR_NAME, registry, SUGGESTED_MAXIMUM_VELOCITY, 0.0, Double.POSITIVE_INFINITY);
   }

   public TunableJointDesiredBehavior(String namePrefix, JointDesiredBehaviorReadOnly other, YoVariableRegistry registry)
   {
      controlMode = new EnumParameter<>(namePrefix + CONTROL_MODE_NAME, registry, JointDesiredControlMode.class, false, other.getControlMode());
      stiffness = new DoubleParameter(namePrefix + STIFFNESS_NAME, registry, other.getStiffness(), 0.0, 10.0);
      damping = new DoubleParameter(namePrefix + DAMPING_NAME, registry, other.getDamping(), 0.0, 10.0);
      masterGain = new DoubleParameter(namePrefix + MASTER_GAIN_NAME, registry, other.getMasterGain(), 0.0, 1.0);
      velocityScaling = new DoubleParameter(namePrefix + VELOCITY_SCALING_NAME, registry, other.getVelocityScaling(), 0.0, 1.0);
      maxPositionError = new DoubleParameter(namePrefix + POSITION_FEEDBACK_MAX_ERROR_NAME, registry, other.getMaxPositionError(), 0.0, Double.POSITIVE_INFINITY);
      maxVelocityError = new DoubleParameter(namePrefix + VELOCITY_FEEDBACK_MAX_ERROR_NAME, registry, other.getMaxVelocityError(), 0.0, Double.POSITIVE_INFINITY);
   }

   @Override
   public JointDesiredControlMode getControlMode()
   {
      return controlMode.getValue();
   }

   @Override
   public double getStiffness()
   {
      return stiffness.getValue();
   }

   @Override
   public double getDamping()
   {
      return damping.getValue();
   }

   @Override
   public double getMasterGain()
   {
      return masterGain.getValue();
   }

   @Override
   public double getVelocityScaling()
   {
      return velocityScaling.getValue();
   }

   @Override
   public double getMaxPositionError()
   {
      return maxPositionError.getValue();
   }

   @Override
   public double getMaxVelocityError()
   {
      return maxVelocityError.getValue();
   }
}
