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
   private final EnumParameter<JointDesiredControlMode> controlMode;
   private final DoubleParameter stiffness;
   private final DoubleParameter damping;
   private final DoubleParameter masterGain;
   private final DoubleParameter velocityScaling;
   private final DoubleParameter maxPositionError;
   private final DoubleParameter maxVelocityError;

   public TunableJointDesiredBehavior(String namePrefix, YoVariableRegistry registry)
   {
      controlMode = new EnumParameter<>(namePrefix + "ControlMode", registry, JointDesiredControlMode.class, false);
      stiffness = new DoubleParameter(namePrefix + "Stiffness", registry, 0.0, 10.0);
      damping = new DoubleParameter(namePrefix + "Damping", registry, 0.0, 10.0);
      masterGain = new DoubleParameter(namePrefix + "MasterGain", registry, 0.0, 1.0);
      velocityScaling = new DoubleParameter(namePrefix + "VelocityScaling", registry, 0.0, 1.0);
      maxPositionError = new DoubleParameter(namePrefix + "MaxPositionError", registry, Double.POSITIVE_INFINITY, 0.0, Double.POSITIVE_INFINITY);
      maxVelocityError = new DoubleParameter(namePrefix + "MaxVelocityError", registry, Double.POSITIVE_INFINITY, 0.0, Double.POSITIVE_INFINITY);
   }

   public TunableJointDesiredBehavior(String namePrefix, JointDesiredBehaviorReadOnly other, YoVariableRegistry registry)
   {
      controlMode = new EnumParameter<>(namePrefix + "ControlMode", registry, JointDesiredControlMode.class, false, other.getControlMode());
      stiffness = new DoubleParameter(namePrefix + "Stiffness", registry, other.getStiffness(), 0.0, 10.0);
      damping = new DoubleParameter(namePrefix + "Damping", registry, other.getDamping(), 0.0, 10.0);
      masterGain = new DoubleParameter(namePrefix + "MasterGain", registry, other.getMasterGain(), 0.0, 1.0);
      velocityScaling = new DoubleParameter(namePrefix + "VelocityScaling", registry, other.getVelocityScaling(), 0.0, 1.0);
      maxPositionError = new DoubleParameter(namePrefix + "PositionFeedbackMaxError", registry, other.getMaxPositionError(), 0.0, Double.POSITIVE_INFINITY);
      maxVelocityError = new DoubleParameter(namePrefix + "VelocityFeedbackMaxError", registry, other.getMaxVelocityError(), 0.0, Double.POSITIVE_INFINITY);
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
