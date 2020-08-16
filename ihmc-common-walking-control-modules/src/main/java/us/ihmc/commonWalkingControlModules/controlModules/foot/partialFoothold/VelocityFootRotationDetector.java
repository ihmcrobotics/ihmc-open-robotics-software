package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class aims at detecting whether or not the foot is rotating. It uses that by integrating up the angular velocity of the foot in order to compute
 * the total angle of rotation.
 */
public class VelocityFootRotationDetector implements FootRotationDetector
{
   private final double dt;
   private final MovingReferenceFrame soleFrame;

   private final YoDouble integratedRotationAngle;
   private final YoDouble absoluteFootOmega;
   private final YoBoolean isRotating;

   private final DoubleProvider omegaThresholdForEstimation;
   private final DoubleProvider decayBreakFrequency;
   private final DoubleProvider rotationThreshold;

   public VelocityFootRotationDetector(RobotSide side,
                                       MovingReferenceFrame soleFrame,
                                       FootholdRotationParameters parameters,
                                       double dt,
                                       YoRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;
      this.dt = dt;

      String namePrefix = side.getLowerCaseName() + "Velocity";

      YoRegistry registry = new YoRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      parentRegistry.addChild(registry);

      omegaThresholdForEstimation = parameters.getOmegaMagnitudeThresholdForEstimation();
      decayBreakFrequency = parameters.getVelocityRotationAngleDecayBreakFrequency();
      rotationThreshold = parameters.getVelocityRotationAngleThreshold();

      integratedRotationAngle = new YoDouble(namePrefix + "IntegratedRotationAngle", registry);
      absoluteFootOmega = new YoDouble(namePrefix + "AbsoluteFootOmega", registry);
      isRotating = new YoBoolean(namePrefix + "IsRotating", registry);

      reset();
   }

   public void reset()
   {
      integratedRotationAngle.set(0.0);
      absoluteFootOmega.set(0.0);
      isRotating.set(false);
   }

   public boolean compute()
   {
      // Using the twist of the foot
      TwistReadOnly soleFrameTwist = soleFrame.getTwistOfFrame();
      absoluteFootOmega.set(soleFrameTwist.getAngularPart().length());
      if (absoluteFootOmega.getValue() > omegaThresholdForEstimation.getValue())
      {
         integratedRotationAngle.add(dt * absoluteFootOmega.getValue());
      }

      isRotating.set(integratedRotationAngle.getValue() > rotationThreshold.getValue());
      integratedRotationAngle.mul(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(decayBreakFrequency.getValue(), dt));

      return isRotating.getValue();
   }

   public boolean isRotating()
   {
      return isRotating.getBooleanValue();
   }
}
