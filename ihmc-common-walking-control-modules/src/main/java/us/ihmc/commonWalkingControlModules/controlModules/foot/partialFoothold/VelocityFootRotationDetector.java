package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class VelocityFootRotationDetector implements FootRotationDetector
{
   private final YoVariableRegistry registry;

   private final double dt;
   private final MovingReferenceFrame soleFrame;

   private final YoDouble integratedRotationAngle;
   private final YoDouble absoluteFootOmega;
   private final YoBoolean isRotating;

   private final DoubleProvider omegaThresholdForEstimation;
   private final DoubleProvider decayBreakFrequency;
   private final DoubleProvider rotationThreshold;

   public VelocityFootRotationDetector(RobotSide side, MovingReferenceFrame soleFrame, double dt, YoVariableRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;
      this.dt = dt;

      registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      parentRegistry.addChild(registry);

      String feetManagerName = FeetManager.class.getSimpleName();
      String paramRegistryName = getClass().getSimpleName() + "Parameters";
      omegaThresholdForEstimation = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "omegaThresholdForEstimation", registry, 3.0);
      decayBreakFrequency = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "decayBreakFrequency", registry, 1.0);
      rotationThreshold = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "rotationThreshold", registry, 0.2);

      integratedRotationAngle = new YoDouble(side.getLowerCaseName() + "IntegratedRotationAngle", registry);
      absoluteFootOmega = new YoDouble(side.getLowerCaseName() + "AbsoluteFootOmega", registry);
      isRotating = new YoBoolean(side.getLowerCaseName() + "IsRotating", registry);

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
      double omegaSquared = soleFrameTwist.getAngularPart().lengthSquared();
      absoluteFootOmega.set(Math.sqrt(omegaSquared));
      if (absoluteFootOmega.getValue() > omegaThresholdForEstimation.getValue())
      {
         double omega = Math.sqrt(omegaSquared);
         integratedRotationAngle.add(dt * omega);
      }

      if (!isRotating.getValue())
      {
         isRotating.set(integratedRotationAngle.getValue() > rotationThreshold.getValue());
      }
      integratedRotationAngle.mul(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(decayBreakFrequency.getValue(), dt));

      return isRotating.getValue();
   }

   public boolean isRotating()
   {
      return isRotating.getBooleanValue();
   }
}
