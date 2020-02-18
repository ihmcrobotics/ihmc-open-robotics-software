package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

/**
 * This class computes whether a foot is rotating.</br>
 * If that is the case provides an estimate of the line of rotation. This class does not rely on a center of pressure
 * estimate and can be used with robots that do not have force-torque sensing in the feet.
 * <p>
 * The strategy employed is to use the measured twist of the foot to compute the current line of rotation. This requires
 * the angular velocity of the foot to be sufficiently large. A threshold determines if that is the case. The speed of
 * rotation is the integrated using a leak rate. If the integral which is a measure of absolute foot rotation exceeds
 * a second threshold the foot is assumed to rotate.]
 *
 * @author Georg Wiedebach
 */
public class NewKinematicFootRotationDetector
{
   private final YoVariableRegistry registry;

   private final double dt;
   private final MovingReferenceFrame soleFrame;

   private final YoDouble integratedRotationAngle;
   private final YoDouble absoluteFootOmega;
   private final YoBoolean isRotating;

   private final DoubleProvider omegaThresholdForRotating;
   private final DoubleProvider decayBreakFrequency;
   private final DoubleProvider rotationThreshold;

   private final KinematicsRotationEdgeCalculator edgeCalculator;

   public NewKinematicFootRotationDetector(RobotSide side, MovingReferenceFrame soleFrame, double dt, YoVariableRegistry parentRegistry,
                                           YoGraphicsListRegistry graphicsRegistry)
   {
      this.soleFrame = soleFrame;
      this.dt = dt;

      edgeCalculator = new KinematicsRotationEdgeCalculator(side, soleFrame, dt, parentRegistry, graphicsRegistry);

      registry = new YoVariableRegistry(getClass().getSimpleName() + side.getPascalCaseName());
      parentRegistry.addChild(registry);

      String feetManagerName = FeetManager.class.getSimpleName();
      String paramRegistryName = getClass().getSimpleName() + "Parameters";
      omegaThresholdForRotating = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "omegaThresholdForRotating", registry, 3.0);
      decayBreakFrequency = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "decayBreakFrequency", registry, 1.0);
      rotationThreshold = ParameterProvider.getOrCreateParameter(feetManagerName, paramRegistryName, "rotationThreshold", registry, 0.2);

      integratedRotationAngle = new YoDouble(side.getLowerCaseName() + "IntegratedRotationAngle", registry);
      absoluteFootOmega = new YoDouble(side.getLowerCaseName() + "AbsoluteFootOmega", registry);
      isRotating = new YoBoolean(side.getLowerCaseName() + "IsRotating", registry);

      reset();
   }

   public boolean isRotating()
   {
      return isRotating.getBooleanValue();
   }

   public double getAbsoluteFootOmega()
   {
      return absoluteFootOmega.getDoubleValue();
   }

   public boolean compute()
   {
      // Using the twist of the foot
      TwistReadOnly soleFrameTwist = soleFrame.getTwistOfFrame();
      double omegaSquared = soleFrameTwist.getAngularPart().lengthSquared();
      absoluteFootOmega.set(Math.sqrt(omegaSquared));
      if (absoluteFootOmega.getValue() > omegaThresholdForRotating.getValue())
      {
         edgeCalculator.compute(null);

         // FIXME this doesn't work well
         double omega = soleFrameTwist.getAngularPart().length();
         integratedRotationAngle.add(dt * omega);
      }
      else if (!isRotating.getValue())
      {
         edgeCalculator.reset();
      }

      if (!isRotating.getValue())
      {
         isRotating.set(integratedRotationAngle.getValue() > rotationThreshold.getValue());
      }
      integratedRotationAngle.mul(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(decayBreakFrequency.getValue(), dt));

      return isRotating.getValue();
   }

   public void reset()
   {
      integratedRotationAngle.set(0.0);
      absoluteFootOmega.set(0.0);
      isRotating.set(false);


   }
}
