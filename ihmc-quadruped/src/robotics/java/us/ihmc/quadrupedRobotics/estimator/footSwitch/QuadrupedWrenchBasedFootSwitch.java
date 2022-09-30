package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedWrenchBasedFootSwitch implements QuadrupedFootSwitchInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final WrenchCalculatorWrapper wrenchCalculator;
   private final ReferenceFrame measurementFrame;

   private final DoubleParameter forceThreshold;

   private final YoFrameVector3D yoMeasuredForceWorld;
   private final YoBoolean hasFootHitGround;

   private final double totalRobotWeight;

   public QuadrupedWrenchBasedFootSwitch(WrenchCalculatorWrapper wrenchCalculator,
                                         ContactablePlaneBody contactablePlaneBody,
                                         double totalRobotWeight,
                                         YoRegistry registry)
   {
      this.wrenchCalculator = wrenchCalculator;
      this.totalRobotWeight = totalRobotWeight;
      measurementFrame = contactablePlaneBody.getSoleFrame();
      String name = contactablePlaneBody.getName();
      forceThreshold = new DoubleParameter(name + "ForceThreshold", registry, 0.04 * totalRobotWeight);

      yoMeasuredForceWorld = new YoFrameVector3D(name + "MeasuredForceWorld", worldFrame, registry);
      hasFootHitGround = new YoBoolean(name + "HasFootHitGround", registry);
   }

   @Override
   public void update()
   {
      wrenchCalculator.calculate();
      yoMeasuredForceWorld.setMatchingFrame(wrenchCalculator.getWrench().getLinearPart());
   }

   @Override
   public boolean hasFootHitGroundSensitive()
   {
      hasFootHitGround.set(Math.abs(yoMeasuredForceWorld.getZ()) > forceThreshold.getValue());
      return hasFootHitGround.getBooleanValue();
   }

   @Override
   public double getFootLoadPercentage()
   {
      return Math.abs(yoMeasuredForceWorld.getZ()) / totalRobotWeight;
   }

   @Override
   public FramePoint2DReadOnly getCenterOfPressure()
   {
      return null;
   }

   @Override
   public WrenchReadOnly getMeasuredWrench()
   {
      return wrenchCalculator.getWrench();
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return measurementFrame;
   }

   @Override
   public void reset()
   {
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
   }

   @Override
   public void trustFootSwitchInSwing(boolean trustFootSwitch)
   {
   }

   @Override
   public void trustFootSwitchInSupport(boolean trustFootSwitch)
   {
   }
}
