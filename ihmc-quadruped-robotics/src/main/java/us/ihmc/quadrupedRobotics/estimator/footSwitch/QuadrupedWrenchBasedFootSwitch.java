package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class QuadrupedWrenchBasedFootSwitch implements FootSwitchInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final WrenchCalculatorWrapper wrenchCalculator;
   private final ReferenceFrame measurementFrame;

   private final DoubleParameter forceThreshold;

   private final YoFrameVector3D yoMeasuredForceWorld;
   private final YoBoolean hasFootHitGround;

   private final double totalRobotWeight;

   public QuadrupedWrenchBasedFootSwitch(WrenchCalculatorWrapper wrenchCalculator, ContactablePlaneBody contactablePlaneBody, double totalRobotWeight, YoVariableRegistry registry)
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
   public void updateMeasurement()
   {
      wrenchCalculator.calculate();
      yoMeasuredForceWorld.setMatchingFrame(wrenchCalculator.getWrench().getLinearPart());
   }

   @Override
   public boolean hasFootHitGround()
   {
      hasFootHitGround.set(Math.abs(yoMeasuredForceWorld.getZ()) > forceThreshold.getValue());
      return hasFootHitGround.getBooleanValue();
   }

   @Override
   public double computeFootLoadPercentage()
   {
      return Math.abs(yoMeasuredForceWorld.getZ()) / totalRobotWeight;
   }

   @Override
   public void computeAndPackCoP(FramePoint2D copToPack)
   {
      copToPack.setToNaN(getMeasurementFrame());
   }

   @Override
   public void updateCoP()
   {
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      footWrenchToPack.setIncludingFrame(wrenchCalculator.getWrench());
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
   public boolean getForceMagnitudePastThreshhold()
   {
      hasFootHitGround.set(Math.abs(yoMeasuredForceWorld.getZ()) > forceThreshold.getValue());
      return hasFootHitGround.getBooleanValue();
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
