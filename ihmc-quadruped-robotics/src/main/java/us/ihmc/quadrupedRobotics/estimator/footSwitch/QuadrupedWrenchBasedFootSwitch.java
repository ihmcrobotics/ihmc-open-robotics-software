package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class QuadrupedWrenchBasedFootSwitch implements FootSwitchInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final WrenchCalculatorWrapper wrenchCalculator;
   private final ReferenceFrame measurementFrame;

   private final YoDouble forceThreshold;

   private final YoFrameVector3D yoMeasuredForceWorld;
   private final YoBoolean hasFootHitGround;

   private final double totalRobotWeight;

   public QuadrupedWrenchBasedFootSwitch(WrenchCalculatorWrapper wrenchCalculator, ContactablePlaneBody contactablePlaneBody, double totalRobotWeight, YoVariableRegistry registry)
   {
      this.wrenchCalculator = wrenchCalculator;
      this.totalRobotWeight = totalRobotWeight;
      measurementFrame = contactablePlaneBody.getSoleFrame();
      String name = contactablePlaneBody.getName();
      forceThreshold = new YoDouble(name + "ForceThreshold", registry);
      forceThreshold.set(0.04 * totalRobotWeight);

      yoMeasuredForceWorld = new YoFrameVector3D(name + "MeasuredForceWorld", worldFrame, registry);
      hasFootHitGround = new YoBoolean(name + "HasFootHitGround", registry);
   }

   public void setForceThreshold(double threshold)
   {
      forceThreshold.set(threshold);
   }

   private void updateMeasurement()
   {
      wrenchCalculator.calculate();
      yoMeasuredForceWorld.setMatchingFrame(wrenchCalculator.getWrench().getLinearPart());
   }

   @Override
   public boolean hasFootHitGround()
   {
      updateMeasurement();
      hasFootHitGround.set(Math.abs(yoMeasuredForceWorld.getZ()) > forceThreshold.getDoubleValue());
      return hasFootHitGround.getBooleanValue();
   }

   @Override
   public double computeFootLoadPercentage()
   {
      updateMeasurement();
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
      updateMeasurement();
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
      updateMeasurement();
      hasFootHitGround.set(Math.abs(yoMeasuredForceWorld.getZ()) > forceThreshold.getDoubleValue());
      return hasFootHitGround.getBooleanValue();
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
   }

   @Override
   public void trustFootSwitch(boolean trustFootSwitch)
   {
   }
}
