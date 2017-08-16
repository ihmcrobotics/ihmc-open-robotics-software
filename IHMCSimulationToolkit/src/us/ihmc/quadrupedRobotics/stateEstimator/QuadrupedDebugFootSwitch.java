package us.ihmc.quadrupedRobotics.stateEstimator;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

public class QuadrupedDebugFootSwitch implements FootSwitchInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final WrenchCalculatorInterface wrenchCalculatorInterface;
   private final ContactablePlaneBody contactablePlaneBody;
   private final ReferenceFrame measurementFrame;

   private final YoDouble forceThreshold;

   private final FrameVector3D measuredForce = new FrameVector3D();
   private final FrameVector3D measuredForceWorld = new FrameVector3D();
   private final YoFrameVector yoMeasuredForceWorld;
   private final YoBoolean hasFootHitGround;

   private final double totalRobotWeight;

   public QuadrupedDebugFootSwitch(WrenchCalculatorInterface wrenchCalculatorInterface, ContactablePlaneBody contactablePlaneBody, double totalRobotWeight, YoVariableRegistry registry)
   {
      this.wrenchCalculatorInterface = wrenchCalculatorInterface;
      this.contactablePlaneBody = contactablePlaneBody;
      this.totalRobotWeight = totalRobotWeight;
      measurementFrame = contactablePlaneBody.getSoleFrame();
      forceThreshold = new YoDouble(contactablePlaneBody.getName() + "ForceThreshold", registry);
      forceThreshold.set(0.04 * totalRobotWeight);
      
      yoMeasuredForceWorld = new YoFrameVector(contactablePlaneBody.getName() + "MeasuredForceWorld", worldFrame, registry);
      hasFootHitGround = new YoBoolean(contactablePlaneBody.getName() + "HasFootHitGround", registry);
   }

   public void setForceThreshold(double threshold)
   {
      forceThreshold.set(threshold);
   }

   private void updateMeasurement()
   {
      wrenchCalculatorInterface.calculate();
      MatrixTools.extractFrameTupleFromEJMLVector(measuredForce, wrenchCalculatorInterface.getWrench(), measurementFrame, 3);
      measuredForceWorld.setIncludingFrame(measuredForce);
      measuredForceWorld.changeFrame(worldFrame);
      yoMeasuredForceWorld.setAndMatchFrame(measuredForce);
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
      ReferenceFrame bodyFixedFrame = contactablePlaneBody.getRigidBody().getBodyFixedFrame();
      footWrenchToPack.setToZero(bodyFixedFrame, measurementFrame);
      footWrenchToPack.setLinearPart(measuredForce);
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
