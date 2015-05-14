package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.yoUtilities.math.filters.DeadzoneYoFrameVector;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class HandCompliantControlHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final BooleanYoVariable[] doCompliantControlLinear;
   private final BooleanYoVariable[] doCompliantControlAngular;
   private final DoubleYoVariable forceDeadzoneSize;
   private final DoubleYoVariable torqueDeadzoneSize;
   private final DeadzoneYoFrameVector deadzoneMeasuredForce;
   private final DeadzoneYoFrameVector deadzoneMeasuredTorque;
   private final DoubleYoVariable measuredForceAlpha;
   private final AlphaFilteredYoFrameVector filteredMeasuredForce;
   private final AlphaFilteredYoFrameVector filteredMeasuredTorque;
   private final YoFrameVector measuredForceAtControlFrame;
   private final YoFrameVector measuredTorqueAtControlFrame;

   private final YoFrameVector desiredForce;
   private final YoFrameVector desiredTorque;

   private final FrameVector totalLinearCorrection = new FrameVector();
   private final FrameVector totalAngularCorrection = new FrameVector();
   private final FrameVector linearCorrection = new FrameVector();
   private final FrameVector angularCorrection = new FrameVector();

   private final DoubleYoVariable linearGain;
   private final DoubleYoVariable angularGain;
   private final DoubleYoVariable compliantControlMaxLinearCorrectionPerTick;
   private final DoubleYoVariable compliantControlMaxLinearDisplacement;
   private final DoubleYoVariable compliantControlMaxAngularCorrectionPerTick;
   private final DoubleYoVariable compliantControlMaxAngularDisplacement;
   private final DoubleYoVariable compliantControlLeakRatio;
   private final YoFrameVector yoCompliantControlLinearDisplacement;
   private final YoFrameVector yoCompliantControlAngularDisplacement;

   private ReferenceFrame controlFrame;
   private final Wrench measuredWrench = new Wrench();
   private final FrameVector measuredForce = new FrameVector();
   private final FrameVector measuredTorque = new FrameVector();
   private final FrameVector errorForce = new FrameVector();
   private final FrameVector errorTorque = new FrameVector();

   private final RobotSide robotSide;
   private final MomentumBasedController momentumBasedController;

   public HandCompliantControlHelper(String namePrefix, RobotSide robotSide, MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + "CompliantControl");
      parentRegistry.addChild(registry);

      this.robotSide = robotSide;
      this.momentumBasedController = momentumBasedController;

      ReferenceFrame forceSensorMeasurementFrame = momentumBasedController.getWristForceSensor(robotSide).getMeasurementFrame();

      forceDeadzoneSize = new DoubleYoVariable(namePrefix + "ForceDeadzoneSize", registry);
      deadzoneMeasuredForce = DeadzoneYoFrameVector.createDeadzoneYoFrameVector(namePrefix + "DeadzoneMeasuredForce", registry, forceDeadzoneSize, forceSensorMeasurementFrame);
      torqueDeadzoneSize = new DoubleYoVariable(namePrefix + "TorqueDeadzoneSize", registry);
      deadzoneMeasuredTorque = DeadzoneYoFrameVector.createDeadzoneYoFrameVector(namePrefix + "DeadzoneMeasuredTorque", registry, torqueDeadzoneSize, forceSensorMeasurementFrame);
      measuredForceAlpha = new DoubleYoVariable(namePrefix + "MeasuredForceAlpha", registry);
      filteredMeasuredForce = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "FilteredMeasuredForce", "", registry, measuredForceAlpha, deadzoneMeasuredForce);
      filteredMeasuredTorque = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "FilteredMeasuredTorque", "", registry, measuredForceAlpha, deadzoneMeasuredTorque);

      measuredForceAtControlFrame = new YoFrameVector(namePrefix + "MeasuredForceAtControlFrame", null, registry);
      measuredTorqueAtControlFrame = new YoFrameVector(namePrefix + "MeasuredTorqueAtControlFrame", null, registry);

      desiredForce = new YoFrameVector(namePrefix + "DesiredForce", null, registry);
      desiredTorque = new YoFrameVector(namePrefix + "DesiredTorque", null, registry);

      doCompliantControlLinear = new BooleanYoVariable[3];
      doCompliantControlLinear[0] = new BooleanYoVariable(namePrefix + "DoCompliantControlLinearX", registry);
      doCompliantControlLinear[1] = new BooleanYoVariable(namePrefix + "DoCompliantControlLinearY", registry);
      doCompliantControlLinear[2] = new BooleanYoVariable(namePrefix + "DoCompliantControlLinearZ", registry);

      doCompliantControlAngular = new BooleanYoVariable[3];
      doCompliantControlAngular[0] = new BooleanYoVariable(namePrefix + "DoCompliantControlAngularX", registry);
      doCompliantControlAngular[1] = new BooleanYoVariable(namePrefix + "DoCompliantControlAngularY", registry);
      doCompliantControlAngular[2] = new BooleanYoVariable(namePrefix + "DoCompliantControlAngularZ", registry);

      for (int i = 0; i < 3; i++)
      {
         doCompliantControlLinear[i].set(true);
//         doCompliantControlAngular[i].set(true);
      }

      linearGain = new DoubleYoVariable(namePrefix + "CompliantControlLinearGain", registry);
      angularGain = new DoubleYoVariable(namePrefix + "CompliantControlAngularGain", registry);
      compliantControlMaxLinearCorrectionPerTick = new DoubleYoVariable(namePrefix + "CompliantControlMaxLinearCorrectionPerTick", registry);
      compliantControlMaxAngularCorrectionPerTick = new DoubleYoVariable(namePrefix + "CompliantControlMaxAngularCorrectionPerTick", registry);
      compliantControlMaxLinearDisplacement = new DoubleYoVariable(namePrefix + "CompliantControlMaxLinearDisplacement", registry);
      compliantControlMaxAngularDisplacement = new DoubleYoVariable(namePrefix + "CompliantControlMaxAngularDisplacement", registry);
      compliantControlLeakRatio = new DoubleYoVariable(namePrefix + "CompliantControlLeakRatio", registry);
      yoCompliantControlLinearDisplacement = new YoFrameVector(namePrefix + "CompliantControlLinearDisplacement", worldFrame, registry);
      yoCompliantControlAngularDisplacement = new YoFrameVector(namePrefix + "CompliantControlAngularDisplacement", worldFrame, registry);

      measuredForceAlpha.set(0.98);
      linearGain.set(0.0001);
      angularGain.set(0.001);
      compliantControlMaxLinearCorrectionPerTick.set(0.0005);
      compliantControlMaxAngularCorrectionPerTick.set(0.001);
      compliantControlMaxLinearDisplacement.set(0.05);
      compliantControlMaxAngularDisplacement.set(0.17);
      compliantControlLeakRatio.set(0.99);
   }

   public void doCompliantControl(FramePoint desiredPosition, FrameOrientation desiredOrientation)
   {
      updateWristMeasuredWrench(measuredForce, measuredTorque);

      yoCompliantControlLinearDisplacement.getFrameTupleIncludingFrame(totalLinearCorrection);
      yoCompliantControlAngularDisplacement.getFrameTupleIncludingFrame(totalAngularCorrection);

      measuredForce.changeFrame(controlFrame);
      measuredTorque.changeFrame(controlFrame);
      totalLinearCorrection.changeFrame(controlFrame);
      totalAngularCorrection.changeFrame(controlFrame);
      linearCorrection.setToZero(controlFrame);
      angularCorrection.setToZero(controlFrame);

      measuredForceAtControlFrame.set(measuredForce.getVector());
      measuredTorqueAtControlFrame.set(measuredTorque.getVector());

      errorForce.setIncludingFrame(controlFrame, desiredForce.getX(), desiredForce.getY(), desiredForce.getZ());
      errorForce.sub(measuredForce);
      errorTorque.setIncludingFrame(controlFrame, desiredTorque.getX(), desiredTorque.getY(), desiredTorque.getZ());
      errorTorque.sub(measuredTorque);

      linearCorrection.scale(linearGain.getDoubleValue(), errorForce);
      angularCorrection.scale(angularGain.getDoubleValue(), errorTorque);

      clipToVectorMagnitude(compliantControlMaxLinearCorrectionPerTick.getDoubleValue(), linearCorrection);
      clipToVectorMagnitude(compliantControlMaxAngularCorrectionPerTick.getDoubleValue(), angularCorrection);

      totalLinearCorrection.scale(compliantControlLeakRatio.getDoubleValue());
      totalAngularCorrection.scale(compliantControlLeakRatio.getDoubleValue());

      totalLinearCorrection.add(linearCorrection);
      totalAngularCorrection.add(angularCorrection);

      if (!doCompliantControlLinear[0].getBooleanValue())
         totalLinearCorrection.setX(0.0);
      if (!doCompliantControlLinear[1].getBooleanValue())
         totalLinearCorrection.setY(0.0);
      if (!doCompliantControlLinear[2].getBooleanValue())
         totalLinearCorrection.setZ(0.0);

      if (!doCompliantControlAngular[0].getBooleanValue())
         totalAngularCorrection.setX(0.0);
      if (!doCompliantControlAngular[1].getBooleanValue())
         totalAngularCorrection.setY(0.0);
      if (!doCompliantControlAngular[2].getBooleanValue())
         totalAngularCorrection.setZ(0.0);

      clipToVectorMagnitude(compliantControlMaxLinearDisplacement.getDoubleValue(), totalLinearCorrection);
      clipToVectorMagnitude(compliantControlMaxAngularDisplacement.getDoubleValue(), totalAngularCorrection);

      yoCompliantControlLinearDisplacement.setAndMatchFrame(totalLinearCorrection);
      yoCompliantControlAngularDisplacement.setAndMatchFrame(totalAngularCorrection);

      ReferenceFrame originalFrame = desiredPosition.getReferenceFrame();
      desiredPosition.changeFrame(controlFrame);
      desiredOrientation.changeFrame(controlFrame);

      desiredPosition.sub(totalLinearCorrection);

      totalAngularCorrection.negate();
      RotationFunctions.setAxisAngleBasedOnRotationVector(angularDisplacementAsAxisAngle, totalAngularCorrection.getVector());
      angularDisplacementAsMatrix.set(angularDisplacementAsAxisAngle);
      desiredOrientation.getMatrix3d(correctedRotationMatrix);
      correctedRotationMatrix.mul(angularDisplacementAsMatrix, correctedRotationMatrix);
      desiredOrientation.set(correctedRotationMatrix);

      desiredPosition.changeFrame(originalFrame);
      desiredOrientation.changeFrame(originalFrame);
   }

   private final AxisAngle4d angularDisplacementAsAxisAngle = new AxisAngle4d();
   private final Matrix3d angularDisplacementAsMatrix = new Matrix3d();
   private final Matrix3d correctedRotationMatrix = new Matrix3d();

   private final FrameVector tempForceVector = new FrameVector();
   private final FrameVector tempTorqueVector = new FrameVector();

   private void updateWristMeasuredWrench(FrameVector measuredForceToPack, FrameVector measuredTorqueToPack)
   {
      momentumBasedController.getWristMeasuredWrenchHandWeightCancelled(measuredWrench, robotSide);
      
      measuredWrench.packLinearPartIncludingFrame(tempForceVector);
      measuredWrench.packAngularPartIncludingFrame(tempTorqueVector);
      
      deadzoneMeasuredForce.update(tempForceVector);
      deadzoneMeasuredTorque.update(tempTorqueVector);
      
      filteredMeasuredForce.update();
      filteredMeasuredTorque.update();

      filteredMeasuredForce.getFrameTupleIncludingFrame(tempForceVector);
      filteredMeasuredTorque.getFrameTupleIncludingFrame(tempTorqueVector);

      measuredWrench.setLinearPart(tempForceVector);
      measuredWrench.setAngularPart(tempTorqueVector);
      measuredWrench.changeFrame(controlFrame);

      measuredWrench.packLinearPartIncludingFrame(measuredForceToPack);
      measuredWrench.packAngularPartIncludingFrame(measuredTorqueToPack);
   }

   private void clipToVectorMagnitude(double maximumMagnitude, FrameVector frameVectorToClip)
   {
      double magnitude = frameVectorToClip.length();
      if (magnitude > maximumMagnitude)
      {
         frameVectorToClip.scale(maximumMagnitude / magnitude);
      }
   }

   public void setHandControlFrame(ReferenceFrame handControlFrame)
   {
      controlFrame = handControlFrame;
   }

   public void reset()
   {
      yoCompliantControlLinearDisplacement.setToZero();
      yoCompliantControlAngularDisplacement.setToZero();
   }
}
