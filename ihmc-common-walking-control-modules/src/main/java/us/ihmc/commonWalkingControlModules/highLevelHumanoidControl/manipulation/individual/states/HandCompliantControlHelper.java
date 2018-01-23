package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandComplianceControlParametersCommand;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.DeadzoneYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class HandCompliantControlHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoBoolean[] doCompliantControlLinear;
   private final YoBoolean[] doCompliantControlAngular;
   private final YoDouble forceDeadzoneSize;
   private final YoDouble torqueDeadzoneSize;
   private final DeadzoneYoFrameVector deadzoneMeasuredForce;
   private final DeadzoneYoFrameVector deadzoneMeasuredTorque;
   private final YoDouble measuredForceAlpha;
   private final AlphaFilteredYoFrameVector filteredMeasuredForce;
   private final AlphaFilteredYoFrameVector filteredMeasuredTorque;
   private final YoFrameVector measuredForceAtControlFrame;
   private final YoFrameVector measuredTorqueAtControlFrame;

   /** This is the desired force that the external environment should apply on the hand */
   private final YoFrameVector desiredForce;
   /** This is the desired torque that the external environment should apply on the hand */
   private final YoFrameVector desiredTorque;

   private final FrameVector3D totalLinearCorrection = new FrameVector3D();
   private final FrameVector3D totalAngularCorrection = new FrameVector3D();
   private final FrameVector3D linearCorrection = new FrameVector3D();
   private final FrameVector3D angularCorrection = new FrameVector3D();

   private final YoDouble linearGain;
   private final YoDouble angularGain;
   private final YoDouble compliantControlMaxLinearCorrectionPerTick;
   private final YoDouble compliantControlMaxLinearDisplacement;
   private final YoDouble compliantControlMaxAngularCorrectionPerTick;
   private final YoDouble compliantControlMaxAngularDisplacement;
   private final YoDouble compliantControlLeakRatio;
   private final YoDouble compliantControlResetLeakRatio;
   private final YoFrameVector yoCompliantControlLinearDisplacement;
   private final YoFrameVector yoCompliantControlAngularDisplacement;

   private ReferenceFrame controlFrame;
   private final Wrench measuredWrench = new Wrench();
   private final FrameVector3D measuredForce = new FrameVector3D();
   private final FrameVector3D measuredTorque = new FrameVector3D();
   private final FrameVector3D errorForce = new FrameVector3D();
   private final FrameVector3D errorTorque = new FrameVector3D();

   private final RobotSide robotSide;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   public HandCompliantControlHelper(String namePrefix, RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + "CompliantControl");
      parentRegistry.addChild(registry);

      this.robotSide = robotSide;
      this.controllerToolbox = controllerToolbox;

      ReferenceFrame forceSensorMeasurementFrame = controllerToolbox.getWristForceSensor(robotSide).getMeasurementFrame();

      forceDeadzoneSize = new YoDouble(namePrefix + "ForceDeadzoneSize", registry);
      deadzoneMeasuredForce = DeadzoneYoFrameVector.createDeadzoneYoFrameVector(namePrefix + "DeadzoneMeasuredForce", registry, forceDeadzoneSize,
            forceSensorMeasurementFrame);
      torqueDeadzoneSize = new YoDouble(namePrefix + "TorqueDeadzoneSize", registry);
      deadzoneMeasuredTorque = DeadzoneYoFrameVector.createDeadzoneYoFrameVector(namePrefix + "DeadzoneMeasuredTorque", registry, torqueDeadzoneSize,
            forceSensorMeasurementFrame);
      measuredForceAlpha = new YoDouble(namePrefix + "MeasuredForceAlpha", registry);
      filteredMeasuredForce = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "FilteredMeasuredForce", "", registry,
            measuredForceAlpha, deadzoneMeasuredForce);
      filteredMeasuredTorque = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "FilteredMeasuredTorque", "", registry,
            measuredForceAlpha, deadzoneMeasuredTorque);

      measuredForceAtControlFrame = new YoFrameVector(namePrefix + "MeasuredForceAtControlFrame", null, registry);
      measuredTorqueAtControlFrame = new YoFrameVector(namePrefix + "MeasuredTorqueAtControlFrame", null, registry);

      desiredForce = new YoFrameVector(namePrefix + "DesiredForce", null, registry);
      desiredTorque = new YoFrameVector(namePrefix + "DesiredTorque", null, registry);

      doCompliantControlLinear = new YoBoolean[3];
      doCompliantControlLinear[0] = new YoBoolean(namePrefix + "DoCompliantControlLinearX", registry);
      doCompliantControlLinear[1] = new YoBoolean(namePrefix + "DoCompliantControlLinearY", registry);
      doCompliantControlLinear[2] = new YoBoolean(namePrefix + "DoCompliantControlLinearZ", registry);

      doCompliantControlAngular = new YoBoolean[3];
      doCompliantControlAngular[0] = new YoBoolean(namePrefix + "DoCompliantControlAngularX", registry);
      doCompliantControlAngular[1] = new YoBoolean(namePrefix + "DoCompliantControlAngularY", registry);
      doCompliantControlAngular[2] = new YoBoolean(namePrefix + "DoCompliantControlAngularZ", registry);

      for (int i = 0; i < 3; i++)
      {
         doCompliantControlLinear[i].set(true);
         //         doCompliantControlAngular[i].set(true);
      }

      linearGain = new YoDouble(namePrefix + "CompliantControlLinearGain", registry);
      angularGain = new YoDouble(namePrefix + "CompliantControlAngularGain", registry);
      compliantControlMaxLinearCorrectionPerTick = new YoDouble(namePrefix + "CompliantControlMaxLinearCorrectionPerTick", registry);
      compliantControlMaxAngularCorrectionPerTick = new YoDouble(namePrefix + "CompliantControlMaxAngularCorrectionPerTick", registry);
      compliantControlMaxLinearDisplacement = new YoDouble(namePrefix + "CompliantControlMaxLinearDisplacement", registry);
      compliantControlMaxAngularDisplacement = new YoDouble(namePrefix + "CompliantControlMaxAngularDisplacement", registry);
      compliantControlLeakRatio = new YoDouble(namePrefix + "CompliantControlLeakRatio", registry);
      compliantControlResetLeakRatio = new YoDouble(namePrefix + "CompliantControlResetLeakRatio", registry);
      yoCompliantControlLinearDisplacement = new YoFrameVector(namePrefix + "CompliantControlLinearDisplacement", worldFrame, registry);
      yoCompliantControlAngularDisplacement = new YoFrameVector(namePrefix + "CompliantControlAngularDisplacement", worldFrame, registry);

      measuredForceAlpha.set(0.98);
      linearGain.set(0.00001);
      angularGain.set(0.001);
      compliantControlMaxLinearCorrectionPerTick.set(0.005);
      compliantControlMaxAngularCorrectionPerTick.set(0.001);
      compliantControlMaxLinearDisplacement.set(0.05);
      compliantControlMaxAngularDisplacement.set(0.2);
      compliantControlLeakRatio.set(0.999);
      compliantControlResetLeakRatio.set(0.9999);

      forceDeadzoneSize.set(10.0);
      torqueDeadzoneSize.set(0.5);
   }

   public void doCompliantControl(FramePoint3D desiredPosition, FrameQuaternion desiredOrientation)
   {
      updateWristMeasuredWrench(measuredForce, measuredTorque);

      totalLinearCorrection.setIncludingFrame(yoCompliantControlLinearDisplacement);
      totalAngularCorrection.setIncludingFrame(yoCompliantControlAngularDisplacement);

      measuredForce.changeFrame(controlFrame);
      measuredTorque.changeFrame(controlFrame);
      totalLinearCorrection.changeFrame(controlFrame);
      totalAngularCorrection.changeFrame(controlFrame);
      linearCorrection.setToZero(controlFrame);
      angularCorrection.setToZero(controlFrame);

      measuredForceAtControlFrame.set(measuredForce);
      measuredTorqueAtControlFrame.set(measuredTorque);

      errorForce.setIncludingFrame(controlFrame, desiredForce.getX(), desiredForce.getY(), desiredForce.getZ());
      errorForce.sub(measuredForce);
      errorTorque.setIncludingFrame(controlFrame, desiredTorque.getX(), desiredTorque.getY(), desiredTorque.getZ());
      errorTorque.sub(measuredTorque);

      linearCorrection.setAndScale(linearGain.getDoubleValue(), errorForce);
      angularCorrection.setAndScale(angularGain.getDoubleValue(), errorTorque);

      clipToVectorMagnitude(compliantControlMaxLinearCorrectionPerTick.getDoubleValue(), linearCorrection);
      clipToVectorMagnitude(compliantControlMaxAngularCorrectionPerTick.getDoubleValue(), angularCorrection);

      totalLinearCorrection.scale(compliantControlLeakRatio.getDoubleValue());
      totalAngularCorrection.scale(compliantControlLeakRatio.getDoubleValue());

      if (doCompliantControlLinear[0].getBooleanValue())
         totalLinearCorrection.setX(totalLinearCorrection.getX() + linearCorrection.getX());
      else
         totalLinearCorrection.setX(compliantControlResetLeakRatio.getDoubleValue() * totalLinearCorrection.getX());

      if (doCompliantControlLinear[1].getBooleanValue())
         totalLinearCorrection.setY(totalLinearCorrection.getY() + linearCorrection.getY());
      else
         totalLinearCorrection.setY(compliantControlResetLeakRatio.getDoubleValue() * totalLinearCorrection.getY());

      if (doCompliantControlLinear[2].getBooleanValue())
         totalLinearCorrection.setZ(totalLinearCorrection.getZ() + linearCorrection.getZ());
      else
         totalLinearCorrection.setZ(compliantControlResetLeakRatio.getDoubleValue() * totalLinearCorrection.getZ());

      if (doCompliantControlAngular[0].getBooleanValue())
         totalAngularCorrection.setX(totalAngularCorrection.getX() + angularCorrection.getX());
      else
         totalAngularCorrection.setX(compliantControlResetLeakRatio.getDoubleValue() * totalAngularCorrection.getX());

      if (doCompliantControlAngular[1].getBooleanValue())
         totalAngularCorrection.setY(totalAngularCorrection.getY() + angularCorrection.getY());
      else
         totalAngularCorrection.setY(compliantControlResetLeakRatio.getDoubleValue() * totalAngularCorrection.getY());

      if (doCompliantControlAngular[2].getBooleanValue())
         totalAngularCorrection.setZ(totalAngularCorrection.getZ() + angularCorrection.getZ());
      else
         totalAngularCorrection.setZ(compliantControlResetLeakRatio.getDoubleValue() * totalAngularCorrection.getZ());

      clipToVectorMagnitude(compliantControlMaxLinearDisplacement.getDoubleValue(), totalLinearCorrection);
      clipToVectorMagnitude(compliantControlMaxAngularDisplacement.getDoubleValue(), totalAngularCorrection);

      yoCompliantControlLinearDisplacement.setAndMatchFrame(totalLinearCorrection);
      yoCompliantControlAngularDisplacement.setAndMatchFrame(totalAngularCorrection);

      applyCorrection(desiredPosition, desiredOrientation);
   }

   private final AxisAngle angularDisplacementAsAxisAngle = new AxisAngle();
   private final RotationMatrix angularDisplacementAsMatrix = new RotationMatrix();
   private final RotationMatrix correctedRotationMatrix = new RotationMatrix();

   private final FrameVector3D tempForceVector = new FrameVector3D();
   private final FrameVector3D tempTorqueVector = new FrameVector3D();

   private void updateWristMeasuredWrench(FrameVector3D measuredForceToPack, FrameVector3D measuredTorqueToPack)
   {
      controllerToolbox.getWristMeasuredWrenchHandWeightCancelled(measuredWrench, robotSide);

      measuredWrench.getLinearPartIncludingFrame(tempForceVector);
      measuredWrench.getAngularPartIncludingFrame(tempTorqueVector);

      deadzoneMeasuredForce.update(tempForceVector);
      deadzoneMeasuredTorque.update(tempTorqueVector);

      filteredMeasuredForce.update();
      filteredMeasuredTorque.update();

      tempForceVector.setIncludingFrame(filteredMeasuredForce);
      tempTorqueVector.setIncludingFrame(filteredMeasuredTorque);

      measuredWrench.setLinearPart(tempForceVector);
      measuredWrench.setAngularPart(tempTorqueVector);
      measuredWrench.changeFrame(controlFrame);

      measuredWrench.getLinearPartIncludingFrame(measuredForceToPack);
      measuredWrench.getAngularPartIncludingFrame(measuredTorqueToPack);
   }

   private void clipToVectorMagnitude(double maximumMagnitude, FrameVector3D frameVectorToClip)
   {
      double magnitude = frameVectorToClip.length();
      if (magnitude > maximumMagnitude)
      {
         frameVectorToClip.scale(maximumMagnitude / magnitude);
      }
   }

   public void setCompliantControlFrame(ReferenceFrame compliantControlFrame)
   {
      controlFrame = compliantControlFrame;
   }

   public void reset()
   {
      yoCompliantControlLinearDisplacement.setToZero();
      yoCompliantControlAngularDisplacement.setToZero();
   }

   public void progressivelyCancelOutCorrection(FramePoint3D desiredPosition, FrameQuaternion desiredOrientation)
   {
      yoCompliantControlLinearDisplacement.scale(compliantControlResetLeakRatio.getDoubleValue());
      yoCompliantControlAngularDisplacement.scale(compliantControlResetLeakRatio.getDoubleValue());

      totalLinearCorrection.setIncludingFrame(yoCompliantControlLinearDisplacement);
      totalAngularCorrection.setIncludingFrame(yoCompliantControlAngularDisplacement);

      totalLinearCorrection.changeFrame(controlFrame);
      totalAngularCorrection.changeFrame(controlFrame);

      applyCorrection(desiredPosition, desiredOrientation);
   }

   private void applyCorrection(FramePoint3D desiredPosition, FrameQuaternion desiredOrientation)
   {
      ReferenceFrame originalFrame = desiredPosition.getReferenceFrame();
      desiredPosition.changeFrame(controlFrame);
      desiredOrientation.changeFrame(controlFrame);

      desiredPosition.sub(totalLinearCorrection);

      totalAngularCorrection.negate();
      angularDisplacementAsAxisAngle.set(totalAngularCorrection);
      angularDisplacementAsMatrix.set(angularDisplacementAsAxisAngle);
      correctedRotationMatrix.set(desiredOrientation);
      correctedRotationMatrix.set(angularDisplacementAsMatrix);
      correctedRotationMatrix.multiply(correctedRotationMatrix);
      desiredOrientation.set(correctedRotationMatrix);

      desiredPosition.changeFrame(originalFrame);
      desiredOrientation.changeFrame(originalFrame);
   }

   public void handleHandComplianceControlParametersMessage(HandComplianceControlParametersCommand message)
   {
      setEnableLinearCompliance(message.getEnableLinearCompliance());
      setEnableAngularCompliance(message.getEnableAngularCompliance());
      setDesiredForceOfHandOntoExternalEnvironment(message.getDesiredForce());
      setDesiredTorqueOfHandOntoExternalEnvironment(message.getDesiredTorque());
      setMeasuredWrenchDeadzoneSize(message.getForceDeadZone(), message.getTorqueDeadZone());
   }

   public void disableLinearCompliance()
   {
      for (int i = 0; i < 3; i++)
         doCompliantControlLinear[i].set(false);
   }

   public void disableAngularCompliance()
   {
      for (int i = 0; i < 3; i++)
         doCompliantControlAngular[i].set(false);
   }

   public void setEnableLinearCompliance(boolean[] enableLinearCompliance)
   {
      if (enableLinearCompliance == null)
      {
         disableLinearCompliance();
      }
      else
      {
         for (int i = 0; i < 3; i++)
            doCompliantControlLinear[i].set(enableLinearCompliance[i]);
      }
   }

   public void setEnableAngularCompliance(boolean[] enableAngularCompliance)
   {
      if (enableAngularCompliance == null)
      {
         disableAngularCompliance();
      }
      else
      {
         for (int i = 0; i < 3; i++)
            doCompliantControlAngular[i].set(enableAngularCompliance[i]);
      }
   }

   public void setDesiredForceOfHandOntoExternalEnvironment(Vector3D desiredForce)
   {
      if (desiredForce == null)
      {
         this.desiredForce.setToZero();
      }
      else
      {
         this.desiredForce.set(desiredForce);
         this.desiredForce.scale(-1.0);
      }
   }

   public void setDesiredTorqueOfHandOntoExternalEnvironment(Vector3D desiredTorque)
   {
      if (desiredTorque == null)
      {
         this.desiredTorque.setToZero();
      }
      else
      {
         this.desiredTorque.set(desiredTorque);
         this.desiredTorque.scale(-1.0);
      }
   }

   public void setMeasuredWrenchDeadzoneSize(double forceDeadzone, double torqueDeadzone)
   {
      if (!Double.isNaN(forceDeadzone))
         forceDeadzoneSize.set(forceDeadzone);
      if (!Double.isNaN(torqueDeadzone))
         torqueDeadzoneSize.set(torqueDeadzone);
   }
}
