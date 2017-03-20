package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandComplianceControlParametersCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.DeadzoneYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.Wrench;

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

   /** This is the desired force that the external environment should apply on the hand */
   private final YoFrameVector desiredForce;
   /** This is the desired torque that the external environment should apply on the hand */
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
   private final DoubleYoVariable compliantControlResetLeakRatio;
   private final YoFrameVector yoCompliantControlLinearDisplacement;
   private final YoFrameVector yoCompliantControlAngularDisplacement;

   private ReferenceFrame controlFrame;
   private final Wrench measuredWrench = new Wrench();
   private final FrameVector measuredForce = new FrameVector();
   private final FrameVector measuredTorque = new FrameVector();
   private final FrameVector errorForce = new FrameVector();
   private final FrameVector errorTorque = new FrameVector();

   private final RobotSide robotSide;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   public HandCompliantControlHelper(String namePrefix, RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + "CompliantControl");
      parentRegistry.addChild(registry);

      this.robotSide = robotSide;
      this.controllerToolbox = controllerToolbox;

      ReferenceFrame forceSensorMeasurementFrame = controllerToolbox.getWristForceSensor(robotSide).getMeasurementFrame();

      forceDeadzoneSize = new DoubleYoVariable(namePrefix + "ForceDeadzoneSize", registry);
      deadzoneMeasuredForce = DeadzoneYoFrameVector.createDeadzoneYoFrameVector(namePrefix + "DeadzoneMeasuredForce", registry, forceDeadzoneSize,
            forceSensorMeasurementFrame);
      torqueDeadzoneSize = new DoubleYoVariable(namePrefix + "TorqueDeadzoneSize", registry);
      deadzoneMeasuredTorque = DeadzoneYoFrameVector.createDeadzoneYoFrameVector(namePrefix + "DeadzoneMeasuredTorque", registry, torqueDeadzoneSize,
            forceSensorMeasurementFrame);
      measuredForceAlpha = new DoubleYoVariable(namePrefix + "MeasuredForceAlpha", registry);
      filteredMeasuredForce = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "FilteredMeasuredForce", "", registry,
            measuredForceAlpha, deadzoneMeasuredForce);
      filteredMeasuredTorque = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(namePrefix + "FilteredMeasuredTorque", "", registry,
            measuredForceAlpha, deadzoneMeasuredTorque);

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
      compliantControlResetLeakRatio = new DoubleYoVariable(namePrefix + "CompliantControlResetLeakRatio", registry);
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

   private final FrameVector tempForceVector = new FrameVector();
   private final FrameVector tempTorqueVector = new FrameVector();

   private void updateWristMeasuredWrench(FrameVector measuredForceToPack, FrameVector measuredTorqueToPack)
   {
      controllerToolbox.getWristMeasuredWrenchHandWeightCancelled(measuredWrench, robotSide);

      measuredWrench.getLinearPartIncludingFrame(tempForceVector);
      measuredWrench.getAngularPartIncludingFrame(tempTorqueVector);

      deadzoneMeasuredForce.update(tempForceVector);
      deadzoneMeasuredTorque.update(tempTorqueVector);

      filteredMeasuredForce.update();
      filteredMeasuredTorque.update();

      filteredMeasuredForce.getFrameTupleIncludingFrame(tempForceVector);
      filteredMeasuredTorque.getFrameTupleIncludingFrame(tempTorqueVector);

      measuredWrench.setLinearPart(tempForceVector);
      measuredWrench.setAngularPart(tempTorqueVector);
      measuredWrench.changeFrame(controlFrame);

      measuredWrench.getLinearPartIncludingFrame(measuredForceToPack);
      measuredWrench.getAngularPartIncludingFrame(measuredTorqueToPack);
   }

   private void clipToVectorMagnitude(double maximumMagnitude, FrameVector frameVectorToClip)
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

   public void progressivelyCancelOutCorrection(FramePoint desiredPosition, FrameOrientation desiredOrientation)
   {
      yoCompliantControlLinearDisplacement.scale(compliantControlResetLeakRatio.getDoubleValue());
      yoCompliantControlAngularDisplacement.scale(compliantControlResetLeakRatio.getDoubleValue());

      yoCompliantControlLinearDisplacement.getFrameTupleIncludingFrame(totalLinearCorrection);
      yoCompliantControlAngularDisplacement.getFrameTupleIncludingFrame(totalAngularCorrection);

      totalLinearCorrection.changeFrame(controlFrame);
      totalAngularCorrection.changeFrame(controlFrame);

      applyCorrection(desiredPosition, desiredOrientation);
   }

   private void applyCorrection(FramePoint desiredPosition, FrameOrientation desiredOrientation)
   {
      ReferenceFrame originalFrame = desiredPosition.getReferenceFrame();
      desiredPosition.changeFrame(controlFrame);
      desiredOrientation.changeFrame(controlFrame);

      desiredPosition.sub(totalLinearCorrection);

      totalAngularCorrection.negate();
      angularDisplacementAsAxisAngle.set(totalAngularCorrection.getVector());
      angularDisplacementAsMatrix.set(angularDisplacementAsAxisAngle);
      desiredOrientation.getMatrix3d(correctedRotationMatrix);
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
