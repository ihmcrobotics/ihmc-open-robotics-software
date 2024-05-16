package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.Map;

public class DiagnosticPostureAdjustmentCalculator implements WholeBodyPostureAdjustmentProvider
{
   private static final String prefix = "diagnostic_posture_";
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoFunctionGeneratorNew[] oneDoFJointFunctionGenerators;
   private final YoFunctionGeneratorNew pelvisHeightFunctionGenerator;
   private final YoFunctionGeneratorNew pelvisOrientationFunctionGenerator;
   private final YoFrameVector3D pelvisOrientationAxis;
   private final FrameVector3D pelvisOrientationAxisNormalized = new FrameVector3D();
   private final MovingReferenceFrame midFeetZUpFrame;
   private final AxisAngle pelvisOrientationOffset = new AxisAngle();

   private final Map<String, YoFunctionGeneratorNew> oneDoFJointNameToGenerator = new HashMap<>();

   public DiagnosticPostureAdjustmentCalculator(OneDoFJointBasics[] oneDoFJoints, MovingReferenceFrame midFeetZUpFrame, double controlDT, YoRegistry parentRegistry)
   {
      oneDoFJointFunctionGenerators = new YoFunctionGeneratorNew[oneDoFJoints.length];

      pelvisHeightFunctionGenerator = new YoFunctionGeneratorNew(prefix + "PelvisHeight", controlDT, registry);
      pelvisOrientationFunctionGenerator = new YoFunctionGeneratorNew(prefix + "PelvisOrientation", controlDT, registry);
      pelvisOrientationAxis = new YoFrameVector3D(prefix + "PelvisOrientationAxis", midFeetZUpFrame, registry);
      this.midFeetZUpFrame = midFeetZUpFrame;

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         oneDoFJointFunctionGenerators[i] = new YoFunctionGeneratorNew(prefix + oneDoFJoints[i].getName(), controlDT, registry);
         oneDoFJointNameToGenerator.put(oneDoFJoints[i].getName(), oneDoFJointFunctionGenerators[i]);
      }

      pelvisOrientationAxis.set(Axis3D.X);
      parentRegistry.addChild(registry);
   }

   public void update()
   {
      pelvisHeightFunctionGenerator.update();

      for (int i = 0; i < oneDoFJointFunctionGenerators.length; i++)
      {
         oneDoFJointFunctionGenerators[i].update();
      }

      pelvisOrientationFunctionGenerator.update();
      pelvisOrientationAxisNormalized.setIncludingFrame(pelvisOrientationAxis);
      pelvisOrientationAxisNormalized.normalize();

      pelvisOrientationOffset.getAxis().set(pelvisOrientationAxisNormalized);
      pelvisOrientationOffset.getAxis().normalize();
      pelvisOrientationOffset.setAngle(pelvisOrientationFunctionGenerator.getValue());
   }

   @Override
   public boolean isEnabled()
   {
      return true;
   }

   @Override
   public double getDesiredJointPositionOffset(String jointName)
   {
      return oneDoFJointNameToGenerator.get(jointName).getValue();
   }

   @Override
   public double getDesiredJointVelocityOffset(String jointName)
   {
      return oneDoFJointNameToGenerator.get(jointName).getValueDot();
   }

   @Override
   public double getDesiredJointAccelerationOffset(String jointName)
   {
      return oneDoFJointNameToGenerator.get(jointName).getValueDDot();
   }

   @Override
   public double getFloatingBasePositionOffsetZ()
   {
      return pelvisHeightFunctionGenerator.getValue();
   }

   @Override
   public double getFloatingBaseVelocityOffsetZ()
   {
      return pelvisHeightFunctionGenerator.getValueDot();
   }

   @Override
   public double getFloatingBaseAccelerationOffsetZ()
   {
      return pelvisHeightFunctionGenerator.getValueDDot();
   }

   @Override
   public void packFloatingBaseOrientationOffset(FrameQuaternionBasics orientationOffsetToPack)
   {
      orientationOffsetToPack.setReferenceFrame(midFeetZUpFrame);
      pelvisOrientationOffset.get(orientationOffsetToPack);
   }

   @Override
   public void packFloatingBaseAngularVelocityOffset(FrameVector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.setReferenceFrame(midFeetZUpFrame);
      angularVelocityToPack.setAndScale(pelvisOrientationFunctionGenerator.getValueDDot(), pelvisOrientationAxisNormalized);
   }

   @Override
   public void packFloatingBaseAngularAccelerationOffset(FrameVector3DBasics angularAccelerationToPack)
   {
      angularAccelerationToPack.setReferenceFrame(midFeetZUpFrame);
      angularAccelerationToPack.setAndScale(pelvisOrientationFunctionGenerator.getValueDDot(), pelvisOrientationAxisNormalized);
   }
}
