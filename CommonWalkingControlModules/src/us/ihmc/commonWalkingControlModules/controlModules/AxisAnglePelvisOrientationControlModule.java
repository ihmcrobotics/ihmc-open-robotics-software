package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Twist;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class AxisAnglePelvisOrientationControlModule implements PelvisOrientationControlModule
{
   private final ProcessedSensorsInterface processedSensors;
   private final YoVariableRegistry registry = new YoVariableRegistry("AngleAxisPelvisOrientationControlModule");
   
   private final DoubleYoVariable proportionalGain = new DoubleYoVariable("pelvisOrientationProportionalGain", registry);
   private final DoubleYoVariable pelvisOrientationError = new DoubleYoVariable("pelvisOrientationError", registry);
   private final YoFrameVector tauPelvis;
   
   private final ReferenceFrame pelvisFrame;
   private final Matrix3d derivativeGainMatrix;

   public AxisAnglePelvisOrientationControlModule(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames commonWalkingReferenceFrames, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.pelvisFrame = commonWalkingReferenceFrames.getPelvisFrame();
      this.tauPelvis = new YoFrameVector("tauPelvis", "", pelvisFrame, registry);
      this.derivativeGainMatrix = new Matrix3d();
      derivativeGainMatrix.setZero();
      
      parentRegistry.addChild(registry);
   }
   
   public void setupParametersForR2()
   {
      setProportionalGain(1500.0);
      setDerivativeGainX(200.0);
      setDerivativeGainY(150.0);
      setDerivativeGainZ(50.0);
   }
   
   public void setupParametersForM2V2()
   {
      setProportionalGain(250.0);
      setDerivativeGainX(80.0);
      setDerivativeGainY(75.0);
      setDerivativeGainZ(30.0);
   }

   public FrameVector computePelvisTorque(RobotSide supportLeg, Orientation desiredPelvisOrientation)
   {
      FrameVector ret = computeProportionalTerm(desiredPelvisOrientation);
      FrameVector derivativeTerm = computeDerivativeTerm();
      ret.add(derivativeTerm);
      tauPelvis.set(ret);
      
      return ret;
   }

   private FrameVector computeProportionalTerm(Orientation desiredPelvisOrientation)
   {
      desiredPelvisOrientation = desiredPelvisOrientation.changeFrameCopy(pelvisFrame);
      Quat4d desiredPelvisQuaternion = desiredPelvisOrientation.getQuaternion();
      AxisAngle4d desiredPelvisAngleAxis = new AxisAngle4d();
      desiredPelvisAngleAxis.set(desiredPelvisQuaternion);
      
      pelvisOrientationError.set(desiredPelvisAngleAxis.getAngle());

      FrameVector proportionalTerm = new FrameVector(desiredPelvisOrientation.getReferenceFrame());
      proportionalTerm.set(desiredPelvisAngleAxis.getX(), desiredPelvisAngleAxis.getY(), desiredPelvisAngleAxis.getZ());
      proportionalTerm.scale(desiredPelvisAngleAxis.getAngle());
      proportionalTerm.scale(proportionalGain.getDoubleValue());
      
      return proportionalTerm;
   }

   private FrameVector computeDerivativeTerm()
   {
      Twist twistOfPelvisWithRespectToWorld = processedSensors.computeTwistOfPelvisWithRespectToWorld();
      twistOfPelvisWithRespectToWorld.changeFrame(pelvisFrame);
      FrameVector derivativeTerm = new FrameVector(twistOfPelvisWithRespectToWorld.getExpressedInFrame(), twistOfPelvisWithRespectToWorld.getAngularPartCopy());
      derivativeGainMatrix.transform(derivativeTerm.getVector());
      derivativeTerm.scale(-1.0);
      
      return derivativeTerm;
   }
   
   public void setProportionalGain(double proportionalGain)
   {
      this.proportionalGain.set(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      for (int i = 0; i < 3; i++)
      {
         derivativeGainMatrix.setElement(i, i, derivativeGain);
      }
   }
   
   public void setDerivativeGainX(double derivativeGainX)
   {
      derivativeGainMatrix.setElement(0, 0, derivativeGainX);
   }
   
   public void setDerivativeGainY(double derivativeGainY)
   {
      derivativeGainMatrix.setElement(1, 1, derivativeGainY);
   }
   
   public void setDerivativeGainZ(double derivativeGainZ)
   {
      derivativeGainMatrix.setElement(2, 2, derivativeGainZ);
   }
}
