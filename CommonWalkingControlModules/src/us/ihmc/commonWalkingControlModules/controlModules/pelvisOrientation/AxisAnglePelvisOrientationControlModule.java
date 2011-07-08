package us.ihmc.commonWalkingControlModules.controlModules.pelvisOrientation;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class AxisAnglePelvisOrientationControlModule implements PelvisOrientationControlModule
{   
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;
   private final YoVariableRegistry registry = new YoVariableRegistry("AxisAnglePelvisOrientationControlModule");
   
   private final DoubleYoVariable proportionalGain = new DoubleYoVariable("pelvisOrientationProportionalGain", registry);
   private final DoubleYoVariable pelvisOrientationError = new DoubleYoVariable("pelvisOrientationError", registry);
   private final YoFrameVector tauPelvis;
   
   private final ReferenceFrame pelvisFrame;
   private final Matrix3d derivativeGainMatrix;
   
   private final boolean useFeedforward;
   
   private final Wrench upperBodyWrench = new Wrench();

   public AxisAnglePelvisOrientationControlModule(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames, CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry, boolean useFeedforward)
   {
      this.processedSensors = processedSensors;
      this.pelvisFrame = referenceFrames.getPelvisFrame();
      this.couplingRegistry = couplingRegistry;
      this.tauPelvis = new YoFrameVector("tauPelvis", "", pelvisFrame, registry);
      this.derivativeGainMatrix = new Matrix3d();
      this.derivativeGainMatrix.setZero();
      this.useFeedforward = useFeedforward;
      
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
      setDerivativeGainX(30.0); // 80.0);
      setDerivativeGainY(18.0); // 75.0);
      setDerivativeGainZ(5.0); // 30.0);
   }

   public FrameVector computePelvisTorque(RobotSide supportLeg, Orientation desiredPelvisOrientation)
   {
      FrameVector ret = computeProportionalTerm(desiredPelvisOrientation);
      
      FrameVector derivativeTerm = computeDerivativeTerm();
      ret.add(derivativeTerm);
      
      if (useFeedforward)
      {
         FrameVector feedForwardTerm = computeFeedForwardTerm();
         ret.add(feedForwardTerm);
      }
      
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
      Twist twistOfPelvisWithRespectToWorld = processedSensors.getTwistOfPelvisWithRespectToWorld();
      twistOfPelvisWithRespectToWorld.changeFrame(pelvisFrame);
      FrameVector derivativeTerm = new FrameVector(twistOfPelvisWithRespectToWorld.getExpressedInFrame(), twistOfPelvisWithRespectToWorld.getAngularPartCopy());
      derivativeGainMatrix.transform(derivativeTerm.getVector());
      derivativeTerm.scale(-1.0);
      
      return derivativeTerm;
   }
   
   private FrameVector computeFeedForwardTerm()
   {
      if (couplingRegistry.getUpperBodyWrench() != null)
      {
         upperBodyWrench.set(couplingRegistry.getUpperBodyWrench());
         upperBodyWrench.changeFrame(pelvisFrame);
         return new FrameVector(upperBodyWrench.getExpressedInFrame(), upperBodyWrench.getTorque());
      }
      else
      {
         return new FrameVector(pelvisFrame);
      }
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
