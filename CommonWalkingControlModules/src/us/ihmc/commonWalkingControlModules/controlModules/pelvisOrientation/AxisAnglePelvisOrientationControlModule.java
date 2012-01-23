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
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class AxisAnglePelvisOrientationControlModule implements PelvisOrientationControlModule
{
   private final ProcessedSensorsInterface processedSensors;
   private final CouplingRegistry couplingRegistry;
   private final YoVariableRegistry registry = new YoVariableRegistry("AxisAnglePelvisOrientationControlModule");

   private final DoubleYoVariable pelvisOrientationError = new DoubleYoVariable("pelvisOrientationError", registry);
   private final DoubleYoVariable[] proportionalGains = new DoubleYoVariable[3], derivativeGains = new DoubleYoVariable[3];
   
   private final YoFrameVector tauPelvis;
   private final YoFrameVector tauSwingLegCompensation;

   private final ReferenceFrame pelvisFrame;
   private final Matrix3d derivativeGainMatrix;

   private final boolean useFeedforward;

   private final Wrench upperBodyWrench = new Wrench();

   public AxisAnglePelvisOrientationControlModule(ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames,
         CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry, boolean useFeedforward)
   {
      this.processedSensors = processedSensors;
      this.pelvisFrame = referenceFrames.getPelvisFrame();
      this.couplingRegistry = couplingRegistry;
      this.tauPelvis = new YoFrameVector("tauPelvis", "", pelvisFrame, registry);
      this.tauSwingLegCompensation = new YoFrameVector("tauSwingLegCompensation", "", pelvisFrame, registry);
      this.derivativeGainMatrix = new Matrix3d();
      this.derivativeGainMatrix.setZero();
      this.useFeedforward = useFeedforward;

      String baseProportionalGainName = "pelvisOrientationProportionalGain";
      proportionalGains[0] = new DoubleYoVariable(baseProportionalGainName + "x", registry);
      proportionalGains[1] = new DoubleYoVariable(baseProportionalGainName + "y", registry);
      proportionalGains[2] = new DoubleYoVariable(baseProportionalGainName + "z", registry);
      
      String baseDerivativeGainName = "pelvisOrientationDerivativeGain";
      derivativeGains[0] = new DoubleYoVariable(baseDerivativeGainName + "x", registry);
      derivativeGains[1] = new DoubleYoVariable(baseDerivativeGainName + "y", registry);
      derivativeGains[2] = new DoubleYoVariable(baseDerivativeGainName + "z", registry);

      for (int i = 0; i < derivativeGains.length; i++)
      {
         derivativeGains[i].addVariableChangedListener(createDerivativeGainVariableUpdater(i));
      }

      parentRegistry.addChild(registry);
   }

   public void setupParametersForR2()
   {
      setProportionalGains(1500.0, 1500.0, 1500.0);
      
      setDerivativeGainX(200.0);
      setDerivativeGainY(150.0);
      setDerivativeGainZ(50.0);
   }

   public void setupParametersForM2V2()
   {
      setProportionalGains(500.0, 500.0, 150.0);
//      setProportionalGains(250.0, 250.0, 150.0);
//      setProportionalGains(150.0, 100.0, 100.0); // TODO: test using these lower gains again
      
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
         FrameVector feedForwardTerm = computeFeedForwardTerm(supportLeg);
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
      
      proportionalTerm.setX(proportionalTerm.getX() * proportionalGains[0].getDoubleValue());
      proportionalTerm.setY(proportionalTerm.getY() * proportionalGains[1].getDoubleValue());
      proportionalTerm.setZ(proportionalTerm.getZ() * proportionalGains[2].getDoubleValue());
      
//      proportionalTerm.scale(proportionalGain.getDoubleValue());

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

   private FrameVector computeFeedForwardTerm(RobotSide supportLeg)
   {
      FrameVector ret;
      if (supportLeg != null && couplingRegistry.getDesiredUpperBodyWrench() != null)
      {
         upperBodyWrench.set(couplingRegistry.getDesiredUpperBodyWrench());
         upperBodyWrench.changeFrame(pelvisFrame);
         ret = new FrameVector(upperBodyWrench.getExpressedInFrame(), upperBodyWrench.getAngularPartCopy());
      } else
      {
         ret = new FrameVector(pelvisFrame);
      }
      tauSwingLegCompensation.set(ret);
      return ret;
   }
   
   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      this.proportionalGains[0].set(proportionalGainX);
      this.proportionalGains[1].set(proportionalGainY);
      this.proportionalGains[2].set(proportionalGainZ);
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
      derivativeGains[0].set(derivativeGainX);
   }

   public void setDerivativeGainY(double derivativeGainY)
   {
      derivativeGains[1].set(derivativeGainY);
   }

   public void setDerivativeGainZ(double derivativeGainZ)
   {
      derivativeGains[2].set(derivativeGainZ);
   }

   private VariableChangedListener createDerivativeGainVariableUpdater(final int i)
   {
      return new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            derivativeGainMatrix.setElement(i, i, v.getValueAsDouble());
         }
      };
   }
}
