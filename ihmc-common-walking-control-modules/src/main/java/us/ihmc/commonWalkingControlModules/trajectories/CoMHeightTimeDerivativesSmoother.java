package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import Jama.Matrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.linearDynamicSystems.ComplexConjugateMode;
import us.ihmc.robotics.linearDynamicSystems.EigenvalueDecomposer;
import us.ihmc.robotics.linearDynamicSystems.SingleRealMode;

public class CoMHeightTimeDerivativesSmoother
{
   private static final boolean DEBUG = false;

   private final double dt;

   //   private final double maximumAcceleration = 10.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FramePoint3D centerOfMassHeightPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());

   private final YoBoolean hasBeenInitialized = new YoBoolean("hasBeenInitialized", registry);

   private final YoDouble inputComHeight = new YoDouble("inputComHeight", registry);
   private final YoDouble inputComHeightVelocity = new YoDouble("inputComHeightVelocity", registry);
   private final YoDouble inputComHeightAcceleration = new YoDouble("inputComHeightAcceleration", registry);

   private final YoDouble smoothComHeight = new YoDouble("smoothComHeight", registry);
   private final YoDouble smoothComHeightVelocity = new YoDouble("smoothComHeightVelocity", registry);
   private final YoDouble smoothComHeightAcceleration = new YoDouble("smoothComHeightAcceleration", registry);
   private final YoDouble smoothComHeightJerk = new YoDouble("smoothComHeightJerk", registry);

   private final YoDouble comHeightGain = new YoDouble("comHeightGain", registry);
   private final YoDouble comHeightVelocityGain = new YoDouble("comHeightVelocityGain", registry);
   private final YoDouble comHeightAccelerationGain = new YoDouble("comHeightAccelerationGain", registry);

   private final YoDouble eigenValueOneReal = new YoDouble("eigenValueOneReal", registry);
   private final YoDouble eigenValueOneImag = new YoDouble("eigenValueOneImag", registry);
   private final YoDouble eigenValueTwoReal = new YoDouble("eigenValueTwoReal", registry);
   private final YoDouble eigenValueTwoImag = new YoDouble("eigenValueTwoImag", registry);
   private final YoDouble eigenValueThreeReal = new YoDouble("eigenValueThreeReal", registry);
   private final YoDouble eigenValueThreeImag = new YoDouble("eigenValueThreeImag", registry);

   private final YoDouble maximumVelocity;
   private final YoDouble maximumAcceleration;
   private final YoDouble maximumJerk;

   public CoMHeightTimeDerivativesSmoother(double dt, YoVariableRegistry parentRegistry)
   {
      this(null, null, null, dt, parentRegistry);
   }

   public CoMHeightTimeDerivativesSmoother(YoDouble maximumVelocity, YoDouble maximumAcceleration, YoDouble maximumJerk, double dt,
         YoVariableRegistry parentRegistry)
   {
      this.dt = dt;

      if (maximumVelocity == null)
      {
         maximumVelocity = new YoDouble("comHeightMaxVelocity", registry);
         maximumVelocity.set(0.25); // Tried 0.25 on the real robot, looked good but need to be well tested.
      }

      if (maximumAcceleration == null)
      {
         maximumAcceleration = new YoDouble("comHeightMaxAcceleration", registry);
         maximumAcceleration.set(0.5 * 9.81);
      }

      if (maximumJerk == null)
      {
         maximumJerk = new YoDouble("comHeightMaxJerk", registry);
         maximumJerk.set(0.5 * 9.81 / 0.05);
      }

      this.maximumVelocity = maximumVelocity;
      this.maximumAcceleration = maximumAcceleration;
      this.maximumJerk = maximumJerk;

      //      comHeightGain.set(200.0); // * 0.001/dt); //200.0;
      //      comHeightVelocityGain.set(80.0); // * 0.001/dt); // 80.0;
      //      comHeightAccelerationGain.set(10.0); // * 0.001/dt); // 10.0

      double w0 = 12.0; //15.0;
      double w1 = 12.0; //15.0;
      double zeta1 = 0.9; //0.7;

      computeGainsByPolePlacement(w0, w1, zeta1);
      parentRegistry.addChild(registry);
      computeEigenvalues();

      hasBeenInitialized.set(false);
   }

   public void computeGainsByPolePlacement(double w0, double w1, double zeta1)
   {
      comHeightGain.set(w0 * w1 * w1);
      comHeightVelocityGain.set(w1 * w1 + 2.0 * zeta1 * w1 * w0);
      comHeightAccelerationGain.set(w0 + 2.0 * zeta1 * w1);
   }

   public void computeEigenvalues()
   {
      double[][] matrixAValues = new double[][] {{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
            {-comHeightGain.getDoubleValue(), -comHeightVelocityGain.getDoubleValue(), -comHeightAccelerationGain.getDoubleValue()}};

      Matrix matrixA = new Matrix(matrixAValues);
      EigenvalueDecomposer eigenvalueDecomposer = new EigenvalueDecomposer(matrixA);

      ComplexNumber[] eigenvalues = eigenvalueDecomposer.getEigenvalues();
      eigenValueOneReal.set(eigenvalues[0].real());
      eigenValueOneImag.set(eigenvalues[0].imag());
      eigenValueTwoReal.set(eigenvalues[1].real());
      eigenValueTwoImag.set(eigenvalues[1].imag());
      eigenValueThreeReal.set(eigenvalues[2].real());
      eigenValueThreeImag.set(eigenvalues[2].imag());

      if (DEBUG)
      {
         ArrayList<SingleRealMode> realModes = eigenvalueDecomposer.getRealModes();
         ArrayList<ComplexConjugateMode> complexConjugateModes = eigenvalueDecomposer.getComplexConjugateModes();

         for (SingleRealMode realMode : realModes)
         {
            System.out.println(realMode);
         }

         for (ComplexConjugateMode complexConjugateMode : complexConjugateModes)
         {
            System.out.println(complexConjugateMode);
         }
      }
   }

   public void smooth(CoMHeightTimeDerivativesData heightZDataOutputToPack, CoMHeightTimeDerivativesData heightZDataInput)
   {
      if (!hasBeenInitialized.getBooleanValue())
         initialize(heightZDataInput);

      heightZDataInput.getComHeight(centerOfMassHeightPoint);
      double heightIn = centerOfMassHeightPoint.getZ();

      double heightVelocityIn = heightZDataInput.getComHeightVelocity();
      double heightAccelerationIn = heightZDataInput.getComHeightAcceleration();

      inputComHeight.set(heightIn);
      inputComHeightVelocity.set(heightVelocityIn);
      inputComHeightAcceleration.set(heightAccelerationIn);

      double heightError = heightIn - smoothComHeight.getDoubleValue();
      double velocityError = heightVelocityIn - smoothComHeightVelocity.getDoubleValue();
      double accelerationError = heightAccelerationIn - smoothComHeightAcceleration.getDoubleValue();

      double jerk = comHeightAccelerationGain.getDoubleValue() * accelerationError + comHeightVelocityGain.getDoubleValue() * velocityError
            + comHeightGain.getDoubleValue() * heightError;
      jerk = MathTools.clamp(jerk, -maximumJerk.getDoubleValue(), maximumJerk.getDoubleValue());

      smoothComHeightJerk.set(jerk);

      smoothComHeightAcceleration.add(jerk * dt);
      smoothComHeightAcceleration.set(MathTools.clamp(smoothComHeightAcceleration.getDoubleValue(), maximumAcceleration.getDoubleValue()));

      double newSmoothComHeightVelocity = smoothComHeightVelocity.getDoubleValue();
      newSmoothComHeightVelocity += smoothComHeightAcceleration.getDoubleValue() * dt;
      newSmoothComHeightVelocity = MathTools.clamp(newSmoothComHeightVelocity, maximumVelocity.getDoubleValue());

      smoothComHeightAcceleration.set(newSmoothComHeightVelocity - smoothComHeightVelocity.getDoubleValue());
      smoothComHeightAcceleration.mul(1.0 / dt);

      smoothComHeightVelocity.set(newSmoothComHeightVelocity);
      smoothComHeight.add(smoothComHeightVelocity.getDoubleValue() * dt);

      heightZDataOutputToPack.setComHeight(centerOfMassHeightPoint.getReferenceFrame(), smoothComHeight.getDoubleValue());
      heightZDataOutputToPack.setComHeightVelocity(smoothComHeightVelocity.getDoubleValue());
      heightZDataOutputToPack.setComHeightAcceleration(smoothComHeightAcceleration.getDoubleValue());
   }

   public void initialize(CoMHeightTimeDerivativesData comHeightDataIn)
   {
      comHeightDataIn.getComHeight(centerOfMassHeightPoint);

      smoothComHeight.set(centerOfMassHeightPoint.getZ());
      smoothComHeightVelocity.set(comHeightDataIn.getComHeightVelocity());
      smoothComHeightAcceleration.set(comHeightDataIn.getComHeightAcceleration());

      this.hasBeenInitialized.set(true);
   }

   public void reset()
   {
      hasBeenInitialized.set(false);
   }
}
