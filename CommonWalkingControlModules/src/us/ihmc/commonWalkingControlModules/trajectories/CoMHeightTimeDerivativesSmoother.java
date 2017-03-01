package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import Jama.Matrix;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.linearDynamicSystems.ComplexConjugateMode;
import us.ihmc.robotics.linearDynamicSystems.EigenvalueDecomposer;
import us.ihmc.robotics.linearDynamicSystems.SingleRealMode;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CoMHeightTimeDerivativesSmoother
{
   private static final boolean DEBUG = false;

   private final double dt;

   //   private final double maximumAcceleration = 10.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final FramePoint centerOfMassHeightPoint = new FramePoint(ReferenceFrame.getWorldFrame());

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);

   private final DoubleYoVariable inputComHeight = new DoubleYoVariable("inputComHeight", registry);
   private final DoubleYoVariable inputComHeightVelocity = new DoubleYoVariable("inputComHeightVelocity", registry);
   private final DoubleYoVariable inputComHeightAcceleration = new DoubleYoVariable("inputComHeightAcceleration", registry);

   private final DoubleYoVariable smoothComHeight = new DoubleYoVariable("smoothComHeight", registry);
   private final DoubleYoVariable smoothComHeightVelocity = new DoubleYoVariable("smoothComHeightVelocity", registry);
   private final DoubleYoVariable smoothComHeightAcceleration = new DoubleYoVariable("smoothComHeightAcceleration", registry);
   private final DoubleYoVariable smoothComHeightJerk = new DoubleYoVariable("smoothComHeightJerk", registry);

   private final DoubleYoVariable comHeightGain = new DoubleYoVariable("comHeightGain", registry);
   private final DoubleYoVariable comHeightVelocityGain = new DoubleYoVariable("comHeightVelocityGain", registry);
   private final DoubleYoVariable comHeightAccelerationGain = new DoubleYoVariable("comHeightAccelerationGain", registry);

   private final DoubleYoVariable eigenValueOneReal = new DoubleYoVariable("eigenValueOneReal", registry);
   private final DoubleYoVariable eigenValueOneImag = new DoubleYoVariable("eigenValueOneImag", registry);
   private final DoubleYoVariable eigenValueTwoReal = new DoubleYoVariable("eigenValueTwoReal", registry);
   private final DoubleYoVariable eigenValueTwoImag = new DoubleYoVariable("eigenValueTwoImag", registry);
   private final DoubleYoVariable eigenValueThreeReal = new DoubleYoVariable("eigenValueThreeReal", registry);
   private final DoubleYoVariable eigenValueThreeImag = new DoubleYoVariable("eigenValueThreeImag", registry);

   private final DoubleYoVariable maximumVelocity;
   private final DoubleYoVariable maximumAcceleration;
   private final DoubleYoVariable maximumJerk;

   public CoMHeightTimeDerivativesSmoother(double dt, YoVariableRegistry parentRegistry)
   {
      this(null, null, null, dt, parentRegistry);
   }

   public CoMHeightTimeDerivativesSmoother(DoubleYoVariable maximumVelocity, DoubleYoVariable maximumAcceleration, DoubleYoVariable maximumJerk, double dt,
         YoVariableRegistry parentRegistry)
   {
      this.dt = dt;

      if (maximumVelocity == null)
      {
         maximumVelocity = new DoubleYoVariable("comHeightMaxVelocity", registry);
         maximumVelocity.set(0.25); // Tried 0.25 on the real robot, looked good but need to be well tested.
      }

      if (maximumAcceleration == null)
      {
         maximumAcceleration = new DoubleYoVariable("comHeightMaxAcceleration", registry);
         maximumAcceleration.set(0.5 * 9.81);
      }

      if (maximumJerk == null)
      {
         maximumJerk = new DoubleYoVariable("comHeightMaxJerk", registry);
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
