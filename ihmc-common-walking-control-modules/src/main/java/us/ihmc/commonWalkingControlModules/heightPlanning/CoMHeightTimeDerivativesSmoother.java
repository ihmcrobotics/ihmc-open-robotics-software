package us.ihmc.commonWalkingControlModules.heightPlanning;

import java.util.ArrayList;

import Jama.Matrix;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.controllers.pidGains.PDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.linearDynamicSystems.ComplexConjugateMode;
import us.ihmc.robotics.linearDynamicSystems.EigenvalueDecomposer;
import us.ihmc.robotics.linearDynamicSystems.SingleRealMode;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class CoMHeightTimeDerivativesSmoother
{
   private static final boolean DEBUG = false;

   private final double dt;

   //   private final double maximumAcceleration = 10.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FramePoint3D centerOfMassHeightPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());

   private final YoBoolean hasBeenInitialized = new YoBoolean("hasBeenInitialized", registry);

   private final YoDouble comHeightError = new YoDouble("smoothComHeightError", registry);
   private final YoDouble comHeightVelocityError = new YoDouble("smoothComHeightVelocityError", registry);
   private final YoDouble comHeightAccelerationError = new YoDouble("smoothComHeightAccelerationError", registry);

   private final YoDouble comHeightFeedback = new YoDouble("smoothComHeightFeedback", registry);
   private final YoDouble comHeightVelocityFeedback = new YoDouble("smoothComHeightVelocityFeedback", registry);
   private final YoDouble comHeightAccelerationFeedback = new YoDouble("smoothComHeightAccelerationFeedback", registry);

   private final YoDouble inputComHeight = new YoDouble("inputComHeight", registry);
   private final YoDouble inputComHeightVelocity = new YoDouble("inputComHeightVelocity", registry);
   private final YoDouble inputComHeightAcceleration = new YoDouble("inputComHeightAcceleration", registry);
   private final YoDouble inputComHeightJerk = new YoDouble("inputComHeightJerk", registry);

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

   private PDGainsReadOnly gains;
   private DoubleProvider maximumVelocity;

   public CoMHeightTimeDerivativesSmoother(double dt, YoRegistry parentRegistry)
   {
      this.dt = dt;

      //      comHeightGain.set(200.0); // * 0.001/dt); //200.0;
      //      comHeightVelocityGain.set(80.0); // * 0.001/dt); // 80.0;
      //      comHeightAccelerationGain.set(10.0); // * 0.001/dt); // 10.0

      double w0 = 20.0; //12.0;
      double w1 = 12.0;
      double zeta1 = 0.9; //0.7;

      computeGainsByPolePlacement(w0, w1, zeta1);
      parentRegistry.addChild(registry);
      computeEigenvalues();

      hasBeenInitialized.set(false);

      createDefaultGains();
   }

   public void computeGainsByPolePlacement(double w0, double w1, double zeta1)
   {
      comHeightGain.set(w0 * w1 * w1);
      comHeightVelocityGain.set(w1 * w1 + 2.0 * zeta1 * w1 * w0);
      comHeightAccelerationGain.set(w0 + 2.0 * zeta1 * w1);
   }

   public void computeEigenvalues()
   {
      double[][] matrixAValues = new double[][] {{0.0, 1.0, 0.0},
                                                 {0.0, 0.0, 1.0},
                                                 {-comHeightGain.getDoubleValue(),
                                                  -comHeightVelocityGain.getDoubleValue(),
                                                  -comHeightAccelerationGain.getDoubleValue()}};

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

   public void smooth(YoCoMHeightTimeDerivativesData heightZDataOutputToPack, YoCoMHeightTimeDerivativesData heightZDataInput)
   {
      if (!hasBeenInitialized.getBooleanValue())
         initialize(heightZDataInput);

      heightZDataInput.getComHeight(centerOfMassHeightPoint);
      double heightIn = centerOfMassHeightPoint.getZ();

      double heightVelocityIn = heightZDataInput.getComHeightVelocity();
      double heightAccelerationIn = heightZDataInput.getComHeightAcceleration();
      double heightJerkIn = heightZDataInput.getComHeightJerk();

      inputComHeight.set(heightIn);
      inputComHeightVelocity.set(heightVelocityIn);
      inputComHeightAcceleration.set(heightAccelerationIn);
      inputComHeightJerk.set(heightJerkIn);

      comHeightError.set(heightIn - smoothComHeight.getDoubleValue());
      comHeightVelocityError.set(heightVelocityIn - smoothComHeightVelocity.getDoubleValue());
      comHeightAccelerationError.set(heightAccelerationIn - smoothComHeightAcceleration.getDoubleValue());

      comHeightFeedback.set(comHeightGain.getDoubleValue() * comHeightError.getDoubleValue());
      comHeightVelocityFeedback.set(comHeightVelocityGain.getDoubleValue() * comHeightVelocityError.getDoubleValue());
      comHeightAccelerationFeedback.set(comHeightAccelerationGain.getDoubleValue() * comHeightAccelerationError.getDoubleValue());

      smoothComHeightJerk.set(inputComHeightJerk.getDoubleValue());
      smoothComHeightJerk.add(comHeightFeedback.getDoubleValue());
      smoothComHeightJerk.add(comHeightVelocityFeedback.getDoubleValue());
      smoothComHeightJerk.add(comHeightAccelerationFeedback.getDoubleValue());
      smoothComHeightJerk.set(MathTools.clamp(smoothComHeightJerk.getDoubleValue(), gains.getMaximumFeedbackRate()));

      double previousAcceleration = smoothComHeightAcceleration.getDoubleValue();
      smoothComHeightAcceleration.add(smoothComHeightJerk.getDoubleValue() * dt);
      smoothComHeightAcceleration.set(MathTools.clamp(smoothComHeightAcceleration.getDoubleValue(), gains.getMaximumFeedback()));

      smoothComHeightJerk.set((smoothComHeightAcceleration.getDoubleValue() - previousAcceleration) / dt);

      double previousVelocity = smoothComHeightVelocity.getDoubleValue();
      smoothComHeightVelocity.add(smoothComHeightAcceleration.getDoubleValue() * dt);
      smoothComHeightVelocity.set(MathTools.clamp(smoothComHeightVelocity.getDoubleValue(), maximumVelocity.getValue()));

      smoothComHeightAcceleration.set((smoothComHeightVelocity.getDoubleValue() - previousVelocity) / dt);

      smoothComHeight.add(smoothComHeightVelocity.getDoubleValue() * dt);

      heightZDataOutputToPack.setComHeight(centerOfMassHeightPoint.getReferenceFrame(), smoothComHeight.getDoubleValue());
      heightZDataOutputToPack.setComHeightVelocity(smoothComHeightVelocity.getDoubleValue());
      heightZDataOutputToPack.setComHeightAcceleration(smoothComHeightAcceleration.getDoubleValue());
      heightZDataOutputToPack.setComHeightJerk(smoothComHeightJerk.getDoubleValue());
   }

   public void initialize(YoCoMHeightTimeDerivativesData comHeightDataIn)
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

   public void setGains(PDGainsReadOnly gains, DoubleProvider maximumVelocity)
   {
      this.gains = gains;
      this.maximumVelocity = maximumVelocity;
   }

   public void createDefaultGains()
   {
      PDGains gains = new PDGains();
      gains.setMaximumFeedback(0.5 * 9.81);
      gains.setMaximumFeedbackRate(0.5 * 9.81 / 0.05);
      DoubleProvider maxVelocity = () -> 0.25;
      setGains(gains, maxVelocity);
   }
}
