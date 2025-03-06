package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoMHeightOffsetCalculator
{
   private double heightOffset;
   private final RateLimitedYoVariable heightOffsetRL;
   private final double minOffset;
   private final double maxOffset;
   private final double integrationDT;
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final DMatrixRMaj centroidalMomentum = new DMatrixRMaj(0);
   private final double robotMass;

   public CoMHeightOffsetCalculator(double integrationDT,
                                    double maxRate,
                                    double minOffset,
                                    double maxOffset,
                                    double robotMass,
                                    CentroidalMomentumCalculator centroidalMomentumCalculator,
                                    YoRegistry registry)
   {
      this.centroidalMomentumCalculator = centroidalMomentumCalculator;
      this.minOffset = minOffset;
      this.maxOffset = maxOffset;
      this.robotMass = robotMass;
      this.integrationDT = integrationDT;

      heightOffsetRL = new RateLimitedYoVariable("comZOffset", registry, maxRate, integrationDT);
   }

   public void reset()
   {
      heightOffset = 0.0;
      heightOffsetRL.set(heightOffset);
   }

   public void integrate(DMatrixRMaj qd)
   {
      CommonOps_DDRM.mult(centroidalMomentumCalculator.getCentroidalMomentumMatrix(), qd, centroidalMomentum);
      int centroidalLinearZIndex = 5;
      double momentumZOffsetVelocity = centroidalMomentum.get(centroidalLinearZIndex);
      double centroidalZVelocity = momentumZOffsetVelocity / robotMass;
      heightOffset += centroidalZVelocity * integrationDT;
      heightOffset = EuclidCoreTools.clamp(heightOffset, minOffset, maxOffset);
      heightOffsetRL.update(heightOffset);
   }

   public void clear(double alphaLeak)
   {
      heightOffset *= alphaLeak;
      heightOffsetRL.update(heightOffset);
   }

   public double getOffset()
   {
      return 0.0; // heightOffsetRL.getDoubleValue(); //
   }
}
