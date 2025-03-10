package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.communication.PostureOptimizerState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CoMHeightRetargeting
{
   private final RateLimitedYoVariable optimizedHeightRL;
   private double nominalHeight;
   private double optimizedHeight;

   private final double minOffset;
   private final double maxOffset;
   private final double integrationDT;
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final DMatrixRMaj centroidalMomentum = new DMatrixRMaj(0);
   private final double robotMass;
   private final ReferenceFrame centerOfMassFrame;

   private final FramePoint3D tempPoint = new FramePoint3D();
   
   private final YoDouble initialCoMZ;
   private final YoDouble nominalCoMZ;
   private final YoDouble centroidalZVelocity;

   public CoMHeightRetargeting(double integrationDT,
                               double maxRate,
                               double minOffset,
                               double maxOffset,
                               double robotMass,
                               ReferenceFrame centerOfMassFrame,
                               CentroidalMomentumCalculator centroidalMomentumCalculator,
                               YoRegistry registry)
   {
      this.centroidalMomentumCalculator = centroidalMomentumCalculator;
      this.centerOfMassFrame = centerOfMassFrame;
      this.minOffset = minOffset;
      this.maxOffset = maxOffset;
      this.robotMass = robotMass;
      this.integrationDT = integrationDT;

      optimizedHeightRL = new RateLimitedYoVariable("optimizedCoMHeight", registry, maxRate, integrationDT);

      initialCoMZ = new YoDouble("initialCoMZ", registry);
      nominalCoMZ = new YoDouble("nominalCoMZ", registry);
      centroidalZVelocity = new YoDouble("centroidalZVelocity", registry);
   }

   public void initialize()
   {
      tempPoint.setToZero(centerOfMassFrame);
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      nominalHeight = tempPoint.getZ();
      optimizedHeight = nominalHeight;

      // call once to avoid edge cases in rate limiting
      this.optimizedHeightRL.set(nominalHeight);
      this.optimizedHeightRL.update(nominalHeight);

      initialCoMZ.set(tempPoint.getZ());
   }

   public void updateNominalHeight(double nominalHeight)
   {
      this.nominalHeight = nominalHeight;
      nominalCoMZ.set(nominalHeight);
   }

   public void update(PostureOptimizerState optimizerState, DMatrixRMaj qd)
   {
      if (optimizerState == PostureOptimizerState.NOMINAL)
      {
         optimizedHeightRL.update(nominalHeight);

         // reset optimized to actual
         optimizedHeight = nominalHeight;
      }
      else if (optimizerState == PostureOptimizerState.OPTIMIZER)
      {
         integrate(qd);
         optimizedHeightRL.update(optimizedHeight);
         optimizedHeightRL.set(EuclidCoreTools.clamp(optimizedHeightRL.getValue(), nominalHeight + minOffset, nominalHeight + maxOffset));
      }
      else
      {
         // freeze, no update (just call this to avoid edge cases in rate limiting)
         optimizedHeightRL.update(optimizedHeightRL.getValue());
      }
   }

   public void integrate(DMatrixRMaj qd)
   {
      CommonOps_DDRM.mult(centroidalMomentumCalculator.getCentroidalMomentumMatrix(), qd, centroidalMomentum);
      int centroidalLinearZIndex = 5;
      double momentumZOffsetVelocity = centroidalMomentum.get(centroidalLinearZIndex);
      double centroidalZVelocity = momentumZOffsetVelocity / robotMass;
      optimizedHeight += centroidalZVelocity * integrationDT;

      this.centroidalZVelocity.set(centroidalZVelocity);
   }

   public double getHeight()
   {
      return optimizedHeightRL.getDoubleValue();
   }
}
