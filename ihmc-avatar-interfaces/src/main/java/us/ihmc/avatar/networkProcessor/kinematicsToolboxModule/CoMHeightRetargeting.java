package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.communication.PostureOptimizerState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.OrientationRetargeting.BLEND_DURATION;

public class CoMHeightRetargeting
{
   private double desiredHeight;
   private double nominalHeight;
   private double optimizedHeight;
   private double blendingHeight;

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

   private final YoDouble comRetargetDelta;
   private double blendStartTime;
   private final DoubleProvider timeProvider;

   public CoMHeightRetargeting(double integrationDT,
                               double minOffset,
                               double maxOffset,
                               double robotMass,
                               DoubleProvider timeProvider,
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
      this.timeProvider = timeProvider;

      initialCoMZ = new YoDouble("initialCoMZ", registry);
      nominalCoMZ = new YoDouble("nominalCoMZ", registry);
      centroidalZVelocity = new YoDouble("centroidalZVelocity", registry);

      comRetargetDelta = new YoDouble("comRetargetDelta", registry);
   }

   public void initialize()
   {
      tempPoint.setToZero(centerOfMassFrame);
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      nominalHeight = tempPoint.getZ();
      optimizedHeight = nominalHeight;
      desiredHeight = nominalHeight;
      blendingHeight = nominalHeight;

      initialCoMZ.set(tempPoint.getZ());
      comRetargetDelta.set(0.0);
   }

   public void updateNominalHeight(double nominalHeight)
   {
      this.nominalHeight = nominalHeight;
      nominalCoMZ.set(nominalHeight);
   }

   public void update(PostureOptimizerState optimizerState, PostureOptimizerState previousState, DMatrixRMaj qd)
   {
      boolean isNewState = previousState != optimizerState;
      if (isNewState && optimizerState == PostureOptimizerState.OPTIMIZER)
      {
         optimizedHeight = desiredHeight;
      }
      else if (isNewState && optimizerState == PostureOptimizerState.NOMINAL)
      {
         blendingHeight = desiredHeight;
         blendStartTime = timeProvider.getValue();
      }

      if (optimizerState == PostureOptimizerState.NOMINAL)
      {
         double blendAlpha = EuclidCoreTools.clamp((timeProvider.getValue() - blendStartTime) / BLEND_DURATION, 0.0, 1.0);
         desiredHeight =  EuclidCoreTools.interpolate(blendingHeight, nominalHeight, blendAlpha);
      }
      else if (optimizerState == PostureOptimizerState.OPTIMIZER)
      {
         integrate(qd);
         desiredHeight = EuclidCoreTools.clamp(optimizedHeight, nominalHeight + minOffset, nominalHeight + maxOffset);
      }
      else
      {
         // freeze, no update (just call this to avoid edge cases in rate limiting)
      }

      comRetargetDelta.set(Math.abs(desiredHeight - nominalHeight));
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
      return desiredHeight;
   }
}
