package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.communication.PostureOptimizerState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
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

   private final YoFramePoint3D nominalCoM;
   private final YoFramePoint3D desiredCoM;

   public CoMHeightRetargeting(double integrationDT,
                               double minOffset,
                               double maxOffset,
                               double robotMass,
                               DoubleProvider timeProvider,
                               ReferenceFrame centerOfMassFrame,
                               CentroidalMomentumCalculator centroidalMomentumCalculator,
                               YoGraphicsListRegistry graphicsListRegistry,
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

      if (OrientationRetargeting.VISUALIZE)
      {
         nominalCoM = new YoFramePoint3D("nominalPosCoM", ReferenceFrame.getWorldFrame(), registry);
         desiredCoM = new YoFramePoint3D("desiredPosCoM", ReferenceFrame.getWorldFrame(), registry);
         YoFrameQuaternion zeroOrientation = new YoFrameQuaternion("zeroOrientation", ReferenceFrame.getWorldFrame(), registry);

         YoGraphicCoordinateSystem nominalOrientation = new YoGraphicCoordinateSystem("nominalCoM", nominalCoM, zeroOrientation, 0.35, YoAppearance.Green());
         YoGraphicCoordinateSystem optimizedOrientation = new YoGraphicCoordinateSystem("optimizedCoM", desiredCoM, zeroOrientation, 0.35, YoAppearance.Red());

         graphicsListRegistry.registerYoGraphic("Coordinate Debug", nominalOrientation);
         graphicsListRegistry.registerYoGraphic("Coordinate Debug", optimizedOrientation);
      }
      else
      {
         nominalCoM = null;
         desiredCoM = null;
      }
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

   public void hideViz()
   {
      nominalCoM.setToNaN();
      desiredCoM.setToNaN();
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

      nominalCoM.setFromReferenceFrame(centerOfMassFrame);
      desiredCoM.setFromReferenceFrame(centerOfMassFrame);

      nominalCoM.setZ(nominalHeight);
      desiredCoM.setZ(desiredHeight);

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
