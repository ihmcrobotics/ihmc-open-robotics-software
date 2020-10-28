package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.functional.IntDoubleConsumer;

import java.util.List;
import java.util.function.*;

public class SplitFractionFromAreaCalculator
{
   private final ConvexPolygon2D previousPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D currentPolygon = new ConvexPolygon2D();

   private final SplitFractionCalculatorParametersReadOnly splitFractionParameters;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> defaultFootPolygons;

   private IntSupplier numberOfStepsProvider;
   private IntFunction<List<? extends Point2DReadOnly>> stepPolygonGetter;
   private IntFunction<RobotSide> stepSideProvider;
   private Supplier<List<? extends Point2DReadOnly>> firstSupportPolygonProvider;
   private DoubleSupplier finalTransferWeightDistributionProvider;
   private DoubleSupplier finalTransferSplitFractionProvider;
   private IntToDoubleFunction transferWeightDistributionProvider;
   private IntToDoubleFunction transferSplitFractionProvider;
   private DoubleConsumer finalTransferWeightDistributionConsumer;
   private DoubleConsumer finalTransferSplitFractionConsumer;
   private IntDoubleConsumer transferWeightDistributionConsumer;
   private IntDoubleConsumer transferSplitFractionConsumer;

   public SplitFractionFromAreaCalculator(SplitFractionCalculatorParametersReadOnly splitFractionParameters,
                                          SideDependentList<? extends ConvexPolygon2DReadOnly> defaultFootPolygons)
   {
      this.splitFractionParameters = splitFractionParameters;
      this.defaultFootPolygons = defaultFootPolygons;
   }

   public void setNumberOfStepsProvider(IntSupplier numberOfStepsProvider)
   {
      this.numberOfStepsProvider = numberOfStepsProvider;
   }

   public void setFinalTransferWeightDistributionProvider(DoubleSupplier finalTransferWeightDistributionProvider)
   {
      this.finalTransferWeightDistributionProvider = finalTransferWeightDistributionProvider;
   }

   public void setFinalTransferSplitFractionProvider(DoubleSupplier finalTransferSplitFractionProvider)
   {
      this.finalTransferSplitFractionProvider = finalTransferSplitFractionProvider;
   }

   public void setFinalTransferWeightDistributionConsumer(DoubleConsumer finalTransferWeightDistributionConsumer)
   {
      this.finalTransferWeightDistributionConsumer = finalTransferWeightDistributionConsumer;
   }

   public void setFinalTransferSplitFractionConsumer(DoubleConsumer finalTransferSplitFractionConsumer)
   {
      this.finalTransferSplitFractionConsumer = finalTransferSplitFractionConsumer;
   }

   public void setTransferWeightDistributionProvider(IntToDoubleFunction transferWeightDistributionProvider)
   {
      this.transferWeightDistributionProvider = transferWeightDistributionProvider;
   }

   public void setTransferSplitFractionProvider(IntToDoubleFunction transferSplitFractionProvider)
   {
      this.transferSplitFractionProvider = transferSplitFractionProvider;
   }

   public void setTransferWeightDistributionConsumer(IntDoubleConsumer transferWeightDistributionConsumer)
   {
      this.transferWeightDistributionConsumer = transferWeightDistributionConsumer;
   }

   public void setTransferSplitFractionConsumer(IntDoubleConsumer transferSplitFractionConsumer)
   {
      this.transferSplitFractionConsumer = transferSplitFractionConsumer;
   }

   public void setFirstSupportPolygonProvider(Supplier<List<? extends Point2DReadOnly>> firstSupportPolygonProvider)
   {
      this.firstSupportPolygonProvider = firstSupportPolygonProvider;
   }

   public void setStepPolygonGetter(IntFunction<List<? extends Point2DReadOnly>> stepPolygonGetter)
   {
      this.stepPolygonGetter = stepPolygonGetter;
   }

   public void setStepSideProvider(IntFunction<RobotSide> stepSideProvider)
   {
      this.stepSideProvider = stepSideProvider;
   }

   public void computeSplitFractionsFromArea()
   {
      if (numberOfStepsProvider.getAsInt() == 0 || !splitFractionParameters.calculateSplitFractionsFromArea())
      {
         return;
      }
      double defaultTransferSplitFraction = splitFractionParameters.getDefaultTransferSplitFraction();
      double defaultWeightDistribution = 0.5;

      for (int stepNumber = 0; stepNumber < numberOfStepsProvider.getAsInt(); stepNumber++)
      {
         if (stepNumber == 0)
         {
            previousPolygon.clear();
            List<? extends Point2DReadOnly> stepPolygon = firstSupportPolygonProvider.get();
            previousPolygon.clear();
            for (int i = 0; i < stepPolygon.size(); i++)
               previousPolygon.addVertex(stepPolygon.get(i));
            previousPolygon.update();
         }
         else
         {
            List<? extends Point2DReadOnly> stepPolygon = stepPolygonGetter.apply(stepNumber - 1);
            if (stepPolygon != null && !stepPolygon.isEmpty())
            {
               previousPolygon.clear();
               for (int i = 0; i < stepPolygon.size(); i++)
                  previousPolygon.addVertex(stepPolygon.get(i));
               previousPolygon.update();
            }
            else
            {
               previousPolygon.set(defaultFootPolygons.get(stepSideProvider.apply(stepNumber - 1)));
            }
         }

         List<? extends Point2DReadOnly> stepPolygon = stepPolygonGetter.apply(stepNumber);

         if (stepPolygon != null && !stepPolygon.isEmpty())
         {
            currentPolygon.clear();
            for (int i = 0; i < stepPolygon.size(); i++)
               currentPolygon.addVertex(stepPolygon.get(i));
            currentPolygon.update();
         }
         else
         {
            previousPolygon.set(defaultFootPolygons.get(stepSideProvider.apply(stepNumber)));
         }

         double currentArea = currentPolygon.getArea();
         double previousArea = previousPolygon.getArea();

         double totalArea = currentArea + previousArea;

         double currentWidth = currentPolygon.getBoundingBoxRangeY();
         double previousWidth = previousPolygon.getBoundingBoxRangeY();

         double totalWidth = currentWidth + previousWidth;

         double percentAreaOnCurrentFoot = totalArea > 0.0 ? currentArea / totalArea : 0.5;
         double percentWidthOnCurrentFoot = totalWidth > 0.0 ? currentWidth / totalWidth : 0.5;

         if (MathTools.epsilonEquals(percentAreaOnCurrentFoot, 0.5, 1.0e-2) && MathTools.epsilonEquals(percentWidthOnCurrentFoot, 0.5, 2.0e-2))
            continue;

         double transferWeightDistributionFromArea = InterpolationTools.linearInterpolate(defaultWeightDistribution,
                                                                                          splitFractionParameters.getFractionLoadIfFootHasFullSupport(),
                                                                                          2.0 * percentAreaOnCurrentFoot - 1.0);
         double transferWeightDistributionFromWidth = InterpolationTools.linearInterpolate(defaultWeightDistribution,
                                                                                           splitFractionParameters.getFractionLoadIfOtherFootHasNoWidth(),
                                                                                           2.0 * percentWidthOnCurrentFoot - 1.0);

         // lower means it spends more time shifting to the center, higher means it spends less time shifting to the center
         // e.g., if we set the fraction to 0 and the trailing foot has no area, the split fraction should be 1 because we spend no time on the first segment
         double transferSplitFractionFromArea = InterpolationTools.linearInterpolate(defaultTransferSplitFraction,
                                                                                     1.0 - splitFractionParameters.getFractionTimeOnFootIfFootHasFullSupport(),
                                                                                     2.0 * percentAreaOnCurrentFoot - 1.0);
         double transferSplitFractionFromWidth = InterpolationTools.linearInterpolate(defaultTransferSplitFraction,
                                                                                      1.0
                                                                                      - splitFractionParameters.getFractionTimeOnFootIfOtherFootHasNoWidth(),
                                                                                      2.0 * percentWidthOnCurrentFoot - 1.0);

         double transferWeightDistribution = 0.5 * (transferWeightDistributionFromArea + transferWeightDistributionFromWidth);
         double transferSplitFraction = 0.5 * (transferSplitFractionFromArea + transferSplitFractionFromWidth);

         transferWeightDistribution = MathTools.clamp(transferWeightDistribution, 0.01, 0.99);
         transferSplitFraction = MathTools.clamp(transferSplitFraction, 0.01, 0.99);

         if (stepNumber == numberOfStepsProvider.getAsInt() - 1)
         { // this is the last step

            double currentSplitFraction = finalTransferSplitFractionProvider.getAsDouble();
            double currentWeightDistribution = finalTransferWeightDistributionProvider.getAsDouble();

            double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
            double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution,
                                                                                         currentWeightDistribution,
                                                                                         defaultWeightDistribution);

            finalTransferSplitFractionConsumer.accept(splitFractionToSet);
            finalTransferWeightDistributionConsumer.accept(weightDistributionToSet);
         }
         else
         {
            double currentSplitFraction = transferSplitFractionProvider.applyAsDouble(stepNumber + 1);
            double currentWeightDistribution = transferWeightDistributionProvider.applyAsDouble(stepNumber + 1);

            double splitFractionToSet = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultTransferSplitFraction);
            double weightDistributionToSet = SplitFractionTools.appendWeightDistribution(transferWeightDistribution,
                                                                                         currentWeightDistribution,
                                                                                         defaultWeightDistribution);

            transferSplitFractionConsumer.accept(stepNumber + 1, splitFractionToSet);
            transferWeightDistributionConsumer.accept(stepNumber + 1, weightDistributionToSet);
         }
      }
   }
}
