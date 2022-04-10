package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicStateInspectorParameters
{

   /** If the ICP is this distance based the heel of the elading foot, toe off should happen, regardless of any of the other conditions. **/
   private final YoDouble distanceForwardFromHeel;

   /** This checks to make sure the ICP isn't falling to the outside of the trailing foot. **/
   private final YoDouble minLateralDistance;

   /**
    * These variables make sure the ICP is far enough from the toe off point. If they're far enough, then there's probably enough control
    * authority to control them
    */
   private final YoDouble minDistanceFromTheToe;
   private final YoDouble minFractionOfStrideFromTheToe;

   private final YoDouble minDistanceFromOutsideEdge;
   private final YoDouble minOrthogonalDistanceFromOutsideEdge;
   private final YoDouble minDistanceFromInsideEdge;
   private final YoDouble minOrthogonalDistanceFromInsideEdge;

   private final YoDouble minNormalizedDistanceFromOutsideEdge;
   private final YoDouble minNormalizedDistanceFromInsideEdge;
   private final YoDouble maxRatioOfControlDecreaseFromToeingOff;
   private final YoDouble maxNecessaryNormalizedError;

   public DynamicStateInspectorParameters(YoRegistry parentRegistry)
   {
      this("", parentRegistry);
   }

   public DynamicStateInspectorParameters(String suffix, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName() + suffix);

      distanceForwardFromHeel = new YoDouble("distanceForwardFromHeel" + suffix, registry);
      minLateralDistance = new YoDouble("minLateralDistance" + suffix, registry);

      minDistanceFromTheToe = new YoDouble("minDistanceFromTheToe" + suffix, registry);
      minFractionOfStrideFromTheToe = new YoDouble("minFractionOfStrideFromTheToe" + suffix, registry);

      minDistanceFromOutsideEdge = new YoDouble("minDistanceFromOutsideEdge" + suffix, registry);
      minOrthogonalDistanceFromOutsideEdge = new YoDouble("minOrthogonalDistanceFromOutsideEdge" + suffix, registry);
      minDistanceFromInsideEdge = new YoDouble("minDistanceFromInsideEdge" + suffix, registry);
      minOrthogonalDistanceFromInsideEdge = new YoDouble("minOrthogonalDistanceFromInsideEdge" + suffix, registry);

      minNormalizedDistanceFromOutsideEdge = new YoDouble("minNormalizedDistanceFromOutsideEdge" + suffix, registry);
      minNormalizedDistanceFromInsideEdge = new YoDouble("minNormalizedDistanceFromInsideEdge" + suffix, registry);
      maxRatioOfControlDecreaseFromToeingOff = new YoDouble("maxRatioOfControlDecreaseFromToeingOff" + suffix, registry);
      maxNecessaryNormalizedError = new YoDouble("maxNecessaryNormalizedError" + suffix, registry);

      minNormalizedDistanceFromOutsideEdge.setToNaN();
      minNormalizedDistanceFromInsideEdge.setToNaN();
      maxNecessaryNormalizedError.setToNaN();
      maxRatioOfControlDecreaseFromToeingOff.set(Double.POSITIVE_INFINITY);

      parentRegistry.addChild(registry);
   }

   public double getDistanceForwardFromHeel()
   {
      return distanceForwardFromHeel.getDoubleValue();
   }

   public double getMinLateralDistance()
   {
      return minLateralDistance.getDoubleValue();
   }

   public double getMinDistanceFromTheToe()
   {
      return minDistanceFromTheToe.getDoubleValue();
   }

   public double getMinFractionOfStrideFromTheToe()
   {
      return minFractionOfStrideFromTheToe.getDoubleValue();
   }

   public double getMinDistanceFromOutsideEdge()
   {
      return minDistanceFromOutsideEdge.getDoubleValue();
   }

   public double getMinNormalizedDistanceFromOutsideEdge()
   {
      return minNormalizedDistanceFromOutsideEdge.getDoubleValue();
   }

   public double getMinNormalizedDistanceFromInsideEdge()
   {
      return minNormalizedDistanceFromInsideEdge.getDoubleValue();
   }

   public double getMinOrthogonalDistanceFromOutsideEdge()
   {
      return minOrthogonalDistanceFromOutsideEdge.getDoubleValue();
   }

   public double getMinDistanceFromInsideEdge()
   {
      return minDistanceFromInsideEdge.getDoubleValue();
   }

   public double getMinOrthogonalDistanceFromInsideEdge()
   {
      return minOrthogonalDistanceFromInsideEdge.getDoubleValue();
   }

   public double getMaxNecessaryNormalizedError()
   {
      return maxNecessaryNormalizedError.getDoubleValue();
   }

   public double getMaxRatioOfControlDecreaseFromToeingOff()
   {
      return maxRatioOfControlDecreaseFromToeingOff.getDoubleValue();
   }

   public void attachParameterChangeListener(YoVariableChangedListener changedListener)
   {
      distanceForwardFromHeel.addListener(changedListener);
      minLateralDistance.addListener(changedListener);
      minDistanceFromTheToe.addListener(changedListener);
      minFractionOfStrideFromTheToe.addListener(changedListener);
      minDistanceFromOutsideEdge.addListener(changedListener);
      minOrthogonalDistanceFromOutsideEdge.addListener(changedListener);
      minDistanceFromInsideEdge.addListener(changedListener);
      minOrthogonalDistanceFromInsideEdge.addListener(changedListener);
      minNormalizedDistanceFromInsideEdge.addListener(changedListener);
      minNormalizedDistanceFromOutsideEdge.addListener(changedListener);
      maxRatioOfControlDecreaseFromToeingOff.addListener(changedListener);
      maxNecessaryNormalizedError.addListener(changedListener);
   }
}
