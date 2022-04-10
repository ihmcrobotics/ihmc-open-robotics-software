package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicStateInspectorParameters
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   /**
    * If the ICP is this distance based the heel of the elading foot, toe off should happen, regardless of any of the other conditions.
    **/
   private final YoDouble distanceForwardFromHeel = new YoDouble("distanceForwardFromHeel", registry);

   /**
    * This checks to make sure the ICP isn't falling to the outside of the trailing foot.
    **/
   private final YoDouble minLateralDistance = new YoDouble("minLateralDistance", registry);

   /**
    * These variables make sure the ICP is far enough from the toe off point. If they're far enough, then there's probably enough control
    * authority to control them
    */
   private final YoDouble minDistanceFromTheToe = new YoDouble("minDistanceFromTheToe", registry);
   private final YoDouble minFractionOfStrideFromTheToe = new YoDouble("minFractionOfStrideFromTheToe", registry);

   private final YoDouble minDistanceFromOutsideEdge = new YoDouble("minDistanceFromOutsideEdge", registry);
   private final YoDouble minOrthogonalDistanceFromOutsideEdge = new YoDouble("minOrthogonalDistanceFromOutsideEdge", registry);
   private final YoDouble minDistanceFromInsideEdge = new YoDouble("minDistanceFromInsideEdge", registry);
   private final YoDouble minOrthogonalDistanceFromInsideEdge = new YoDouble("minOrthogonalDistanceFromInsideEdge", registry);

   public DynamicStateInspectorParameters(YoRegistry parentRegistry)
   {
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
   }
}
