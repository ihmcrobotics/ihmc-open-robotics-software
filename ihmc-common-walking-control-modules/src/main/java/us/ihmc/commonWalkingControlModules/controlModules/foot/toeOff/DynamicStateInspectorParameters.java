package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicStateInspectorParameters
{
   /** This checks to make sure the ICP isn't falling to the outside of the trailing foot. **/
   private final YoDouble minLateralDistanceInside;

   /**
    * These variables make sure the ICP is far enough from the toe off point. If they're far enough, then there's probably enough control
    * authority to control them
    */
   private final YoDouble minDistanceFromTheToe;
   private final YoDouble minFractionOfStrideFromTheToe;

   private final YoDouble minDistanceAlongErrorFromOutsideEdge;
   private final YoDouble minOrthogonalDistanceFromOutsideEdge;
   private final YoDouble minDistanceAlongErrorFromInsideEdge;
   private final YoDouble minOrthogonalDistanceFromInsideEdge;

   private final YoDouble minNormalizedDistanceFromOutsideEdge;
   private final YoDouble minNormalizedDistanceFromInsideEdge;
   private final YoDouble maxRatioOfControlDecreaseFromToeingOff;
   private final YoDouble maxNormalizedErrorNeededForControl;

   public DynamicStateInspectorParameters(YoRegistry parentRegistry)
   {
      this("", parentRegistry);
   }

   public DynamicStateInspectorParameters(String suffix, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName() + suffix);

      minLateralDistanceInside = new YoDouble("minLatDistInside" + suffix, registry);

      minDistanceFromTheToe = new YoDouble("minDistanceFromToe" + suffix, registry);
      minFractionOfStrideFromTheToe = new YoDouble("minFractionOfStrideFromToe" + suffix, registry);

      minDistanceAlongErrorFromOutsideEdge = new YoDouble("minDistAlongErrorFromOutEdge" + suffix, registry);
      minOrthogonalDistanceFromOutsideEdge = new YoDouble("minOrthoDistFromOutEdge" + suffix, registry);
      minDistanceAlongErrorFromInsideEdge = new YoDouble("minDistAlongErrorFromInEdge" + suffix, registry);
      minOrthogonalDistanceFromInsideEdge = new YoDouble("minOrthoDistFromInEdge" + suffix, registry);

      minNormalizedDistanceFromOutsideEdge = new YoDouble("minNormDistFromOutEdge" + suffix, registry);
      minNormalizedDistanceFromInsideEdge = new YoDouble("minNormDistFromInEdge" + suffix, registry);
      maxRatioOfControlDecreaseFromToeingOff = new YoDouble("maxRatioOfControlDecreaseFromToeingOff" + suffix, registry);
      maxNormalizedErrorNeededForControl = new YoDouble("maxNormErrorNeededForControl" + suffix, registry);

      maxNormalizedErrorNeededForControl.set(1.0);
      maxRatioOfControlDecreaseFromToeingOff.set(2.0);

      minDistanceAlongErrorFromInsideEdge.set(-0.01);
      minOrthogonalDistanceFromInsideEdge.set(-0.0075);

      minDistanceAlongErrorFromOutsideEdge.set(-0.025);
      minOrthogonalDistanceFromOutsideEdge.set(-0.015);

      minNormalizedDistanceFromInsideEdge.set(0.3);
      minNormalizedDistanceFromOutsideEdge.set(0.35);

      minLateralDistanceInside.set(0.05);
      minFractionOfStrideFromTheToe.set(0.5);
      minDistanceFromTheToe.set(0.05);


      parentRegistry.addChild(registry);
   }

   public double getMinLateralDistanceInside()
   {
      return minLateralDistanceInside.getDoubleValue();
   }

   public double getMinDistanceFromTheToe()
   {
      return minDistanceFromTheToe.getDoubleValue();
   }

   public double getMinFractionOfStrideFromTheToe()
   {
      return minFractionOfStrideFromTheToe.getDoubleValue();
   }

   public double getMinDistanceAlongErrorFromOutsideEdge()
   {
      return minDistanceAlongErrorFromOutsideEdge.getDoubleValue();
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

   public double getMinDistanceAlongErrorFromInsideEdge()
   {
      return minDistanceAlongErrorFromInsideEdge.getDoubleValue();
   }

   public double getMinOrthogonalDistanceFromInsideEdge()
   {
      return minOrthogonalDistanceFromInsideEdge.getDoubleValue();
   }

   public double getMaxNormalizedErrorNeededForControl()
   {
      return maxNormalizedErrorNeededForControl.getDoubleValue();
   }

   public double getMaxRatioOfControlDecreaseFromToeingOff()
   {
      return maxRatioOfControlDecreaseFromToeingOff.getDoubleValue();
   }

   public void attachParameterChangeListener(YoVariableChangedListener changedListener)
   {
      minLateralDistanceInside.addListener(changedListener);
      minDistanceFromTheToe.addListener(changedListener);
      minFractionOfStrideFromTheToe.addListener(changedListener);
      minDistanceAlongErrorFromOutsideEdge.addListener(changedListener);
      minOrthogonalDistanceFromOutsideEdge.addListener(changedListener);
      minDistanceAlongErrorFromInsideEdge.addListener(changedListener);
      minOrthogonalDistanceFromInsideEdge.addListener(changedListener);
      minNormalizedDistanceFromInsideEdge.addListener(changedListener);
      minNormalizedDistanceFromOutsideEdge.addListener(changedListener);
      maxRatioOfControlDecreaseFromToeingOff.addListener(changedListener);
      maxNormalizedErrorNeededForControl.addListener(changedListener);
   }
}
