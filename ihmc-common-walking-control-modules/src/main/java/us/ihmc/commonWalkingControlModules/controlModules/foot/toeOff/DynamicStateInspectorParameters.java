package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.yoVariables.listener.YoParameterChangedListener;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class DynamicStateInspectorParameters
{
   /** This checks to make sure the ICP isn't falling to the outside of the trailing foot. **/
   private final DoubleParameter minLateralDistanceInside;

   /**
    * These variables make sure the ICP is far enough from the toe off point. If they're far enough, then there's probably enough control
    * authority to control them
    */
   private final DoubleParameter minDistanceFromTheToe;
   private final DoubleParameter minFractionOfStrideFromTheToe;

   private final DoubleParameter minDistanceAlongErrorFromOutsideEdge;
   private final DoubleParameter minOrthogonalDistanceFromOutsideEdge;
   private final DoubleParameter minDistanceAlongErrorFromInsideEdge;
   private final DoubleParameter minOrthogonalDistanceFromInsideEdge;

   private final DoubleParameter minNormalizedDistanceFromOutsideEdge;
   private final DoubleParameter minNormalizedDistanceFromInsideEdge;
   private final DoubleParameter maxRatioOfControlDecreaseFromToeingOff;
   private final DoubleParameter maxNormalizedErrorNeededForControl;

   private final DoubleParameter maxDistanceToMoveECMP;
   private final DoubleParameter loadThresholdToCheckCoP;
   private final DoubleParameter maxDistanceToMoveCoP;

   public DynamicStateInspectorParameters(YoRegistry parentRegistry)
   {
      this("", parentRegistry);
   }

   public DynamicStateInspectorParameters(String suffix, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName() + suffix);

      minLateralDistanceInside = new DoubleParameter("minLatDistInside" + suffix, registry, 0.05);

      minDistanceFromTheToe = new DoubleParameter("minDistanceFromToe" + suffix, registry, 0.05);
      minFractionOfStrideFromTheToe = new DoubleParameter("minFractionOfStrideFromToe" + suffix, registry, 0.5);

      minDistanceAlongErrorFromOutsideEdge = new DoubleParameter("minDistAlongErrorFromOutEdge" + suffix, registry, -0.025);
      minOrthogonalDistanceFromOutsideEdge = new DoubleParameter("minOrthoDistFromOutEdge" + suffix, registry, -0.035);
      minDistanceAlongErrorFromInsideEdge = new DoubleParameter("minDistAlongErrorFromInEdge" + suffix, registry, -0.01);
      minOrthogonalDistanceFromInsideEdge = new DoubleParameter("minOrthoDistFromInEdge" + suffix, registry, -0.025);

      minNormalizedDistanceFromOutsideEdge = new DoubleParameter("minNormDistFromOutEdge" + suffix, registry, 0.35);
      minNormalizedDistanceFromInsideEdge = new DoubleParameter("minNormDistFromInEdge" + suffix, registry, 0.3);
      maxRatioOfControlDecreaseFromToeingOff = new DoubleParameter("maxRatioOfControlDecreaseFromToeingOff" + suffix, registry, 2.0);
      maxNormalizedErrorNeededForControl = new DoubleParameter("maxNormErrorNeededForControl" + suffix, registry, 1.0);

      maxDistanceToMoveECMP = new DoubleParameter("maxDistanceToMoveECMP" + suffix, registry, 0.02);
      loadThresholdToCheckCoP = new DoubleParameter("loadThresholdToCheckCoP" + suffix, registry, 0.2);
      maxDistanceToMoveCoP = new DoubleParameter("maxDistanceToMoveCoP" + suffix, registry, 0.05);

      parentRegistry.addChild(registry);
   }

   public double getMinLateralDistanceInside()
   {
      return minLateralDistanceInside.getValue();
   }

   public double getMinDistanceFromTheToe()
   {
      return minDistanceFromTheToe.getValue();
   }

   public double getMinFractionOfStrideFromTheToe()
   {
      return minFractionOfStrideFromTheToe.getValue();
   }

   public double getMinDistanceAlongErrorFromOutsideEdge()
   {
      return minDistanceAlongErrorFromOutsideEdge.getValue();
   }

   public double getMinNormalizedDistanceFromOutsideEdge()
   {
      return minNormalizedDistanceFromOutsideEdge.getValue();
   }

   public double getMinNormalizedDistanceFromInsideEdge()
   {
      return minNormalizedDistanceFromInsideEdge.getValue();
   }

   public double getMinOrthogonalDistanceFromOutsideEdge()
   {
      return minOrthogonalDistanceFromOutsideEdge.getValue();
   }

   public double getMinDistanceAlongErrorFromInsideEdge()
   {
      return minDistanceAlongErrorFromInsideEdge.getValue();
   }

   public double getMinOrthogonalDistanceFromInsideEdge()
   {
      return minOrthogonalDistanceFromInsideEdge.getValue();
   }

   public double getMaxNormalizedErrorNeededForControl()
   {
      return maxNormalizedErrorNeededForControl.getValue();
   }

   public double getMaxRatioOfControlDecreaseFromToeingOff()
   {
      return maxRatioOfControlDecreaseFromToeingOff.getValue();
   }

   public double getMaxDistanceToMoveECMP()
   {
      return maxDistanceToMoveECMP.getValue();
   }

   public double getMaxDistanceToMoveCoP()
   {
      return maxDistanceToMoveCoP.getValue();
   }

   public double getLoadThresholdToCheckCoP()
   {
      return loadThresholdToCheckCoP.getValue();
   }

   public void attachParameterChangeListener(YoParameterChangedListener changedListener)
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
      maxDistanceToMoveECMP.addListener(changedListener);
      maxDistanceToMoveCoP.addListener(changedListener);
      loadThresholdToCheckCoP.addListener(changedListener);
   }
}
