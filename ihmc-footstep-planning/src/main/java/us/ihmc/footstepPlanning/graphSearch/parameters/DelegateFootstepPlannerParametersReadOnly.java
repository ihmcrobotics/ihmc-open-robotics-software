package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.List;

/**
 * Allows for easily swapping different parameter sets while not changing the API of existing code.
 */
public class DelegateFootstepPlannerParametersReadOnly implements FootstepPlannerParametersReadOnly
{
   private FootstepPlannerParametersReadOnly parameters;

   public void setParameters(FootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public FootstepPlannerParametersReadOnly getParameters()
   {
      return parameters;
   }

   public DoubleProvider getAStarHeuristicsWeight()
   {
      return parameters.getAStarHeuristicsWeight();
   }

   public int getMaximumBranchFactor()
   {
      return parameters.getMaximumBranchFactor();
   }

   public boolean getEnabledExpansionMask()
   {
      return parameters.getEnabledExpansionMask();
   }

   public double getIdealFootstepWidth()
   {
      return parameters.getIdealFootstepWidth();
   }

   public double getIdealFootstepLength()
   {
      return parameters.getIdealFootstepLength();
   }

   public double getIdealSideStepWidth()
   {
      return parameters.getIdealSideStepWidth();
   }

   public double getIdealBackStepLength()
   {
      return parameters.getIdealBackStepLength();
   }

   public double getIdealStepLengthAtMaxStepZ()
   {
      return parameters.getIdealStepLengthAtMaxStepZ();
   }

   public boolean getUseStepReachabilityMap()
   {
      return parameters.getUseStepReachabilityMap();
   }

   public double getSolutionQualityThreshold()
   {
      return parameters.getSolutionQualityThreshold();
   }

   public double getMinimumStepWidth()
   {
      return parameters.getMinimumStepWidth();
   }

   public double getMinimumStepLength()
   {
      return parameters.getMinimumStepLength();
   }

   public double getMinimumSurfaceInclineRadians()
   {
      return parameters.getMinimumSurfaceInclineRadians();
   }

   public double getMinimumStepYaw()
   {
      return parameters.getMinimumStepYaw();
   }

   public double getMinimumStepZWhenFullyPitched()
   {
      return parameters.getMinimumStepZWhenFullyPitched();
   }

   public double getMinimumFootholdPercent()
   {
      return parameters.getMinimumFootholdPercent();
   }

   public double getMinClearanceFromStance()
   {
      return parameters.getMinClearanceFromStance();
   }

   public double getRMSErrorThreshold()
   {
      return parameters.getRMSErrorThreshold();
   }

   public double getRMSErrorCost()
   {
      return parameters.getRMSErrorCost();
   }

   public double getRMSMinErrorToPenalize()
   {
      return parameters.getRMSMinErrorToPenalize();
   }

   public double getHeightMapSnapThreshold()
   {
      return parameters.getHeightMapSnapThreshold();
   }

   public double getMaximumStepWidth()
   {
      return parameters.getMaximumStepWidth();
   }

   public double getMaximumStepReach()
   {
      return parameters.getMaximumStepReach();
   }

   public double getMaximumStepYaw()
   {
      return parameters.getMaximumStepYaw();
   }

   public double getMaximumStepXWhenFullyPitched()
   {
      return parameters.getMaximumStepXWhenFullyPitched();
   }

   public double getMaximumStepXWhenForwardAndDown()
   {
      return parameters.getMaximumStepXWhenForwardAndDown();
   }

   public double getMaximumStepYWhenForwardAndDown()
   {
      return parameters.getMaximumStepYWhenForwardAndDown();
   }

   public double getMaximumStepZWhenForwardAndDown()
   {
      return parameters.getMaximumStepZWhenForwardAndDown();
   }

   public double getMaximumStepReachWhenSteppingUp()
   {
      return parameters.getMaximumStepReachWhenSteppingUp();
   }

   public double getMaximumStepWidthWhenSteppingUp()
   {
      return parameters.getMaximumStepWidthWhenSteppingUp();
   }

   public double getMaximumStepZWhenSteppingUp()
   {
      return parameters.getMaximumStepZWhenSteppingUp();
   }

   public double getMaxStepZ()
   {
      return parameters.getMaxStepZ();
   }

   public double getMaxSwingZ()
   {
      return parameters.getMaxSwingZ();
   }

   public double getMaxSwingReach()
   {
      return parameters.getMaxSwingReach();
   }

   public double getStepYawReductionFactorAtMaxReach()
   {
      return parameters.getStepYawReductionFactorAtMaxReach();
   }

   public double getYawWeight()
   {
      return parameters.getYawWeight();
   }

   public double getForwardWeight()
   {
      return parameters.getForwardWeight();
   }

   public double getLateralWeight()
   {
      return parameters.getLateralWeight();
   }

   public double getCostPerStep()
   {
      return parameters.getCostPerStep();
   }

   public double getStepUpWeight()
   {
      return parameters.getStepUpWeight();
   }

   public double getStepDownWeight()
   {
      return parameters.getStepDownWeight();
   }

   public double getRollWeight()
   {
      return parameters.getRollWeight();
   }

   public double getPitchWeight()
   {
      return parameters.getPitchWeight();
   }

   public double getFootholdAreaWeight()
   {
      return parameters.getFootholdAreaWeight();
   }

   public double getReferencePlanAlpha()
   {
      return parameters.getReferencePlanAlpha();
   }

   public double getWiggleInsideDeltaTarget()
   {
      return parameters.getWiggleInsideDeltaTarget();
   }

   public double getWiggleInsideDeltaMinimum()
   {
      return parameters.getWiggleInsideDeltaMinimum();
   }

   public boolean getEnableConcaveHullWiggler()
   {
      return parameters.getEnableConcaveHullWiggler();
   }

   public boolean getWiggleWhilePlanning()
   {
      return parameters.getWiggleWhilePlanning();
   }

   public double getMaximumXYWiggleDistance()
   {
      return parameters.getMaximumXYWiggleDistance();
   }

   public double getMaximumYawWiggle()
   {
      return parameters.getMaximumYawWiggle();
   }

   public double getMaximumZPenetrationOnValleyRegions()
   {
      return parameters.getMaximumZPenetrationOnValleyRegions();
   }

   public double getMaximumSnapHeight()
   {
      return parameters.getMaximumSnapHeight();
   }

   public double getFinalTurnProximity()
   {
      return parameters.getFinalTurnProximity();
   }

   public double getDistanceFromPathTolerance()
   {
      return parameters.getDistanceFromPathTolerance();
   }

   public double getDeltaYawFromReferenceTolerance()
   {
      return parameters.getDeltaYawFromReferenceTolerance();
   }

   public boolean checkForBodyBoxCollisions()
   {
      return parameters.checkForBodyBoxCollisions();
   }

   public double getBodyBoxHeight()
   {
      return parameters.getBodyBoxHeight();
   }

   public double getBodyBoxDepth()
   {
      return parameters.getBodyBoxDepth();
   }

   public double getBodyBoxWidth()
   {
      return parameters.getBodyBoxWidth();
   }

   public double getBodyBoxBaseX()
   {
      return parameters.getBodyBoxBaseX();
   }

   public double getBodyBoxBaseY()
   {
      return parameters.getBodyBoxBaseY();
   }

   public double getBodyBoxBaseZ()
   {
      return parameters.getBodyBoxBaseZ();
   }

   public int getIntermediateBodyBoxChecks()
   {
      return parameters.getIntermediateBodyBoxChecks();
   }

   public boolean getEnableShinCollisionCheck()
   {
      return parameters.getEnableShinCollisionCheck();
   }

   public double getShinToeClearance()
   {
      return parameters.getShinToeClearance();
   }

   public double getShinHeelClearance()
   {
      return parameters.getShinHeelClearance();
   }

   public double getShinLength()
   {
      return parameters.getShinLength();
   }

   public double getShinHeightOffset()
   {
      return parameters.getShinHeightOffset();
   }

   public boolean checkForPathCollisions()
   {
      return parameters.checkForPathCollisions();
   }

   public double getCliffBaseHeightToAvoid()
   {
      return parameters.getCliffBaseHeightToAvoid();
   }

   public double getMinimumDistanceFromCliffBottoms()
   {
      return parameters.getMinimumDistanceFromCliffBottoms();
   }

   public double getCliffTopHeightToAvoid()
   {
      return parameters.getCliffTopHeightToAvoid();
   }

   public double getMinimumDistanceFromCliffTops()
   {
      return parameters.getMinimumDistanceFromCliffTops();
   }

   @Override
   public double get(DoubleStoredPropertyKey key)
   {
      return parameters.get(key);
   }

   @Override
   public int get(IntegerStoredPropertyKey key)
   {
      return parameters.get(key);
   }

   @Override
   public boolean get(BooleanStoredPropertyKey key)
   {
      return parameters.get(key);
   }

   @Override
   public <T> T get(StoredPropertyKey<T> key)
   {
      return parameters.get(key);
   }

   @Override
   public <T> StoredPropertyReadOnly<T> getProperty(StoredPropertyKey<T> key)
   {
      return parameters.getProperty(key);
   }

   @Override
   public List<Object> getAll()
   {
      return parameters.getAll();
   }

   @Override
   public List<String> getAllAsStrings()
   {
      return parameters.getAllAsStrings();
   }

   @Override
   public String getTitle()
   {
      return parameters.getTitle();
   }

   @Override
   public String getCurrentVersionSuffix()
   {
      return parameters.getCurrentVersionSuffix();
   }

   @Override
   public String getCapitalizedClassName()
   {
      return parameters.getCapitalizedClassName();
   }
}
