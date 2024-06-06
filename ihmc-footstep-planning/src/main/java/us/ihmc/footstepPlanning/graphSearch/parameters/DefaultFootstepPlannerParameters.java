package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class DefaultFootstepPlannerParameters extends StoredPropertySet implements DefaultFootstepPlannerParametersBasics
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey astarHeuristicsWeight = keys.addDoubleKey("AStar heuristics weight");
   public static final IntegerStoredPropertyKey maxBranchFactor = keys.addIntegerKey("Max branch factor");
   public static final BooleanStoredPropertyKey enableExpansionMask = keys.addBooleanKey("Enable expansion mask");
   public static final DoubleStoredPropertyKey idealFootstepWidth = keys.addDoubleKey("Ideal footstep width");
   public static final DoubleStoredPropertyKey idealFootstepLength = keys.addDoubleKey("Ideal footstep length");
   public static final DoubleStoredPropertyKey idealSideStepWidth = keys.addDoubleKey("Ideal side step width");
   public static final DoubleStoredPropertyKey idealBackStepLength = keys.addDoubleKey("Ideal back step length");
   public static final DoubleStoredPropertyKey idealStepLengthAtMaxStepZ = keys.addDoubleKey("Ideal step length at max step z");
   public static final BooleanStoredPropertyKey useReachabilityMap = keys.addBooleanKey("Use reachability map");
   public static final DoubleStoredPropertyKey solutionQualityThreshold = keys.addDoubleKey("Solution quality threshold");
   public static final DoubleStoredPropertyKey minStepWidth = keys.addDoubleKey("Min step width");
   public static final DoubleStoredPropertyKey minStepLength = keys.addDoubleKey("Min step length");
   public static final DoubleStoredPropertyKey minSurfaceIncline = keys.addDoubleKey("Min surface incline");
   public static final DoubleStoredPropertyKey minStepYaw = keys.addDoubleKey("Min step yaw");
   public static final DoubleStoredPropertyKey minStepZWhenFullyPitched = keys.addDoubleKey("Min step Z when fully pitched");
   public static final DoubleStoredPropertyKey minFootholdPercent = keys.addDoubleKey("Min foothold percent");
   public static final DoubleStoredPropertyKey minClearanceFromStance = keys.addDoubleKey("Min clearance from stance");
   public static final DoubleStoredPropertyKey rmsErrorThreshold = keys.addDoubleKey("RMS Error Threshold");
   public static final DoubleStoredPropertyKey rmsErrorCost = keys.addDoubleKey("RMS Error Cost");
   public static final DoubleStoredPropertyKey rmsMinErrorToPenalize = keys.addDoubleKey("RMS Min Error To Penalize");
   public static final DoubleStoredPropertyKey heightMapSnapThreshold = keys.addDoubleKey("Height map snap threshold");
   public static final DoubleStoredPropertyKey maxStepWidth = keys.addDoubleKey("Max step width");
   public static final DoubleStoredPropertyKey maxStepReach = keys.addDoubleKey("Max step reach");
   public static final DoubleStoredPropertyKey maxStepYaw = keys.addDoubleKey("Max step yaw");
   public static final DoubleStoredPropertyKey maxStepXWhenFullyPitched = keys.addDoubleKey("Max step X when fully pitched");
   public static final DoubleStoredPropertyKey maxStepXWhenForwardAndDown = keys.addDoubleKey("Max step X when forward and down");
   public static final DoubleStoredPropertyKey maxStepYWhenForwardAndDown = keys.addDoubleKey("Max step Y when forward and down");
   public static final DoubleStoredPropertyKey maxStepZWhenForwardAndDown = keys.addDoubleKey("Max step Z when forward and down");
   public static final DoubleStoredPropertyKey maxStepReachWhenSteppingUp = keys.addDoubleKey("Max step reach when stepping up");
   public static final DoubleStoredPropertyKey maxStepWidthWhenSteppingUp = keys.addDoubleKey("Max step width when stepping up");
   public static final DoubleStoredPropertyKey maxStepZWhenSteppingUp = keys.addDoubleKey("Max step Z when stepping up");
   public static final DoubleStoredPropertyKey maxStepZ = keys.addDoubleKey("Max step Z");
   public static final DoubleStoredPropertyKey maxSwingZ = keys.addDoubleKey("Max swing Z");
   public static final DoubleStoredPropertyKey maxSwingReach = keys.addDoubleKey("Max swing reach");
   public static final DoubleStoredPropertyKey stepYawReductionFactorAtMaxReach = keys.addDoubleKey("Step yaw reduction factor at max reach");
   public static final DoubleStoredPropertyKey yawWeight = keys.addDoubleKey("Yaw weight");
   public static final DoubleStoredPropertyKey forwardWeight = keys.addDoubleKey("Forward weight");
   public static final DoubleStoredPropertyKey lateralWeight = keys.addDoubleKey("Lateral weight");
   public static final DoubleStoredPropertyKey costPerStep = keys.addDoubleKey("Cost per step");
   public static final DoubleStoredPropertyKey stepUpWeight = keys.addDoubleKey("Step up weight");
   public static final DoubleStoredPropertyKey stepDownWeight = keys.addDoubleKey("Step down weight");
   public static final DoubleStoredPropertyKey rollWeight = keys.addDoubleKey("Roll weight");
   public static final DoubleStoredPropertyKey pitchWeight = keys.addDoubleKey("Pitch weight");
   public static final DoubleStoredPropertyKey footholdAreaWeight = keys.addDoubleKey("Foothold area weight");
   public static final DoubleStoredPropertyKey referencePlanAlpha = keys.addDoubleKey("Reference plan alpha");
   public static final DoubleStoredPropertyKey wiggleInsideDeltaTarget = keys.addDoubleKey("Wiggle inside delta target");
   public static final DoubleStoredPropertyKey wiggleInsideDeltaMinimum = keys.addDoubleKey("Wiggle inside delta minimum");
   public static final BooleanStoredPropertyKey enableConcaveHullWiggler = keys.addBooleanKey("Enable concave hull wiggler");
   public static final BooleanStoredPropertyKey wiggleWhilePlanning = keys.addBooleanKey("Wiggle while planning");
   public static final DoubleStoredPropertyKey maxXYWiggleDistance = keys.addDoubleKey("Max XY wiggle distance");
   public static final DoubleStoredPropertyKey maxYawWiggle = keys.addDoubleKey("Max yaw wiggle");
   public static final DoubleStoredPropertyKey maxZPenetrationOnValleyRegions = keys.addDoubleKey("Max Z penetration on valley regions");
   public static final DoubleStoredPropertyKey maximumSnapHeight = keys.addDoubleKey("Maximum snap height");
   public static final DoubleStoredPropertyKey finalTurnProximity = keys.addDoubleKey("Final turn proximity");
   public static final DoubleStoredPropertyKey distanceFromPathTolerance = keys.addDoubleKey("Distance from path tolerance");
   public static final DoubleStoredPropertyKey deltaYawFromReferenceTolerance = keys.addDoubleKey("Delta yaw from reference tolerance");
   public static final BooleanStoredPropertyKey checkForBodyBoxCollisions = keys.addBooleanKey("Check for body box collisions");
   public static final DoubleStoredPropertyKey bodyBoxWidth = keys.addDoubleKey("Body box width");
   public static final DoubleStoredPropertyKey bodyBoxHeight = keys.addDoubleKey("Body box height");
   public static final DoubleStoredPropertyKey bodyBoxDepth = keys.addDoubleKey("Body box depth");
   public static final DoubleStoredPropertyKey bodyBoxBaseX = keys.addDoubleKey("Body box base X");
   public static final DoubleStoredPropertyKey bodyBoxBaseY = keys.addDoubleKey("Body box base Y");
   public static final DoubleStoredPropertyKey bodyBoxBaseZ = keys.addDoubleKey("Body box base Z");
   public static final IntegerStoredPropertyKey intermediateBodyBoxChecks = keys.addIntegerKey("Intermediate body box checks");
   public static final BooleanStoredPropertyKey enableShinCollisionCheck = keys.addBooleanKey("Enable shin collision check");
   public static final DoubleStoredPropertyKey shinLength = keys.addDoubleKey("Shin length");
   public static final DoubleStoredPropertyKey shinToeClearance = keys.addDoubleKey("Shin toe clearance");
   public static final DoubleStoredPropertyKey shinHeelClearance = keys.addDoubleKey("Shin heel clearance");
   public static final DoubleStoredPropertyKey shinHeightOffset = keys.addDoubleKey("Shin height offset");
   public static final BooleanStoredPropertyKey checkForPathCollisions = keys.addBooleanKey("Check for path collisions");
   public static final DoubleStoredPropertyKey cliffBottomHeightToAvoid = keys.addDoubleKey("Cliff bottom height to avoid");
   public static final DoubleStoredPropertyKey minDistanceFromCliffBottoms = keys.addDoubleKey("Min distance from cliff bottoms");
   public static final DoubleStoredPropertyKey cliffTopHeightToAvoid = keys.addDoubleKey("Cliff top height to avoid");
   public static final DoubleStoredPropertyKey minDistanceFromCliffTops = keys.addDoubleKey("Min distance from cliff tops");
   public static final DoubleStoredPropertyKey scaledFootPolygonPercentage = keys.addDoubleKey("Scaled foot polygon percentage");
   public static final DoubleStoredPropertyKey cliffHeightThreshold = keys.addDoubleKey("Cliff height threshold");


   public DefaultFootstepPlannerParameters() // for tests and stuff that's probably not gonna save
   {
      this(null);
   }

   private DefaultFootstepPlannerParameters(DefaultFootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      super(keys, DefaultFootstepPlannerParameters.class);

      if (footstepPlannerParameters != null)
      {
         set(footstepPlannerParameters);
      }
      else
      {
         loadUnsafe();
      }
   }

   /**
    * Run to update file with new parameters.
    */
   public static void main(String[] args)
   {
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      parameters.save();
   }
}
