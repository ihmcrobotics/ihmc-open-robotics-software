package us.ihmc.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.FootstepPlannerCostParametersPacket;
import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoFootstepPlannerParameters implements FootstepPlannerParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoBoolean checkForBodyBoxCollisions = new YoBoolean("checkForBodyBoxCollisions", registry);
   private final YoBoolean performHeuristicSearchPolicies = new YoBoolean("performHeuristicSearchPolicies", registry);
   private final YoDouble maximumStepReach = new YoDouble("maximumStepReach", registry);
   private final YoDouble minimumFootholdPercent = new YoDouble("minimumFootholdPercent", registry);
   private final YoDouble idealFootstepLength = new YoDouble("idealFootstepLength", registry);
   private final YoDouble idealFootstepWidth = new YoDouble("idealFootstepWidth", registry);
   private final YoDouble maximumStepZ = new YoDouble("maximumStepZ", registry);
   private final YoDouble maximumStepYaw = new YoDouble("maximumStepYaw", registry);
   private final YoDouble maximumStepWidth = new YoDouble("maximumStepWidth", registry);
   private final YoDouble minimumStepWidth = new YoDouble("minimumStepWidth", registry);
   private final YoDouble minimumStepLength = new YoDouble("minimumStepLength", registry);
   private final YoDouble minimumStepYaw = new YoDouble("minimumStepYaw", registry);
   private final YoDouble maximumStepReachWhenSteppingUp= new YoDouble("maximumStepReachWhenSteppingUp", registry);
   private final YoDouble maximumStepZWhenSteppingUp= new YoDouble("maximumStepZWhenSteppingUp", registry);
   private final YoDouble maximumStepXWhenForwardAndDown = new YoDouble("maximumStepXWhenForwardAndDown", registry);
   private final YoDouble maximumStepZWhenForwardAndDown = new YoDouble("maximumStepZWhenForwardAndDown", registry);
   private final YoDouble wiggleInsideDelta = new YoDouble("wiggleInsideDelta", registry);
   private final YoBoolean rejectIfCannotFullyWiggleInside = new YoBoolean("rejectIfCannotFullyWiggleInside", registry);
   private final YoBoolean wiggleIntoConvexHullOfPlanarRegions = new YoBoolean("WiggleIntoConvexHullOfPlanarRegions", registry);
   private final YoDouble maximumXYWiggleDistance = new YoDouble("maximumXYWiggleDistance", registry);
   private final YoDouble maximumYawWiggle = new YoDouble("maximumYawWiggle", registry);
   private final YoDouble cliffHeightToShiftAwayFrom = new YoDouble("cliffHeightToShiftAwayFrom", registry);
   private final YoDouble minimumDistanceFromCliffBottoms = new YoDouble("minimumDistanceFromCliffBottoms", registry);
   private final YoDouble minimumSurfaceInclineRadians = new YoDouble("minimumSurfaceInclineRadians", registry);
   private final YoDouble maximumZPenetrationOnValleyRegions = new YoDouble("maximumZPenetrationOnValleyRegions", registry);
   private final YoDouble bodyGroundClearance = new YoDouble("bodyGroundClearance", registry);
   private final YoDouble bodyBoxHeight = new YoDouble("bodyBoxHeight", registry);
   private final YoDouble bodyBoxWidth = new YoDouble("bodyBoxWidth", registry);
   private final YoDouble bodyBoxDepth = new YoDouble("bodyBoxDepth", registry);
   private final YoDouble bodyBoxBaseX = new YoDouble("bodyBoxBaseX", registry);
   private final YoDouble bodyBoxBaseY = new YoDouble("bodyBoxBaseY", registry);
   private final YoDouble bodyBoxBaseZ = new YoDouble("bodyBoxBaseZ", registry);
   private final YoBoolean returnBestEffortPlan = new YoBoolean("returnBestEffortPlan", registry);
   private final YoInteger minimumStepForBestEffortPlan = new YoInteger("minimumStepForBestEffortPlan", registry);
   private final YoDouble minXClearanceFromStance = new YoDouble("minXClearanceFromStance", registry);
   private final YoDouble minYClearanceFromStance = new YoDouble("minYClearanceFromStance", registry);
   private final YoDouble goalTurnRadius = new YoDouble("goalTurnRadius", registry);

   private final YoFootstepPlannerCostParameters costParameters;

   public YoFootstepPlannerParameters(YoVariableRegistry parentRegistry, FootstepPlannerParameters defaults)
   {
      costParameters = new YoFootstepPlannerCostParameters(registry, defaults.getCostParameters());

      parentRegistry.addChild(registry);
      set(defaults);
   }

   public void set(FootstepPlannerParameters defaults)
   {
      setCheckForBodyBoxCollisions(defaults.checkForBodyBoxCollisions());
      setPerformHeuristicSearchPolicies(defaults.performHeuristicSearchPolicies());
      maximumStepReach.set(defaults.getMaximumStepReach());
      minimumFootholdPercent.set(defaults.getMinimumFootholdPercent());
      idealFootstepLength.set(defaults.getIdealFootstepLength());
      idealFootstepWidth.set(defaults.getIdealFootstepWidth());
      maximumStepZ.set(defaults.getMaximumStepZ());
      maximumStepYaw.set(defaults.getMaximumStepYaw());
      maximumStepWidth.set(defaults.getMaximumStepWidth());
      minimumStepWidth.set(defaults.getMinimumStepWidth());
      minimumStepLength.set(defaults.getMinimumStepLength());
      minimumStepYaw.set(defaults.getMinimumStepYaw());
      maximumStepReachWhenSteppingUp.set(defaults.getMaximumStepReachWhenSteppingUp());
      maximumStepZWhenSteppingUp.set(defaults.getMaximumStepZWhenSteppingUp());
      maximumStepXWhenForwardAndDown.set(defaults.getMaximumStepXWhenForwardAndDown());
      maximumStepZWhenForwardAndDown.set(defaults.getMaximumStepZWhenForwardAndDown());
      wiggleInsideDelta.set(defaults.getWiggleInsideDelta());
      rejectIfCannotFullyWiggleInside.set(defaults.getRejectIfCannotFullyWiggleInside());
      wiggleIntoConvexHullOfPlanarRegions.set(defaults.getWiggleIntoConvexHullOfPlanarRegions());
      maximumXYWiggleDistance.set(defaults.getMaximumXYWiggleDistance());
      maximumYawWiggle.set(defaults.getMaximumYawWiggle());
      cliffHeightToShiftAwayFrom.set(defaults.getCliffHeightToAvoid());
      minimumDistanceFromCliffBottoms.set(defaults.getMinimumDistanceFromCliffBottoms());
      minimumSurfaceInclineRadians.set(defaults.getMinimumSurfaceInclineRadians());
      maximumZPenetrationOnValleyRegions.set(defaults.getMaximumZPenetrationOnValleyRegions());
      bodyGroundClearance.set(defaults.getBodyGroundClearance());
      bodyBoxDepth.set(defaults.getBodyBoxDepth());
      bodyBoxHeight.set(defaults.getBodyBoxHeight());
      bodyBoxWidth.set(defaults.getBodyBoxWidth());
      bodyBoxBaseX.set(defaults.getBodyBoxBaseX());
      bodyBoxBaseY.set(defaults.getBodyBoxBaseY());
      bodyBoxBaseZ.set(defaults.getBodyBoxBaseZ());
      returnBestEffortPlan.set(defaults.getReturnBestEffortPlan());
      minimumStepForBestEffortPlan.set(defaults.getMinimumStepsForBestEffortPlan());
      minXClearanceFromStance.set(defaults.getMinXClearanceFromStance());
      minYClearanceFromStance.set(defaults.getMinYClearanceFromStance());
      goalTurnRadius.set(defaults.getGoalTurnRadius());

      costParameters.set(defaults.getCostParameters());
   }

   @Override
   public boolean checkForBodyBoxCollisions()
   {
      return checkForBodyBoxCollisions.getBooleanValue();
   }

   @Override
   public boolean performHeuristicSearchPolicies()
   {
      return performHeuristicSearchPolicies.getBooleanValue();
   }

   @Override
   public double getIdealFootstepWidth()
   {
      return idealFootstepWidth.getDoubleValue();
   }

   @Override
   public double getIdealFootstepLength()
   {
      return idealFootstepLength.getDoubleValue();
   }

   @Override
   public double getWiggleInsideDelta()
   {
      return wiggleInsideDelta.getDoubleValue();
   }

   @Override
   public double getMaximumStepReach()
   {
      return maximumStepReach.getDoubleValue();
   }

   @Override
   public double getMaximumStepYaw()
   {
      return maximumStepYaw.getDoubleValue();
   }

   @Override
   public double getMinimumStepWidth()
   {
      return minimumStepWidth.getDoubleValue();
   }

   @Override
   public double getMinimumStepLength()
   {
      return minimumStepLength.getDoubleValue();
   }

   @Override
   public double getMinimumStepYaw()
   {
      return minimumStepYaw.getDoubleValue();
   }

   @Override
   public double getMaximumStepReachWhenSteppingUp()
   {
      return maximumStepReachWhenSteppingUp.getDoubleValue();
   }

   @Override
   public double getMaximumStepZWhenSteppingUp()
   {
      return maximumStepZWhenSteppingUp.getDoubleValue();
   }

   @Override
   public double getMaximumStepXWhenForwardAndDown()
   {
      return maximumStepXWhenForwardAndDown.getDoubleValue();
   }

   @Override
   public double getMaximumStepZWhenForwardAndDown()
   {
      return maximumStepZWhenForwardAndDown.getDoubleValue();
   }

   @Override
   public double getMaximumStepZ()
   {
      return maximumStepZ.getDoubleValue();
   }

   @Override
   public double getMinimumFootholdPercent()
   {
      return minimumFootholdPercent.getDoubleValue();
   }

   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minimumSurfaceInclineRadians.getDoubleValue();
   }

   @Override
   public boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return wiggleIntoConvexHullOfPlanarRegions.getBooleanValue();
   }

   @Override
   public boolean getRejectIfCannotFullyWiggleInside()
   {
      return rejectIfCannotFullyWiggleInside.getBooleanValue();
   }

   @Override
   public double getMaximumXYWiggleDistance()
   {
      return maximumXYWiggleDistance.getDoubleValue();
   }

   @Override
   public double getMaximumYawWiggle()
   {
      return maximumYawWiggle.getDoubleValue();
   }

   @Override
   public double getMaximumZPenetrationOnValleyRegions()
   {
      return maximumZPenetrationOnValleyRegions.getDoubleValue();
   }

   @Override
   public double getMaximumStepWidth()
   {
      return maximumStepWidth.getDoubleValue();
   }

   @Override
   public double getCliffHeightToAvoid()
   {
      return cliffHeightToShiftAwayFrom.getDoubleValue();
   }

   @Override
   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimumDistanceFromCliffBottoms.getDoubleValue();
   }

   @Override
   public double getBodyGroundClearance()
   {
      return bodyGroundClearance.getDoubleValue();
   }

   @Override
   public double getBodyBoxHeight()
   {
      return bodyBoxHeight.getDoubleValue();
   }

   @Override
   public double getBodyBoxDepth()
   {
      return bodyBoxDepth.getDoubleValue();
   }

   @Override
   public double getBodyBoxWidth()
   {
      return bodyBoxWidth.getDoubleValue();
   }

   @Override
   public double getBodyBoxBaseX()
   {
      return bodyBoxBaseX.getDoubleValue();
   }

   @Override
   public double getBodyBoxBaseY()
   {
      return bodyBoxBaseY.getDoubleValue();
   }

   @Override
   public double getBodyBoxBaseZ()
   {
      return bodyBoxBaseZ.getDoubleValue();
   }

   @Override
   public boolean getReturnBestEffortPlan()
   {
      return returnBestEffortPlan.getBooleanValue();
   }

   @Override
   public int getMinimumStepsForBestEffortPlan()
   {
      return minimumStepForBestEffortPlan.getIntegerValue();
   }

   @Override
   public double getMinXClearanceFromStance()
   {
      return minXClearanceFromStance.getDoubleValue();
   }

   @Override
   public double getMinYClearanceFromStance()
   {
      return minYClearanceFromStance.getDoubleValue();
   }

   @Override
   public FootstepPlannerCostParameters getCostParameters()
   {
      return costParameters;
   }

   @Override
   public double getGoalTurnRadius()
   {
      return goalTurnRadius.getDoubleValue();
   }

   public void set(FootstepPlannerParametersPacket parametersPacket)
   {
      setCheckForBodyBoxCollisions(parametersPacket.getCheckForBodyBoxCollisions());
      setPerformHeuristicSearchPolicies(parametersPacket.getPerformHeuristicSearchPolicies());
      if (parametersPacket.getIdealFootstepWidth() != -1.0)
         setIdealFootstepWidth(parametersPacket.getIdealFootstepWidth());
      if (parametersPacket.getIdealFootstepLength() != -1.0)
         setIdealFootstepLength(parametersPacket.getIdealFootstepLength());
      if (parametersPacket.getWiggleInsideDelta() != -1.0)
         setWiggleInsideDelta(parametersPacket.getWiggleInsideDelta());
      if (parametersPacket.getMaximumStepReach() != -1.0)
         setMaximumStepReach(parametersPacket.getMaximumStepReach());
      if (parametersPacket.getMaximumStepYaw() != 1.0)
         setMaximumStepYaw(parametersPacket.getMaximumStepYaw());
      if (parametersPacket.getMinimumStepWidth() != 1.0)
         setMinimumStepWidth(parametersPacket.getMinimumStepWidth());
      if (parametersPacket.getMinimumStepLength() != -1.0)
         setMinimumStepLength(parametersPacket.getMinimumStepLength());
      if (parametersPacket.getMinimumStepYaw() != -1.0)
         setMinimumStepYaw(parametersPacket.getMinimumStepYaw());
      if (parametersPacket.getMaximumStepReachWhenSteppingUp() != -1.0)
         setMaximumStepReachWhenSteppingUp(parametersPacket.getMaximumStepReachWhenSteppingUp());
      if (parametersPacket.getMaximumStepZWhenSteppingUp() != -1.0)
         setMaximumStepZWhenSteppingUp(parametersPacket.getMaximumStepZWhenSteppingUp());
      if (parametersPacket.getMaximumStepXWhenForwardAndDown() != -1.0)
         setMaximumStepXWhenForwardAndDown(parametersPacket.getMaximumStepXWhenForwardAndDown());
      if (parametersPacket.getMaximumStepZWhenForwardAndDown() != -1.0)
         setMaximumStepZWhenForwardAndDown(parametersPacket.getMaximumStepZWhenForwardAndDown());
      if (parametersPacket.getMaximumStepZ() != -1.0)
         setMaximumStepZ(parametersPacket.getMaximumStepZ());
      if (parametersPacket.getMinimumFootholdPercent() != -1.0)
         setMinimumFootholdPercent(parametersPacket.getMinimumFootholdPercent());
      if (parametersPacket.getMinimumSurfaceInclineRadians() != -1.0)
         setMinimumSurfaceInclineRadians(parametersPacket.getMinimumSurfaceInclineRadians());
      setWiggleIntoConvexHullOfPlanarRegions(parametersPacket.getWiggleIntoConvexHullOfPlanarRegions());
      setRejectIfCannotFullyWiggleInside(parametersPacket.getRejectIfCannotFullyWiggleInside());
      if (parametersPacket.getMaximumXyWiggleDistance() != -1.0)
         setMaximumXYWiggleDistance(parametersPacket.getMaximumXyWiggleDistance());
      if (parametersPacket.getMaximumYawWiggle() != -1.0)
         setMaximumYawWiggle(parametersPacket.getMaximumYawWiggle());
      if (parametersPacket.getMaximumZPenetrationOnValleyRegions() != -1.0)
         setMaximumZPenetrationOnValleyRegions(parametersPacket.getMaximumZPenetrationOnValleyRegions());
      if (parametersPacket.getMaximumStepWidth() != -1.0)
         setMaximumStepWidth(parametersPacket.getMaximumStepWidth());
      if (parametersPacket.getCliffHeightToAvoid() != -1.0)
         setCliffHeightToShiftAwayFrom(parametersPacket.getCliffHeightToAvoid());
      if (parametersPacket.getMinimumDistanceFromCliffBottoms() != -1.0)
         setMinimumDistanceFromCliffBottoms(parametersPacket.getMinimumDistanceFromCliffBottoms());
      setReturnBestEffortPlan(parametersPacket.getReturnBestEffortPlan());
      if (parametersPacket.getMinimumStepsForBestEffortPlan() > 0)
         setMinimumStepForBestEffortPlan((int) parametersPacket.getMinimumStepsForBestEffortPlan());
      if (parametersPacket.getBodyGroundClearance() != -1.0)
         setBodyGroundClearance(parametersPacket.getBodyGroundClearance());
      if (parametersPacket.getBodyBoxHeight() != -1.0)
         setBodyBoxHeight(parametersPacket.getBodyBoxHeight());
      if (parametersPacket.getBodyBoxDepth() != -1.0)
         setBodyBoxDepth(parametersPacket.getBodyBoxDepth());         
      if (parametersPacket.getBodyBoxWidth() != -1.0)
         setBodyBoxWidth(parametersPacket.getBodyBoxWidth());
      if (parametersPacket.getBodyBoxBaseX() != -1.0)
         setBodyBoxBaseX(parametersPacket.getBodyBoxBaseX());
      if (parametersPacket.getBodyBoxBaseY() != -1.0)
         setBodyBoxBaseY(parametersPacket.getBodyBoxBaseY());
      if (parametersPacket.getBodyBoxBaseZ() != -1.0)
         setBodyBoxBaseZ(parametersPacket.getBodyBoxBaseZ());
      if (parametersPacket.getMinXClearanceFromStance() != -1.0)
         setMinXClearanceFromStance(parametersPacket.getMinXClearanceFromStance());
      if (parametersPacket.getMinYClearanceFromStance() != -1.0)
         setMinYClearanceFromStance(parametersPacket.getMinYClearanceFromStance());
      setGoalTurnRadius(parametersPacket.getGoalTurnRadius());

      setCostParameters(parametersPacket.getCostParameters());
   }

   public void setCheckForBodyBoxCollisions(boolean checkForBodyBoxCollisions)
   {
      this.checkForBodyBoxCollisions.set(checkForBodyBoxCollisions);
   }

   public void setPerformHeuristicSearchPolicies(boolean performHeuristicSearchPolicies)
   {
      this.performHeuristicSearchPolicies.set(performHeuristicSearchPolicies);
   }

   public void setIdealFootstepWidth(double idealFootstepWidth)
   {
      this.idealFootstepWidth.set(idealFootstepWidth);
   }

   public void setIdealFootstepLength(double idealFootstepLength)
   {
      this.idealFootstepLength.set(idealFootstepLength);
   }

   public void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      this.wiggleInsideDelta.set(wiggleInsideDelta);
   }

   public void setMaximumStepReach(double maximumStepReach)
   {
      this.maximumStepReach.set(maximumStepReach);
   }

   public void setMaximumStepYaw(double maximumStepYaw)
   {
      this.maximumStepYaw.set(maximumStepYaw);
   }

   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth.set(minimumStepWidth);
   }

   public void setMinimumStepLength(double minimumStepLength)
   {
      this.minimumStepLength.set(minimumStepLength);
   }

   public void setMinimumStepYaw(double minimumStepYaw)
   {
      this.minimumStepYaw.set(minimumStepYaw);
   }

   public void setMaximumStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      this.maximumStepReachWhenSteppingUp.set(maximumStepReachWhenSteppingUp);
   }

   public void setMaximumStepZWhenSteppingUp(double maximumStepZWhenSteppingUp)
   {
      this.maximumStepZWhenSteppingUp.set(maximumStepZWhenSteppingUp);
   }

   public void setMaximumStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      this.maximumStepXWhenForwardAndDown.set(maximumStepXWhenForwardAndDown);
   }

   public void setMaximumStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      this.maximumStepZWhenForwardAndDown.set(maximumStepZWhenForwardAndDown);
   }

   public void setMaximumStepZ(double maximumStepZ)
   {
      this.maximumStepZ.set(maximumStepZ);
   }

   public void setMinimumFootholdPercent(double minimumFootholdPercent)
   {
      this.minimumFootholdPercent.set(minimumFootholdPercent);
   }

   public void setMinimumSurfaceInclineRadians(double minimumSurfaceInclineRadians)
   {
      this.minimumSurfaceInclineRadians.set(minimumSurfaceInclineRadians);
   }

   public void setWiggleIntoConvexHullOfPlanarRegions(boolean wiggleIntoConvexHullOfPlanarRegions)
   {
      this.wiggleIntoConvexHullOfPlanarRegions.set(wiggleIntoConvexHullOfPlanarRegions);
   }

   public void setRejectIfCannotFullyWiggleInside(boolean rejectIfCannotFullyWiggleInside)
   {
      this.rejectIfCannotFullyWiggleInside.set(rejectIfCannotFullyWiggleInside);
   }

   public void setMaximumXYWiggleDistance(double maximumXYWiggleDistance)
   {
      this.maximumXYWiggleDistance.set(maximumXYWiggleDistance);
   }

   public void setMaximumYawWiggle(double maximumYawWiggle)
   {
      this.maximumYawWiggle.set(maximumYawWiggle);
   }

   public void setMaximumZPenetrationOnValleyRegions(double maximumZPenetrationOnValleyRegions)
   {
      this.maximumZPenetrationOnValleyRegions.set(maximumZPenetrationOnValleyRegions);
   }

   public void setMaximumStepWidth(double maximumStepWidth)
   {
      this.maximumStepWidth.set(maximumStepWidth);
   }

   public void setCliffHeightToShiftAwayFrom(double cliffHeightToAvoid)
   {
      this.cliffHeightToShiftAwayFrom.set(cliffHeightToAvoid);
   }

   public void setMinimumDistanceFromCliffBottoms(double minimumDistanceFromCliffBottoms)
   {
      this.minimumDistanceFromCliffBottoms.set(minimumDistanceFromCliffBottoms);
   }

   public void setReturnBestEffortPlan(boolean returnBestEffortPlan)
   {
      this.returnBestEffortPlan.set(returnBestEffortPlan);
   }

   public void setMinimumStepForBestEffortPlan(int minimumStepForBestEffortPlan)
   {
      this.minimumStepForBestEffortPlan.set(minimumStepForBestEffortPlan);
   }

   public void setBodyGroundClearance(double bodyGroundClearance)
   {
      this.bodyGroundClearance.set(bodyGroundClearance);
   }

   public void setBodyBoxHeight(double bodyBoxHeight)
   {
      this.bodyBoxHeight.set(bodyBoxHeight);
   }

   public void setBodyBoxDepth(double bodyBoxDepth)
   {
      this.bodyBoxDepth.set(bodyBoxDepth);
   }

   public void setBodyBoxWidth(double bodyBoxWidth)
   {
      this.bodyBoxWidth.set(bodyBoxWidth);
   }

   public void setBodyBoxBaseX(double bodyBoxBaseZ)
   {
      this.bodyBoxBaseX.set(bodyBoxBaseZ);
   }

   public void setBodyBoxBaseY(double bodyBoxBaseZ)
   {
      this.bodyBoxBaseY.set(bodyBoxBaseZ);
   }

   public void setBodyBoxBaseZ(double bodyBoxBaseZ)
   {
      this.bodyBoxBaseZ.set(bodyBoxBaseZ);
   }

   public void setMinXClearanceFromStance(double minXClearanceFromStance)
   {
      this.minXClearanceFromStance.set(minXClearanceFromStance);
   }

   public void setMinYClearanceFromStance(double minYClearanceFromStance)
   {
      this.minYClearanceFromStance.set(minYClearanceFromStance);
   }

   public void setGoalTurnRadius(double goalTurnRadius)
   {
      this.goalTurnRadius.set(goalTurnRadius);
   }

   public void setCostParameters(FootstepPlannerCostParameters parameters)
   {
      this.costParameters.set(parameters);
   }

   public void setCostParameters(FootstepPlannerCostParametersPacket packet)
   {
      this.costParameters.set(packet);
   }
}
