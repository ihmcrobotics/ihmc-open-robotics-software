package us.ihmc.footstepPlanning.graphSearch.parameters;

public class SettableFootstepPlannerParameters implements FootstepPlannerParameters
{
   public static final String CONFIGURATION_FILE_NAME = "./saved-configurations/footstepPlannerParameters.txt";

   private double idealFootstepWidth;
   private double idealFootstepLength;

   private double wiggleInsideDelta;
   private boolean wiggleIntoConvexHullOfPlanarRegions;
   private boolean rejectIfCannotFullyWiggleInside;
   private double maximumXYWiggleDistance;
   private double maximumYawWiggle;

   private double maxStepReach;
   private double maxStepYaw;
   private double minStepWidth;
   private double minStepLength;
   private double minStepYaw;
   private double maxStepZ;
   private double minFootholdPercent;
   private double minSurfaceIncline;
   private double maxStepWidth;
   private double minXClearanceFromStance;
   private double minYClearanceFromStance;

   private double maximumStepReachWhenSteppingUp;
   private double maximumStepZWhenSteppingUp;
   private double maximumStepXWhenForwardAndDown;
   private double maximumStepZWhenForwardAndDown;
   private double maximumZPenetrationOnValleyRegions;
   private double cliffHeightToAvoid;
   private double minimumDistanceFromCliffBottoms;
   private double goalTurnRadius;

   private boolean returnBestEffortPlan;
   private int minimumStepsForBestEffortPlan;

   private double bodyGroundClearance;
   private boolean checkForBodyBoxCollisions;
   private boolean performHeuristicSearchPolicies;
   private double bodyBoxWidth;
   private double bodyBoxHeight;
   private double bodyBoxDepth;
   private double bodyBoxBaseX;
   private double bodyBoxBaseY;
   private double bodyBoxBaseZ;

   private final SettableFootstepPlannerCostParameters costParameters;

   public SettableFootstepPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      this.costParameters = new SettableFootstepPlannerCostParameters(footstepPlannerParameters.getCostParameters());

      set(footstepPlannerParameters);
   }

   public void set(FootstepPlannerParameters footstepPlannerParameters)
   {
      this.idealFootstepWidth = footstepPlannerParameters.getIdealFootstepWidth();
      this.idealFootstepLength = footstepPlannerParameters.getIdealFootstepLength();

      this.wiggleInsideDelta = footstepPlannerParameters.getWiggleInsideDelta();
      this.wiggleIntoConvexHullOfPlanarRegions = footstepPlannerParameters.getWiggleIntoConvexHullOfPlanarRegions();
      this.rejectIfCannotFullyWiggleInside = footstepPlannerParameters.getRejectIfCannotFullyWiggleInside();
      this.maximumXYWiggleDistance = footstepPlannerParameters.getMaximumXYWiggleDistance();
      this.maximumYawWiggle = footstepPlannerParameters.getMaximumYawWiggle();

      this.maxStepReach = footstepPlannerParameters.getMaximumStepReach();
      this.maxStepYaw = footstepPlannerParameters.getMaximumStepYaw();
      this.minStepWidth = footstepPlannerParameters.getMinimumStepWidth();
      this.minStepLength = footstepPlannerParameters.getMinimumStepLength();
      this.minStepYaw = footstepPlannerParameters.getMinimumStepYaw();
      this.maxStepZ = footstepPlannerParameters.getMaximumStepZ();
      this.maxStepWidth = footstepPlannerParameters.getMaximumStepWidth();
      this.minFootholdPercent = footstepPlannerParameters.getMinimumFootholdPercent();
      this.minSurfaceIncline = footstepPlannerParameters.getMinimumSurfaceInclineRadians();
      this.minXClearanceFromStance = footstepPlannerParameters.getMinXClearanceFromStance();
      this.minYClearanceFromStance = footstepPlannerParameters.getMinYClearanceFromStance();

      this.maximumStepReachWhenSteppingUp = footstepPlannerParameters.getMaximumStepReachWhenSteppingUp();
      this.maximumStepZWhenSteppingUp = footstepPlannerParameters.getMaximumStepZWhenSteppingUp();
      this.maximumStepXWhenForwardAndDown = footstepPlannerParameters.getMaximumStepXWhenForwardAndDown();
      this.maximumStepZWhenForwardAndDown = footstepPlannerParameters.getMaximumStepZWhenForwardAndDown();
      this.maximumZPenetrationOnValleyRegions = footstepPlannerParameters.getMaximumZPenetrationOnValleyRegions();
      this.cliffHeightToAvoid = footstepPlannerParameters.getCliffHeightToAvoid();
      this.minimumDistanceFromCliffBottoms = footstepPlannerParameters.getMinimumDistanceFromCliffBottoms();
      this.goalTurnRadius = footstepPlannerParameters.getGoalTurnRadius();

      this.returnBestEffortPlan = footstepPlannerParameters.getReturnBestEffortPlan();
      this.minimumStepsForBestEffortPlan = footstepPlannerParameters.getMinimumStepsForBestEffortPlan();

      this.bodyGroundClearance = footstepPlannerParameters.getBodyGroundClearance();
      this.checkForBodyBoxCollisions = footstepPlannerParameters.checkForBodyBoxCollisions();
      this.performHeuristicSearchPolicies = footstepPlannerParameters.performHeuristicSearchPolicies();
      this.bodyBoxHeight = footstepPlannerParameters.getBodyBoxHeight();
      this.bodyBoxWidth = footstepPlannerParameters.getBodyBoxWidth();
      this.bodyBoxDepth = footstepPlannerParameters.getBodyBoxDepth();
      this.bodyBoxBaseX = footstepPlannerParameters.getBodyBoxBaseX();
      this.bodyBoxBaseY = footstepPlannerParameters.getBodyBoxBaseY();
      this.bodyBoxBaseZ = footstepPlannerParameters.getBodyBoxBaseZ();

      this.costParameters.set(footstepPlannerParameters.getCostParameters());
   }

   public void setIdealFootstepWidth(double idealFootstepWidth)
   {
      this.idealFootstepWidth = idealFootstepWidth;
   }

   public void setIdealFootstepLength(double idealFootstepLength)
   {
      this.idealFootstepLength = idealFootstepLength;
   }

   public void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      this.wiggleInsideDelta = wiggleInsideDelta;
   }

   public void setWiggleIntoConvexHullOfPlanarRegions(boolean wiggleIntoConvexHullOfPlanarRegions)
   {
      this.wiggleIntoConvexHullOfPlanarRegions = wiggleIntoConvexHullOfPlanarRegions;
   }

   public void setRejectIfCannotFullyWiggleInside(boolean rejectIfCannotFullyWiggleInside)
   {
      this.rejectIfCannotFullyWiggleInside = rejectIfCannotFullyWiggleInside;
   }

   public void setMaximumXYWiggleDistance(double maximumXYWiggleDistance)
   {
      this.maximumXYWiggleDistance = maximumXYWiggleDistance;
   }

   public void setMaximumYawWiggle(double maximumYawWiggle)
   {
      this.maximumYawWiggle = maximumYawWiggle;
   }

   public void setMaximumStepReach(double maxStepReach)
   {
      this.maxStepReach = maxStepReach;
   }

   public void setMaximumStepYaw(double maxStepYaw)
   {
      this.maxStepYaw = maxStepYaw;
   }

   public void setMinimumStepWidth(double minStepWidth)
   {
      this.minStepWidth = minStepWidth;
   }

   public void setMinimumStepLength(double minStepLength)
   {
      this.minStepLength = minStepLength;
   }

   public void setMinimumStepYaw(double minStepYaw)
   {
      this.minStepYaw = minStepYaw;
   }

   public void setMaximumStepZ(double maxStepZ)
   {
      this.maxStepZ = maxStepZ;
   }

   public void setMaximumStepWidth(double maxStepWidth)
   {
      this.maxStepWidth = maxStepWidth;
   }

   public void setMinimumFootholdPercent(double minFootholdPercent)
   {
      this.minFootholdPercent = minFootholdPercent;
   }

   public void setMinimumSurfaceInclineRadians(double minSurfaceIncline)
   {
      this.minSurfaceIncline = minSurfaceIncline;
   }

   public void setMaximumStepReachWhenSteppingUp(double maximumStepReachWhenSteppingUp)
   {
      this.maximumStepReachWhenSteppingUp = maximumStepReachWhenSteppingUp;
   }

   public void setMaximumStepZWhenSteppingUp(double maximumStepZWhenSteppingUp)
   {
      this.maximumStepZWhenSteppingUp = maximumStepZWhenSteppingUp;
   }

   public void setMaximumStepXWhenForwardAndDown(double maximumStepXWhenForwardAndDown)
   {
      this.maximumStepXWhenForwardAndDown = maximumStepXWhenForwardAndDown;
   }

   public void setMaximumStepZWhenForwardAndDown(double maximumStepZWhenForwardAndDown)
   {
      this.maximumStepZWhenForwardAndDown = maximumStepZWhenForwardAndDown;
   }

   public void setMinXClearanceFromStance(double minXClearanceFromStance)
   {
      this.minXClearanceFromStance = minXClearanceFromStance;
   }

   public void setMinYClearanceFromStance(double minYClearanceFromStance)
   {
      this.minYClearanceFromStance = minYClearanceFromStance;
   }

   public void setMaximumZPenetrationOnValleyRegions(double maximumZPenetrationOnValleyRegions)
   {
      this.maximumZPenetrationOnValleyRegions = maximumZPenetrationOnValleyRegions;
   }

   public void setCliffHeightToAvoid(double cliffHeightToAvoid)
   {
      this.cliffHeightToAvoid = cliffHeightToAvoid;
   }

   public void setMinimumDistanceFromCliffBottoms(double minimumDistanceFromCliffBottoms)
   {
      this.minimumDistanceFromCliffBottoms = minimumDistanceFromCliffBottoms;
   }

   public void setReturnBestEffortPlan(boolean returnBestEffortPlan)
   {
      this.returnBestEffortPlan = returnBestEffortPlan;
   }

   public void setMinimumStepsForBestEffortPlan(int minimumStepsForBestEffortPlan)
   {
      this.minimumStepsForBestEffortPlan = minimumStepsForBestEffortPlan;
   }

   public void setYawWeight(double yawWeight)
   {
      costParameters.setYawWeight(yawWeight);
   }

   public void setPitchWeight(double pitchWeight)
   {
      costParameters.setPitchWeight(pitchWeight);
   }

   public void setRollWeight(double rollWeight)
   {
      costParameters.setRollWeight(rollWeight);
   }

   public void setForwardWeight(double forwardWeight)
   {
      costParameters.setForwardWeight(forwardWeight);
   }

   public void setLateralWeight(double lateralWeight)
   {
      costParameters.setLateralWeight(lateralWeight);
   }

   public void setStepUpWeight(double stepUpWeight)
   {
      costParameters.setStepUpWeight(stepUpWeight);
   }

   public void setStepDownWeight(double stepDownWeight)
   {
      costParameters.setStepDownWeight(stepDownWeight);
   }

   public void setUseQuadraticDistanceCost(boolean useQuadraticDistanceCost)
   {
      costParameters.setUseQuadraticDistanceCost(useQuadraticDistanceCost);
   }

   public void setUseQuadraticHeightCost(boolean useQuadraticHeightCost)
   {
      costParameters.setUseQuadraticHeightCost(useQuadraticHeightCost);
   }

   public void setCostPerStep(double costPerStep)
   {
      costParameters.setCostPerStep(costPerStep);
   }

   public void setAStarHeuristicsWeight(double heuristicsWeight)
   {
      costParameters.setAStarHeuristicsWeight(heuristicsWeight);
   }

   public void setVisGraphWithAStarHeuristicsWeight(double heuristicsWeight)
   {
      costParameters.setVisGraphWithAStarHeuristicsWeight(heuristicsWeight);
   }

   public void setDepthFirstHeuristicsWeight(double heuristicsWeight)
   {
      costParameters.setDepthFirstHeuristicsWeight(heuristicsWeight);
   }

   public void setBodyPathBasedHeuristicsWeight(double heuristicsWeight)
   {
      costParameters.setBodyPathBasedHeuristicsWeight(heuristicsWeight);
   }

   public void setBodyGroundClearance(double bodyGroundClearance)
   {
      this.bodyGroundClearance = bodyGroundClearance;
   }

   public void setCheckForBodyBoxCollisions(boolean checkForBodyBoxCollisions)
   {
      this.checkForBodyBoxCollisions = checkForBodyBoxCollisions;
   }

   public void setPerformHeuristicSearchPolicies(boolean performHeuristicSearchPolicies)
   {
      this.performHeuristicSearchPolicies = performHeuristicSearchPolicies;
   }

   public void setBodyBoxWidth(double bodyBoxWidth)
   {
      this.bodyBoxWidth = bodyBoxWidth;
   }

   public void setBodyBoxHeight(double bodyBoxHeight)
   {
      this.bodyBoxHeight = bodyBoxHeight;
   }

   public void setBodyBoxDepth(double bodyBoxDepth)
   {
      this.bodyBoxDepth = bodyBoxDepth;
   }

   public void setBodyBoxBaseX(double bodyBoxBaseX)
   {
      this.bodyBoxBaseX = bodyBoxBaseX;
   }

   public void setBodyBoxBaseY(double bodyBoxBaseY)
   {
      this.bodyBoxBaseY = bodyBoxBaseY;
   }

   public void setBodyBoxBaseZ(double bodyBoxBaseZ)
   {
      this.bodyBoxBaseZ = bodyBoxBaseZ;
   }

   public void setMaximum2dDistanceFromBoundingBoxToPenalize(double maximum2dDistanceFromBoundingBoxToPenalize)
   {
      this.costParameters.setMaximum2dDistanceFromBoundingBoxToPenalize(maximum2dDistanceFromBoundingBoxToPenalize);
   }

   public void setBoundingBoxCost(double boundingBoxCost)
   {
      this.costParameters.setBoundingBoxCost(boundingBoxCost);
   }

   public void setGoalTurnRadius(double goalTurnRadius)
   {
      this.goalTurnRadius = goalTurnRadius;
   }

   @Override
   public double getIdealFootstepWidth()
   {
      return idealFootstepWidth;
   }

   @Override
   public double getIdealFootstepLength()
   {
      return idealFootstepLength;
   }

   @Override
   public double getWiggleInsideDelta()
   {
      return wiggleInsideDelta;
   }

   @Override
   public boolean getWiggleIntoConvexHullOfPlanarRegions()
   {
      return wiggleIntoConvexHullOfPlanarRegions;
   }

   @Override
   public boolean getRejectIfCannotFullyWiggleInside()
   {
      return rejectIfCannotFullyWiggleInside;
   }

   @Override
   public double getMaximumXYWiggleDistance()
   {
      return maximumXYWiggleDistance;
   }

   @Override
   public double getMaximumYawWiggle()
   {
      return maximumYawWiggle;
   }

   @Override
   public double getMaximumStepReach()
   {
      return maxStepReach;
   }

   @Override
   public double getMaximumStepYaw()
   {
      return maxStepYaw;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return minStepWidth;
   }

   @Override
   public double getMinimumStepLength()
   {
      return minStepLength;
   }

   @Override
   public double getMinimumStepYaw()
   {
      return minStepYaw;
   }

   @Override
   public double getMaximumStepZ()
   {
      return maxStepZ;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return maxStepWidth;
   }

   @Override
   public double getMaximumStepReachWhenSteppingUp()
   {
      return maximumStepReachWhenSteppingUp;
   }

   @Override
   public double getMaximumStepZWhenSteppingUp()
   {
      return maximumStepZWhenSteppingUp;
   }

   @Override
   public double getMaximumStepXWhenForwardAndDown()
   {
      return maximumStepXWhenForwardAndDown;
   }

   @Override
   public double getMaximumStepZWhenForwardAndDown()
   {
      return maximumStepZWhenForwardAndDown;
   }

   @Override
   public double getMinXClearanceFromStance()
   {
      return minXClearanceFromStance;
   }

   @Override
   public double getMinYClearanceFromStance()
   {
      return minYClearanceFromStance;
   }

   @Override
   public double getMinimumFootholdPercent()
   {
      return minFootholdPercent;
   }

   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minSurfaceIncline;
   }

   @Override
   public double getMaximumZPenetrationOnValleyRegions()
   {
      return maximumZPenetrationOnValleyRegions;
   }

   @Override
   public double getCliffHeightToAvoid()
   {
      return cliffHeightToAvoid;
   }

   @Override
   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimumDistanceFromCliffBottoms;
   }

   @Override
   public boolean getReturnBestEffortPlan()
   {
      return returnBestEffortPlan;
   }

   @Override
   public int getMinimumStepsForBestEffortPlan()
   {
      return minimumStepsForBestEffortPlan;
   }

   @Override
   public double getBodyGroundClearance()
   {
      return bodyGroundClearance;
   }

   @Override
   public boolean checkForBodyBoxCollisions()
   {
      return checkForBodyBoxCollisions;
   }

   @Override
   public boolean performHeuristicSearchPolicies()
   {
      return performHeuristicSearchPolicies;
   }

   @Override
   public double getBodyBoxHeight()
   {
      return bodyBoxHeight;
   }

   @Override
   public double getBodyBoxWidth()
   {
      return bodyBoxWidth;
   }

   @Override
   public double getBodyBoxDepth()
   {
      return bodyBoxDepth;
   }

   @Override
   public double getBodyBoxBaseX()
   {
      return bodyBoxBaseX;
   }

   @Override
   public double getBodyBoxBaseY()
   {
      return bodyBoxBaseY;
   }

   @Override
   public double getBodyBoxBaseZ()
   {
      return bodyBoxBaseZ;
   }

   @Override
   public double getGoalTurnRadius()
   {
      return goalTurnRadius;
   }

   public boolean useQuadraticDistanceCost()
   {
      return costParameters.useQuadraticDistanceCost();
   }

   public boolean useQuadraticHeightCost()
   {
      return costParameters.useQuadraticHeightCost();
   }

   public double getYawWeight()
   {
      return costParameters.getYawWeight();
   }

   public double getPitchWeight()
   {
      return costParameters.getPitchWeight();
   }

   public double getRollWeight()
   {
      return costParameters.getRollWeight();
   }

   public double getForwardWeight()
   {
      return costParameters.getForwardWeight();
   }

   public double getLateralWeight()
   {
      return costParameters.getLateralWeight();
   }

   public double getStepUpWeight()
   {
      return costParameters.getStepUpWeight();
   }

   public double getStepDownWeight()
   {
      return costParameters.getStepDownWeight();
   }

   public double getCostPerStep()
   {
      return costParameters.getCostPerStep();
   }

   public double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return costParameters.getMaximum2dDistanceFromBoundingBoxToPenalize();
   }

   public double getBoundingBoxCost()
   {
      return costParameters.getBoundingBoxCost();
   }

   public double getAStarHeuristicsWeight()
   {
      return costParameters.getAStarHeuristicsWeight().getValue();
   }

   public double getVisGraphWithAStarHeuristicsWeight()
   {
      return costParameters.getVisGraphWithAStarHeuristicsWeight().getValue();
   }

   public double getDepthFirstHeuristicsWeight()
   {
      return costParameters.getDepthFirstHeuristicsWeight().getValue();
   }

   public double getBodyPathBasedHeuristicsWeight()
   {
      return costParameters.getBodyPathBasedHeuristicsWeight().getValue();
   }

   @Override
   public SettableFootstepPlannerCostParameters getCostParameters()
   {
      return costParameters;
   }
}
