package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFootstepPlannerParameters implements FootstepPlannerParametersBasics
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble maximumStepReach = new YoDouble("maximumStepReach", registry);
   private final YoDouble maximumStepLength = new YoDouble("maximumStepLength", registry);
   private final YoDouble minimumStepLength = new YoDouble("minimumStepLength", registry);
   private final YoDouble maximumStepWidth = new YoDouble("maximumStepWidth", registry);
   private final YoDouble minimumStepWidth = new YoDouble("minimumStepWidth", registry);
   private final YoDouble minimumStepYaw = new YoDouble("minimumStepYaw", registry);
   private final YoDouble maximumStepYaw = new YoDouble("maximumStepYaw", registry);
   private final YoDouble maximumStepChangeZ = new YoDouble("maximumStepChangeZ", registry);
   private final YoDouble bodyGroundClearance = new YoDouble("bodyGroundClearance", registry);
   private final YoDouble distanceHeuristicWeight = new YoDouble("distanceHeuristicWeight", registry);
   private final YoDouble yawWeight = new YoDouble("yawWeight", registry);
   private final YoDouble xGaitWeight = new YoDouble("xGaitWeight", registry);
   private final YoDouble costPerStep = new YoDouble("costPerStep", registry);
   private final YoDouble stepUpWeight = new YoDouble("stepUpWeight", registry);
   private final YoDouble stepDownWeight = new YoDouble("stepDownWeight", registry);
   private final YoDouble heuristicsInflationWeight = new YoDouble("heuristicsInflationWeight", registry);
   private final YoDouble minXClearanceFromFoot = new YoDouble("minXClearanceFromFoot", registry);
   private final YoDouble minYClearanceFromFoot = new YoDouble("minYClearanceFromFoot", registry);
   private final YoDouble crawlSpeed = new YoDouble("crawlSpeed", registry);
   private final YoDouble trotSpeed = new YoDouble("trotSpeed", registry);
   private final YoDouble paceSpeed = new YoDouble("paceSpeed", registry);
   private final YoDouble projectionInsideDistance = new YoDouble("projectionInsideDistance", registry);
   private final YoDouble minimumSurfaceInclineRadians = new YoDouble("minimumSurfaceInclineRadians", registry);
   private final YoDouble cliffHeightToAvoid = new YoDouble("cliffHeightToAvoid", registry);
   private final YoDouble minimumDistanceFromCliffBottoms = new YoDouble("minimumCliffHeightFromBottoms", registry);
   private final YoDouble minimumDistanceFromCliffTops = new YoDouble("minimumCliffHeightFromTops", registry);

   public YoFootstepPlannerParameters(FootstepPlannerParameters parameters, YoVariableRegistry parentRegistry)
   {
      set(parameters);
      parentRegistry.addChild(registry);
   }

   public void setMaximumStepReach(double maximumStepReach)
   {
      this.maximumStepReach.set(maximumStepReach);
   }

   public void setMaximumStepLength(double maximumStepLength)
   {
      this.maximumStepLength.set(maximumStepLength);
   }

   public void setMinimumStepLength(double minimumStepLength)
   {
      this.minimumStepLength.set(minimumStepLength);
   }

   public void setMaximumStepWidth(double maximumStepWidth)
   {
      this.maximumStepWidth.set(maximumStepWidth);
   }

   public void setMinimumStepWidth(double minimumStepWidth)
   {
      this.minimumStepWidth.set(minimumStepWidth);
   }

   public void setMinimumStepYaw(double minimumStepYaw)
   {
      this.minimumStepYaw.set(minimumStepYaw);
   }

   public void setMaximumStepYaw(double maximumStepYaw)
   {
      this.maximumStepYaw.set(maximumStepYaw);
   }

   public void setMaximumStepChangeZ(double maximumStepChangeZ)
   {
      this.maximumStepChangeZ.set(maximumStepChangeZ);
   }

   public void setBodyGroundClearance(double bodyGroundClearance)
   {
      this.bodyGroundClearance.set(bodyGroundClearance);
   }

   public void setDistanceHeuristicWeight(double distanceHeuristicWeight)
   {
      this.distanceHeuristicWeight.set(distanceHeuristicWeight);
   }

   public void setYawWeight(double yawWeight)
   {
      this.yawWeight.set(yawWeight);
   }

   public void setXGaitWeight(double xGaitWeight)
   {
      this.xGaitWeight.set(xGaitWeight);
   }

   public void setCostPerStep(double costPerStep)
   {
      this.costPerStep.set(costPerStep);
   }

   public void setStepUpWeight(double stepUpWeight)
   {
      this.stepUpWeight.set(stepUpWeight);
   }

   public void setStepDownWeight(double stepDownWeight)
   {
      this.stepDownWeight.set(stepDownWeight);
   }

   public void setHeuristicsInflationWeight(double heuristicsInflationWeight)
   {
      this.heuristicsInflationWeight.set(heuristicsInflationWeight);
   }

   public void setMinXClearanceFromFoot(double minXClearanceFromFoot)
   {
      this.minXClearanceFromFoot.set(minXClearanceFromFoot);
   }

   public void setMinYClearanceFromFoot(double minYClearanceFromFoot)
   {
      this.minYClearanceFromFoot.set(minYClearanceFromFoot);
   }

   public void setCrawlSpeed(double crawlSpeed)
   {
      this.crawlSpeed.set(crawlSpeed);
   }

   public void setTrotSpeed(double trotSpeed)
   {
      this.trotSpeed.set(trotSpeed);
   }

   public void setPaceSpeed(double paceSpeed)
   {
      this.paceSpeed.set(paceSpeed);
   }

   public void setProjectInsideDistance(double projectionInsideDistance)
   {
      this.projectionInsideDistance.set(projectionInsideDistance);
   }

   public void setMinimumSurfaceInclineRadians(double minimumSurfaceIncline)
   {
      this.minimumSurfaceInclineRadians.set(minimumSurfaceIncline);
   }

   public void setCliffHeightToAvoid(double cliffHeightToAvoid)
   {
      this.cliffHeightToAvoid.set(cliffHeightToAvoid);
   }

   public void setMinimumDistanceFromCliffBottoms(double distance)
   {
      minimumDistanceFromCliffBottoms.set(distance);
   }

   public void setMinimumDistanceFromCliffTops(double distance)
   {
      minimumDistanceFromCliffTops.set(distance);
   }

   /** {@inheritDoc} */
   public double getMaximumStepReach()
   {
      return maximumStepReach.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMaximumStepLength()
   {
      return maximumStepLength.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMinimumStepLength()
   {
      return minimumStepLength.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMaximumStepWidth()
   {
      return maximumStepWidth.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMinimumStepWidth()
   {
      return minimumStepWidth.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMinimumStepYaw()
   {
      return minimumStepYaw.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMaximumStepYaw()
   {
      return maximumStepYaw.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMaximumStepChangeZ()
   {
      return maximumStepChangeZ.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getBodyGroundClearance()
   {
      return bodyGroundClearance.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getDistanceHeuristicWeight()
   {
      return distanceHeuristicWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getYawWeight()
   {
      return yawWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getXGaitWeight()
   {
      return xGaitWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getCostPerStep()
   {
      return costPerStep.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getStepUpWeight()
   {
      return stepUpWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getStepDownWeight()
   {
      return stepDownWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getHeuristicsInflationWeight()
   {
      return heuristicsInflationWeight.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMinXClearanceFromFoot()
   {
      return minXClearanceFromFoot.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMinYClearanceFromFoot()
   {
      return minYClearanceFromFoot.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getCrawlSpeed()
   {
      return crawlSpeed.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getTrotSpeed()
   {
      return trotSpeed.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getPaceSpeed()
   {
      return paceSpeed.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getProjectInsideDistance()
   {
      return projectionInsideDistance.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMinimumSurfaceInclineRadians()
   {
      return minimumSurfaceInclineRadians.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getCliffHeightToAvoid()
   {
      return cliffHeightToAvoid.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMinimumDistanceFromCliffBottoms()
   {
      return minimumDistanceFromCliffBottoms.getDoubleValue();
   }

   /** {@inheritDoc} */
   public double getMinimumDistanceFromCliffTops()
   {
      return minimumDistanceFromCliffTops.getDoubleValue();
   }
}
