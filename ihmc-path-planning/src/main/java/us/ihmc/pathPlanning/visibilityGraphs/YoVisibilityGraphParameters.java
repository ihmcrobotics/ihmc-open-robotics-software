package us.ihmc.pathPlanning.visibilityGraphs;

import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoVisibilityGraphParameters implements VisibilityGraphsParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry("VisibilityGraphParameters");

   private final YoDouble maxInterRegionConnectionLength = new YoDouble("maxInterRegionConnectionLength", registry);
   private final YoDouble normalZThresholdForAccessibleRegions = new YoDouble("normalZThresholdForAccessibleRegions", registry);
   private final YoDouble regionOrthogonalAngle = new YoDouble("regionOrthogonalAngle", registry);
   private final YoDouble extrusionDistance = new YoDouble("extrusionDistance", registry);
   private final YoDouble preferredExtrusionDistance = new YoDouble("preferredExtrusionDistance", registry);
   private final YoDouble extrusionDistanceIfNotTooHighToStep = new YoDouble("extrusionDistanceIfNotTooHighToStep", registry);
   private final YoDouble tooHighToStepDistance = new YoDouble("tooHighToStepDistance", registry);
   private final YoDouble clusterResolution = new YoDouble("clusterResolution", registry);
   private final YoDouble explorationDistance = new YoDouble("explorationDistanceFromStartGoal", registry);
   private final YoDouble planarRegionMinArea = new YoDouble("planarRegionMinArea", registry);
   private final YoInteger planarRegionMinSize = new YoInteger("planarRegionMinSize", registry);
   private final YoDouble searchHostRegionEpsilon = new YoDouble("searchHostRegionEpsilon", registry);

   public YoVisibilityGraphParameters(VisibilityGraphsParameters defaults, YoVariableRegistry parentRegistry)
   {
      set(defaults);

      parentRegistry.addChild(registry);
   }

   public void set(VisibilityGraphsParameters parameters)
   {
      setMaxInterRegionConnectionLength(parameters.getMaxInterRegionConnectionLength());
      setNormalZThresholdForAccessibleRegions(parameters.getNormalZThresholdForAccessibleRegions());
      setRegionOrthogonalAngle(parameters.getRegionOrthogonalAngle());
      setExtrusionDistance(parameters.getObstacleExtrusionDistance());
      setPreferredExtrusionDistance(parameters.getPreferredObstacleExtrusionDistance());
      setExtrusionDistanceIfNotTooHighToStep(parameters.getObstacleExtrusionDistanceIfNotTooHighToStep());
      setTooHighToStepDistance(parameters.getTooHighToStepDistance());
      setClusterResolution(parameters.getClusterResolution());
      setExplorationDistanceFromStartGoal(parameters.getExplorationDistanceFromStartGoal());
      setPlanarRegionMinArea(parameters.getPlanarRegionMinArea());
      setPlanarRegionMinSize(parameters.getPlanarRegionMinSize());
      setSearchHostRegionEpsilon(parameters.getSearchHostRegionEpsilon());
   }

   public void set(VisibilityGraphsParametersPacket packet)
   {
      if (packet.getMaxInterRegionConnectionLength() != -1.0)
         setMaxInterRegionConnectionLength(packet.getMaxInterRegionConnectionLength());
      if (packet.getNormalZThresholdForAccessibleRegions() != -1.0)
         setNormalZThresholdForAccessibleRegions(packet.getNormalZThresholdForAccessibleRegions());
      if (packet.getExtrusionDistance() != -1.0)
         setExtrusionDistance(packet.getExtrusionDistance());
      if (packet.getPreferredExtrusionDistance() != -1.0)
         setPreferredExtrusionDistance(packet.getPreferredExtrusionDistance());
      if (packet.getExtrusionDistanceIfNotTooHighToStep() != -1.0)
         setExtrusionDistanceIfNotTooHighToStep(packet.getExtrusionDistanceIfNotTooHighToStep());
      if (packet.getTooHighToStepDistance() != -1.0)
         setTooHighToStepDistance(packet.getTooHighToStepDistance());
      if (packet.getClusterResolution() != -1.0)
         setClusterResolution(packet.getClusterResolution());
      if (packet.getExplorationDistanceFromStartGoal() != -1.0)
         setExplorationDistanceFromStartGoal(packet.getExplorationDistanceFromStartGoal());
      if (packet.getPlanarRegionMinArea() != -1.0)
         setPlanarRegionMinArea(packet.getPlanarRegionMinArea());
      if (packet.getPlanarRegionMinSize() != -1)
         setPlanarRegionMinSize((int) packet.getPlanarRegionMinSize());
      if (packet.getRegionOrthogonalAngle() != -1.0)
         setRegionOrthogonalAngle(packet.getRegionOrthogonalAngle());
      if (packet.getSearchHostRegionEpsilon() != -1.0)
         setSearchHostRegionEpsilon(packet.getSearchHostRegionEpsilon());
   }

   @Override
   public double getMaxInterRegionConnectionLength()
   {
      return maxInterRegionConnectionLength.getDoubleValue();
   }

   @Override
   public double getNormalZThresholdForAccessibleRegions()
   {
      return normalZThresholdForAccessibleRegions.getDoubleValue();
   }

   @Override
   public double getRegionOrthogonalAngle()
   {
      return regionOrthogonalAngle.getDoubleValue();
   }

   @Override
   public double getPreferredObstacleExtrusionDistance()
   {
      return preferredExtrusionDistance.getDoubleValue();
   }

   @Override
   public double getObstacleExtrusionDistance()
   {
      return extrusionDistance.getDoubleValue();
   }

   @Override
   public double getObstacleExtrusionDistanceIfNotTooHighToStep()
   {
      return extrusionDistanceIfNotTooHighToStep.getDoubleValue();
   }

   @Override
   public double getTooHighToStepDistance()
   {
      return tooHighToStepDistance.getDoubleValue();
   }

   @Override
   public double getClusterResolution()
   {
      return clusterResolution.getDoubleValue();
   }

   @Override
   public double getExplorationDistanceFromStartGoal()
   {
      return explorationDistance.getDoubleValue();
   }

   @Override
   public double getPlanarRegionMinArea()
   {
      return planarRegionMinArea.getDoubleValue();
   }

   @Override
   public int getPlanarRegionMinSize()
   {
      return planarRegionMinSize.getIntegerValue();
   }

   @Override
   public double getSearchHostRegionEpsilon()
   {
      return searchHostRegionEpsilon.getDoubleValue();
   }

   public void setMaxInterRegionConnectionLength(double maxInterRegionConnectionLength)
   {
      this.maxInterRegionConnectionLength.set(maxInterRegionConnectionLength);
   }

   public void setNormalZThresholdForAccessibleRegions(double normalZThresholdForAccessibleRegions)
   {
      this.normalZThresholdForAccessibleRegions.set(normalZThresholdForAccessibleRegions);
   }

   public void setRegionOrthogonalAngle(double regionOrthogonalAngle)
   {
      this.regionOrthogonalAngle.set(regionOrthogonalAngle);
   }

   public void setPreferredExtrusionDistance(double preferredExtrusionDistance)
   {
      this.preferredExtrusionDistance.set(preferredExtrusionDistance);
   }

   public void setExtrusionDistance(double extrusionDistance)
   {
      this.extrusionDistance.set(extrusionDistance);
   }

   public void setExtrusionDistanceIfNotTooHighToStep(double extrusionDistanceIfNotTooHighToStep)
   {
      this.extrusionDistanceIfNotTooHighToStep.set(extrusionDistanceIfNotTooHighToStep);
   }

   public void setTooHighToStepDistance(double tooHighToStepDistance)
   {
      this.tooHighToStepDistance.set(tooHighToStepDistance);
   }

   public void setClusterResolution(double clusterResolution)
   {
      this.clusterResolution.set(clusterResolution);
   }

   public void setExplorationDistanceFromStartGoal(double explorationDistanceFromStartGoal)
   {
      this.explorationDistance.set(explorationDistanceFromStartGoal);
   }

   public void setPlanarRegionMinArea(double planarRegionMinArea)
   {
      this.planarRegionMinArea.set(planarRegionMinArea);
   }

   public void setPlanarRegionMinSize(int planarRegionMinSize)
   {
      this.planarRegionMinSize.set(planarRegionMinSize);
   }

   public void setSearchHostRegionEpsilon(double searchHostRegionEpsilon)
   {
      this.searchHostRegionEpsilon.set(searchHostRegionEpsilon);
   }
}
