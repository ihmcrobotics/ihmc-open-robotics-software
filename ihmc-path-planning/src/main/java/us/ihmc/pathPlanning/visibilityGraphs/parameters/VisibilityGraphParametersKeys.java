package us.ihmc.pathPlanning.visibilityGraphs.parameters;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;

public class VisibilityGraphParametersKeys
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey maxInterRegionConnectionLength              = keys.addDoubleKey("Max inter region connection length", 0.45);
   public static final DoubleStoredPropertyKey normalZThresholdForAccessibleRegions        = keys.addDoubleKey("Normal Z threshold for accessible regions", Math.cos(Math.toRadians(30.0)));
   public static final DoubleStoredPropertyKey navigableExtrusionDistance                  = keys.addDoubleKey("Navigable extrusion distance", 0.02);
   public static final DoubleStoredPropertyKey obstacleExtrusionDistance                   = keys.addDoubleKey("Obstacle extrusion distance", 0.4);
   public static final DoubleStoredPropertyKey preferredObstacleExtrusionDistance          = keys.addDoubleKey("Preferred obstacle extrusion distance", 1.0);
   public static final DoubleStoredPropertyKey obstacleExtrusionDistanceIfNotTooHighToStep = keys.addDoubleKey("Obstacle extrusion distance if not too high to step", 0.05);
   public static final DoubleStoredPropertyKey tooHighToStepDistance                       = keys.addDoubleKey("Too high to step distance", 0.28);
   public static final DoubleStoredPropertyKey clusterResolution                           = keys.addDoubleKey("Cluster resolution", 0.2);
   public static final DoubleStoredPropertyKey explorationDistanceFromStartGoal            = keys.addDoubleKey("Exploration distance from start goal", Double.POSITIVE_INFINITY);
   public static final DoubleStoredPropertyKey planarRegionMinArea                         = keys.addDoubleKey("Planar region min area", 0.0);
   public static final IntegerStoredPropertyKey planarRegionMinSize                        = keys.addIntegerKey("Planar region min size", 0);
   public static final DoubleStoredPropertyKey regionOrthogonalAngle                       = keys.addDoubleKey("Region orthogonal angle", Math.toRadians(75.0));
   public static final DoubleStoredPropertyKey searchHostRegionEpsilon                     = keys.addDoubleKey("Search host region epsilon", 0.03);
   public static final DoubleStoredPropertyKey canDuckUnderHeight                          = keys.addDoubleKey("Can duck under height", 2.0);
   public static final DoubleStoredPropertyKey canEasilyStepOverHeight                     = keys.addDoubleKey("Can easily step over height", 0.03);
   public static final DoubleStoredPropertyKey lengthForLongInterRegionEdge                = keys.addDoubleKey("Length for long inter region edge", 0.3);

   public static final DoubleStoredPropertyKey heuristicWeight                             = keys.addDoubleKey("Heuristic weight", 2.0);
   public static final DoubleStoredPropertyKey distanceWeight                              = keys.addDoubleKey("Distance weight", 1.5);
   public static final DoubleStoredPropertyKey elevationWeight                             = keys.addDoubleKey("Elevation weight", 0.0);
}
