package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.*;

/**
 * The JSON file for this property set is located here:
 * ihmc-footstep-planning/src/main/resources/us/ihmc/footstepPlanning/AStarBodyPathPlannerParameters.json
 *
 * This class was auto generated. Property attributes must be edited in the JSON file,
 * after which this class should be regenerated by running the main. This class uses
 * the generator to assist in the addition, removal, and modification of property keys.
 * It is permissible to forgo these benefits and abandon the generator, in which case
 * you should also move it from the generated-java folder to the java folder.
 *
 * If the constant paths have changed, change them in this file and run the main to regenerate.
 */
public class AStarBodyPathPlannerParameters extends StoredPropertySet implements AStarBodyPathPlannerParametersBasics
{
   public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "ihmc-open-robotics-software";
   public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "ihmc-footstep-planning/src/main/resources";
   public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "ihmc-footstep-planning/src/main/generated-java";

   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   /**
    * whether or not the planner checks for collisions.
    */
   public static final BooleanStoredPropertyKey checkForCollisions = keys.addBooleanKey("Check for collisions");
   /**
    * Whether the planner computes surface normals. If false, traversibility and roll
    * are also not computed
    */
   public static final BooleanStoredPropertyKey computeSurfaceNormalCost = keys.addBooleanKey("Compute surface normal cost");
   /**
    * Whether the planner computes and checks traversibility
    */
   public static final BooleanStoredPropertyKey computeTraversibility = keys.addBooleanKey("Compute traversibility");
   /**
    * Whether the body path plan is post-processed with the smoother.
    */
   public static final BooleanStoredPropertyKey performSmoothing = keys.addBooleanKey("Perform smoothing");
   /**
    * This is the weight assigned to roll in the search space. Increasing this value
    * will decrease the likelihood of the path moving sideways across slopes.
    */
   public static final DoubleStoredPropertyKey rollCostWeight = keys.addDoubleKey("Roll cost weight");
   /**
    * The angle in degrees of the deadband applied to the computed roll. Increasing
    * this value decreases the effect small rolls have on the path.
    */
   public static final DoubleStoredPropertyKey rollCostDeadband = keys.addDoubleKey("Roll cost deadband");
   /**
    * When the roll is below this angle, the cost is linearly discounted to zero.
    * Increasing this value decreases the effect roll has on the path.
    */
   public static final DoubleStoredPropertyKey maxPenalizedRollAngle = keys.addDoubleKey("Max penalized roll angle");
   /**
    * The node height of a vertex is determined as the average height of all cells
    * within this radius.
    */
   public static final DoubleStoredPropertyKey snapRadius = keys.addDoubleKey("Snap radius");
   /**
    * When computing the vertex height, cells that are this distance below the max
    * height are ignored when taking the average
    */
   public static final DoubleStoredPropertyKey minSnapHeightThreshold = keys.addDoubleKey("Min snap height threshold");
   /**
    * This is the weight assigned to minimizing the inccline the path takes. The cost
    * is determined by the difference between the edge incline from the nominal
    * incline.
    */
   public static final DoubleStoredPropertyKey inclineCostWeight = keys.addDoubleKey("Incline cost weight");
   /**
    * This is a deadband applied to the incline in the search.
    */
   public static final DoubleStoredPropertyKey inclineCostDeadband = keys.addDoubleKey("Incline cost deadband");
   /**
    * The max incline, in degrees, that is allowed for the body path planner to
    * traverse.
    */
   public static final DoubleStoredPropertyKey maxIncline = keys.addDoubleKey("Max incline");
   /**
    * Width of the collision box used for checking collisions with the environment.
    */
   public static final DoubleStoredPropertyKey collisionBoxSizeY = keys.addDoubleKey("Collision box size Y");
   /**
    * Depth of the collision box used for checking collisions with the environment.
    */
   public static final DoubleStoredPropertyKey collisionBoxSizeX = keys.addDoubleKey("Collision box size X");
   /**
    * Intersection height below which collisions are ignored.
    */
   public static final DoubleStoredPropertyKey collisionBoxGroundClearance = keys.addDoubleKey("Collision box ground clearance");
   /**
    * Weight placed on maximizing traversibility in the plan. Increasing this weight
    * will tend the plan towards flat sections.
    */
   public static final DoubleStoredPropertyKey traversibilityWeight = keys.addDoubleKey("Traversibility weight");
   /**
    * weight placed on the traversibility at the start node when computing the overall
    * traversibility score.
    */
   public static final DoubleStoredPropertyKey traversibilityStanceWeight = keys.addDoubleKey("Traversibility stance weight");
   /**
    * weight placed on the traversibility at the end node when computing the overall
    * traversibility score.
    */
   public static final DoubleStoredPropertyKey traversibilityStepWeight = keys.addDoubleKey("Traversibility step weight");
   /**
    * Min score on the start node traversibility to say an edge is traversible.
    */
   public static final DoubleStoredPropertyKey minTraversibilityScore = keys.addDoubleKey("Min traversibility score");
   /**
    * Min angle in degrees to penalize in the traversibility score.
    */
   public static final DoubleStoredPropertyKey minNormalAngleToPenalizeForTraversibility = keys.addDoubleKey("Min normal angle to penalize for traversibility");
   /**
    * Max angle in degrees to penalize in the traversibility score.
    */
   public static final DoubleStoredPropertyKey maxNormalAngleToPenalizeForTraversibility = keys.addDoubleKey("Max normal angle to penalize for traversibility");
   /**
    * Weight to place on the surface normals when computing the traversibility score.
    */
   public static final DoubleStoredPropertyKey traversibilityInclineWeight = keys.addDoubleKey("Traversibility incline weight");
   /**
    * Box width of cells to include when performing the traversibility calculation
    */
   public static final DoubleStoredPropertyKey traversibilitySearchWidth = keys.addDoubleKey("Traversibility search width");
   /**
    * This is the minimum number of occupied neighbor cells to say a vertex is
    * traversibile.
    */
   public static final IntegerStoredPropertyKey minOccupiedNeighborsForTraversibility = keys.addIntegerKey("Min occupied neighbors for traversibility");
   /**
    * This is half the typical stance width of the robot, used to compute the
    * traversibility
    */
   public static final DoubleStoredPropertyKey halfStanceWidth = keys.addDoubleKey("Half stance width");
   /**
    * This the width of the height window of cells to include in the traversibility
    * calculation. The height distance must be within this value of the start and end
    * nodes. Increasing this value will increase the number of cells included in the
    * traversibility calculation
    */
   public static final DoubleStoredPropertyKey traversibilityHeightWindowWidth = keys.addDoubleKey("Traversibility height window width");
   /**
    * This is the deadband applied to the height distance of cells in the
    * traversibility calculation. Increasing this value will increase the overall
    * traversibility scores.
    */
   public static final DoubleStoredPropertyKey traversibilityHeightWindowDeadband = keys.addDoubleKey("Traversibility height window deadband");
   /**
    * This is the distance to the ground plane estimate that is needed for saying the
    * two nodes are in the ground plane.
    */
   public static final DoubleStoredPropertyKey heightProximityForSayingWalkingOnGround = keys.addDoubleKey("Height proximity for saying walking on ground");
   /**
    * This is the minimum discount applied to non-ground cells when computing the
    * traversibility when the edge is located in the ground plane. Increasing this
    * value will increase the traversibility score when walking on the ground.
    */
   public static final DoubleStoredPropertyKey traversibilityNonGroundDiscountWhenWalkingOnGround = keys.addDoubleKey("Traversibility non ground discount when walking on ground");
   /**
    * Weight placed on the gradient for avoiding collisions
    */
   public static final DoubleStoredPropertyKey smootherCollisionWeight = keys.addDoubleKey("Smoother collision weight");
   /**
    * Weight placed on the gradient for minimizing the angle between successive
    * segments of the body path
    */
   public static final DoubleStoredPropertyKey smootherSmoothnessWeight = keys.addDoubleKey("Smoother smoothness weight");
   /**
    * Discount applied to the smoothness gradients of turn points.
    */
   public static final DoubleStoredPropertyKey smootherTurnPointSmoothnessDiscount = keys.addDoubleKey("Smoother turn point smoothness discount");
   /**
    * Min curvature in degrees to penalize with a gradient.
    */
   public static final DoubleStoredPropertyKey smootherMinCurvatureToPenalize = keys.addDoubleKey("Smoother min curvature to penalize");
   /**
    * Weight placed on the gradient for making the vertices of the body path plan an
    * equal distance apart.
    */
   public static final DoubleStoredPropertyKey smootherEqualSpacingWeight = keys.addDoubleKey("Smoother equal spacing weight");
   /**
    * Weight placed on the gradient for minimizing the roll cost of the body path plan
    */
   public static final DoubleStoredPropertyKey smootherRollWeight = keys.addDoubleKey("Smoother roll weight");
   /**
    * Weight placed on a gradient that drives the waypoint towards the initial value
    */
   public static final DoubleStoredPropertyKey smootherDisplacementWeight = keys.addDoubleKey("Smoother displacement weight");
   /**
    * Weight placed on the gradient for maximizing the traversibility of the body path
    * plan.
    */
   public static final DoubleStoredPropertyKey smootherTraversibilityWeight = keys.addDoubleKey("Smoother traversibility weight");
   /**
    * Weight placed on the gradient pushing the body path plan towards the most cells
    * in the ground plane.
    */
   public static final DoubleStoredPropertyKey smootherGroundPlaneWeight = keys.addDoubleKey("Smoother ground plane weight");
   /**
    * Traversibility threshold that results in a zero gradient for traversibility
    */
   public static final DoubleStoredPropertyKey smootherMinimumTraversibilityToSearchFor = keys.addDoubleKey("Smoother minimum traversibility to search for");
   /**
    * Traversibility threshold above which the traversibility gradient begins to carry
    * less weight.
    */
   public static final DoubleStoredPropertyKey smootherTraversibilityThresholdForNoDiscount = keys.addDoubleKey("Smoother traversibility threshold for no discount");
   /**
    * Gain applied to the smoother gradient for iterative modifications
    */
   public static final DoubleStoredPropertyKey smootherHillClimbGain = keys.addDoubleKey("Smoother hill climb gain");
   /**
    * Minimum gradient vector magnitude to terminate the smoother iterations
    */
   public static final DoubleStoredPropertyKey smootherGradientThresholdToTerminate = keys.addDoubleKey("Smoother gradient threshold to terminate");

   /**
    * Loads this property set.
    */
   public AStarBodyPathPlannerParameters()
   {
      this("");
   }

   /**
    * Loads an alternate version of this property set in the same folder.
    */
   public AStarBodyPathPlannerParameters(String versionSpecifier)
   {
      this(AStarBodyPathPlannerParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
   }

   /**
    * Loads an alternate version of this property set in other folders.
    */
   public AStarBodyPathPlannerParameters(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String versionSuffix)
   {
      super(keys, classForLoading, AStarBodyPathPlannerParameters.class, directoryNameToAssumePresent, subsequentPathToResourceFolder, versionSuffix);
      load();
   }

   public AStarBodyPathPlannerParameters(StoredPropertySetReadOnly other)
   {
      super(keys, AStarBodyPathPlannerParameters.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
      set(other);
   }

   public static void main(String[] args)
   {
      StoredPropertySet parameters = new StoredPropertySet(keys,
                                                           AStarBodyPathPlannerParameters.class,
                                                           DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                           SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
      parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
   }
}
