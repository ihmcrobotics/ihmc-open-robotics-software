package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.AStarBodyPathEdgeData;
import us.ihmc.footstepPlanning.log.AStarBodyPathIterationData;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatMemory;
import us.ihmc.perception.opencl.OpenCLIntBuffer;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class GPUAStarBodyPathPlanner implements AStarBodyPathPlannerInterface
{
   private static final int numberOfNeighborsPerExpansion = 16;
   private static final int defaultCells = (int) (5.0 / 0.03);
   private static final int defaultNodes = (int) (5.0 / BodyPathLatticePoint.gridSizeXY);

   private static final boolean debug = false;


   // Inputs to the planner
   private final DefaultFootstepPlannerParametersReadOnly parameters;
   private final AStarBodyPathPlannerParametersReadOnly plannerParameters;
   private HeightMapData heightMapData;

   // yo variables to be logged
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean containsCollision = new YoBoolean("containsCollision", registry);
   private final YoDouble edgeCost = new YoDouble("edgeCost", registry);
   private final YoDouble deltaHeight = new YoDouble("deltaHeight", registry);
   private final YoDouble snapHeight = new YoDouble("snapHeight", registry);
   private final YoDouble incline = new YoDouble("incline", registry);
   private final YoDouble inclineCost = new YoDouble("inclineCost", registry);
   private final YoDouble traversibilityCost = new YoDouble("traversibilityCost", registry);
   private final YoDouble roll = new YoDouble("roll", registry);
   private final YoDouble rollCost = new YoDouble("rollCost", registry);
   private final YoDouble nominalIncline = new YoDouble("nominalIncline", registry);
   private final YoFrameVector3D leastSqNormal = new YoFrameVector3D("leastSqNormal", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble heuristicCost = new YoDouble("heuristicCost", registry);
   private final YoDouble totalCost = new YoDouble("totalCost", registry);
   private final YoEnum<RejectionReason> rejectionReason = new YoEnum<>("rejectionReason", registry, RejectionReason.class, true);

   private final SideDependentList<YoDouble> stanceScore;
   private final SideDependentList<YoDouble> stepScores;
   private final YoDouble stanceTraversibility;

   // parameters used for the actual planning process
   private final PriorityQueue<BodyPathLatticePoint> stack;
   private BodyPathLatticePoint startNode, goalNode;
   private BodyPathLatticePoint leastCostNode = null;
   private double leastCost = Double.POSITIVE_INFINITY;

   private final TIntArrayList xSnapOffsets = new TIntArrayList();
   private final TIntArrayList ySnapOffsets = new TIntArrayList();

   private final List<AStarBodyPathIterationData> iterationData = new ArrayList<>();
   private final HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap = new HashMap<>();

   private final List<Consumer<FootstepPlannerOutput>> statusCallbacks;
   private final Stopwatch stopwatch;
   private double planningStartTime;
   private BodyPathPlanningResult result = null;
   private boolean reachedGoal = false;
   private final AtomicBoolean haltRequested = new AtomicBoolean();
   private static final int maxIterations = 3000;

   private final AStarBodyPathEdgeData edgeData;
   private final HashSet<BodyPathLatticePoint> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<BodyPathLatticePoint> graph = new DirectedGraph<>();
   private final TIntArrayList neighborsOffsetX = new TIntArrayList();
   private final TIntArrayList neighborsOffsetY = new TIntArrayList();
   private final List<BodyPathLatticePoint> neighbors = new ArrayList<>();

   /////// all the open cl memory, programs, and kernels /////
   private final OpenCLManager openCLManager = new OpenCLManager();
   private _cl_program pathPlannerProgram;
   private _cl_kernel computeNormalsWithLeastSquaresKernel;
   private _cl_kernel computeNormalsWithRansacKernel;
   private _cl_kernel snapVerticesKernel;
   private _cl_kernel computeEdgeDataKernel;
   private _cl_kernel computeHeuristicCostKernel;

   private final OpenCLFloatBuffer heightMapParametersBuffer = new OpenCLFloatBuffer(6);
   private final OpenCLFloatBuffer pathPlanningParametersBuffer = new OpenCLFloatBuffer(31);
   private final OpenCLFloatBuffer ransacNormalParametersBuffer = new OpenCLFloatBuffer(8);

   private final OpenCLIntBuffer leastSquaresOffsetBuffer = new OpenCLIntBuffer(6);
   private final OpenCLIntBuffer ransacOffsetBuffer = new OpenCLIntBuffer(4);
   private final OpenCLIntBuffer snapOffsetsBuffer = new OpenCLIntBuffer(6);
   private final OpenCLIntBuffer traversibilityOffsetsBuffer = new OpenCLIntBuffer(8);
   private final OpenCLIntBuffer collisionOffsetsBuffer = new OpenCLIntBuffer(8);
   private final OpenCLIntBuffer neighborOffsetsBuffer = new OpenCLIntBuffer(33);

   private final OpenCLFloatBuffer heightMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer leastSquaresNormalXYZBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatMemory ransacNormalXYZBuffer = new OpenCLFloatMemory(1);
   private final OpenCLFloatBuffer sampledHeightBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer snappedNodeHeightBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLIntBuffer edgeRejectionReasonBuffer = new OpenCLIntBuffer(1);
   private final OpenCLFloatBuffer deltaHeightMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer inclineMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer rollMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer stanceTraversibilityMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer stepTraversibilityMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer inclineCostMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer rollCostMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer traversibilityCostMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer edgeCostMapBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer heuristicCostMapBuffer = new OpenCLFloatBuffer(1);

   private final GPUAStarBodyPathSmoother smoother;

   private int cellsPerSide = -1;
   private int nodesPerSide = -1;
   private int nodeCenterIndex = -1;

   private boolean firstTick = true;

   public GPUAStarBodyPathPlanner(DefaultFootstepPlannerParametersReadOnly parameters,
                                  AStarBodyPathPlannerParametersReadOnly plannerParameters,
                                  SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this(parameters, plannerParameters, footPolygons, new Stopwatch());
   }

   public GPUAStarBodyPathPlanner(DefaultFootstepPlannerParametersReadOnly parameters,
                                  AStarBodyPathPlannerParametersReadOnly plannerParameters,
                                  SideDependentList<ConvexPolygon2D> footPolygons,
                                  Stopwatch stopwatch)
   {
      this(parameters, plannerParameters, footPolygons, new ArrayList<>(), stopwatch);
   }

   public GPUAStarBodyPathPlanner(DefaultFootstepPlannerParametersReadOnly parameters,
                                  AStarBodyPathPlannerParametersReadOnly plannerParameters,
                                  SideDependentList<ConvexPolygon2D> footPolygons,
                                  List<Consumer<FootstepPlannerOutput>> statusCallbacks,
                                  Stopwatch stopwatch)
   {
      this.parameters = parameters;
      this.plannerParameters = plannerParameters;
      this.statusCallbacks = statusCallbacks;
      this.stopwatch = stopwatch;
      stack = new PriorityQueue<>(new NodeComparator<>(graph, this::getHeuristicCost));

      this.stanceScore = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "StanceScore", registry));
      this.stepScores = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "StepScore", registry));
      this.stanceTraversibility = new YoDouble("stanceTraversibility", registry);

      // These are the 16 neighbor offsets
      packNeighborOffsets(neighborsOffsetX, neighborsOffsetY);

      // Makes sure to destroy the open CL memory by adding a shutdown hook
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroyOpenCLStuff));

      // Sets up the post-processing waypoint smoother
      smoother = new GPUAStarBodyPathSmoother(plannerParameters, null, openCLManager, null, null);

      createOpenCLStuff(defaultCells, defaultNodes);

      // this sets up all the data to be stored by the logger. This callback is called whenever the graph is expanded, where it then creates a new big of edge
      // data, and stores it in a map, and resets all the variables. This gets called inside the planning loop.
      List<YoVariable> allVariables = registry.collectSubtreeVariables();
      this.edgeData = new AStarBodyPathEdgeData(allVariables.size());
      graph.setGraphExpansionCallback(edge ->
                                      {
                                         for (int i = 0; i < allVariables.size(); i++)
                                         {
                                            edgeData.setData(i, allVariables.get(i).getValueAsLongBits());
                                         }

                                         edgeData.setParentNode(edge.getStartNode());
                                         edgeData.setChildNode(edge.getEndNode());
                                         edgeData.setChildSnapHeight(snapHeight.getDoubleValue());

                                         edgeDataMap.put(edge, edgeData.getCopyAndClear());

                                         containsCollision.set(false);
                                         deltaHeight.set(Double.NaN);
                                         edgeCost.set(Double.NaN);
                                         deltaHeight.set(Double.NaN);
                                         rejectionReason.set(null);
                                         leastSqNormal.setToZero();
                                         roll.set(0.0);
                                         incline.set(0.0);
                                         heuristicCost.setToNaN();
                                         totalCost.setToNaN();
                                         for (RobotSide side : RobotSide.values)
                                         {
                                            stanceScore.get(side).setToNaN();
                                            stepScores.get(side).setToNaN();
                                         }
                                         stanceTraversibility.setToNaN();
                                      });
   }

   /**
    * This creates all the open cl managers, programs, and kernels used in the planner
    */
   private void createOpenCLStuff(int numberOfCells, int numberOfNodes)
   {
      cellsPerSide = numberOfCells;
      this.nodesPerSide = numberOfNodes;
      this.nodeCenterIndex = (nodesPerSide - 1) / 2;

      pathPlannerProgram = openCLManager.loadProgram("BodyPathPlanning", "HeightMapUtils.cl");
      computeNormalsWithLeastSquaresKernel = openCLManager.createKernel(pathPlannerProgram, "computeSurfaceNormalsWithLeastSquares");
      computeNormalsWithRansacKernel = openCLManager.createKernel(pathPlannerProgram, "computeSurfaceNormalsWithRANSAC");
      snapVerticesKernel = openCLManager.createKernel(pathPlannerProgram, "snapVertices");
      computeEdgeDataKernel = openCLManager.createKernel(pathPlannerProgram, "computeEdgeData");
      computeHeuristicCostKernel = openCLManager.createKernel(pathPlannerProgram, "computeHeuristicCost");

      smoother.createOpenCLStuff(pathPlannerProgram, numberOfCells, numberOfNodes);
   }

   /**
    * On the first time the planner is called, this sets up all the OpenCL memory. It's useful to do this after creation to allow the process to start
    * more quickly.
    */
   private void firstTickSetup()
   {
      heightMapParametersBuffer.createOpenCLBufferObject(openCLManager);
      pathPlanningParametersBuffer.createOpenCLBufferObject(openCLManager);
      ransacNormalParametersBuffer.createOpenCLBufferObject(openCLManager);
      leastSquaresOffsetBuffer.createOpenCLBufferObject(openCLManager);
      ransacOffsetBuffer.createOpenCLBufferObject(openCLManager);
      snapOffsetsBuffer.createOpenCLBufferObject(openCLManager);
      traversibilityOffsetsBuffer.createOpenCLBufferObject(openCLManager);
      collisionOffsetsBuffer.createOpenCLBufferObject(openCLManager);
      neighborOffsetsBuffer.createOpenCLBufferObject(openCLManager);

      heightMapBuffer.createOpenCLBufferObject(openCLManager);
      leastSquaresNormalXYZBuffer.createOpenCLBufferObject(openCLManager);
      ransacNormalXYZBuffer.createOpenCLBufferObject(openCLManager);
      sampledHeightBuffer.createOpenCLBufferObject(openCLManager);
      snappedNodeHeightBuffer.createOpenCLBufferObject(openCLManager);
      edgeRejectionReasonBuffer.createOpenCLBufferObject(openCLManager);
      deltaHeightMapBuffer.createOpenCLBufferObject(openCLManager);
      inclineMapBuffer.createOpenCLBufferObject(openCLManager);
      rollMapBuffer.createOpenCLBufferObject(openCLManager);
      stanceTraversibilityMapBuffer.createOpenCLBufferObject(openCLManager);
      stepTraversibilityMapBuffer.createOpenCLBufferObject(openCLManager);
      inclineCostMapBuffer.createOpenCLBufferObject(openCLManager);
      rollCostMapBuffer.createOpenCLBufferObject(openCLManager);
      traversibilityCostMapBuffer.createOpenCLBufferObject(openCLManager);
      edgeCostMapBuffer.createOpenCLBufferObject(openCLManager);
      heuristicCostMapBuffer.createOpenCLBufferObject(openCLManager);

      smoother.firstTickSetup();

      firstTick = false;
   }

   /**
    * Deallocates everything from OpenCL memory, and destroys the OpenCL manager. Nominally, this is called when the process is terminated.
    **/
   public void destroyOpenCLStuff()
   {
      pathPlannerProgram.close();
      computeNormalsWithLeastSquaresKernel.close();
      computeNormalsWithRansacKernel.close();
      snapVerticesKernel.close();
      computeEdgeDataKernel.close();
      computeHeuristicCostKernel.close();

      heightMapParametersBuffer.destroy(openCLManager);
      pathPlanningParametersBuffer.destroy(openCLManager);
      ransacNormalParametersBuffer.destroy(openCLManager);
      leastSquaresOffsetBuffer.destroy(openCLManager);
      ransacOffsetBuffer.destroy(openCLManager);
      snapOffsetsBuffer.destroy(openCLManager);
      traversibilityOffsetsBuffer.destroy(openCLManager);
      collisionOffsetsBuffer.destroy(openCLManager);
      neighborOffsetsBuffer.destroy(openCLManager);

      heightMapBuffer.destroy(openCLManager);
      leastSquaresNormalXYZBuffer.destroy(openCLManager);
      ransacNormalXYZBuffer.destroy(openCLManager);
      sampledHeightBuffer.destroy(openCLManager);
      edgeRejectionReasonBuffer.destroy(openCLManager);
      deltaHeightMapBuffer.destroy(openCLManager);
      inclineMapBuffer.destroy(openCLManager);
      rollMapBuffer.destroy(openCLManager);
      stanceTraversibilityMapBuffer.destroy(openCLManager);
      stepTraversibilityMapBuffer.destroy(openCLManager);
      inclineCostMapBuffer.destroy(openCLManager);
      rollCostMapBuffer.destroy(openCLManager);
      traversibilityCostMapBuffer.destroy(openCLManager);
      edgeCostMapBuffer.destroy(openCLManager);
      heuristicCostMapBuffer.destroy(openCLManager);

      smoother.destroyOpenCLStuff();

      openCLManager.destroy();
   }

   private enum RejectionReason
   {
      INVALID_SNAP,
      TOO_STEEP,
      STEP_TOO_HIGH,
      COLLISION,
      NON_TRAVERSIBLE
   }

   /**
    * Computes the body path plan using the information contained in {@param request}, and packs into the output {@param outputToPack}.
    */
   @Override
   public void handleRequest(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      heightMapData = request.getHeightMapData();
      if (firstTick)
      {
         firstTickSetup();
      }
      if (cellsPerSide != heightMapData.getCellsPerAxis())
      {
         this.cellsPerSide = heightMapData.getCellsPerAxis();
         this.nodeCenterIndex = HeightMapTools.computeCenterIndex(heightMapData.getGridSizeXY(), BodyPathLatticePoint.gridSizeXY);
         this.nodesPerSide = 2 * nodeCenterIndex + 1;
         resizeOpenCLObjects();
         smoother.resizeOpenCLObjects(cellsPerSide);
      }

      haltRequested.set(false);
      int iterations = 0;
      reachedGoal = false;
      stopwatch.start();
      result = BodyPathPlanningResult.PLANNING;
      planningStartTime = stopwatch.totalElapsed();
      stopwatch.lap();

      iterationData.clear();
      edgeDataMap.clear();

      /////// set up the planner preliminaries, like the start and goal pose, nominal incline, and the search stack ///////
      // Get the start and goal midfoot buffers
      Pose3D startPose = new Pose3D();
      Pose3D goalPose = new Pose3D();

      startPose.interpolate(request.getStartFootPoses().get(RobotSide.LEFT), request.getStartFootPoses().get(RobotSide.RIGHT), 0.5);
      goalPose.interpolate(request.getGoalFootPoses().get(RobotSide.LEFT), request.getGoalFootPoses().get(RobotSide.RIGHT), 0.5);

      startNode = new BodyPathLatticePoint(startPose.getX(), startPose.getY());
      goalNode = new BodyPathLatticePoint(goalPose.getX(), goalPose.getY());
      stack.clear();
      stack.add(startNode);
      graph.initialize(startNode);
      expandedNodeSet.clear();
      leastCostNode = startNode;
      nominalIncline.set(Math.atan2(goalPose.getZ() - startPose.getZ(), goalPose.getPosition().distanceXY(startPose.getPosition())));

      /////// set up the GPU buffers with all the input data including parameters and search directions ///////
      // compute the offset vertices for getting the node height from the height map
      AStarBodyPathPlanner.packRadialOffsets(heightMapData, plannerParameters.getSnapRadius(), xSnapOffsets, ySnapOffsets);

      // populate the offset vertices for node height into a GPU buffer
      populateSnapHeightOffsetsBuffer();
      // populate the offset vertices for computing the traversibility into a GPU buffer
      populateTraversibilityOffsetsBuffer();
      // populate the offset vertices for checking for collisions into a GPU buffer
      populateCollisionsOffsetsBuffer(heightMapData.getGridResolutionXY(), plannerParameters.getCollisionBoxSizeX(), plannerParameters.getCollisionBoxSizeY());
      // populate the offset vertices for computing the children node into the GPU buffer
      populateNeighborOffsetsBuffer();
      // populate the parameters that define the height map extrinsics
      double patchWidth = 0.3;
      populateHeightMapParameterBuffer(patchWidth);
      // populate the buffer that includes all the path planning parameters
      populatePathPlanningParametersBuffer();
      // populate the buffer that represents the height map
      populateHeightMapBuffer();

      /////// compute all the data on the GPU, and get the results out back into CPU memory ///////
      // compute the snapped node height from the height map data
      computeSnapHeightsOnTheGPU();
      // Compute the two different versions of the height map normal, one which fits the local slope, and one that is better at deadling with discontinuous
      // surfaces (RANSAC)
      if (plannerParameters.getComputeSurfaceNormalCost())
      {
         computeSurfaceNormalsWithLeastSquares(patchWidth);
         computeSurfaceNormalsWithRansac();
      }
      // Compute the heuristic cost for every possible location
      computeHeuristicCostMapOnTheGPU();
      // Compute all data for all the possible edges that could be included in the plan
      computeEdgeDataCostAndValidityOnTheGPU();

      // Get the current lowest cost ending node
      leastCost = getHeuristicCost(leastCostNode);

      /////// perform the actual search, using the data that was precomputed on the GPU ///////
      planningLoop:
      while (true)
      {
         iterations++;
         // update the number of iterations we've taken.
         outputToPack.getPlannerTimings().setPathPlanningIterations(iterations);

         if (stopwatch.totalElapsed() >= request.getTimeout())
         {
            // we're out of time. Terminate the planning loop and report the result
            result = BodyPathPlanningResult.TIMED_OUT_BEFORE_SOLUTION;
            break;
         }
         if (haltRequested.get())
         {
            // Someone requested that we halt planning. Termiante the planning loop and report the result.
            result = BodyPathPlanningResult.HALTED;
            break;
         }
         if (iterations > maxIterations)
         {
            // We've taken more iterations than allowed. Terminate the planning loop and report the result.
            result = BodyPathPlanningResult.MAXIMUM_ITERATIONS_REACHED;
            break;
         }

         // Get the next node. This will be the cheapest node in the stack, with the cost being the path to that node plus the heuristic to goal.
         BodyPathLatticePoint node = getNextNode();
         if (node == null)
         {
            // There are no more nodes that haven't been expanded. This means we've either exhaustively searched the entire grid map and there are no more
            // children nodes, or there are no viable transitions to get to the goal. The goal is either off the modeled world, or in an island that is
            // unreachable.
            result = BodyPathPlanningResult.NO_PATH_EXISTS;
            if (debug)
            {
               LogTools.info("Stack is empty, no path exists...");
            }
            break;
         }

         // Get the buffer key of the parent node.
         int parentNodeGraphKey = getNodeGraphKey(node);
         // Get the edge key of the first possible child of the parent node. Each node has (nubmerOfneighborsPerExpansion) possible children, so use this value.
         int edgeKeyStart = parentNodeGraphKey * numberOfNeighborsPerExpansion;
         // Compute all the candidate children nodes. Not all of these are feasible, but that's determined in the search
         populateNeighbors(node);

         // Get the height of the current node being expanded.
         double parentSnapHeight = getSnapHeight(parentNodeGraphKey);
         // Search through all the possible children, and check the viability and score of each.
         for (int neighborIndex = 0; neighborIndex < neighbors.size(); neighborIndex++)
         {
            BodyPathLatticePoint neighbor = neighbors.get(neighborIndex);

            // This is the node key for the candidate child node.
            int neighborNodeKey = getNodeGraphKey(neighbor);
            // If the node key is invalid, it's off the modeled environment, so we shouldn't allow that transition. This information is polled from the GPU
            // buffers
            if (neighborNodeKey < 0)
            {
               rejectionReason.set(RejectionReason.INVALID_SNAP);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            // This is the key for the edge data of the candidate node.
            int edgeKey = edgeKeyStart + neighborIndex;

            // Check if this edge is valid. If it's not, this method adds that edge to the graph with infinite weight
            if (!checkEdge(node, neighbor, neighborNodeKey, edgeKey))
               continue;

            // Populate the heuristic cost of the child node.
            heuristicCost.set(heuristicCostMapBuffer.getBackingDirectFloatBuffer().get(neighborNodeKey));

            // Get the data for this edge cost from the GPU buffers.
            computeEdgeCost(node, neighbor, edgeKey);

            // The total cost of this node is the cost to get to the node from the start plus the estimated cost to the goal.
            totalCost.set(heuristicCost.getValue() + edgeCost.getValue());
            // add this data to the graph. the expansion callback records it into the log.
            graph.checkAndSetEdge(node, neighbor, edgeCost.getValue());
            // add the node to the stack.
            stack.add(neighbor);

            // If this candidate node is the goal, terminate the plan. This could be a local minima, and there could be a cheaper path, but that doesn't matter.
            if (neighbor.equals(goalNode))
            {
               reachedGoal = true;
               result = BodyPathPlanningResult.FOUND_SOLUTION;
               break planningLoop;
            }
            // always keep track of the least cost node, as this is the terminal node if we terminate without having reached the goal.
            else if (heuristicCost.getValue() < leastCost)
            {
               leastCostNode = neighbor;
               leastCost = heuristicCost.getValue();
            }
         }

         // Record that this node has been expanded, so that we don't expand it again
         expandedNodeSet.add(node);

         // record all the iteration data for this newly expanded node. This could be done before or after the evaluation loop
         AStarBodyPathIterationData iterationData = new AStarBodyPathIterationData();
         iterationData.setParentNode(node);
         iterationData.getChildNodes().addAll(neighbors);
         iterationData.setParentNodeHeight(parentSnapHeight);
         this.iterationData.add(iterationData);

         // If we've not reported the status in a while, reprot it.
         if (shouldPublishStatus(request))
         {
            reportStatus(request, outputToPack);
            stopwatch.lap();
         }
      }

      // Report the result of the planning step.
      reportStatus(request, outputToPack);
   }

   /**
    * Convenience method for getting the key of a planning node. Returns -1 if the location is out of bounds.
    */
   private int getNodeGraphKey(BodyPathLatticePoint node)
   {
      int nodesPerSide = 2 * nodeCenterIndex + 1;
      int nodeGraphXIdx = HeightMapTools.coordinateToIndex(node.getX(), heightMapData.getGridCenter().getX(), BodyPathLatticePoint.gridSizeXY, nodeCenterIndex);
      int nodeGraphYIdx = HeightMapTools.coordinateToIndex(node.getY(), heightMapData.getGridCenter().getY(), BodyPathLatticePoint.gridSizeXY, nodeCenterIndex);

      if (nodeGraphYIdx < 0 || nodeGraphXIdx < 0 || nodeGraphXIdx >= nodesPerSide || nodeGraphYIdx >= nodesPerSide)
         return -1;

      return HeightMapTools.indicesToKey(nodeGraphXIdx, nodeGraphYIdx, nodeCenterIndex);
   }

   /**
    * Makes sure all the OpenCL objects are the correct size in memory
    */
   private void resizeOpenCLObjects()
   {
      int totalCells = cellsPerSide * cellsPerSide;
      heightMapBuffer.resize(totalCells, openCLManager);
      leastSquaresNormalXYZBuffer.resize(3 * totalCells, openCLManager);
      ransacNormalXYZBuffer.resize(3 * totalCells, openCLManager);
      sampledHeightBuffer.resize(totalCells, openCLManager);

      int totalNodes = nodesPerSide * nodesPerSide;
      int totalEdges = numberOfNeighborsPerExpansion * totalNodes;
      snappedNodeHeightBuffer.resize(totalNodes, openCLManager);
      edgeRejectionReasonBuffer.resize(totalEdges, openCLManager);
      deltaHeightMapBuffer.resize(totalEdges, openCLManager);
      inclineMapBuffer.resize(totalEdges, openCLManager);
      rollMapBuffer.resize(totalEdges, openCLManager);
      stanceTraversibilityMapBuffer.resize(2 * totalEdges, openCLManager);
      stepTraversibilityMapBuffer.resize(2 * totalEdges, openCLManager);
      inclineCostMapBuffer.resize(totalEdges, openCLManager);
      rollCostMapBuffer.resize(totalEdges, openCLManager);
      traversibilityCostMapBuffer.resize(totalEdges, openCLManager);
      edgeCostMapBuffer.resize(totalEdges, openCLManager);
      heuristicCostMapBuffer.resize(totalNodes, openCLManager);
   }


   /**
    * Populates the GPU buffer with the indices of the neighboring cells to use when computing the average height at a snapped location. These is used for
    * determining the height of the body path from the height map.
    */
   private void populateSnapHeightOffsetsBuffer()
   {
      int connections = xSnapOffsets.size();
      snapOffsetsBuffer.resize(2 * connections + 1, openCLManager);
      IntPointer intPointer = snapOffsetsBuffer.getBytedecoIntBufferPointer();
      int index = 0;
      intPointer.put(index, connections);
      index++;
      for (int x = 0; x < xSnapOffsets.size(); x++)
      {
         // pack the x offsets
         intPointer.put(index, xSnapOffsets.get(x));
         // pack the y offsets
         intPointer.put(connections + index, ySnapOffsets.get(x));
         index++;
      }

      snapOffsetsBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Populates the GPU buffer with the indices of the neighboring cells to check when calculating the traversibility. This sets the search radius for the
    * traversibility metric.
    */
   private void populateTraversibilityOffsetsBuffer()
   {
      TIntArrayList zeroDegCollisionOffsetsX = new TIntArrayList();
      TIntArrayList zeroDegCollisionOffsetsY = new TIntArrayList();
      TIntArrayList fourtyFiveDegCollisionOffsetsX = new TIntArrayList();
      TIntArrayList fourtyFiveDegCollisionOffsetsY = new TIntArrayList();
      TIntArrayList twentyTwoCollisionOffsetsX = new TIntArrayList();
      TIntArrayList twentyTwoCollisionOffsetsY = new TIntArrayList();

      double width = plannerParameters.getTraversibilitySearchWidth() / 2.0;
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(),
                                            zeroDegCollisionOffsetsX,
                                            zeroDegCollisionOffsetsY,
                                            width,
                                            width,
                                            0.0);
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(),
                                            fourtyFiveDegCollisionOffsetsX,
                                            fourtyFiveDegCollisionOffsetsY,
                                            width,
                                            width,
                                            Math.toRadians(45.0));
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(),
                                            twentyTwoCollisionOffsetsX,
                                            twentyTwoCollisionOffsetsY,
                                            width,
                                            width,
                                            Math.toRadians(22.5));

      int offsets0 = zeroDegCollisionOffsetsX.size();
      int offsets1 = fourtyFiveDegCollisionOffsetsY.size();
      int offsets2 = twentyTwoCollisionOffsetsY.size();
      traversibilityOffsetsBuffer.resize(2 * offsets0 + 2 * offsets1 + 2 * offsets2 + 3, openCLManager);
      int index = 0;
      IntPointer intPointer = traversibilityOffsetsBuffer.getBytedecoIntBufferPointer();

      intPointer.put(0, offsets0);
      intPointer.put(1, offsets1);
      intPointer.put(2, offsets2);
      int xStart = 3;
      int yStart = 3 + offsets0;
      for (int i = 0; i < offsets0; i++)
      {
         intPointer.put(xStart + i, zeroDegCollisionOffsetsX.get(i));
         intPointer.put(yStart + i, zeroDegCollisionOffsetsY.get(i));
      }
      xStart += 2 * offsets0;
      yStart += offsets0 + offsets1;
      for (int i = 0; i < offsets1; i++)
      {
         intPointer.put(xStart + i, fourtyFiveDegCollisionOffsetsX.get(i));
         intPointer.put(yStart + i, fourtyFiveDegCollisionOffsetsY.get(i));
      }
      xStart += 2 * offsets1;
      yStart += offsets1 + offsets2;
      for (int i = 0; i < offsets2; i++)
      {
         intPointer.put(xStart + i, twentyTwoCollisionOffsetsX.get(i));
         intPointer.put(yStart + i, twentyTwoCollisionOffsetsY.get(i));
      }

      traversibilityOffsetsBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Populates the GPU buffer with the indices of the neighboring cells to check when looking for collisions. These are offset from the cell in question.
    * These include cells at different angles.
    */
   private void populateCollisionsOffsetsBuffer(double gridResolutionXY, double boxSizeX, double boxSizeY)
   {
      TIntArrayList collisionOffsetsX1 = new TIntArrayList();
      TIntArrayList collisionOffsetsY1 = new TIntArrayList();
      TIntArrayList collisionOffsetsX2 = new TIntArrayList();
      TIntArrayList collisionOffsetsY2 = new TIntArrayList();
      TIntArrayList collisionOffsetsX3 = new TIntArrayList();
      TIntArrayList collisionOffsetsY3 = new TIntArrayList();

      BodyPathCollisionDetector.packOffsets(gridResolutionXY, collisionOffsetsX1, collisionOffsetsY1, boxSizeX, boxSizeY, 0 * Math.PI / 8.0);
      BodyPathCollisionDetector.packOffsets(gridResolutionXY, collisionOffsetsX2, collisionOffsetsY2, boxSizeX, boxSizeY, 1 * Math.PI / 8.0);
      BodyPathCollisionDetector.packOffsets(gridResolutionXY, collisionOffsetsX3, collisionOffsetsY3, boxSizeX, boxSizeY, 2 * Math.PI / 8.0);

      int offsets0 = collisionOffsetsX1.size();
      int offsets1 = collisionOffsetsX2.size();
      int offsets2 = collisionOffsetsX3.size();
      collisionOffsetsBuffer.resize(2 * offsets0 + 2 * offsets1 + 2 * offsets2 + 3, openCLManager);
      IntPointer intPointer = collisionOffsetsBuffer.getBytedecoIntBufferPointer();

      intPointer.put(0, offsets0);
      intPointer.put(1, offsets1);
      intPointer.put(2, offsets2);
      int xStart = 3;
      int yStart = 3 + offsets0;
      for (int i = 0; i < offsets0; i++)
      {
         intPointer.put(xStart + i, collisionOffsetsX1.get(i));
         intPointer.put(yStart + i, collisionOffsetsY1.get(i));
      }
      xStart += 2 * offsets0;
      yStart += offsets0 + offsets1;
      for (int i = 0; i < offsets1; i++)
      {
         intPointer.put(xStart + i, collisionOffsetsX2.get(i));
         intPointer.put(yStart + i, collisionOffsetsY2.get(i));
      }
      xStart += 2 * offsets1;
      yStart += offsets1 + offsets2;
      for (int i = 0; i < offsets2; i++)
      {
         intPointer.put(xStart + i, collisionOffsetsX3.get(i));
         intPointer.put(yStart + i, collisionOffsetsY3.get(i));
      }

      collisionOffsetsBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Computes the offset indices for the neighboring nodes when performing the search. These are the grid coordinates of the candidate child nodes when
    * expanding the cheapest node in the A* planner.collisionDetected
    */
   private static void packNeighborOffsets(TIntArrayList xOffsets, TIntArrayList yOffsets)
   {
      xOffsets.clear();
      yOffsets.clear();

      xOffsets.add(1); yOffsets.add(0);
      xOffsets.add(2); yOffsets.add(1);
      xOffsets.add(1); yOffsets.add(1);
      xOffsets.add(1); yOffsets.add(2);

      xOffsets.add(0); yOffsets.add(1);
      xOffsets.add(-1); yOffsets.add(2);
      xOffsets.add(-1); yOffsets.add(1);
      xOffsets.add(-2); yOffsets.add(1);

      xOffsets.add(-1); yOffsets.add(0);
      xOffsets.add(-2); yOffsets.add(-1);
      xOffsets.add(-1); yOffsets.add(-1);
      xOffsets.add(-1); yOffsets.add(-2);

      xOffsets.add(0); yOffsets.add(-1);
      xOffsets.add(1); yOffsets.add(-2);
      xOffsets.add(1); yOffsets.add(-1);
      xOffsets.add(2); yOffsets.add(-1);

      if (xOffsets.size() != numberOfNeighborsPerExpansion || yOffsets.size() != numberOfNeighborsPerExpansion)
         throw new RuntimeException("Neighbor set is wrong.");
   }

   /**
    * Populates the GPU buffer with the indices of the candidate children when performing the path plan.
    */
   private void populateNeighborOffsetsBuffer()
   {
      int offsets = neighborsOffsetX.size();
      neighborOffsetsBuffer.resize(2 * offsets + 1, openCLManager);
      int index = 0;
      IntPointer intPointer = neighborOffsetsBuffer.getBytedecoIntBufferPointer();

      intPointer.put(index++, offsets);
      for (int i = 0; i < offsets; i++)
      {
         intPointer.put(index, neighborsOffsetX.get(i));
         intPointer.put(offsets + index++, neighborsOffsetY.get(i));
      }

      neighborOffsetsBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Populates the height map data buffer, which contains the height map values in a buffered list.
    */
   private void populateHeightMapBuffer()
   {
      for (int x = 0; x < cellsPerSide; x++)
      {
         for (int y = 0; y < cellsPerSide; y++)
         {
            int key = HeightMapTools.indicesToKey(x, y, heightMapData.getCenterIndex());
            heightMapBuffer.getBackingDirectFloatBuffer().put(key, (float) heightMapData.getHeightAt(key));
         }
      }

      heightMapBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Populates the parameter buffer used for representing the height map data.
    */
   private void populateHeightMapParameterBuffer(double patchWidth)
   {
      double maxIncline = Math.toRadians(45.0);
      float snapHeightThreshold = (float) (patchWidth * Math.sin(maxIncline));

      FloatPointer floatPointer = heightMapParametersBuffer.getBytedecoFloatBufferPointer();
      floatPointer.put(0, (float) heightMapData.getGridResolutionXY());
      floatPointer.put(1, (float) heightMapData.getCenterIndex());
      floatPointer.put(2, (float) heightMapData.getGridCenter().getX());
      floatPointer.put(3, (float) heightMapData.getGridCenter().getY());
      floatPointer.put(4, snapHeightThreshold);
      floatPointer.put(5, (float) heightMapData.getEstimatedGroundHeight());

      heightMapParametersBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Populates the parameter buffer used for path planning.
    */
   private void populatePathPlanningParametersBuffer()
   {
      FloatPointer floatPointer = pathPlanningParametersBuffer.getBytedecoFloatBufferPointer();
      floatPointer.put(0, (float) BodyPathLatticePoint.gridSizeXY);
      floatPointer.put(1, (float) nodeCenterIndex);
      floatPointer.put(2, (float) startNode.getXIndex());
      floatPointer.put(3, (float) startNode.getYIndex());
      floatPointer.put(4, (float) goalNode.getX());
      floatPointer.put(5, (float) goalNode.getY());
      floatPointer.put(6, (float) nominalIncline.getValue());
      floatPointer.put(7, (plannerParameters.getCheckForCollisions() ? 1.0f : 0.0f));
      floatPointer.put(8, (plannerParameters.getComputeSurfaceNormalCost() ? 1.0f : 0.0f));
      floatPointer.put(9, (plannerParameters.getComputeTraversibility() ? 1.0f : 0.0f));
      floatPointer.put(10, (float) plannerParameters.getMinSnapHeightThreshold());
      floatPointer.put(11, (float) plannerParameters.getCollisionBoxGroundClearance());
      floatPointer.put(12, (float) Math.toRadians(plannerParameters.getMaxIncline()));
      floatPointer.put(13, (float) plannerParameters.getInclineCostWeight());
      floatPointer.put(14, (float) plannerParameters.getInclineCostDeadband());
      floatPointer.put(15, (float) plannerParameters.getRollCostWeight());
      floatPointer.put(16, (float) Math.toRadians(plannerParameters.getRollCostDeadband()));
      floatPointer.put(17, (float) Math.toRadians(plannerParameters.getMaxPenalizedRollAngle()));
      floatPointer.put(18, (float) plannerParameters.getTraversibilityStanceWeight());
      floatPointer.put(19, (float) plannerParameters.getTraversibilityStepWeight());
      floatPointer.put(20, (float) plannerParameters.getMinTraversibilityScore());
      floatPointer.put(21, (float) plannerParameters.getHalfStanceWidth());
      floatPointer.put(22, (float) plannerParameters.getTraversibilityHeightWindowWidth());
      floatPointer.put(23, (float) plannerParameters.getMinNormalAngleToPenalizeForTraversibility());
      floatPointer.put(24, (float) plannerParameters.getMaxNormalAngleToPenalizeForTraversibility());
      floatPointer.put(25, (float) plannerParameters.getTraversibilityInclineWeight());
      floatPointer.put(26, (float) plannerParameters.getTraversibilityWeight());
      floatPointer.put(27, (float) plannerParameters.getTraversibilityHeightWindowDeadband());
      floatPointer.put(28, (float) plannerParameters.getHeightProximityForSayingWalkingOnGround());
      floatPointer.put(29, (float) plannerParameters.getTraversibilityNonGroundDiscountWhenWalkingOnGround());
      floatPointer.put(30, (float) plannerParameters.getMinOccupiedNeighborsForTraversibility());

      pathPlanningParametersBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Populates the ransac offset buffer, which is the array of offsets to search with RANSAC. This has to be called before you can compute the surface normals.
    */
   private void populateLeastSquaresOffsetBuffer(double patchWidth)
   {
      int patchCellHalfWidth = (int) ((patchWidth / 2.0) / heightMapData.getGridResolutionXY());
      if (patchCellHalfWidth % 2 == 0)
         patchCellHalfWidth++;

      int size = 2 * patchCellHalfWidth + 1;
      int connections = size * size;
      leastSquaresOffsetBuffer.resize(2 * connections + 1, openCLManager);
      IntPointer intPointer = leastSquaresOffsetBuffer.getBytedecoIntBufferPointer();
      int index = 0;
      intPointer.put(index++, connections);
      for (int x = -patchCellHalfWidth; x <= patchCellHalfWidth; x++)
      {
         for (int y = -patchCellHalfWidth; y <= patchCellHalfWidth; y++)
         {
            // pack the x offsets
            intPointer.put(index, y);
            // pack the y offsets
            intPointer.put(connections + index++, x);
         }
      }

      leastSquaresOffsetBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Computes the surface normal using least squares at each location of the height map on the GPU, and then reads these data back into the CPU.
    */
   private void computeSurfaceNormalsWithLeastSquares(double patchWidth)
   {
      populateLeastSquaresOffsetBuffer(patchWidth);

      // set the kernel arguments
      openCLManager.setKernelArgument(computeNormalsWithLeastSquaresKernel, 0, heightMapParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsWithLeastSquaresKernel, 1, leastSquaresOffsetBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsWithLeastSquaresKernel, 2, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsWithLeastSquaresKernel, 3, leastSquaresNormalXYZBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsWithLeastSquaresKernel, 4, sampledHeightBuffer.getOpenCLBufferObject());

      int totalCells = cellsPerSide * cellsPerSide;
      openCLManager.execute1D(computeNormalsWithLeastSquaresKernel, totalCells);

      // get the data from the GPU
      sampledHeightBuffer.readOpenCLBufferObject(openCLManager);
      leastSquaresNormalXYZBuffer.readOpenCLBufferObject(openCLManager);
   }

   /**
    * Populates the parameter buffer for the ransac calculator. This has to be called before you can compute the surface normals.
    */
   private void populateRansacNormalParametersBuffer()
   {
      FloatPointer floatPointer = ransacNormalParametersBuffer.getBytedecoFloatBufferPointer();
      floatPointer.put(0, (float) HeightMapRANSACNormalCalculator.iterations);
      floatPointer.put(1, (float) HeightMapRANSACNormalCalculator.distanceEpsilon);
      floatPointer.put(2, (float) HeightMapRANSACNormalCalculator.minNormalZ);
      floatPointer.put(3, (float) HeightMapRANSACNormalCalculator.acceptableConcensus);

      ransacNormalParametersBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Populates the ransac offset buffer, which is the array of offsets to search with RANSAC. This has to be called before you can compute the surface normals.
    */
   private void populateRansacOffsetBuffer()
   {
      int maxOffset = (int) Math.round(HeightMapRANSACNormalCalculator.maxRansacRadius / heightMapData.getGridResolutionXY());

      TIntArrayList xRansacOffsets = new TIntArrayList();
      TIntArrayList yRansacOffsets = new TIntArrayList();
      TIntArrayList xConsensusOffsets = new TIntArrayList();
      TIntArrayList yConsensusOffsets = new TIntArrayList();

      for (int xi = -maxOffset; xi <= maxOffset; xi++)
      {
         for (int yi = -maxOffset; yi <= maxOffset; yi++)
         {
            double radius = heightMapData.getGridResolutionXY() * EuclidCoreTools.norm(xi, yi);
            if (radius > HeightMapRANSACNormalCalculator.minRansacRadius && radius < HeightMapRANSACNormalCalculator.maxRansacRadius)
            {
               xRansacOffsets.add(xi);
               yRansacOffsets.add(yi);
            }
            if (radius < HeightMapRANSACNormalCalculator.consensusRadius)
            {
               xConsensusOffsets.add(xi);
               yConsensusOffsets.add(yi);
            }
         }
      }

      int offsets = xRansacOffsets.size();
      int consensusOffsets = xConsensusOffsets.size();
      ransacOffsetBuffer.resize(2 * (offsets + consensusOffsets + 1), openCLManager);
      IntPointer intPointer = ransacOffsetBuffer.getBytedecoIntBufferPointer();

      int index = 0;
      intPointer.put(index, offsets);
      index++;
      intPointer.put(index, consensusOffsets);
      index++;
      for (int i = 0; i < offsets; i++)
      {
         intPointer.put(index, xRansacOffsets.get(i));
         intPointer.put(offsets + index, yRansacOffsets.get(i));
         index++;
      }
      for (int i = 0; i < consensusOffsets; i++)
      {
         intPointer.put(index, xConsensusOffsets.get(i));
         intPointer.put(consensusOffsets + index, yConsensusOffsets.get(i));
         index++;
      }

      ransacOffsetBuffer.writeOpenCLBufferObject(openCLManager);
   }

   /**
    * Computes the surface normal using ransac at each location of the height map on the GPU, and then reads these data back into the CPU.
    */
   private void computeSurfaceNormalsWithRansac()
   {
      populateRansacNormalParametersBuffer();
      populateRansacOffsetBuffer();

      // set the kernel arguments
      openCLManager.setKernelArgument(computeNormalsWithRansacKernel, 0, heightMapParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsWithRansacKernel, 1, ransacNormalParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsWithRansacKernel, 2, ransacOffsetBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsWithRansacKernel, 3, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsWithRansacKernel, 4, ransacNormalXYZBuffer.getOpenCLBufferObject());

      int totalCells = cellsPerSide * cellsPerSide;
      openCLManager.execute1D(computeNormalsWithRansacKernel, totalCells);
   }

   /**
    * Computes the height at each planning node from the height map as the approximately locally averaged height on the GPU, and then reads these data back into the CPU.
    */
   private void computeSnapHeightsOnTheGPU()
   {
      openCLManager.setKernelArgument(snapVerticesKernel, 0, heightMapParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(snapVerticesKernel, 1, pathPlanningParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(snapVerticesKernel, 2, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(snapVerticesKernel, 3, snapOffsetsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(snapVerticesKernel, 4, snappedNodeHeightBuffer.getOpenCLBufferObject());

      int totalCells = nodesPerSide * nodesPerSide;
      openCLManager.execute1D(snapVerticesKernel, totalCells);

      snappedNodeHeightBuffer.readOpenCLBufferObject(openCLManager);
   }

   /**
    * Computes the heuristic cost at every possible location in the mapped world on the GPU, and then reads these data back into the CPU.
    */
   private void computeHeuristicCostMapOnTheGPU()
   {
      openCLManager.setKernelArgument(computeHeuristicCostKernel, 0, heightMapParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeHeuristicCostKernel, 1, pathPlanningParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeHeuristicCostKernel, 2, heuristicCostMapBuffer.getOpenCLBufferObject());

      int totalCells = nodesPerSide * nodesPerSide;
      if (heightMapParametersBuffer.getBackingDirectFloatBuffer().limit() != 6)
         throw new RuntimeException("Bad height map parameters buffer length");
      if (pathPlanningParametersBuffer.getBackingDirectFloatBuffer().limit() != 31)
         throw new RuntimeException("Bad path planning parameters buffer length");
      if (heuristicCostMapBuffer.getBackingDirectFloatBuffer().limit() != totalCells)
         throw new RuntimeException("Bad buffer length");

      openCLManager.execute1D(computeHeuristicCostKernel, totalCells);

      heuristicCostMapBuffer.readOpenCLBufferObject(openCLManager);
   }

   /**
    * Computes the edge validity and cost metrics for the entire map on the GPU, and then reads these data back into the CPU.
    */
   private void computeEdgeDataCostAndValidityOnTheGPU()
   {
      openCLManager.setKernelArgument(computeEdgeDataKernel, 0, heightMapParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 1, pathPlanningParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 2, neighborOffsetsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 3, traversibilityOffsetsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 4, collisionOffsetsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 5, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 6, snappedNodeHeightBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 7, leastSquaresNormalXYZBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 8, ransacNormalXYZBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 9, edgeRejectionReasonBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 10, deltaHeightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 11, inclineMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 12, rollMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 13, stanceTraversibilityMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 14, stepTraversibilityMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 15, inclineCostMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 16, rollCostMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 17, traversibilityCostMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeEdgeDataKernel, 18, edgeCostMapBuffer.getOpenCLBufferObject());

      openCLManager.execute3D(computeEdgeDataKernel, nodesPerSide, nodesPerSide, numberOfNeighborsPerExpansion);

      edgeRejectionReasonBuffer.readOpenCLBufferObject(openCLManager);
      deltaHeightMapBuffer.readOpenCLBufferObject(openCLManager);
      inclineMapBuffer.readOpenCLBufferObject(openCLManager);
      rollMapBuffer.readOpenCLBufferObject(openCLManager);
      stanceTraversibilityMapBuffer.readOpenCLBufferObject(openCLManager);
      stepTraversibilityMapBuffer.readOpenCLBufferObject(openCLManager);
      inclineCostMapBuffer.readOpenCLBufferObject(openCLManager);
      rollCostMapBuffer.readOpenCLBufferObject(openCLManager);
      traversibilityCostMapBuffer.readOpenCLBufferObject(openCLManager);
      edgeCostMapBuffer.readOpenCLBufferObject(openCLManager);
   }

   /**
    * Convenience method to get the surface normal found using least squares from the GPU buffer at a certain location.
    * @return Surface normal using least squares at this location.
    */
   private UnitVector3DReadOnly getSurfaceNormal(int key)
   {
      float x = leastSquaresNormalXYZBuffer.getBackingDirectFloatBuffer().get(3 * key);
      float y = leastSquaresNormalXYZBuffer.getBackingDirectFloatBuffer().get(3 * key + 1);
      float z = leastSquaresNormalXYZBuffer.getBackingDirectFloatBuffer().get(3 * key + 2);
      return new UnitVector3D(x, y, z);
   }

   /**
    * Computes whether the edge is a valid edge, and populates the different metrics from the GPU memory. The only value that is actively used in the planning
    * process is the rejection reason, which is used to compute the return of this method. The rest of the fields are used for logging.
    * @return whether the edge is valid, and can be used.
    */
   private boolean checkEdge(BodyPathLatticePoint node, BodyPathLatticePoint neighbor, int childNodeKey, int edgeKey)
   {
      int localRejectionReason = edgeRejectionReasonBuffer.getBackingDirectIntBuffer().get(edgeKey);

      if (localRejectionReason == 0)
      {
         this.snapHeight.setToNaN();
         rejectionReason.set(RejectionReason.INVALID_SNAP);
         graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
         return false;
      }
      this.snapHeight.set(getSnapHeight(childNodeKey));

      deltaHeight.set(deltaHeightMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));
      incline.set(inclineMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));

      if (localRejectionReason == 1)
      {
         rejectionReason.set(RejectionReason.TOO_STEEP);
         graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
         return false;
      }

      if (localRejectionReason == 3)
      {
         this.containsCollision.set(true);
         rejectionReason.set(RejectionReason.COLLISION);
         graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
         return false;
      }

      stanceScore.get(RobotSide.LEFT).set(stanceTraversibilityMapBuffer.getBackingDirectFloatBuffer().get(2 * edgeKey));
      stanceScore.get(RobotSide.RIGHT).set(stanceTraversibilityMapBuffer.getBackingDirectFloatBuffer().get(2 * edgeKey + 1));
      stepScores.get(RobotSide.LEFT).set(stepTraversibilityMapBuffer.getBackingDirectFloatBuffer().get(2 * edgeKey));
      stepScores.get(RobotSide.RIGHT).set(stepTraversibilityMapBuffer.getBackingDirectFloatBuffer().get(2 * edgeKey + 1));
      traversibilityCost.set(traversibilityCostMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));
      stanceTraversibility.set(Math.max(stanceScore.get(RobotSide.LEFT).getDoubleValue(), stanceScore.get(RobotSide.RIGHT).getDoubleValue()));

      if (localRejectionReason == 4)
      {
         rejectionReason.set(RejectionReason.NON_TRAVERSIBLE);
         graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
         return false;
      }

      return true;
   }

   /**
    * Populates the edge cost fields from the GPU buffers. The only one that is actively used by the planner is the {@link #edgeCost}, as the rest of the costs
    * compose this edge cost. The rest of the variables are only used for logging.
    */
   private void computeEdgeCost(BodyPathLatticePoint node, BodyPathLatticePoint neighbor, int edgeKey)
   {
      // composite cost, which consists of other values like roll cost and traversibility cost. Calculated on the GPU.
      edgeCost.set(edgeCostMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));
      // Cost of that edge, when compared to the nominal incline. Calculated on the GPU. Purely a function of the start, goal, and edge beginning and end heights.
      inclineCost.set(inclineCostMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));

      if (plannerParameters.getComputeSurfaceNormalCost())
      {
         // This the roll of the current edge, which is the incline of the world perpendicular to the edge direction.
         roll.set(rollMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));
         // The cost of the roll
         rollCost.set(rollCostMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));

         // Get the midpoint of the edge.
         double bodyX = 0.5 * (neighbor.getX() + node.getX());
         double bodyY = 0.5 * (neighbor.getY() + node.getY());

         // ensure that this body position is within bounds. Sometimes rounding errors cause this to fail.
         double halfWidth = heightMapData.getGridSizeXY() / 2.0;
         bodyX = MathTools.clamp(bodyX, heightMapData.getGridCenter().getX() - halfWidth, heightMapData.getGridCenter().getX() + halfWidth);
         bodyY = MathTools.clamp(bodyY, heightMapData.getGridCenter().getY() - halfWidth, heightMapData.getGridCenter().getY() + halfWidth);

         // Sets the normal value at the midpoint of the edge, found using least squares on the GPU.
         leastSqNormal.set(getSurfaceNormal(HeightMapTools.coordinateToKey(bodyX,
                                                                           bodyY,
                                                                           heightMapData.getGridCenter().getX(),
                                                                           heightMapData.getGridCenter().getY(),
                                                                           heightMapData.getGridResolutionXY(),
                                                                           heightMapData.getCenterIndex())));
      }

      if (edgeCost.getValue() < 0.0)
      {
         throw new RuntimeException("Negative edge cost!");
      }
   }

   /**
    * Takes the output body path found by the A star search and converts it into an Output message. This includes optionally performing a smoothing step.
    */
   private void reportStatus(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      if (debug)
      {
         LogTools.info("Reporting status");
      }

      // We don't want to do any real smoothing if we didn't reach the goal.
      boolean performSmoothing = plannerParameters.getPerformSmoothing() && result == BodyPathPlanningResult.FOUND_SOLUTION;

      // reset the initial aspects of the output message
      outputToPack.setBodyPathPlanningResult(result);
      outputToPack.getBodyPath().clear();
      outputToPack.getBodyPathUnsmoothed().clear();

      // If we don't reach the goal, we want the node that has the lowest predicted cost to the goal.
      BodyPathLatticePoint terminalNode = reachedGoal ? goalNode : leastCostNode;
      // This is the optimal path in the grid map coordinates
      List<BodyPathLatticePoint> path = graph.getPathFromStart(terminalNode);
      // This is the body path that will either be smoothed or added to the output message
      List<Point3D> bodyPath = new ArrayList<>();

      for (int i = 0; i < path.size(); i++)
      {
         // get the waypoint from the 2D path position and the snapped height at that position
         Point3D waypoint = new Point3D(path.get(i).getX(), path.get(i).getY(), getSnapHeight(getNodeGraphKey(path.get(i))));
         // pack that waypoint into the body path
         bodyPath.add(waypoint);
         // set that waypoint as part of the unsmoothed body path
         outputToPack.getBodyPathUnsmoothed().add(waypoint);
      }

      if (performSmoothing)
      {
         List<Pose3D> smoothedPath = smoother.doSmoothing(bodyPath,
                                                          heightMapData,
                                                          heightMapParametersBuffer,
                                                          pathPlanningParametersBuffer,
                                                          heightMapBuffer,
                                                          snappedNodeHeightBuffer,
                                                          ransacNormalParametersBuffer,
                                                          leastSquaresNormalXYZBuffer,
                                                          sampledHeightBuffer);
         for (int i = 0; i < bodyPath.size(); i++)
         {
            Pose3D waypoint = new Pose3D(smoothedPath.get(i));

            if (i == 0)
            {
               // Set the starting yaw as the middle of the stance feet
               double yaw = AngleTools.interpolateAngle(request.getStartFootPoses().get(RobotSide.LEFT).getYaw(), request.getStartFootPoses().get(RobotSide.RIGHT).getYaw(), 0.5);
               waypoint.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
            }
            else if (i == bodyPath.size() - 1)
            {
               // Set the ending yaw as the middle of the goal feet
               double yaw = AngleTools.interpolateAngle(request.getGoalFootPoses().get(RobotSide.LEFT).getYaw(), request.getGoalFootPoses().get(RobotSide.RIGHT).getYaw(), 0.5);
               waypoint.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
            }

            outputToPack.getBodyPath().add(waypoint);
         }
      }
      else
      {
         for (int i = 0 ; i < bodyPath.size(); i++)
         {
            Pose3D waypoint = new Pose3D();
            waypoint.getPosition().set(bodyPath.get(i));

            if (i == 0)
            {
               // Set the starting yaw as the middle of the stance feet
               double yaw = AngleTools.interpolateAngle(request.getStartFootPoses().get(RobotSide.LEFT).getYaw(), request.getStartFootPoses().get(RobotSide.RIGHT).getYaw(), 0.5);
               waypoint.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
            }
            else if (i == bodyPath.size() - 1)
            {
               // Set the ending yaw as the middle of the goal feet
               double yaw = AngleTools.interpolateAngle(request.getGoalFootPoses().get(RobotSide.LEFT).getYaw(), request.getGoalFootPoses().get(RobotSide.RIGHT).getYaw(), 0.5);
               waypoint.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
            }
            else
            {
               // Since we didn't do smoothing, set the yaw as the heading to get to the next waypoint.
               waypoint.getOrientation().setToYawOrientation(AngleTools.calculateHeading(bodyPath.get(i), bodyPath.get(i + 1), 0.0));
            }

            outputToPack.getBodyPath().add(waypoint);
         }

      }

      outputToPack.getPlannerTimings().setTimePlanningBodyPathSeconds(stopwatch.totalElapsed() - planningStartTime);
      outputToPack.getPlannerTimings().setTotalElapsedSeconds(stopwatch.totalElapsed());

      if (reachedGoal)
      {
         // If we reached the goal, move onto the planning step.
         outputToPack.setFootstepPlanningResult(FootstepPlanningResult.PLANNING);
      }

      markSolutionEdges(terminalNode);
      statusCallbacks.forEach(callback -> callback.accept(outputToPack));
   }

   /**
    *  This clears out the old logged data maps. If this isn't done, these maps will continue to get populated on subsequent steps. However, this should not
    *  be done automatically as part of the planning process, since the logs are saved aftwards.
     */

   @Override
   public void clearLoggedData()
   {
      edgeDataMap.clear();
      iterationData.clear();
   }

   /**
    * Computes whether the planner should incrementally publish its status. This allows the planner to output its current status to a remote process, if
    * the plan is taking a long time, using the message.
    */
   private boolean shouldPublishStatus(FootstepPlannerRequest request)
   {
      double statusPublishPeriod = request.getStatusPublishPeriod();
      if (statusPublishPeriod <= 0.0)
      {
         return false;
      }

      return stopwatch.lapElapsed() > statusPublishPeriod && !MathTools.epsilonEquals(stopwatch.totalElapsed(), request.getTimeout(), 0.8 * request.getStatusPublishPeriod());
   }

   private void markSolutionEdges(BodyPathLatticePoint terminalNode)
   {
      edgeDataMap.values().forEach(data -> data.setSolutionEdge(false));
      List<BodyPathLatticePoint> path = graph.getPathFromStart(terminalNode);
      for (int i = 1; i < path.size(); i++)
      {
         edgeDataMap.get(new GraphEdge<>(path.get(i - 1), path.get(i))).setSolutionEdge(true);
      }
   }

   /**
    * Returns the best next node to expand from the plan queue.
    */
   @Override
   public BodyPathLatticePoint getNextNode()
   {
      while (!stack.isEmpty())
      {
         BodyPathLatticePoint nextNode = stack.poll();
         if (!expandedNodeSet.contains(nextNode))
         {
            return nextNode;
         }
      }

      return null;
   }


   /**
    * Populates a 16-connected grid starting along +x and moving clockwise
    */
   private void populateNeighbors(BodyPathLatticePoint latticePoint)
   {
      neighbors.clear();

      for (int i = 0; i < neighborsOffsetX.size(); i++)
      {
         neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + neighborsOffsetX.get(i), latticePoint.getYIndex() + neighborsOffsetY.get(i)));
      }
   }

   /**
    * Returns the snapped height as a certain location. As the grid size of the planning map is different than the height map, this is approximately the local
    * average height of the height map at this location.
    */
   private double getSnapHeight(int nodeKey)
   {
      return snappedNodeHeightBuffer.getBackingDirectFloatBuffer().get(nodeKey);
   }

   /**
    * Returns the heuristic cost for the node point.
    */
   private double getHeuristicCost(BodyPathLatticePoint node)
   {
      return heuristicCostMapBuffer.getBackingDirectFloatBuffer().get(getNodeGraphKey(node));
   }

   /**
    * Ceases the iterative planning at the current iteration, and will return the best un-smoothed plan that has been found so far
    */
   @Override
   public void halt()
   {
      haltRequested.set(true);
   }

   /**
    * Retuns the list of all the iteration data for the planner. This is used for logging.
    */
   @Override
   public List<AStarBodyPathIterationData> getIterationData()
   {
      return iterationData;
   }

   /**
    * Returns the map of all edge data in the graph that has been calculated so far. This is used for logging.
    */
   @Override
   public HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> getEdgeDataMap()
   {
      return edgeDataMap;
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }
}
