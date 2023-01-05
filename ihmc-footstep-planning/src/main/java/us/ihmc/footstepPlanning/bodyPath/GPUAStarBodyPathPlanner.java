package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.AStarBodyPathEdgeData;
import us.ihmc.footstepPlanning.log.AStarBodyPathIterationData;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;
import us.ihmc.perception.*;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.Activator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class GPUAStarBodyPathPlanner
{
   private static final int numberOfNeighborsPerExpansion = 16;
   private static final int defaultCells = (int) (5.0 / 0.03);
   private static final int defaultNodes = (int) (5.0 / BodyPathLatticePoint.gridSizeXY);

   private static final boolean debug = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly parameters;
   private final AStarBodyPathPlannerParametersReadOnly plannerParameters;
   private final AStarBodyPathEdgeData edgeData;
   private HeightMapData heightMapData;
   private final HashSet<BodyPathLatticePoint> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<BodyPathLatticePoint> graph = new DirectedGraph<>();
   private final TIntArrayList neighborsOffsetX = new TIntArrayList();
   private final TIntArrayList neighborsOffsetY = new TIntArrayList();
   private final List<BodyPathLatticePoint> neighbors = new ArrayList<>();

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
   private final YoFrameVector3D ransacNormal = new YoFrameVector3D("ransacNormal", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble heuristicCost = new YoDouble("heuristicCost", registry);
   private final YoDouble totalCost = new YoDouble("totalCost", registry);

   private final SideDependentList<YoDouble> stanceScore;
   private final SideDependentList<YoDouble> stepScores;
   private final YoDouble stanceTraversibility;

   private final PriorityQueue<BodyPathLatticePoint> stack;
   private BodyPathLatticePoint startNode, goalNode;
   private BodyPathLatticePoint leastCostNode = null;
   private final YoEnum<RejectionReason> rejectionReason = new YoEnum<>("rejectionReason", registry, RejectionReason.class, true);

   private final TIntArrayList xSnapOffsets = new TIntArrayList();
   private final TIntArrayList ySnapOffsets = new TIntArrayList();

   private final List<AStarBodyPathIterationData> iterationData = new ArrayList<>();
   private final HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap = new HashMap<>();

   private final List<Consumer<FootstepPlannerOutput>> statusCallbacks;
   private final Stopwatch stopwatch;
   private double planningStartTime;
   private int iterations = 0;
   private BodyPathPlanningResult result = null;
   private boolean reachedGoal = false;
   private final AtomicBoolean haltRequested = new AtomicBoolean();
   private static final int maxIterations = 3000;

   private final OpenCLManager openCLManager;
   private _cl_program pathPlannerProgram;
   private _cl_kernel computeNormalsWithLeastSquaresKernel;
   private _cl_kernel computeNormalsWithRansacKernel;
   private _cl_kernel snapVerticesKernel;
   private _cl_kernel computeEdgeDataKernel;
   private _cl_kernel computeHeuristicCostKernel;

   private OpenCLFloatBuffer heightMapParametersBuffer = new OpenCLFloatBuffer(6);
   private OpenCLFloatBuffer pathPlanningParametersBuffer = new OpenCLFloatBuffer(20);
   private OpenCLFloatBuffer ransacNormalParametersBuffer = new OpenCLFloatBuffer(8);

   private OpenCLIntBuffer leastSquaresOffsetBuffer = new OpenCLIntBuffer(6);
   private OpenCLIntBuffer ransacOffsetBuffer = new OpenCLIntBuffer(4);
   private OpenCLIntBuffer snapOffsetsBuffer = new OpenCLIntBuffer(6);
   private OpenCLIntBuffer traversibilityOffsetsBuffer = new OpenCLIntBuffer(8);
   private OpenCLIntBuffer collisionOffsetsBuffer = new OpenCLIntBuffer(8);
   private OpenCLIntBuffer neighborOffsetsBuffer = new OpenCLIntBuffer(33);

   // TODO once the things that use the normal are all on the GPU, these can be replaced with _cl_mem object = openCLManager.createImage as the backing data in
   // the cpu is no longer needed
   private OpenCLFloatBuffer heightMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer leastSquaresNormalXYZBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer ransacNormalXYZBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer sampledHeightBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer snappedNodeHeightBuffer = new OpenCLFloatBuffer(1);
   private OpenCLIntBuffer edgeRejectionReasonBuffer = new OpenCLIntBuffer(1);
   private OpenCLFloatBuffer deltaHeightMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer inclineMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer rollMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer stanceTraversibilityMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer stepTraversibilityMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer inclineCostMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer rollCostMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer traversibilityCostMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer edgeCostMapBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer heuristicCostMapBuffer = new OpenCLFloatBuffer(1);

   private GPUAStarBodyPathSmoother smoother;


   private int cellsPerSide = -1;
   private int nodesPerSide = -1;
   private int nodeCenterIndex = -1;

   private boolean firstTick = true;

   /* Parameters to extract */
   static final double groundClearance = 0.3;
   static final double maxIncline = Math.toRadians(55.0);
   static final double snapRadius = 0.15;
   static final double boxSizeY = 1.2;
   static final double boxSizeX = 0.35;

   public GPUAStarBodyPathPlanner(FootstepPlannerParametersReadOnly parameters,
                                  AStarBodyPathPlannerParametersReadOnly plannerParameters,
                                  SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this(parameters, plannerParameters, footPolygons, new Stopwatch());
   }

   public GPUAStarBodyPathPlanner(FootstepPlannerParametersReadOnly parameters,
                                  AStarBodyPathPlannerParametersReadOnly plannerParameters,
                                  SideDependentList<ConvexPolygon2D> footPolygons,
                                  Stopwatch stopwatch)
   {
      this(parameters, plannerParameters, footPolygons, new ArrayList<>(), stopwatch);
   }

   public GPUAStarBodyPathPlanner(FootstepPlannerParametersReadOnly parameters,
                                  AStarBodyPathPlannerParametersReadOnly plannerParameters,
                                  SideDependentList<ConvexPolygon2D> footPolygons,
                                  List<Consumer<FootstepPlannerOutput>> statusCallbacks,
                                  Stopwatch stopwatch)
   {
      this.parameters = parameters;
      this.plannerParameters = plannerParameters;
      this.statusCallbacks = statusCallbacks;
      this.stopwatch = stopwatch;
      // TODO get the heuristics from the map
      stack = new PriorityQueue<>(new NodeComparator<>(graph, this::heuristics));

      this.stanceScore = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "StanceScore", registry));
      this.stepScores = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "StepScore", registry));
      this.stanceTraversibility = new YoDouble("stanceTraversibility", registry);


      openCLManager = new OpenCLManager();
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroyOpenCLStuff));

      smoother = new GPUAStarBodyPathSmoother(null, openCLManager, null, null);

      Activator nativeLoader = BytedecoTools.loadNativesOnAThread();
      boolean doneLoading = false;

      while (!doneLoading)
      {
         if (nativeLoader.poll())
         {
            if (nativeLoader.isNewlyActivated())
            {
               createOpenCLStuff(defaultCells, defaultNodes);
               doneLoading = true;
            }
         }
      }

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
                                         ransacNormal.setToZero();
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

   public void createOpenCLStuff(int numberOfCells, int numberOfNodes)
   {
      cellsPerSide = numberOfCells;
      this.nodesPerSide = numberOfNodes;
      this.nodeCenterIndex = (nodesPerSide - 1) / 2;

      openCLManager.create();

      pathPlannerProgram = openCLManager.loadProgram("BodyPathPlanning");
      computeNormalsWithLeastSquaresKernel = openCLManager.createKernel(pathPlannerProgram, "computeSurfaceNormalsWithLeastSquares");
      computeNormalsWithRansacKernel = openCLManager.createKernel(pathPlannerProgram, "computeSurfaceNormalsWithRANSAC");
      snapVerticesKernel = openCLManager.createKernel(pathPlannerProgram, "snapVertices");
      computeEdgeDataKernel = openCLManager.createKernel(pathPlannerProgram, "computeEdgeData");
      computeHeuristicCostKernel = openCLManager.createKernel(pathPlannerProgram, "computeHeuristicCost");

      smoother.createOpenCLStuff(pathPlannerProgram, numberOfCells, numberOfNodes);
   }

   public void firstTickSetup()
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
      heuristicCostMapBuffer.resize(totalEdges, openCLManager);
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   static void packRadialOffsets(HeightMapData heightMapData, double radius, TIntArrayList xOffsets, TIntArrayList yOffsets)
   {
      int minMaxOffsetXY = (int) Math.round(radius / heightMapData.getGridResolutionXY());

      xOffsets.clear();
      yOffsets.clear();

      for (int i = -minMaxOffsetXY; i <= minMaxOffsetXY; i++)
      {
         for (int j = -minMaxOffsetXY; j <= minMaxOffsetXY; j++)
         {
            double x = i * heightMapData.getGridResolutionXY();
            double y = j * heightMapData.getGridResolutionXY();
            if (EuclidCoreTools.norm(x, y) < radius && !(i == 0 && j == 0))
            {
               xOffsets.add(i);
               yOffsets.add(j);
            }
         }
      }
   }

   void populateRadialOffsetsBuffer()
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

   private void populateTraversibilityOffsetsBuffer()
   {
      TIntArrayList zeroDegCollisionOffsetsX = new TIntArrayList();
      TIntArrayList zeroDegCollisionOffsetsY = new TIntArrayList();
      TIntArrayList fourtyFiveDegCollisionOffsetsX = new TIntArrayList();
      TIntArrayList fourtyFiveDegCollisionOffsetsY = new TIntArrayList();
      TIntArrayList twentyTwoCollisionOffsetsX = new TIntArrayList();
      TIntArrayList twentyTwoCollisionOffsetsY = new TIntArrayList();

      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(),
                                            zeroDegCollisionOffsetsX,
                                            zeroDegCollisionOffsetsY,
                                            BodyPathRANSACTraversibilityCalculator.sampleSizeX,
                                            BodyPathRANSACTraversibilityCalculator.sampleSizeY,
                                            0.0);
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(),
                                            fourtyFiveDegCollisionOffsetsX,
                                            fourtyFiveDegCollisionOffsetsY,
                                            BodyPathRANSACTraversibilityCalculator.sampleSizeX,
                                            BodyPathRANSACTraversibilityCalculator.sampleSizeY,
                                            Math.toRadians(45.0));
      BodyPathCollisionDetector.packOffsets(heightMapData.getGridResolutionXY(),
                                            twentyTwoCollisionOffsetsX,
                                            twentyTwoCollisionOffsetsY,
                                            BodyPathRANSACTraversibilityCalculator.sampleSizeX,
                                            BodyPathRANSACTraversibilityCalculator.sampleSizeY,
                                            Math.toRadians(22.5));

      int offsets0 = zeroDegCollisionOffsetsX.size();
      int offsets1 = fourtyFiveDegCollisionOffsetsY.size();
      int offsets2 = twentyTwoCollisionOffsetsY.size();
      traversibilityOffsetsBuffer.resize(2 * offsets0 + 2 * offsets1 + 2 * offsets2 + 3, openCLManager);
      int index = 0;
      IntPointer intPointer = traversibilityOffsetsBuffer.getBytedecoIntBufferPointer();

      intPointer.put(index++, offsets0);
      intPointer.put(index++, offsets1);
      intPointer.put(index++, offsets2);
      for (int i = 0; i < offsets0; i++)
      {
         intPointer.put(index, zeroDegCollisionOffsetsX.get(i));
         intPointer.put(offsets0 + index++, zeroDegCollisionOffsetsY.get(i));
      }
      for (int i = 0; i < offsets1; i++)
      {
         intPointer.put(index, fourtyFiveDegCollisionOffsetsX.get(i));
         intPointer.put(offsets1 + index++, fourtyFiveDegCollisionOffsetsY.get(i));
      }
      for (int i = 0; i < offsets2; i++)
      {
         intPointer.put(index, twentyTwoCollisionOffsetsX.get(i));
         intPointer.put(offsets2 + index++, twentyTwoCollisionOffsetsY.get(i));
      }

      traversibilityOffsetsBuffer.writeOpenCLBufferObject(openCLManager);
   }

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
      int index = 0;
      IntPointer intPointer = collisionOffsetsBuffer.getBytedecoIntBufferPointer();

      intPointer.put(index++, offsets0);
      intPointer.put(index++, offsets1);
      intPointer.put(index++, offsets2);
      for (int i = 0; i < offsets0; i++)
      {
         intPointer.put(index, collisionOffsetsX1.get(i));
         intPointer.put(offsets0 + index++, collisionOffsetsY1.get(i));
      }
      for (int i = 0; i < offsets1; i++)
      {
         intPointer.put(index, collisionOffsetsX2.get(i));
         intPointer.put(offsets1 + index++, collisionOffsetsY2.get(i));
      }
      for (int i = 0; i < offsets2; i++)
      {
         intPointer.put(index, collisionOffsetsX3.get(i));
         intPointer.put(offsets2 + index++, collisionOffsetsY3.get(i));
      }

      collisionOffsetsBuffer.writeOpenCLBufferObject(openCLManager);
   }

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


   private enum RejectionReason
   {
      INVALID_SNAP,
      TOO_STEEP,
      STEP_TOO_HIGH,
      COLLISION,
      NON_TRAVERSIBLE
   }

   public void handleRequest(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
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
      iterations = 0;
      reachedGoal = false;
      stopwatch.start();
      result = BodyPathPlanningResult.PLANNING;
      planningStartTime = stopwatch.totalElapsed();
      stopwatch.lap();

      iterationData.clear();
      edgeDataMap.clear();

      packRadialOffsets(heightMapData, snapRadius, xSnapOffsets, ySnapOffsets);
      packNeighborOffsets(neighborsOffsetX, neighborsOffsetY);

      populateRadialOffsetsBuffer();
      populateTraversibilityOffsetsBuffer();
      populateCollisionsOffsetsBuffer(heightMapData.getGridResolutionXY(), boxSizeX, boxSizeY);
      populateNeighborOffsetsBuffer();

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

      populateHeightMapImage();

      double patchWidth = 0.3;
      populateHeightMapParameterBuffer(patchWidth);
      populatePathPlanningParametersBuffer();

      if (plannerParameters.getComputeSurfaceNormalCost())
      {
         computeSurfaceNormalsWithLeastSquares(patchWidth);
      }
      computeSnapHeights();
      computeSurfaceNormalsWithRansac();
      computeHeuristicCost();
      computeEdgeData();

      planningLoop:
      while (true)
      {
         iterations++;
         outputToPack.getPlannerTimings().setPathPlanningIterations(iterations);

         if (stopwatch.totalElapsed() >= request.getTimeout())
         {
            result = BodyPathPlanningResult.TIMED_OUT_BEFORE_SOLUTION;
            break;
         }
         if (haltRequested.get())
         {
            result = BodyPathPlanningResult.HALTED;
            break;
         }
         if (iterations > maxIterations)
         {
            result = BodyPathPlanningResult.MAXIMUM_ITERATIONS_REACHED;
            break;
         }

         BodyPathLatticePoint node = getNextNode();
         if (node == null)
         {
            result = BodyPathPlanningResult.NO_PATH_EXISTS;
            if (debug)
            {
               LogTools.info("Stack is empty, no path exists...");
            }
            break;
         }

         int parentNodeGraphKey = getNodeGraphKey(node);
         int edgeKeyStart = parentNodeGraphKey * numberOfNeighborsPerExpansion;
         populateNeighbors(node);

         double parentSnapHeight = snap(parentNodeGraphKey);
         for (int neighborIndex = 0; neighborIndex < neighbors.size(); neighborIndex++)
         {
            BodyPathLatticePoint neighbor = neighbors.get(neighborIndex);
            int neighborNodeKey = getNodeGraphKey(neighbor);
            ransacNormal.set(getRansacSurfaceNormal(neighborNodeKey));

            int edgeKey = edgeKeyStart + neighborIndex;

            if (!checkEdge(node, neighbor, neighborNodeKey, edgeKey))
               continue;

            heuristicCost.set(heuristicCostMapBuffer.getBackingDirectFloatBuffer().get(neighborNodeKey));

            computeEdgeCost(node, neighbor, edgeKey);

            totalCost.set(heuristicCost.getValue() + edgeCost.getValue());
            graph.checkAndSetEdge(node, neighbor, edgeCost.getValue());
            stack.add(neighbor);

            if (node.equals(goalNode))
            {
               reachedGoal = true;
               result = BodyPathPlanningResult.FOUND_SOLUTION;
               break planningLoop;
            }
            else if (heuristics(node) < heuristics(leastCostNode))
            {
               leastCostNode = node;
            }
         }

         expandedNodeSet.add(node);

         AStarBodyPathIterationData iterationData = new AStarBodyPathIterationData();
         iterationData.setParentNode(node);
         iterationData.getChildNodes().addAll(neighbors);
         iterationData.setParentNodeHeight(parentSnapHeight);
         this.iterationData.add(iterationData);

         if (publishStatus(request))
         {
            reportStatus(request, outputToPack);
            stopwatch.lap();
         }
      }

      reportStatus(request, outputToPack);
   }

   private int getNodeGraphKey(BodyPathLatticePoint node)
   {
      int nodeGraphXIdx = HeightMapTools.coordinateToIndex(node.getX(), heightMapData.getGridCenter().getX(), BodyPathLatticePoint.gridSizeXY, nodeCenterIndex);
      int nodeGraphYIdx = HeightMapTools.coordinateToIndex(node.getY(), heightMapData.getGridCenter().getY(), BodyPathLatticePoint.gridSizeXY, nodeCenterIndex);

      return HeightMapTools.indicesToKey(nodeGraphXIdx, nodeGraphYIdx, nodeCenterIndex);
   }

   private void populateHeightMapImage()
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

   private void populatePathPlanningParametersBuffer()
   {
      FloatPointer floatPointer = pathPlanningParametersBuffer.getBytedecoFloatBufferPointer();
      floatPointer.put(0, (float) BodyPathLatticePoint.gridSizeXY);
      floatPointer.put(1, (float) nodeCenterIndex);
      floatPointer.put(2, (float) startNode.getXIndex());
      floatPointer.put(3, (float) startNode.getYIndex());
      floatPointer.put(4, (float) goalNode.getX());
      floatPointer.put(5, (float) goalNode.getY());
      floatPointer.put(6, (float) groundClearance);
      floatPointer.put(7, (float) maxIncline);
      floatPointer.put(8, (float) nominalIncline.getValue());
      floatPointer.put(9, (float) plannerParameters.getInclineCostWeight());
      floatPointer.put(10, (float) plannerParameters.getInclineCostDeadband());
      floatPointer.put(11, (float) plannerParameters.getRollCostWeight());
      floatPointer.put(12, (float) BodyPathRANSACTraversibilityCalculator.alphaStance);
      floatPointer.put(13, (float) BodyPathRANSACTraversibilityCalculator.alphaStep);
      floatPointer.put(14, (float) BodyPathRANSACTraversibilityCalculator.minPercent);
      floatPointer.put(15, (float) BodyPathRANSACTraversibilityCalculator.halfStanceWidth);
      floatPointer.put(16, (float) BodyPathRANSACTraversibilityCalculator.heightWindow);
      floatPointer.put(17, (float) BodyPathRANSACTraversibilityCalculator.minNormalToPenalize);
      floatPointer.put(18, (float) BodyPathRANSACTraversibilityCalculator.maxNormalToPenalize);
      floatPointer.put(19, (float) BodyPathRANSACTraversibilityCalculator.inclineWeight);
//      floatPointer.put(5, (float) heightMapData.getEstimatedGroundHeight());

      pathPlanningParametersBuffer.writeOpenCLBufferObject(openCLManager);
   }

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

      openCLManager.finish();
   }

   private void populateRansacNormalParametersBuffer()
   {
      FloatPointer floatPointer = ransacNormalParametersBuffer.getBytedecoFloatBufferPointer();
      floatPointer.put(0, (float) HeightMapRANSACNormalCalculator.iterations);
      floatPointer.put(1, (float) HeightMapRANSACNormalCalculator.distanceEpsilon);
      floatPointer.put(2, (float) HeightMapRANSACNormalCalculator.minNormalZ);
      floatPointer.put(3, (float) HeightMapRANSACNormalCalculator.acceptableConcensus);

      ransacNormalParametersBuffer.writeOpenCLBufferObject(openCLManager);
   }

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

      // get the data from the GPU
      ransacNormalXYZBuffer.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();
   }

   private void computeSnapHeights()
   {
      openCLManager.setKernelArgument(snapVerticesKernel, 0, heightMapParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(snapVerticesKernel, 1, pathPlanningParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(snapVerticesKernel, 2, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(snapVerticesKernel, 3, snapOffsetsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(snapVerticesKernel, 4, snappedNodeHeightBuffer.getOpenCLBufferObject());

      int totalCells = nodesPerSide * nodesPerSide;
      openCLManager.execute1D(snapVerticesKernel, totalCells);

      snappedNodeHeightBuffer.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();
   }

   private void computeHeuristicCost()
   {
      openCLManager.setKernelArgument(computeHeuristicCostKernel, 0, heightMapParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeHeuristicCostKernel, 1, pathPlanningParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeHeuristicCostKernel, 2, heuristicCostMapBuffer.getOpenCLBufferObject());

      int totalCells = nodesPerSide * nodesPerSide;
      openCLManager.execute1D(computeHeuristicCostKernel, totalCells);

      heuristicCostMapBuffer.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();
   }

   private void computeEdgeData()
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

      int totalCells = nodesPerSide * nodesPerSide;
      openCLManager.execute1D(computeEdgeDataKernel, totalCells);

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

      openCLManager.finish();
   }

   private UnitVector3DReadOnly getSurfaceNormal(int key)
   {
      float x = leastSquaresNormalXYZBuffer.getBackingDirectFloatBuffer().get(3 * key);
      float y = leastSquaresNormalXYZBuffer.getBackingDirectFloatBuffer().get(3 * key + 1);
      float z = leastSquaresNormalXYZBuffer.getBackingDirectFloatBuffer().get(3 * key + 2);
      return new UnitVector3D(x, y, z);
   }

   private UnitVector3DReadOnly getRansacSurfaceNormal(int xIndex, int yIndex)
   {
      return getRansacSurfaceNormal(HeightMapTools.indicesToKey(xIndex, yIndex, heightMapData.getCenterIndex()));
   }

   private UnitVector3DReadOnly getRansacSurfaceNormal(int key)
   {
      float x = leastSquaresNormalXYZBuffer.getBackingDirectFloatBuffer().get(3 * key);
      float y = leastSquaresNormalXYZBuffer.getBackingDirectFloatBuffer().get(3 * key + 1);
      float z = leastSquaresNormalXYZBuffer.getBackingDirectFloatBuffer().get(3 * key + 2);
      return new UnitVector3D(x, y, z);
   }

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
      this.snapHeight.set(snap(childNodeKey));

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

   private void computeEdgeCost(BodyPathLatticePoint node, BodyPathLatticePoint neighbor, int edgeKey)
   {
      edgeCost.set(edgeCostMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));
      inclineCost.set(inclineCostMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));

      if (plannerParameters.getComputeSurfaceNormalCost())
      {
         roll.set(rollMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));
         rollCost.set(rollCostMapBuffer.getBackingDirectFloatBuffer().get(edgeKey));

         Point2D bodyPose = new Point2D(neighbor.getX() + node.getX(), neighbor.getY() + node.getY());
         bodyPose.scale(0.5);
         leastSqNormal.set(getSurfaceNormal(HeightMapTools.coordinateToKey(bodyPose.getX(),
                                                                           bodyPose.getY(),
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

   private void reportStatus(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      if (debug)
      {
         LogTools.info("Reporting status");
      }

      boolean performSmoothing = plannerParameters.getPerformSmoothing() && result == BodyPathPlanningResult.FOUND_SOLUTION;

      outputToPack.setBodyPathPlanningResult(result);
      outputToPack.getBodyPath().clear();
      outputToPack.getBodyPathUnsmoothed().clear();

      BodyPathLatticePoint terminalNode = reachedGoal ? goalNode : leastCostNode;
      List<BodyPathLatticePoint> path = graph.getPathFromStart(terminalNode);
      List<Point3D> bodyPath = new ArrayList<>();

      for (int i = 0; i < path.size(); i++)
      {
         Point3D waypoint = new Point3D(path.get(i).getX(), path.get(i).getY(), snap(getNodeGraphKey(path.get(i))));
         bodyPath.add(waypoint);
         outputToPack.getBodyPathUnsmoothed().add(waypoint);

         if (!performSmoothing)
            outputToPack.getBodyPath().add(new Pose3D(waypoint, new Quaternion()));
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
               double yaw = AngleTools.interpolateAngle(request.getStartFootPoses().get(RobotSide.LEFT).getYaw(), request.getStartFootPoses().get(RobotSide.RIGHT).getYaw(), 0.5);
               waypoint.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
            }
            else if (i == bodyPath.size() - 1)
            {
               double yaw = AngleTools.interpolateAngle(request.getGoalFootPoses().get(RobotSide.LEFT).getYaw(), request.getGoalFootPoses().get(RobotSide.RIGHT).getYaw(), 0.5);
               waypoint.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
            }

            outputToPack.getBodyPath().add(waypoint);
         }
      }



      outputToPack.getPlannerTimings().setTimePlanningBodyPathSeconds(stopwatch.totalElapsed() - planningStartTime);
      outputToPack.getPlannerTimings().setTotalElapsedSeconds(stopwatch.totalElapsed());

      if (reachedGoal)
      {
         outputToPack.setFootstepPlanningResult(FootstepPlanningResult.PLANNING);
      }

      markSolutionEdges(terminalNode);
      statusCallbacks.forEach(callback -> callback.accept(outputToPack));
   }

   public void clearLoggedData()
   {
      edgeDataMap.clear();
      iterationData.clear();
   }

   private boolean publishStatus(FootstepPlannerRequest request)
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

   public BodyPathLatticePoint getNextNode()
   {
      while (!stack.isEmpty())
      {
//         BodyPathLatticePoint nextNode = stack.pollFirst();
         BodyPathLatticePoint nextNode = stack.poll();
         if (!expandedNodeSet.contains(nextNode))
         {
            return nextNode;
         }
      }

      return null;
   }

   static void packNeighborOffsets(TIntArrayList xOffsets, TIntArrayList yOffsets)
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

   private double snap(int nodeKey)
   {
      return snappedNodeHeightBuffer.getBackingDirectFloatBuffer().get(nodeKey);
   }

   private double heuristics(BodyPathLatticePoint node)
   {
      return heuristicCostMapBuffer.getBytedecoFloatBufferPointer().get(getNodeGraphKey(node));
   }

   public void halt()
   {
      haltRequested.set(true);
   }

   public List<AStarBodyPathIterationData> getIterationData()
   {
      return iterationData;
   }

   public HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> getEdgeDataMap()
   {
      return edgeDataMap;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }
}
