package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.apache.commons.lang3.tuple.Pair;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencl.*;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import static us.ihmc.footstepPlanning.bodyPath.AStarBodyPathSmoother.*;

public class GPUAStarBodyPathSmoother
{
   static final int yawDiscretizations = 8;

   private static final int iterations = 180;

   private static final int turnPointIteration = 12;
   private final TIntArrayList turnPointIndices = new TIntArrayList();
   private static final int minTurnPointProximity = 7;
   private static final double turnPointYawThreshold = Math.toRadians(30.0);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger iteration = new YoInteger("iterations", registry);
   private final YoDouble maxCollision = new YoDouble("maxCollision", registry);
   private final TickAndUpdatable tickAndUpdatable;
   private final boolean visualize;

   private int pathSize;
   private final GPUAStarBodyPathSmootherWaypoint[] waypoints = new GPUAStarBodyPathSmootherWaypoint[maxPoints];

   /* Cost gradient in the direction of higher cost for each coordinate */
   private final YoVector2D[] gradients = new YoVector2D[maxPoints];

   private final OpenCLManager openCLManager;

   private _cl_kernel computeCollisionGradientMapKernel;
   private _cl_kernel computeCurrentTraversibilityMapKernel;
   private _cl_kernel computeTraversibilityForGradientMapKernel;
   private _cl_kernel computeGroundPlaneGradientKernel;
   private _cl_kernel computeWaypointSmoothessGradientKernel;
   private _cl_kernel getWaypointCurrentTraversibilityKernel;
   private _cl_kernel computeWaypointMapGradientsKernel;

   private final OpenCLFloatBuffer smoothingParametersBuffer = new OpenCLFloatBuffer(15);
   private final OpenCLFloatBuffer traversibilityOffsetsForGradientBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLFloatBuffer traversibilityOffsetsForNominalBuffer = new OpenCLFloatBuffer(1);
   private final OpenCLIntBuffer offsetsForGroundPlaneGradientBuffer = new OpenCLIntBuffer(1);

   private final OpenCLFloatMemory collisionGradientsMapBuffer = new OpenCLFloatMemory(1);
   private final OpenCLIntMemory maxCollisionsMapBuffer = new OpenCLIntMemory(1);
   private final OpenCLFloatMemory leftTraversibilitiesMapBuffer = new OpenCLFloatMemory(1);
   private final OpenCLFloatMemory rightTraversibilitiesMapBuffer = new OpenCLFloatMemory(1);
   private final OpenCLFloatMemory leftTraversibilitiesForGradientMapBuffer = new OpenCLFloatMemory(1);
   private final OpenCLFloatMemory rightTraversibilitiesForGradientMapBuffer = new OpenCLFloatMemory(1);
   private final OpenCLIntMemory leftGroundPlaneCellsMapBuffer = new OpenCLIntMemory(1);
   private final OpenCLIntMemory rightGroundPlaneCellsMapBuffer = new OpenCLIntMemory(1);
   private final OpenCLFloatMemory groundPlaneGradientMapBuffer = new OpenCLFloatMemory(1);

   private OpenCLFloatBuffer waypointXYZYawBuffer = new OpenCLFloatBuffer(1);
   private OpenCLIntBuffer waypointTurnPointsBuffer = new OpenCLIntBuffer(1);
   private OpenCLFloatBuffer waypointSmoothnessGradients = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer waypointTravesibilityValues = new OpenCLFloatBuffer(1);
   private OpenCLIntBuffer waypointMaxCollisions = new OpenCLIntBuffer(1);
   private OpenCLFloatBuffer waypointCollisionGradients = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer waypointTraversibilitySamples = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer waypointTraversibilityGradients = new OpenCLFloatBuffer(1);
   private OpenCLIntBuffer waypointGroundPlaneCells = new OpenCLIntBuffer(1);
   private OpenCLFloatBuffer waypointGroundPlaneGradients = new OpenCLFloatBuffer(1);

   private int cellsPerSide = -1;

   private final AStarBodyPathPlannerParametersReadOnly plannerParameters;

   public GPUAStarBodyPathSmoother(AStarBodyPathPlannerParametersReadOnly plannerParameters,
                                   TickAndUpdatable tickAndUpdatable,
                                   OpenCLManager openCLManager,
                                   YoGraphicsListRegistry graphicsListRegistry,
                                   YoRegistry parentRegistry)
   {
      this.plannerParameters = plannerParameters;
      this.openCLManager = openCLManager;

      for (int i = 0; i < maxPoints; i++)
      {
         gradients[i] = new YoVector2D("gradient" + i, registry);
      }

      for (int i = 0; i < maxPoints; i++)
      {
         waypoints[i] = new GPUAStarBodyPathSmootherWaypoint(plannerParameters, i, graphicsListRegistry, (parentRegistry == null) ? null : registry);
      }

      if (parentRegistry == null)
      {
         visualize = false;
         this.tickAndUpdatable = null;
      }
      else
      {
         graphicsListRegistry.getYoGraphicsLists().stream().filter(list -> list.getLabel().equals("Collisions")).forEach(list -> list.setVisible(false));
         graphicsListRegistry.getYoGraphicsLists().stream().filter(list -> list.getLabel().equals("Normals")).forEach(list -> list.setVisible(false));
         graphicsListRegistry.getYoGraphicsLists().stream().filter(list -> list.getLabel().equals("Step Poses")).forEach(list -> list.setVisible(false));
         this.tickAndUpdatable = tickAndUpdatable;
         parentRegistry.addChild(registry);
         visualize = true;
      }
   }

   static int yawToIndex(double yaw)
   {
      return yawToIndex(yaw, yawDiscretizations);
   }

   static int yawToIndex(double yaw, int yawDiscretizations)
   {
      yaw = AngleTools.trimAngleMinusPiToPi(yaw);
      return ((int) (yaw / yawDiscretizations)) + (yawDiscretizations / 2);
   }

   public void createOpenCLStuff(_cl_program pathPlannerProgram, int numberOfCells, int numberOfNodes)
   {
      cellsPerSide = numberOfCells;

      computeCollisionGradientMapKernel = openCLManager.createKernel(pathPlannerProgram, "computeCollisionGradientMap");
      computeCurrentTraversibilityMapKernel = openCLManager.createKernel(pathPlannerProgram, "computeCurrentTraversibilityMap");
      computeTraversibilityForGradientMapKernel = openCLManager.createKernel(pathPlannerProgram, "computeTraversibilityForGradientMap");
      computeGroundPlaneGradientKernel = openCLManager.createKernel(pathPlannerProgram, "computeGroundPlaneGradientMap");
      computeWaypointSmoothessGradientKernel = openCLManager.createKernel(pathPlannerProgram, "computeWaypointSmoothnessGradient");
      getWaypointCurrentTraversibilityKernel = openCLManager.createKernel(pathPlannerProgram, "getWaypointCurrentTraversibility");
      computeWaypointMapGradientsKernel = openCLManager.createKernel(pathPlannerProgram, "computeWaypointMapGradients");
   }

   public void firstTickSetup()
   {
      smoothingParametersBuffer.createOpenCLBufferObject(openCLManager);
      traversibilityOffsetsForGradientBuffer.createOpenCLBufferObject(openCLManager);
      traversibilityOffsetsForNominalBuffer.createOpenCLBufferObject(openCLManager);
      offsetsForGroundPlaneGradientBuffer.createOpenCLBufferObject(openCLManager);

      collisionGradientsMapBuffer.createOpenCLBufferObject(openCLManager);
      maxCollisionsMapBuffer.createOpenCLBufferObject(openCLManager);
      leftTraversibilitiesMapBuffer.createOpenCLBufferObject(openCLManager);
      rightTraversibilitiesMapBuffer.createOpenCLBufferObject(openCLManager);
      leftTraversibilitiesForGradientMapBuffer.createOpenCLBufferObject(openCLManager);
      rightTraversibilitiesForGradientMapBuffer.createOpenCLBufferObject(openCLManager);
      leftGroundPlaneCellsMapBuffer.createOpenCLBufferObject(openCLManager);
      rightGroundPlaneCellsMapBuffer.createOpenCLBufferObject(openCLManager);
      groundPlaneGradientMapBuffer.createOpenCLBufferObject(openCLManager);

      waypointXYZYawBuffer.createOpenCLBufferObject(openCLManager);
      waypointTurnPointsBuffer.createOpenCLBufferObject(openCLManager);
      waypointSmoothnessGradients.createOpenCLBufferObject(openCLManager);
      waypointTravesibilityValues.createOpenCLBufferObject(openCLManager);

      waypointMaxCollisions.createOpenCLBufferObject(openCLManager);
      waypointCollisionGradients.createOpenCLBufferObject(openCLManager);
      waypointTraversibilitySamples.createOpenCLBufferObject(openCLManager);
      waypointTraversibilityGradients.createOpenCLBufferObject(openCLManager);
      waypointGroundPlaneCells.createOpenCLBufferObject(openCLManager);
      waypointGroundPlaneGradients.createOpenCLBufferObject(openCLManager);

      populateGroundPlaneOffsetsGradientBuffer();
   }

   public void destroyOpenCLStuff()
   {
      computeCollisionGradientMapKernel.close();
      computeCurrentTraversibilityMapKernel.close();
      computeTraversibilityForGradientMapKernel.close();
      computeGroundPlaneGradientKernel.close();
      computeWaypointSmoothessGradientKernel.close();
      getWaypointCurrentTraversibilityKernel.close();
      computeWaypointMapGradientsKernel.close();

      smoothingParametersBuffer.destroy(openCLManager);
      traversibilityOffsetsForGradientBuffer.destroy(openCLManager);
      traversibilityOffsetsForNominalBuffer.destroy(openCLManager);
      offsetsForGroundPlaneGradientBuffer.destroy(openCLManager);

      collisionGradientsMapBuffer.destroy(openCLManager);
      maxCollisionsMapBuffer.destroy(openCLManager);
      leftTraversibilitiesMapBuffer.destroy(openCLManager);
      rightTraversibilitiesMapBuffer.destroy(openCLManager);
      leftTraversibilitiesForGradientMapBuffer.destroy(openCLManager);
      rightTraversibilitiesForGradientMapBuffer.destroy(openCLManager);
      leftGroundPlaneCellsMapBuffer.destroy(openCLManager);
      rightGroundPlaneCellsMapBuffer.destroy(openCLManager);
      groundPlaneGradientMapBuffer.destroy(openCLManager);

      waypointXYZYawBuffer.destroy(openCLManager);
      waypointTurnPointsBuffer.destroy(openCLManager);
      waypointSmoothnessGradients.destroy(openCLManager);
      waypointTravesibilityValues.destroy(openCLManager);

      waypointMaxCollisions.destroy(openCLManager);
      waypointCollisionGradients.destroy(openCLManager);
      waypointTraversibilitySamples.destroy(openCLManager);
      waypointTraversibilityGradients.destroy(openCLManager);
      waypointGroundPlaneCells.destroy(openCLManager);
      waypointGroundPlaneGradients.destroy(openCLManager);
   }

   public void resizeOpenCLObjects(int cellsPerSide)
   {
      int totalCells = cellsPerSide * cellsPerSide;
      int cellsWithYaw = yawDiscretizations * totalCells;
      collisionGradientsMapBuffer.resize(2 * yawDiscretizations * totalCells, openCLManager);
      maxCollisionsMapBuffer.resize(yawDiscretizations * totalCells, openCLManager);
      leftTraversibilitiesMapBuffer.resize(yawDiscretizations * totalCells, openCLManager);
      rightTraversibilitiesMapBuffer.resize(yawDiscretizations * totalCells, openCLManager);
      leftTraversibilitiesForGradientMapBuffer.resize(2 * cellsWithYaw, openCLManager);
      rightTraversibilitiesForGradientMapBuffer.resize(2 * cellsWithYaw, openCLManager);
      leftGroundPlaneCellsMapBuffer.resize(cellsWithYaw, openCLManager);
      rightGroundPlaneCellsMapBuffer.resize(cellsWithYaw, openCLManager);
      groundPlaneGradientMapBuffer.resize(2 * cellsWithYaw, openCLManager);
   }

   private void resizeForWayponts(int waypoints)
   {
      waypointXYZYawBuffer.resize(4 * waypoints, openCLManager);
      waypointTurnPointsBuffer.resize(waypoints, openCLManager);
      waypointSmoothnessGradients.resize(2 * waypoints, openCLManager);
      waypointTravesibilityValues.resize(2 * waypoints, openCLManager);

      waypointMaxCollisions.resize(waypoints, openCLManager);
      waypointCollisionGradients.resize(2 * waypoints, openCLManager);
      waypointTraversibilitySamples.resize(4 * waypoints, openCLManager);
      waypointTraversibilityGradients.resize(4 * waypoints, openCLManager);
      waypointGroundPlaneCells.resize(2 * waypoints, openCLManager);
      waypointGroundPlaneGradients.resize(2 * waypoints, openCLManager);
   }

   public List<Pose3D> doSmoothing(List<Point3D> bodyPath,
                                   HeightMapData heightMapData,
                                   OpenCLFloatBuffer heightMapParametersBuffer,
                                   OpenCLFloatBuffer plannerParametersBuffer,
                                   OpenCLFloatBuffer heightMapBuffer,
                                   OpenCLFloatBuffer snappedNodeHeightBuffer,
                                   OpenCLFloatBuffer ransacNormalsBuffer,
                                   OpenCLFloatBuffer leastSquaresNormalBuffer,
                                   OpenCLFloatBuffer leastSquaresSampledHeightBuffer)
   {
      LogTools.info("Starting waypoint optimization");
      pathSize = bodyPath.size();
      turnPointIndices.clear();

      if (pathSize > maxPoints)
      {
         throw new RuntimeException("Too many body path waypoints to smooth. Path size = " + bodyPath.size() + ", Max points = " + maxPoints);
      }

      if (pathSize == 2)
      {
         Point3D start = bodyPath.get(0);
         Point3D goal = bodyPath.get(1);
         double yaw = Math.atan2(goal.getY() - start.getY(), goal.getX() - start.getX());
         List<Pose3D> waypoints = new ArrayList<>();
         waypoints.add(new Pose3D(start, new Quaternion(yaw, 0.0, 0.0)));
         waypoints.add(new Pose3D(goal, new Quaternion(yaw, 0.0, 0.0)));
         return waypoints;
      }

      populateSmoothingParametersBuffer();

      populateTraversibilityOffsetsForGradientBuffer(heightMapData);
      populateTraversibilityOffsetsForNominalBuffer(heightMapData);

      computeTraversibilities(heightMapParametersBuffer, plannerParametersBuffer, heightMapBuffer, snappedNodeHeightBuffer, ransacNormalsBuffer);
      computeCollisionsGradient(heightMapParametersBuffer, plannerParametersBuffer, heightMapBuffer, snappedNodeHeightBuffer);
      computeTraversibilityForGradient(heightMapParametersBuffer, plannerParametersBuffer, heightMapBuffer, snappedNodeHeightBuffer, ransacNormalsBuffer);
      computeGroundPlaneGradient(heightMapParametersBuffer, plannerParametersBuffer, heightMapBuffer);

      for (int i = 0; i < pathSize; i++)
      {
         waypoints[i].setNeighbors(waypoints);
      }

      for (int i = 0; i < maxPoints; i++)
      {
         waypoints[i].initialize(bodyPath);
      }

      for (int i = 1; i < bodyPath.size() - 1; i++)
      {
         waypoints[i].update(true, heightMapData, snappedNodeHeightBuffer);
      }

      if (visualize)
      {
         iteration.set(-1);
         tickAndUpdatable.tickAndUpdate();
      }

      resizeForWayponts(pathSize);

      for (iteration.set(0); iteration.getValue() < iterations; iteration.increment())
      {
         maxCollision.set(0.0);

         populateWaypointStateBuffers();
         computeSmoothnessGradientsForAllWaypoints();
         getCurrentTraversibilityValues(heightMapParametersBuffer);
         if (heightMapData != null)
            computeWaypointMapGradients(heightMapParametersBuffer.getOpenCLBufferObject());

         for (int waypointIndex = 1; waypointIndex < pathSize - 1; waypointIndex++)
         {
            computeSmoothenessGradientFromGPU(waypointIndex, gradients[waypointIndex]);

            Tuple3DReadOnly displacementGradient = waypoints[waypointIndex].computeDisplacementGradient();
            gradients[waypointIndex].add(displacementGradient.getX(), displacementGradient.getY());
         }

         if (heightMapData != null)
         {
            for (int waypointIndex = 1; waypointIndex < pathSize - 1; waypointIndex++)
            {
               waypoints[waypointIndex].computeCurrentTraversibilityFromGPU(waypointTravesibilityValues);
            }

            for (int waypointIndex = 2; waypointIndex < pathSize - 2; waypointIndex++)
            {
               /* Collision gradient */
               gradients[waypointIndex].add(waypoints[waypointIndex].computeCollisionGradientFromGPU(waypointIndex, waypointMaxCollisions, waypointCollisionGradients));
               maxCollision.set(Math.max(waypoints[waypointIndex].getMaxCollision(), maxCollision.getValue()));

               /* Traversibility gradient */
               Tuple3DReadOnly traversibilityGradient = waypoints[waypointIndex].computeTraversibilityGradientFromGPU(waypointTraversibilitySamples,
                                                                                                                      waypointTraversibilityGradients);
               gradients[waypointIndex].sub(traversibilityGradient.getX(), traversibilityGradient.getY());

               if (waypoints[waypointIndex].isTurnPoint())
               {
                  continue;
               }

               /* Ground plane gradient */
               Tuple3DReadOnly groundPlaneGradient = waypoints[waypointIndex].computeGroundPlaneGradientFromGPU(waypointGroundPlaneCells,
                                                                                                                waypointGroundPlaneGradients);
               gradients[waypointIndex].sub(groundPlaneGradient.getX(), groundPlaneGradient.getY());

               /* Roll-z gradient */
               Vector2DReadOnly rollGradient = waypoints[waypointIndex].computeRollInclineGradient(leastSquaresNormalBuffer, leastSquaresSampledHeightBuffer);
               gradients[waypointIndex - 1].sub(rollGradient);
               gradients[waypointIndex + 1].add(rollGradient);

               if (visualize)
               {
                  if (waypointIndex - 1 != 0)
                  {
                     waypoints[waypointIndex - 1].updateRollGraphics(-rollGradient.getX(), -rollGradient.getY());
                  }
                  if (waypointIndex + 1 != pathSize - 1)
                  {
                     waypoints[waypointIndex + 1].updateRollGraphics(rollGradient.getX(), rollGradient.getY());
                  }
               }
            }

            double gradientMagnitudeSq = 0.0;
            for (int i = 0; i < gradients.length; i++)
            {
               gradientMagnitudeSq += EuclidCoreTools.normSquared(gradients[i].getX(), gradients[i].getY());
            }

            if (gradientMagnitudeSq < MathTools.square(plannerParameters.getSmootherGradientThresholdToTerminate()) && iteration.getValue() > minIterations)
            {
               break;
            }
         }

         for (int j = 1; j < pathSize - 1; j++)
         {
            waypoints[j].getPosition().subX(plannerParameters.getSmootherHillClimbGain() * gradients[j].getX());
            waypoints[j].getPosition().subY(plannerParameters.getSmootherHillClimbGain() * gradients[j].getY());
         }

         for (int j = 1; j < pathSize - 1; j++)
         {
            waypoints[j].update(false, heightMapData, snappedNodeHeightBuffer);
         }

         if (iteration.getValue() == turnPointIteration)
         {
            computeTurnPoints();
         }

         if (visualize)
         {
            tickAndUpdatable.tickAndUpdate();
         }
      }

      List<Pose3D> smoothedPath = new ArrayList<>();
      for (int i = 0; i < pathSize; i++)
      {
         smoothedPath.add(new Pose3D(waypoints[i].getPose()));
      }

      return smoothedPath;
   }

   private void populateSmoothingParametersBuffer()
   {
      FloatPointer floatPointer = smoothingParametersBuffer.getBytedecoFloatBufferPointer();
      floatPointer.put(0, (float) pathSize);
      floatPointer.put(1, (float) plannerParameters.getSmootherEqualSpacingWeight());
      floatPointer.put(2, (float) Math.toRadians(plannerParameters.getSmootherMinCurvatureToPenalize()));
      floatPointer.put(3, (float) gradientEpsilon);
      floatPointer.put(4, (float) plannerParameters.getSmootherSmoothnessWeight());
      floatPointer.put(5, (float) plannerParameters.getCollisionBoxSizeX());
      floatPointer.put(6, (float) plannerParameters.getCollisionBoxSizeY());
      floatPointer.put(7, (float) plannerParameters.getCollisionBoxGroundClearance());
      floatPointer.put(8, (float) plannerParameters.getSmootherCollisionWeight());
      floatPointer.put(9, (float) yawDiscretizations);
      floatPointer.put(10, (float) plannerParameters.getSmootherGroundPlaneWeight());
      floatPointer.put(11, (float) AStarBodyPathSmootherWaypoint.traversibilitySmoothingHeightDeadband);
      floatPointer.put(12, (float) plannerParameters.getSmootherTurnPointSmoothnessDiscount());
      floatPointer.put(13, (float) plannerParameters.getSmootherMinimumTraversibilityToSearchFor());
      floatPointer.put(14, (float) plannerParameters.getSmootherTraversibilityThresholdForNoDiscount());

      smoothingParametersBuffer.writeOpenCLBufferObject(openCLManager);
   }

   private void computeCollisionsGradient(OpenCLFloatBuffer heightMapParamsBuffer,
                                          OpenCLFloatBuffer plannerParamsBuffer,
                                          OpenCLFloatBuffer heightMapBuffer,
                                          OpenCLFloatBuffer snappedNodeHeightBuffer)
   {
      openCLManager.setKernelArgument(computeCollisionGradientMapKernel, 0, heightMapParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientMapKernel, 1, plannerParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientMapKernel, 2, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientMapKernel, 3, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientMapKernel, 4, snappedNodeHeightBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientMapKernel, 5, collisionGradientsMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientMapKernel, 6, maxCollisionsMapBuffer.getOpenCLBufferObject());

      openCLManager.execute3D(computeCollisionGradientMapKernel, cellsPerSide, cellsPerSide, yawDiscretizations);
   }

   private void populateTraversibilityOffsetsForNominalBuffer(HeightMapData heightMapData)
   {
      int minXTraversibilityGradWindow = (int) Math.round(-0.5 * traversibilitySampleWindowX / heightMapData.getGridResolutionXY());
      int maxXTraversibilityGradWindow = (int) Math.round(0.5 * traversibilitySampleWindowX / heightMapData.getGridResolutionXY());
      int minMaxYTraversibilityNomWindow = (int) Math.round(0.5 * yOffsetTraversibilityNominalWindow / heightMapData.getGridResolutionXY());

      TDoubleArrayList xTraversibilityNominalOffsets = new TDoubleArrayList();
      TDoubleArrayList yTraversibilityNominalOffsets = new TDoubleArrayList();

      for (int xi = minXTraversibilityGradWindow; xi <= maxXTraversibilityGradWindow; xi++)
      {
         for (int yi = -minMaxYTraversibilityNomWindow; yi <= minMaxYTraversibilityNomWindow; yi++)
         {
            double dx = xi * heightMapData.getGridResolutionXY();
            double dy = yi * heightMapData.getGridResolutionXY();
            xTraversibilityNominalOffsets.add(dx);
            yTraversibilityNominalOffsets.add(dy);
         }
      }

      int size = xTraversibilityNominalOffsets.size();
      traversibilityOffsetsForNominalBuffer.resize(1 + 2 * size, openCLManager);
      FloatPointer pointer = traversibilityOffsetsForNominalBuffer.getBytedecoFloatBufferPointer();
      pointer.put(0, (float) size);
      for (int i = 0; i < size; i++)
      {
         pointer.put(1 + i, (float) xTraversibilityNominalOffsets.get(i));
         pointer.put(1 + size + i, (float) xTraversibilityNominalOffsets.get(i));
      }

      traversibilityOffsetsForNominalBuffer.writeOpenCLBufferObject(openCLManager);
   }

   private void computeTraversibilities(OpenCLFloatBuffer heightMapParamsBuffer,
                                        OpenCLFloatBuffer plannerParamsBuffer,
                                        OpenCLFloatBuffer heightMapBuffer,
                                        OpenCLFloatBuffer snappedNodeHeightBuffer,
                                        OpenCLFloatBuffer normalXYZBuffer)
   {
      openCLManager.setKernelArgument(computeCurrentTraversibilityMapKernel, 0, heightMapParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCurrentTraversibilityMapKernel, 1, plannerParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCurrentTraversibilityMapKernel, 2, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCurrentTraversibilityMapKernel, 3, traversibilityOffsetsForNominalBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCurrentTraversibilityMapKernel, 4, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCurrentTraversibilityMapKernel, 5, snappedNodeHeightBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCurrentTraversibilityMapKernel, 6, normalXYZBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCurrentTraversibilityMapKernel, 7, leftTraversibilitiesMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCurrentTraversibilityMapKernel, 8, rightTraversibilitiesMapBuffer.getOpenCLBufferObject());

      openCLManager.execute3D(computeCurrentTraversibilityMapKernel, cellsPerSide, cellsPerSide, yawDiscretizations);
   }

   private void populateTraversibilityOffsetsForGradientBuffer(HeightMapData heightMapData)
   {
      int minYTraversibilityGradWindow = (int) Math.round((yOffsetTraversibilityGradientWindow - 0.5 * traversibilitySampleWindowY) / heightMapData.getGridResolutionXY());
      int maxYTraversibilityGradWindow = (int) Math.round((yOffsetTraversibilityGradientWindow + 0.5 * traversibilitySampleWindowY) / heightMapData.getGridResolutionXY());
      int minXTraversibilityGradWindow = (int) Math.round(-0.5 * traversibilitySampleWindowX / heightMapData.getGridResolutionXY());
      int maxXTraversibilityGradWindow = (int) Math.round(0.5 * traversibilitySampleWindowX / heightMapData.getGridResolutionXY());

      TDoubleArrayList xTraversibilityGradientOffsets = new TDoubleArrayList();
      TDoubleArrayList yTraversibilityGradientOffsets = new TDoubleArrayList();

      for (int xi = minXTraversibilityGradWindow; xi <= maxXTraversibilityGradWindow; xi++)
      {
         for (int yi = minYTraversibilityGradWindow; yi <= maxYTraversibilityGradWindow; yi++)
         {
            double dx = xi * heightMapData.getGridResolutionXY();
            double dy = yi * heightMapData.getGridResolutionXY();
            xTraversibilityGradientOffsets.add(dx);
            yTraversibilityGradientOffsets.add(dy);
         }

      }

      int size = xTraversibilityGradientOffsets.size();
      traversibilityOffsetsForGradientBuffer.resize(1 + 2 * size, openCLManager);
      FloatPointer pointer = traversibilityOffsetsForGradientBuffer.getBytedecoFloatBufferPointer();
      pointer.put(0, (float) size);
      for (int i = 0; i < size; i++)
      {
         pointer.put(1 + i, (float) xTraversibilityGradientOffsets.get(i));
         pointer.put(1 + size + i, (float) yTraversibilityGradientOffsets.get(i));
      }

      traversibilityOffsetsForGradientBuffer.writeOpenCLBufferObject(openCLManager);
   }

   private void computeTraversibilityForGradient(OpenCLFloatBuffer heightMapParamsBuffer,
                                                 OpenCLFloatBuffer plannerParamsBuffer,
                                                 OpenCLFloatBuffer heightMapBuffer,
                                                 OpenCLFloatBuffer snappedNodeHeightBuffer,
                                                 OpenCLFloatBuffer normalXYZBuffer)
   {
      openCLManager.setKernelArgument(computeTraversibilityForGradientMapKernel, 0, heightMapParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientMapKernel, 1, plannerParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientMapKernel, 2, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientMapKernel, 3, traversibilityOffsetsForGradientBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientMapKernel, 4, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientMapKernel, 5, snappedNodeHeightBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientMapKernel, 6, normalXYZBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientMapKernel, 7, leftTraversibilitiesForGradientMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientMapKernel, 8, rightTraversibilitiesForGradientMapBuffer.getOpenCLBufferObject());

      openCLManager.execute3D(computeTraversibilityForGradientMapKernel, cellsPerSide, cellsPerSide, yawDiscretizations);
   }

   private void populateGroundPlaneOffsetsGradientBuffer()
   {
      int minMaxXOffsetGroundPlane = 1;
      int minYOffsetGroundPlane = 0;
      int maxYOffsetGroundPlane = 4;
      TIntArrayList groundPlaneXOffsets = new TIntArrayList();
      TIntArrayList groundPlaneYOffsets = new TIntArrayList();
      for (int xi = -minMaxXOffsetGroundPlane; xi <= minMaxXOffsetGroundPlane; xi++)
      {
         for (int yi = minYOffsetGroundPlane; yi <= maxYOffsetGroundPlane; yi++)
         {
            groundPlaneXOffsets.add(xi);
            groundPlaneYOffsets.add(yi);
         }
      }

      int size = groundPlaneYOffsets.size();
      offsetsForGroundPlaneGradientBuffer.resize(1 + 2 * size, openCLManager);
      IntPointer pointer = offsetsForGroundPlaneGradientBuffer.getBytedecoIntBufferPointer();
      pointer.put(0, size);
      for (int i = 0; i < size; i++)
      {
         pointer.put(1 + i, groundPlaneXOffsets.get(i));
         pointer.put(1 + size + i, groundPlaneYOffsets.get(i));
      }

      offsetsForGroundPlaneGradientBuffer.writeOpenCLBufferObject(openCLManager);
   }

   private void computeGroundPlaneGradient(OpenCLFloatBuffer heightMapParamsBuffer, OpenCLFloatBuffer plannerParamsBuffer, OpenCLFloatBuffer heightMapBuffer)
   {
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 0, heightMapParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 1, plannerParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 2, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 3, offsetsForGroundPlaneGradientBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 4, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 5, leftGroundPlaneCellsMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 6, rightGroundPlaneCellsMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 7, groundPlaneGradientMapBuffer.getOpenCLBufferObject());

      openCLManager.execute3D(computeGroundPlaneGradientKernel, cellsPerSide, cellsPerSide, yawDiscretizations);
   }

   private void populateWaypointStateBuffers()
   {
      for (int i = 0; i < pathSize; i++)
      {
         waypointXYZYawBuffer.getBytedecoFloatBufferPointer().put(4 * i, (float) waypoints[i].getPosition().getX());
         waypointXYZYawBuffer.getBytedecoFloatBufferPointer().put(4 * i + 1, (float) waypoints[i].getPosition().getY());
         waypointXYZYawBuffer.getBytedecoFloatBufferPointer().put(4 * i + 2, (float) waypoints[i].getPosition().getZ());
         waypointXYZYawBuffer.getBytedecoFloatBufferPointer().put(4 * i + 3, (float) waypoints[i].getHeading());
         waypointTurnPointsBuffer.getBytedecoIntBufferPointer().put(i, waypoints[i].isTurnPoint() ? 1 : 0);
      }
      waypointXYZYawBuffer.writeOpenCLBufferObject(openCLManager);
      waypointTurnPointsBuffer.writeOpenCLBufferObject(openCLManager);
   }


   private void computeSmoothnessGradientsForAllWaypoints()
   {
      openCLManager.setKernelArgument(computeWaypointSmoothessGradientKernel, 0, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointSmoothessGradientKernel, 1, waypointXYZYawBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointSmoothessGradientKernel, 2, waypointTurnPointsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointSmoothessGradientKernel, 3, waypointSmoothnessGradients.getOpenCLBufferObject());

      openCLManager.execute1D(computeWaypointSmoothessGradientKernel, pathSize);

      waypointSmoothnessGradients.readOpenCLBufferObject(openCLManager);
   }

   private void getCurrentTraversibilityValues(OpenCLFloatBuffer heightMapParamsBuffer)
   {
      openCLManager.setKernelArgument(getWaypointCurrentTraversibilityKernel, 0, heightMapParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(getWaypointCurrentTraversibilityKernel, 1, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(getWaypointCurrentTraversibilityKernel, 2, waypointXYZYawBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(getWaypointCurrentTraversibilityKernel, 3, leftTraversibilitiesMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(getWaypointCurrentTraversibilityKernel, 4, rightTraversibilitiesMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(getWaypointCurrentTraversibilityKernel, 5, waypointTravesibilityValues.getOpenCLBufferObject());

      openCLManager.execute1D(getWaypointCurrentTraversibilityKernel, pathSize);

      waypointTravesibilityValues.readOpenCLBufferObject(openCLManager);
   }


   private void computeWaypointMapGradients(_cl_mem heightMapParamsBuffer)
   {
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 0, heightMapParamsBuffer);
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 1, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 2, waypointXYZYawBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 3, waypointTurnPointsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 4, collisionGradientsMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 5, maxCollisionsMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 6, leftTraversibilitiesForGradientMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 7, rightTraversibilitiesForGradientMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 8, waypointTravesibilityValues.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 9, leftGroundPlaneCellsMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 10, rightGroundPlaneCellsMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 11, groundPlaneGradientMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 12, waypointMaxCollisions.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 13, waypointCollisionGradients.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 14, waypointTraversibilitySamples.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 15, waypointTraversibilityGradients.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 16, waypointGroundPlaneCells.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeWaypointMapGradientsKernel, 17, waypointGroundPlaneGradients.getOpenCLBufferObject());

      openCLManager.execute1D(computeWaypointMapGradientsKernel, pathSize);

      waypointMaxCollisions.readOpenCLBufferObject(openCLManager);
      waypointCollisionGradients.readOpenCLBufferObject(openCLManager);
      waypointTraversibilitySamples.readOpenCLBufferObject(openCLManager);
      waypointTraversibilityGradients.readOpenCLBufferObject(openCLManager);
      waypointGroundPlaneCells.readOpenCLBufferObject(openCLManager);
      waypointGroundPlaneGradients.readOpenCLBufferObject(openCLManager);
   }

   private void computeSmoothenessGradientFromGPU(int waypointIndex, Vector2DBasics gradientToSet)
   {
      gradientToSet.setX(waypointSmoothnessGradients.getBackingDirectFloatBuffer().get(2 * waypointIndex));
      gradientToSet.setY(waypointSmoothnessGradients.getBackingDirectFloatBuffer().get(2 * waypointIndex + 1));
   }

   private static double computeDeltaHeadingMagnitude(double x0, double y0, double x1, double y1, double x2, double y2, double deadband)
   {
      double heading0 = Math.atan2(y1 - y0, x1 - x0);
      double heading1 = Math.atan2(y2 - y1, x2 - x1);
      return Math.max(Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(heading1, heading0)) - deadband, 0.0);
   }

   private void computeTurnPoints()
   {
      int minPathSizeForTurnPoints = 5;
      if (pathSize < minPathSizeForTurnPoints)
      {
         return;
      }

      List<Pair<Double, Integer>> headingIndexList = new ArrayList<>();
      for (int i = 2; i < pathSize - 2; i++)
      {
         double x0 = waypoints[i - 1].getPosition().getX();
         double y0 = waypoints[i - 1].getPosition().getY();
         double x1 = waypoints[i].getPosition().getX();
         double y1 = waypoints[i].getPosition().getY();
         double x2 = waypoints[i + 1].getPosition().getX();
         double y2 = waypoints[i + 1].getPosition().getY();

         double deltaHeading = computeDeltaHeadingMagnitude(x0, y0, x1, y1, x2, y2, 0.0);
         headingIndexList.add(Pair.of(deltaHeading, i));
      }

      headingIndexList = headingIndexList.stream()
                                         .filter(hi -> hi.getKey() > turnPointYawThreshold)
                                         .sorted(Comparator.comparingDouble(Pair::getKey))
                                         .collect(Collectors.toList());

      boolean[] canBeTurnPoint = new boolean[pathSize];
      Arrays.fill(canBeTurnPoint, true);

      for (int i = headingIndexList.size() - 1; i >= 0; i--)
      {
         int index = headingIndexList.get(i).getRight();
         if (canBeTurnPoint[index])
         {
            waypoints[index].setTurnPoint();

            for (int j = 1; j < minTurnPointProximity; j++)
            {
               canBeTurnPoint[Math.max(0, index - j)] = false;
               canBeTurnPoint[Math.min(pathSize - 1, index + j)] = false;
            }
         }
      }
   }
}
