package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.apache.commons.lang3.tuple.Pair;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLIntBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.SideDependentList;
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
   static final int yawDiscretizations = 16;

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
   private HeightMapLeastSquaresNormalCalculator leastSquaresNormalCalculator = new HeightMapLeastSquaresNormalCalculator();

   private final GPUAStarBodyPathSmootherWaypoint[] waypoints = new GPUAStarBodyPathSmootherWaypoint[maxPoints];

   /* Cost gradient in the direction of higher cost for each coordinate */
   private final YoVector2D[] gradients = new YoVector2D[maxPoints];

   private final OpenCLManager openCLManager;

   private _cl_program pathPlannerProgram;
   private _cl_kernel computeCollisionGradientKernel;
   private _cl_kernel computeTraversibilityKernel;
   private _cl_kernel computeTraversibilityForGradientKernel;
   private _cl_kernel computeGroundPlaneGradientKernel;

   private OpenCLFloatBuffer smoothingParametersBuffer = new OpenCLFloatBuffer(11);
   private OpenCLFloatBuffer collisionGradientsBuffer = new OpenCLFloatBuffer(1);

   private OpenCLFloatBuffer traversibilityOffsetsForGradientBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer traversibilityOffsetsForNominalBuffer = new OpenCLFloatBuffer(1);
   private OpenCLIntBuffer offsetsForGroundPlaneGradientBuffer = new OpenCLIntBuffer(1);

   private OpenCLIntBuffer maxCollisionsBuffer = new OpenCLIntBuffer(1);
   private OpenCLFloatBuffer leftTraversibilitiesBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer rightTraversibilitiesBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer leftTraversibilitiesForGradientBuffer = new OpenCLFloatBuffer(1);
   private OpenCLFloatBuffer rightTraversibilitiesForGradientBuffer = new OpenCLFloatBuffer(1);
   private OpenCLIntBuffer leftGroundPlaneCellsBuffer = new OpenCLIntBuffer(1);
   private OpenCLIntBuffer rightGroundPlaneCellsBuffer = new OpenCLIntBuffer(1);
   private OpenCLFloatBuffer groundPlaneGradientBuffer = new OpenCLFloatBuffer(1);

   private int cellsPerSide = -1;
   private int nodesPerSide = -1;
   private int nodeCenterIndex = -1;

   public GPUAStarBodyPathSmoother(TickAndUpdatable tickAndUpdatable,
                                   OpenCLManager openCLManager,
                                   YoGraphicsListRegistry graphicsListRegistry,
                                   YoRegistry parentRegistry)
   {
      this.openCLManager = openCLManager;

      for (int i = 0; i < maxPoints; i++)
      {
         gradients[i] = new YoVector2D("gradient" + i, registry);
      }

      for (int i = 0; i < maxPoints; i++)
      {
         waypoints[i] = new GPUAStarBodyPathSmootherWaypoint(i, graphicsListRegistry, (parentRegistry == null) ? null : registry);
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
      this.nodesPerSide = numberOfNodes;
      this.nodeCenterIndex = (nodesPerSide - 1) / 2;

      this.pathPlannerProgram = pathPlannerProgram;
      computeCollisionGradientKernel = openCLManager.createKernel(pathPlannerProgram, "computeWaypointCollisionGradientMap");
      computeTraversibilityKernel = openCLManager.createKernel(pathPlannerProgram, "computeWaypointCurrentTraversibilityMap");
      computeTraversibilityForGradientKernel = openCLManager.createKernel(pathPlannerProgram, "computeWaypointTraversibilityForGradientMap");
      computeGroundPlaneGradientKernel = openCLManager.createKernel(pathPlannerProgram, "computeWaypointGroundPlaneGradientMap");
   }

   public void firstTickSetup()
   {
      smoothingParametersBuffer.createOpenCLBufferObject(openCLManager);
      traversibilityOffsetsForGradientBuffer.createOpenCLBufferObject(openCLManager);
      traversibilityOffsetsForNominalBuffer.createOpenCLBufferObject(openCLManager);
      offsetsForGroundPlaneGradientBuffer.createOpenCLBufferObject(openCLManager);

      collisionGradientsBuffer.createOpenCLBufferObject(openCLManager);
      maxCollisionsBuffer.createOpenCLBufferObject(openCLManager);
      leftTraversibilitiesBuffer.createOpenCLBufferObject(openCLManager);
      rightTraversibilitiesBuffer.createOpenCLBufferObject(openCLManager);
      leftTraversibilitiesForGradientBuffer.createOpenCLBufferObject(openCLManager);
      rightTraversibilitiesForGradientBuffer.createOpenCLBufferObject(openCLManager);
      leftGroundPlaneCellsBuffer.createOpenCLBufferObject(openCLManager);
      rightGroundPlaneCellsBuffer.createOpenCLBufferObject(openCLManager);
      groundPlaneGradientBuffer.createOpenCLBufferObject(openCLManager);

      populateGroundPlaneOffsetsGradientBuffer();
   }

   public void destroyOpenCLStuff()
   {
      computeCollisionGradientKernel.close();
      computeTraversibilityKernel.close();
      computeTraversibilityForGradientKernel.close();
      computeGroundPlaneGradientKernel.close();

      smoothingParametersBuffer.destroy(openCLManager);
      traversibilityOffsetsForGradientBuffer.destroy(openCLManager);
      traversibilityOffsetsForNominalBuffer.destroy(openCLManager);
      offsetsForGroundPlaneGradientBuffer.destroy(openCLManager);

      collisionGradientsBuffer.destroy(openCLManager);
      maxCollisionsBuffer.destroy(openCLManager);
      leftTraversibilitiesBuffer.destroy(openCLManager);
      rightTraversibilitiesBuffer.destroy(openCLManager);
      leftTraversibilitiesForGradientBuffer.destroy(openCLManager);
      rightTraversibilitiesForGradientBuffer.destroy(openCLManager);
      leftGroundPlaneCellsBuffer.destroy(openCLManager);
      rightGroundPlaneCellsBuffer.destroy(openCLManager);
      groundPlaneGradientBuffer.destroy(openCLManager);
   }

   public void resizeOpenCLObjects(int cellsPerSide)
   {
      int totalCells = cellsPerSide * cellsPerSide;
      collisionGradientsBuffer.resize(2 * yawDiscretizations * totalCells, openCLManager);
      maxCollisionsBuffer.resize(yawDiscretizations * totalCells, openCLManager);
      leftTraversibilitiesBuffer.resize(yawDiscretizations * totalCells, openCLManager);
      rightTraversibilitiesBuffer.resize(yawDiscretizations * totalCells, openCLManager);
      leftTraversibilitiesForGradientBuffer.resize(2 * yawDiscretizations * totalCells, openCLManager);
      rightTraversibilitiesForGradientBuffer.resize(2 * yawDiscretizations * totalCells, openCLManager);
      leftGroundPlaneCellsBuffer.resize(yawDiscretizations * totalCells, openCLManager);
      rightGroundPlaneCellsBuffer.resize(yawDiscretizations * totalCells, openCLManager);
      groundPlaneGradientBuffer.resize(2 * yawDiscretizations * totalCells, openCLManager);
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
      computeGroundPlaneGradient(heightMapBuffer, plannerParametersBuffer, heightMapBuffer);

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

      for (iteration.set(0); iteration.getValue() < iterations; iteration.increment())
      {
         maxCollision.set(0.0);

         for (int waypointIndex = 1; waypointIndex < pathSize - 1; waypointIndex++)
         {
            computeSmoothenessGradient(waypointIndex, gradients[waypointIndex]);

            Tuple3DReadOnly displacementGradient = waypoints[waypointIndex].computeDisplacementGradient();
            gradients[waypointIndex].add(displacementGradient.getX(), displacementGradient.getY());
         }

         if (heightMapData != null)
         {
            for (int waypointIndex = 1; waypointIndex < pathSize - 1; waypointIndex++)
            {
               waypoints[waypointIndex].computeCurrentTraversibility(leftTraversibilitiesBuffer, rightTraversibilitiesBuffer);
            }

            for (int waypointIndex = 2; waypointIndex < pathSize - 2; waypointIndex++)
            {
               /* Collision gradient */
               gradients[waypointIndex].add(waypoints[waypointIndex].computeCollisionGradient(heightMapData, maxCollisionsBuffer, collisionGradientsBuffer));
               maxCollision.set(Math.max(waypoints[waypointIndex].getMaxCollision(), maxCollision.getValue()));

               /* Traversibility gradient */
               Tuple3DReadOnly traversibilityGradient = waypoints[waypointIndex].computeTraversibilityGradient(leftTraversibilitiesForGradientBuffer,
                                                                                                               rightTraversibilitiesForGradientBuffer);
               gradients[waypointIndex].sub(traversibilityGradient.getX(), traversibilityGradient.getY());

               if (waypoints[waypointIndex].isTurnPoint())
               {
                  continue;
               }

               /* Ground plane gradient */
               Tuple3DReadOnly groundPlaneGradient = waypoints[waypointIndex].computeGroundPlaneGradient(heightMapData,
                                                                                                         leftGroundPlaneCellsBuffer,
                                                                                                         rightGroundPlaneCellsBuffer,
                                                                                                         groundPlaneGradientBuffer);
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

            if (gradientMagnitudeSq < gradientMagnitudeToTerminate && iteration.getValue() > minIterations)
            {
               break;
            }
         }

         for (int j = 1; j < pathSize - 1; j++)
         {
            waypoints[j].getPosition().subX(hillClimbGain * gradients[j].getX());
            waypoints[j].getPosition().subY(hillClimbGain * gradients[j].getY());
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
      floatPointer.put(1, (float) equalSpacingWeight);
      floatPointer.put(2, (float) minCurvatureToPenalize);
      floatPointer.put(3, (float) gradientEpsilon);
      floatPointer.put(4, (float) smoothnessWeight);
      floatPointer.put(5, (float) AStarBodyPathPlanner.boxSizeX);
      floatPointer.put(6, (float) AStarBodyPathPlanner.boxSizeY);
      floatPointer.put(7, (float) AStarBodyPathSmootherWaypoint.boxGroundOffset);
      floatPointer.put(8, (float) collisionWeight);
      floatPointer.put(9, (float) yawDiscretizations);
      floatPointer.put(10, (float) flatGroundWeight);
   }

   private void computeCollisionsGradient(OpenCLFloatBuffer heightMapParamsBuffer,
                                          OpenCLFloatBuffer plannerParamsBuffer,
                                          OpenCLFloatBuffer heightMapBuffer,
                                          OpenCLFloatBuffer snappedNodeHeightBuffer)
   {
      openCLManager.setKernelArgument(computeCollisionGradientKernel, 0, heightMapParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientKernel, 1, plannerParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientKernel, 2, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientKernel, 3, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientKernel, 4, snappedNodeHeightBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientKernel, 5, collisionGradientsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeCollisionGradientKernel, 6, maxCollisionsBuffer.getOpenCLBufferObject());

      int totalCells = cellsPerSide * cellsPerSide;
      openCLManager.execute2D(computeCollisionGradientKernel, totalCells, yawDiscretizations);

      collisionGradientsBuffer.readOpenCLBufferObject(openCLManager);
      maxCollisionsBuffer.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();
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
      openCLManager.setKernelArgument(computeTraversibilityKernel, 0, heightMapParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityKernel, 1, plannerParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityKernel, 2, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityKernel, 3, traversibilityOffsetsForNominalBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityKernel, 4, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityKernel, 5, snappedNodeHeightBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityKernel, 6, normalXYZBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityKernel, 7, leftTraversibilitiesBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityKernel, 8, rightTraversibilitiesBuffer.getOpenCLBufferObject());

      int totalCells = cellsPerSide * cellsPerSide;
      openCLManager.execute2D(computeTraversibilityKernel, totalCells, yawDiscretizations);

      leftTraversibilitiesBuffer.readOpenCLBufferObject(openCLManager);
      rightTraversibilitiesBuffer.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();
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
      openCLManager.setKernelArgument(computeTraversibilityForGradientKernel, 0, heightMapParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientKernel, 1, plannerParamsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientKernel, 2, smoothingParametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientKernel, 3, traversibilityOffsetsForGradientBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientKernel, 4, heightMapBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientKernel, 5, snappedNodeHeightBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientKernel, 6, normalXYZBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientKernel, 7, leftTraversibilitiesForGradientBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeTraversibilityForGradientKernel, 8, rightTraversibilitiesForGradientBuffer.getOpenCLBufferObject());

      int totalCells = cellsPerSide * cellsPerSide;
      openCLManager.execute2D(computeTraversibilityForGradientKernel, totalCells, yawDiscretizations);

      leftTraversibilitiesForGradientBuffer.readOpenCLBufferObject(openCLManager);
      rightTraversibilitiesForGradientBuffer.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();
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
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 5, leftGroundPlaneCellsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 6, rightGroundPlaneCellsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeGroundPlaneGradientKernel, 7, groundPlaneGradientBuffer.getOpenCLBufferObject());

      int totalCells = cellsPerSide * cellsPerSide;
      openCLManager.execute2D(computeGroundPlaneGradientKernel, totalCells, yawDiscretizations);

      leftGroundPlaneCellsBuffer.readOpenCLBufferObject(openCLManager);
      rightGroundPlaneCellsBuffer.readOpenCLBufferObject(openCLManager);
      groundPlaneGradientBuffer.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();
   }

   private void computeSmoothenessGradient(int waypointIndex, Vector2DBasics gradientToSet)
   {
      boolean isTurnPoint = waypoints[waypointIndex].isTurnPoint();

      double x0 = waypoints[waypointIndex - 1].getPosition().getX();
      double y0 = waypoints[waypointIndex - 1].getPosition().getY();
      double x1 = waypoints[waypointIndex].getPosition().getX();
      double y1 = waypoints[waypointIndex].getPosition().getY();
      double x2 = waypoints[waypointIndex + 1].getPosition().getX();
      double y2 = waypoints[waypointIndex + 1].getPosition().getY();

      /* Equal spacing gradient */
      double spacingGradientX = -4.0 * AStarBodyPathSmoother.equalSpacingWeight * (x2 - 2.0 * x1 + x0);
      double spacingGradientY = -4.0 * AStarBodyPathSmoother.equalSpacingWeight * (y2 - 2.0 * y1 + y0);
      double alphaTurnPoint = isTurnPoint ? 0.1 : 1.0;
      gradientToSet.setX(alphaTurnPoint * spacingGradientX);
      gradientToSet.setY(alphaTurnPoint * spacingGradientY);

      /* Smoothness gradient */
      double smoothnessGradientX, smoothnessGradientY;
      if (isTurnPoint)
      {
         smoothnessGradientX = 0.0;
         smoothnessGradientY = 0.0;
      }
      else
      {
         double exp = 1.5;
         double f0 = Math.pow(computeDeltaHeadingMagnitude(x0, y0, x1, y1, x2, y2, minCurvatureToPenalize), exp);
         double fPdx = Math.pow(computeDeltaHeadingMagnitude(x0, y0, x1 + gradientEpsilon, y1, x2, y2, minCurvatureToPenalize), exp);
         double fPdy = Math.pow(computeDeltaHeadingMagnitude(x0, y0, x1, y1 + gradientEpsilon, x2, y2, minCurvatureToPenalize), exp);
         smoothnessGradientX = smoothnessWeight * (fPdx - f0) / gradientEpsilon;
         smoothnessGradientY = smoothnessWeight * (fPdy - f0) / gradientEpsilon;
         gradientToSet.addX(smoothnessGradientX);
         gradientToSet.addY(smoothnessGradientY);
      }

      if (visualize)
      {
         waypoints[waypointIndex].updateGradientGraphics(spacingGradientX, spacingGradientY, smoothnessGradientX, smoothnessGradientY);
      }
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
