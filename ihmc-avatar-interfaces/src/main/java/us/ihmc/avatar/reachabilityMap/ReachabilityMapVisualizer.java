package us.ihmc.avatar.reachabilityMap;

import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3DDefinition;

import java.io.File;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Predicate;

import org.ejml.data.DMatrixRMaj;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper.VisualizationType;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.VoxelExtraData;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DBuilder;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReachabilityMapVisualizer
{
   private static final double bufferGrowthFactor = 1.1;

   private final ReachabilityMapRobotInformation robotInformation;
   private Voxel3DGrid reachabilityMap;

   private boolean visualizePositionReach = true;
   private boolean visualizeRayReach = true;
   private boolean visualizePoseReach = true;

   private Predicate<Voxel3DData> positionFilter = null;
   private RayPredicate rayFilter = null;
   private PosePredicate poseFilter = null;

   private final YoRegistry registry = new YoRegistry(ReachabilityMapTools.class.getSimpleName());
   private final YoEnum<VisualizationType> currentEvaluation = new YoEnum<>("currentEvaluation", registry, VisualizationType.class);
   private final AtomicReference<VisualizationType> previousEvaluation = new AtomicReference<>(currentEvaluation.getValue());
   private final YoFramePose3D currentEvaluationPose = new YoFramePose3D("currentEvaluationPose", SimulationSession.DEFAULT_INERTIAL_FRAME, registry);
   private final YoDouble R = new YoDouble("R", registry);
   private final YoDouble R2 = new YoDouble("R2", registry);
   private final YoInteger numberOfRays = new YoInteger("numberOfRays", registry);
   private final YoInteger numberOfPoses = new YoInteger("numberOfPoses", registry);
   private final YoInteger numberOfReachableRays = new YoInteger("numberOfReachableRays", registry);
   private final YoInteger numberOfReachableRotationsAroundRay = new YoInteger("numberOfReachableRotationsAroundRay", registry);
   private final YoInteger numberOfReachablePoses = new YoInteger("numberOfReachablePoses", registry);

   public ReachabilityMapVisualizer(ReachabilityMapRobotInformation robotInformation)
   {
      this.robotInformation = robotInformation;
   }

   public void setVisualizePositionReach(boolean visualizePositionReach)
   {
      this.visualizePositionReach = visualizePositionReach;
   }

   public void setVisualizeRayReach(boolean visualizeRayReach)
   {
      this.visualizeRayReach = visualizeRayReach;
   }

   public void setVisualizePoseReach(boolean visualizePoseReach)
   {
      this.visualizePoseReach = visualizePoseReach;
   }

   public void setPositionReachVoxelFilter(Predicate<Voxel3DData> positionReachVoxelFilter)
   {
      positionFilter = positionReachVoxelFilter;
   }

   public void setRayFilter(RayPredicate rayFilter)
   {
      this.rayFilter = rayFilter;
   }

   public void setPoseFilter(PosePredicate poseFilter)
   {
      this.poseFilter = poseFilter;
   }

   public boolean loadReachabilityMapFromLatestFile(Class<?> classForFilePath)
   {
      ReachabilityMapSpreadsheetImporter importer = new ReachabilityMapSpreadsheetImporter();
      File file = importer.findLatestFile(classForFilePath, robotInformation);
      if (file == null)
         return false;
      return loadReachabilityMapFromFile(importer, file);
   }

   public boolean loadReachabilityMapFromFile()
   {
      ReachabilityMapSpreadsheetImporter importer = new ReachabilityMapSpreadsheetImporter();
      File file = importer.openSelectionFileDialog();
      if (file == null)
         return false;

      return loadReachabilityMapFromFile(importer, file);
   }

   public boolean loadReachabilityMapFromFile(ReachabilityMapFileReader importer, File file)
   {
      long startTime = System.nanoTime();
      System.out.println("Loading reachability map");

      reachabilityMap = importer.read(file, robotInformation);

      if (reachabilityMap == null)
         return false;

      System.out.println("Done loading reachability map. Took: " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) + " seconds.");
      return true;
   }

   public void setReachabilityMap(Voxel3DGrid reachabilityMap)
   {
      this.reachabilityMap = reachabilityMap;
   }

   public void visualize()
   {
      RobotDefinition robotDefinition = robotInformation.getRobotDefinition();
      robotDefinition.ignoreAllJoints();

      SimulationSession session = new SimulationSession(robotDefinition.getName() + " Reachability Map Visualizer");
      SimulationSessionControls sessionControls = session.getSimulationSessionControls();
      session.getRootRegistry().addChild(registry);
      Robot robot = session.addRobot(robotDefinition);
      SessionVisualizerControls guiControls = SessionVisualizer.startSessionVisualizer(session);
      guiControls.waitUntilFullyUp();
      session.stopSessionThread();

      Pose3DReadOnly controlFramePose = robotInformation.getControlFramePoseInParentJoint();

      RigidBodyBasics endEffector = robot.getRigidBody(robotInformation.getEndEffectorName());
      ReferenceFrame controlFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("controlFrame",
                                                                                                      endEffector.getParentJoint().getFrameAfterJoint(),
                                                                                                      new RigidBodyTransform(controlFramePose.getOrientation(),
                                                                                                                             controlFramePose.getPosition()));

      guiControls.addStaticVisuals(ReachabilityMapTools.createReachibilityColorScaleVisuals());
      guiControls.addStaticVisuals(ReachabilityMapTools.createBoundingBoxVisuals(reachabilityMap));
      guiControls.addYoGraphic(newYoGraphicCoordinateSystem3DDefinition("currentEvaluationPose", currentEvaluationPose, 0.15, ColorDefinitions.HotPink()));
      guiControls.addYoGraphic(newYoGraphicCoordinateSystem3DDefinition("controlFrame", controlFramePose, 0.05, ColorDefinitions.parse("#A1887F")));

      Map<VisualizationType, List<VisualDefinition>> voxelVisualization = new EnumMap<>(VisualizationType.class);

      for (VisualizationType visualizationType : VisualizationType.values())
      {
         voxelVisualization.put(visualizationType, new ArrayList<>());
      }

      currentEvaluation.addListener(v ->
      {
         guiControls.removeStaticVisuals(voxelVisualization.get(previousEvaluation.get()));
         guiControls.addStaticVisuals(voxelVisualization.get(currentEvaluation.getValue()));
         previousEvaluation.set(currentEvaluation.getValue());
      });

      TriangleMesh3DBuilder rayReachableVizMeshBuilder = new TriangleMesh3DBuilder();
      TriangleMesh3DBuilder rayUnreachableVizMeshBuilder = new TriangleMesh3DBuilder();

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
         {
            voxel = reachabilityMap.getOrCreateVoxel(voxelIndex);
            voxelVisualization.get(VisualizationType.Unreachable).add(ReachabilityMapTools.createRReachabilityVisual(voxel, 0.1, (double) -1));
            reachabilityMap.destroy(voxel);
         }
         else
         {
            voxelVisualization.get(VisualizationType.PositionReach).add(ReachabilityMapTools.createPositionReachabilityVisual(voxel, 0.2, true));

            if (voxel.getR() > 1e-3)
            {
               ReachabilityMapTools.createRReachabilityVisualStyle2(voxel, 0.25, rayReachableVizMeshBuilder, rayUnreachableVizMeshBuilder);
               //               voxelVisualization.get(VisualizationType.RayReach).add(ReachabilityMapTools.createRReachabilityVisual(voxel, 0.25, voxel.getR()));
               voxelVisualization.get(VisualizationType.PoseReach).add(ReachabilityMapTools.createRReachabilityVisual(voxel, 0.25, voxel.getR2()));
            }
            else
            {
               voxelVisualization.get(VisualizationType.Unreachable).add(ReachabilityMapTools.createRReachabilityVisual(voxel, 0.1, (double) -1));
            }
         }
      }

      voxelVisualization.get(VisualizationType.RayReach)
                        .add(new VisualDefinition(rayReachableVizMeshBuilder.generateTriangleMesh3D(), new MaterialDefinition(ColorDefinitions.GreenYellow())));
      voxelVisualization.get(VisualizationType.RayReach)
                        .add(new VisualDefinition(rayUnreachableVizMeshBuilder.generateTriangleMesh3D(), new MaterialDefinition(ColorDefinitions.IndianRed())));

      LogTools.info("Done generating visuals");

      if (visualizePositionReach)
      {
         LogTools.info("Start exploring position reach");
         visualizePositionReach(session, robot, controlFrame);
         LogTools.info("Done exploring position reach");
      }
      if (visualizeRayReach)
      {
         LogTools.info("Start exploring ray reach");
         visualizeRayReach(session, robot);
         LogTools.info("Done exploring ray reach");
      }
      if (visualizePoseReach)
      {
         LogTools.info("Start exploring ray reach");
         visualizePoseReach(session, robot);
         LogTools.info("Done exploring ray reach");
      }

      LogTools.info("Cropping buffer");
      sessionControls.cropBuffer();
      LogTools.info("Restarting session's thread");
      session.startSessionThread();
      LogTools.info("Done");
      guiControls.waitUntilDown();
   }

   public void visualizePositionReach(SimulationSession session, Robot robot, ReferenceFrame controlFrame)
   {
      currentEvaluation.set(VisualizationType.PositionReach);

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
            continue;
         if (positionFilter != null && !positionFilter.test(voxel))
            continue;

         VoxelExtraData positionExtraData = voxel.getPositionExtraData();

         if (positionExtraData == null)
            continue;

         writeVoxelJointData(positionExtraData, robot);
         currentEvaluationPose.getPosition().set(positionExtraData.getDesiredPosition());
         currentEvaluationPose.getOrientation().setFromReferenceFrame(controlFrame);
         simulationStep(session);
      }
   }

   public void visualizeRayReach(SimulationSession session, Robot robot)
   {
      SphereVoxelShape sphereVoxelShape = reachabilityMap.getSphereVoxelShape();
      currentEvaluation.set(VisualizationType.RayReach);

      List<VoxelExtraData> filteredRayExtraDataList = new ArrayList<>();

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
            continue;
         if (positionFilter != null && !positionFilter.test(voxel))
            continue;

         VoxelExtraData positionExtraData = voxel.getPositionExtraData();

         if (positionExtraData == null)
            continue;

         filteredRayExtraDataList.clear();
         numberOfRays.set(0);
         numberOfReachableRays.set(0);

         for (int rayIndex = 0; rayIndex < sphereVoxelShape.getNumberOfRays(); rayIndex++)
         {
            if (rayFilter != null && rayFilter.test(sphereVoxelShape, rayIndex))
               continue;

            numberOfRays.increment();

            VoxelExtraData rayExtraData = voxel.getRayExtraData(rayIndex);

            if (rayExtraData == null)
               continue;

            filteredRayExtraDataList.add(rayExtraData);
            numberOfReachableRays.increment();
         }

         R.set(numberOfReachableRays.getValueAsDouble() / numberOfRays.getValueAsDouble());

         for (VoxelExtraData rayExtraData : filteredRayExtraDataList)
         {
            writeVoxelJointData(rayExtraData, robot);
            currentEvaluationPose.getPosition().set(rayExtraData.getDesiredPosition());
            currentEvaluationPose.getOrientation().set(rayExtraData.getDesiredOrientation());
            simulationStep(session);
         }
      }
   }

   public void visualizePoseReach(SimulationSession session, Robot robot)
   {
      SphereVoxelShape sphereVoxelShape = reachabilityMap.getSphereVoxelShape();
      currentEvaluation.set(VisualizationType.PoseReach);

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
            continue;

         VoxelExtraData positionExtraData = voxel.getPositionExtraData();

         if (positionExtraData == null)
            continue;

         List<List<VoxelExtraData>> filteredPoseExtraData2DList = new ArrayList<>();
         TIntArrayList reachableRotationsList = new TIntArrayList();
         numberOfRays.set(0);
         numberOfPoses.set(0);

         for (int rayIndex = 0; rayIndex < sphereVoxelShape.getNumberOfRays(); rayIndex++)
         {
            if (rayFilter != null && rayFilter.test(sphereVoxelShape, rayIndex))
               continue;

            List<VoxelExtraData> filteredPoseExtraDataList = null;
            boolean hasIncrementedNumberOfRays = false;
            boolean hasIncrementedNumberOfReachableRays = false;
            int numberOfReachableRotations = 0;

            for (int rotationIndex = 0; rotationIndex < sphereVoxelShape.getNumberOfRotationsAroundRay(); rotationIndex++)
            {
               if (poseFilter != null && !poseFilter.test(sphereVoxelShape, rayIndex, rotationIndex))
                  continue;

               if (!hasIncrementedNumberOfRays)
               {
                  numberOfRays.increment();
                  hasIncrementedNumberOfRays = true;
               }
               numberOfPoses.increment();

               VoxelExtraData poseExtraData = voxel.getPoseExtraData(rayIndex, rotationIndex);

               if (poseExtraData == null)
                  continue;

               if (filteredPoseExtraDataList == null)
                  filteredPoseExtraDataList = new ArrayList<>();

               filteredPoseExtraDataList.add(poseExtraData);

               numberOfReachableRotations++;
               if (!hasIncrementedNumberOfReachableRays)
               {
                  numberOfReachableRays.increment();
                  hasIncrementedNumberOfReachableRays = true;
               }
               numberOfReachablePoses.increment();
            }

            if (filteredPoseExtraDataList != null)
            {
               filteredPoseExtraData2DList.add(filteredPoseExtraDataList);
               reachableRotationsList.add(numberOfReachableRotations);
            }
         }

         R2.set(numberOfReachablePoses.getValueAsDouble() / numberOfReachablePoses.getValueAsDouble());

         for (int rayIndex = 0; rayIndex < filteredPoseExtraData2DList.size(); rayIndex++)
         {
            List<VoxelExtraData> filteredPoseExtraDataList = filteredPoseExtraData2DList.get(rayIndex);
            numberOfReachableRotationsAroundRay.set(reachableRotationsList.get(rayIndex));

            for (VoxelExtraData poseExtraData : filteredPoseExtraDataList)
            {
               writeVoxelJointData(poseExtraData, robot);
               currentEvaluationPose.getPosition().set(poseExtraData.getDesiredPosition());
               currentEvaluationPose.getOrientation().set(poseExtraData.getDesiredOrientation());
               simulationStep(session);
            }
         }
      }
   }

   private void simulationStep(SimulationSession session)
   {
      YoBufferPropertiesReadOnly bufferProperties = session.getBufferProperties();
      session.getSimulationSessionControls().simulateNow(1);

      if (bufferProperties.getActiveBufferLength() >= bufferProperties.getSize() - 1)
         session.submitBufferSizeRequestAndWait((int) (bufferGrowthFactor * bufferProperties.getSize()));
   }

   public static void writeVoxelJointData(VoxelExtraData voxelExtraData, Robot robot)
   {
      DMatrixRMaj jointPositions = toVectorMatrix(voxelExtraData.getJointPositions());
      DMatrixRMaj jointTorques = toVectorMatrix(voxelExtraData.getJointTorques());

      int positionIndex = 0;
      int torqueIndex = 0;

      for (JointBasics joint : robot.getAllJoints())
      {
         positionIndex = joint.setJointConfiguration(positionIndex, jointPositions);
         torqueIndex = joint.setJointTau(torqueIndex, jointTorques);
      }
      robot.updateFrames();
   }

   public static DMatrixRMaj toVectorMatrix(float[] array)
   {
      DMatrixRMaj vector = new DMatrixRMaj(array.length, 1);
      for (int i = 0; i < array.length; i++)
      {
         vector.set(i, array[i]);
      }
      return vector;
   }

   public static interface RayPredicate
   {
      boolean test(SphereVoxelShape sphereVoxelShape, int rayIndex);
   }

   public static interface PosePredicate
   {
      boolean test(SphereVoxelShape sphereVoxelShape, int rayIndex, int rotationIndex);
   }

   /**
    * Creates new ray filter that selects all ray which dot product with the given vector is positive.
    * <p>
    * For instance, if {@link Axis3D#Y} is given, then only the rays with {@code ray.y >= 0.0} with be
    * considered.
    * </p>
    */
   public static RayPredicate newHemisphereFilter(Vector3DReadOnly filteringRayDirection)
   {
      Vector3D ray = new Vector3D();

      return new RayPredicate()
      {
         @Override
         public boolean test(SphereVoxelShape sphereVoxelShape, int rayIndex)
         {
            sphereVoxelShape.getRay(ray, rayIndex);
            return ray.dot(filteringRayDirection) < 0.0;
         }
      };
   }
}
