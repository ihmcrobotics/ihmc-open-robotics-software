package us.ihmc.avatar.reachabilityMap;

import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3DDefinition;

import java.io.File;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper.VisualizationType;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.VoxelExtraData;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
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

   private final YoRegistry registry = new YoRegistry(ReachabilityMapTools.class.getSimpleName());
   private final YoEnum<VisualizationType> currentEvaluation = new YoEnum<>("currentEvaluation", registry, VisualizationType.class);
   private final AtomicReference<VisualizationType> previousEvaluation = new AtomicReference<>(currentEvaluation.getValue());
   private final YoFramePose3D currentEvaluationPose = new YoFramePose3D("currentEvaluationPose", SimulationSession.DEFAULT_INERTIAL_FRAME, registry);
   private final YoDouble D = new YoDouble("D", registry);
   private final YoDouble D0 = new YoDouble("D0", registry);
   private final YoInteger numberOfReachableRays = new YoInteger("numberOfReachableRays", registry);
   private final YoInteger numberOfReachableRotationsAroundRay = new YoInteger("numberOfReachableRotationsAroundRay", registry);
   private final YoInteger numberOfReachablePoses = new YoInteger("numberOfReachablePoses", registry);

   public ReachabilityMapVisualizer(ReachabilityMapRobotInformation robotInformation)
   {
      this.robotInformation = robotInformation;
   }

   public boolean loadReachabilityMapFromFile()
   {
      ReachabilityMapSpreadsheetImporter importer = new ReachabilityMapSpreadsheetImporter();
      File file = importer.openSelectionFileDialog();
      if (file == null)
         return false;

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
      ReferenceFrame controlFrame = ReferenceFrameTools.constructFrameWithChangingTransformFromParent("controlFrame",
                                                                                                      endEffector.getParentJoint().getFrameAfterJoint(),
                                                                                                      new RigidBodyTransform(controlFramePose.getOrientation(),
                                                                                                                             controlFramePose.getPosition()));

      guiControls.addStaticVisuals(ReachabilityMapTools.createReachibilityColorScaleVisuals());
      guiControls.addStaticVisuals(ReachabilityMapTools.createBoundingBoxVisuals(reachabilityMap));
      guiControls.addYoGraphic(newYoGraphicCoordinateSystem3DDefinition("currentEvaluationPose", currentEvaluationPose, 0.15, ColorDefinitions.HotPink()));
      guiControls.addYoGraphic(newYoGraphicCoordinateSystem3DDefinition("controlFrame", controlFramePose, 0.05, ColorDefinitions.parse("#A1887F")));

      SphereVoxelShape sphereVoxelShape = reachabilityMap.getSphereVoxelShape();
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

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
         {
            voxelVisualization.get(VisualizationType.Unreachable)
                              .add(sphereVoxelShape.createDReachabilityVisual(reachabilityMap.getVoxelPosition(voxelIndex), 0.1, -1));
         }
         else
         {
            voxelVisualization.get(VisualizationType.PositionReach).add(sphereVoxelShape.createPositionReachabilityVisual(voxel.getPosition(), 0.2, true));

            if (voxel.getD() > 1e-3)
            {
               voxelVisualization.get(VisualizationType.RayReach).add(sphereVoxelShape.createDReachabilityVisual(voxel.getPosition(), 0.25, voxel.getD()));
               voxelVisualization.get(VisualizationType.PoseReach).add(sphereVoxelShape.createDReachabilityVisual(voxel.getPosition(), 0.25, voxel.getD0()));
            }
            else
            {
               voxelVisualization.get(VisualizationType.Unreachable).add(sphereVoxelShape.createDReachabilityVisual(voxel.getPosition(), 0.1, -1));
            }
         }
      }

      visualizePositionReach(session, robot, controlFrame);
      visualizeRayReach(session, robot);
      visualizePoseReach(session, robot);

      sessionControls.cropBuffer();
      session.startSessionThread();
      guiControls.waitUntilDown();
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

         D0.set(voxel.getD0());
         numberOfReachableRays.set(voxel.getNumberOfReachableRays());
         numberOfReachablePoses.set(voxel.getNumberOfReachablePoses());

         for (int rayIndex = 0; rayIndex < sphereVoxelShape.getNumberOfRays(); rayIndex++)
         {
            numberOfReachableRotationsAroundRay.set(voxel.getNumberOfReachableRotationsAroundRay(rayIndex));

            for (int rotationIndex = 0; rotationIndex < sphereVoxelShape.getNumberOfRotationsAroundRay(); rotationIndex++)
            {
               VoxelExtraData poseExtraData = voxel.getPoseExtraData(rayIndex, rotationIndex);

               if (poseExtraData != null)
               {
                  writeVoxelJointData(poseExtraData, robot);
                  currentEvaluationPose.getPosition().set(poseExtraData.getDesiredPosition());
                  currentEvaluationPose.getOrientation().set(poseExtraData.getDesiredOrientation());
                  simulationStep(session);
               }
            }
         }
      }
   }

   public void visualizeRayReach(SimulationSession session, Robot robot)
   {
      SphereVoxelShape sphereVoxelShape = reachabilityMap.getSphereVoxelShape();
      currentEvaluation.set(VisualizationType.RayReach);

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
            continue;

         VoxelExtraData positionExtraData = voxel.getPositionExtraData();

         if (positionExtraData == null)
            continue;

         D.set(voxel.getD());
         numberOfReachableRays.set(voxel.getNumberOfReachableRays());

         for (int rayIndex = 0; rayIndex < sphereVoxelShape.getNumberOfRays(); rayIndex++)
         {
            VoxelExtraData rayExtraData = voxel.getRayExtraData(rayIndex);

            if (rayExtraData != null)
            {
               writeVoxelJointData(rayExtraData, robot);
               currentEvaluationPose.getPosition().set(rayExtraData.getDesiredPosition());
               currentEvaluationPose.getOrientation().set(rayExtraData.getDesiredOrientation());
               simulationStep(session);
            }
         }
      }
   }

   public void visualizePositionReach(SimulationSession session, Robot robot, ReferenceFrame controlFrame)
   {
      currentEvaluation.set(VisualizationType.PositionReach);

      for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);

         if (voxel == null)
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
}
