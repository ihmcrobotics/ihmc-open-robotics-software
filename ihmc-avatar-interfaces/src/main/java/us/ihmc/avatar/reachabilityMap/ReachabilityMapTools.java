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
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.TriangleMesh3DFactories;
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

public class ReachabilityMapTools
{
   public static List<VisualDefinition> createBoundingBoxVisuals(Voxel3DGrid voxel3DGrid)
   {
      return createBoundingBoxVisuals(voxel3DGrid.getMinPoint(), voxel3DGrid.getMaxPoint());
   }

   public static List<VisualDefinition> createBoundingBoxVisuals(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      double width = 0.01;
      ColorDefinition color = ColorDefinitions.LightBlue();
      VisualDefinitionFactory boundingBox = new VisualDefinitionFactory();
      FramePoint3D modifiableMin = new FramePoint3D(min);
      modifiableMin.changeFrame(ReferenceFrame.getWorldFrame());
      FramePoint3D modifiableMax = new FramePoint3D(max);
      modifiableMax.changeFrame(ReferenceFrame.getWorldFrame());
      double x0 = modifiableMin.getX();
      double y0 = modifiableMin.getY();
      double z0 = modifiableMin.getZ();
      double x1 = modifiableMax.getX();
      double y1 = modifiableMax.getY();
      double z1 = modifiableMax.getZ();
      // The three segments originating from min
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z0, x1, y0, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z0, x0, y1, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z0, x0, y0, z1, width), color);
      // The three segments originating from min
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y1, z1, x0, y1, z1, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y1, z1, x1, y0, z1, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y1, z1, x1, y1, z0, width), color);

      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y0, z0, x1, y1, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x1, y0, z0, x1, y0, z1, width), color);

      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y1, z0, x1, y1, z0, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y1, z0, x0, y1, z1, width), color);

      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z1, x1, y0, z1, width), color);
      boundingBox.addGeometryDefinition(TriangleMesh3DFactories.Line(x0, y0, z1, x0, y1, z1, width), color);

      return boundingBox.getVisualDefinitions();
   }

   public static List<VisualDefinition> createReachibilityColorScaleVisuals()
   {
      VisualDefinitionFactory voxelViz = new VisualDefinitionFactory();
      double maxReachability = 0.7;
      double resolution = 0.1;
      voxelViz.appendTranslation(-1.0, -1.0, 0.0);

      for (double z = 0; z <= maxReachability; z += maxReachability * resolution)
      {
         ColorDefinition color = ColorDefinitions.hsb(z * 360.0, 1.0, 1.0);
         voxelViz.appendTranslation(0.0, 0.0, resolution);
         voxelViz.addSphere(0.025, color);
      }

      return voxelViz.getVisualDefinitions();
   }

   public static void loadVisualizeReachabilityMap(ReachabilityMapRobotInformation robotInformation)
   {
      long startTime = System.nanoTime();
      System.out.println("Loading reachability map");
      ReachabilityMapSpreadsheetImporter importer = new ReachabilityMapSpreadsheetImporter();
      File file = importer.openSelectionFileDialog();
      if (file == null)
         return;
      Voxel3DGrid reachabilityMap = importer.read(file, robotInformation);

      System.out.println("Done loading reachability map. Took: " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) + " seconds.");

      SphereVoxelShape sphereVoxelShape = reachabilityMap.getSphereVoxelShape();

      Map<VisualizationType, List<VisualDefinition>> voxelVisualization = new EnumMap<>(VisualizationType.class);

      YoRegistry registry = new YoRegistry(ReachabilityMapTools.class.getSimpleName());

      YoEnum<VisualizationType> currentEvaluation = new YoEnum<>("currentEvaluation", registry, VisualizationType.class);
      AtomicReference<VisualizationType> previousEvaluation = new AtomicReference<>(currentEvaluation.getValue());
      YoFramePose3D currentEvaluationPose = new YoFramePose3D("currentEvaluationPose", ReferenceFrame.getWorldFrame(), registry);
      YoDouble D = new YoDouble("D", registry);
      YoDouble D0 = new YoDouble("D0", registry);
      YoInteger numberOfReachableRays = new YoInteger("numberOfReachableRays", registry);
      YoInteger numberOfReachableRotationsAroundRay = new YoInteger("numberOfReachableRotationsAroundRay", registry);
      YoInteger numberOfReachablePoses = new YoInteger("numberOfReachablePoses", registry);

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

      guiControls.addStaticVisuals(createReachibilityColorScaleVisuals());
      guiControls.addStaticVisuals(ReachabilityMapTools.createBoundingBoxVisuals(reachabilityMap));
      guiControls.addYoGraphic(newYoGraphicCoordinateSystem3DDefinition("currentEvaluationPose", currentEvaluationPose, 0.15, ColorDefinitions.HotPink()));
      guiControls.addYoGraphic(newYoGraphicCoordinateSystem3DDefinition("controlFrame", controlFramePose, 0.05, ColorDefinitions.parse("#A1887F")));

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

      currentEvaluation.set(VisualizationType.PositionReach);
      double bufferGrowthFactor = 1.1;
      YoBufferPropertiesReadOnly bufferProperties = session.getBufferProperties();

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
         sessionControls.simulateNow(1);

         if (bufferProperties.getActiveBufferLength() >= bufferProperties.getSize() - 1)
            session.submitBufferSizeRequestAndWait((int) (bufferGrowthFactor * bufferProperties.getSize()));
      }

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
               sessionControls.simulateNow(1);

               if (bufferProperties.getActiveBufferLength() >= bufferProperties.getSize() - 1)
                  session.submitBufferSizeRequestAndWait((int) (bufferGrowthFactor * bufferProperties.getSize()));
            }
         }
      }

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
                  sessionControls.simulateNow(1);

                  if (bufferProperties.getActiveBufferLength() >= bufferProperties.getSize() - 1)
                     session.submitBufferSizeRequestAndWait((int) (bufferGrowthFactor * bufferProperties.getSize()));
               }
            }
         }
      }

      sessionControls.cropBuffer();
      session.startSessionThread();
      guiControls.waitUntilDown();
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
