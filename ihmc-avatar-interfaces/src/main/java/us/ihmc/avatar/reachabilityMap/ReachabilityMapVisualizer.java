package us.ihmc.avatar.reachabilityMap;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import javafx.application.Platform;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.VBox;
import javafx.util.converter.DoubleStringConverter;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapSimulationHelper.VisualizationType;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.VoxelExtraData;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
import us.ihmc.scs2.definition.visual.TriangleMesh3DBuilder;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
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
   private final YoFramePose3D currentEvaluationPose = new YoFramePose3D("currentEvaluationPose", SimulationSession.DEFAULT_INERTIAL_FRAME, registry);
   private final YoDouble R = new YoDouble("R", registry);
   private final YoDouble R2 = new YoDouble("R2", registry);
   private final YoDouble singularity = new YoDouble("singularity", registry);
   private final YoInteger numberOfRays = new YoInteger("numberOfRays", registry);
   private final YoInteger numberOfPoses = new YoInteger("numberOfPoses", registry);
   private final YoInteger numberOfReachableRays = new YoInteger("numberOfReachableRays", registry);
   private final YoInteger numberOfReachableRotationsAroundRay = new YoInteger("numberOfReachableRotationsAroundRay", registry);
   private final YoInteger numberOfReachablePoses = new YoInteger("numberOfReachablePoses", registry);

   private SimRigidBodyBasics robotBase;
   private SimRigidBodyBasics robotEndEffector;
   private OneDoFJointBasics[] robotArmJoints;

   private SessionVisualizerControls guiControls;
   private VisualizationControlsStageController controller;

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
      ReachabilityMapMatlabImporter matlabImporter = new ReachabilityMapMatlabImporter();
      File file = matlabImporter.findLatestFile(classForFilePath, robotInformation);
      if (file != null)
         return loadReachabilityMapFromFile(matlabImporter, file);

      ReachabilityMapSpreadsheetImporter spreadSheetImporter = new ReachabilityMapSpreadsheetImporter();
      file = spreadSheetImporter.findLatestFile(classForFilePath, robotInformation);
      if (file == null)
      {
         LogTools.error("Failed to load latest file.");
         return false;
      }
      return loadReachabilityMapFromFile(spreadSheetImporter, file);
   }

   public boolean loadReachabilityMapFromFile()
   {
      ReachabilityMapMatlabImporter importer = new ReachabilityMapMatlabImporter();
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
      robotBase = robot.getRigidBody(robotInformation.getBaseName());
      robotEndEffector = robot.getRigidBody(robotInformation.getEndEffectorName());
      robotArmJoints = MultiBodySystemTools.createOneDoFJointPath(robotBase, robotEndEffector);
      RigidBodyBasics endEffector = robot.getRigidBody(robotInformation.getEndEffectorName());
      Pose3DReadOnly controlFramePose = robotInformation.getControlFramePoseInParentJoint();
      RigidBodyTransform frameTransform = new RigidBodyTransform(controlFramePose.getOrientation(), controlFramePose.getPosition());
      MovingReferenceFrame parentFrame = endEffector.getParentJoint().getFrameAfterJoint();
      ReferenceFrame controlFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("controlFrame", parentFrame, frameTransform);

      guiControls = SessionVisualizer.startSessionVisualizer(session);
      guiControls.waitUntilVisualizerFullyUp();
      session.stopSessionThread();

      guiControls.addStaticVisuals(ReachabilityMapTools.createReachibilityColorScaleVisuals());
      guiControls.addYoGraphic(newYoGraphicCoordinateSystem3D("currentEvaluationPose", currentEvaluationPose, 0.15, ColorDefinitions.HotPink()));
      guiControls.addYoGraphic(newYoGraphicCoordinateSystem3D("controlFrame", new FramePose3D(controlFrame), 0.05, ColorDefinitions.parse("#A1887F")));

      createVisualizationControls();

      LogTools.info("Done generating visuals");

      if (visualizePositionReach)
      {
         LogTools.info("Start exploring position reach");
         visualizePositionReach(session, controlFrame);
         LogTools.info("Done exploring position reach");
      }
      if (visualizeRayReach)
      {
         LogTools.info("Start exploring ray reach");
         visualizeRayReach(session);
         LogTools.info("Done exploring ray reach");
      }
      if (visualizePoseReach)
      {
         LogTools.info("Start exploring ray reach");
         visualizePoseReach(session);
         LogTools.info("Done exploring ray reach");
      }

      LogTools.info("Cropping buffer");
      sessionControls.cropBuffer();
      LogTools.info("Restarting session's thread");
      session.startSessionThread();
      LogTools.info("Done");
   }

   public SessionVisualizerControls getGuiControls()
   {
      return guiControls;
   }

   private void createVisualizationControls()
   {
      URL resource = getClass().getClassLoader()
                               .getResource(getClass().getPackage().getName().replace('.', '/') + "/ReachabilityMapVisualizationControlsStage.fxml");
      FXMLLoader fxmlLoader = new FXMLLoader(resource);
      controller = new VisualizationControlsStageController();
      fxmlLoader.setController(controller);

      CountDownLatch latch = new CountDownLatch(1);

      Platform.runLater(() ->
      {
         try
         {
            guiControls.addCustomGUIPane("Reachability map", fxmlLoader.load());
            controller.initialize();

            latch.countDown();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      });

      try
      {
         latch.await();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   public void visualizePositionReach(SimulationSession session, ReferenceFrame controlFrame)
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

         singularity.set(computeFullJacobianSingularityMetric(voxel.getPositionExtraData()));
         writeVoxelJointData(positionExtraData, robotArmJoints);
         currentEvaluationPose.getPosition().set(positionExtraData.getDesiredPosition());
         currentEvaluationPose.getOrientation().setFromReferenceFrame(controlFrame);
         simulationStep(session);
      }
   }

   public void visualizeRayReach(SimulationSession session)
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
            singularity.set(computeFullJacobianSingularityMetric(rayExtraData));
            writeVoxelJointData(rayExtraData, robotArmJoints);
            currentEvaluationPose.getPosition().set(rayExtraData.getDesiredPosition());
            currentEvaluationPose.getOrientation().set(rayExtraData.getDesiredOrientation());
            simulationStep(session);
         }
      }
   }

   public void visualizePoseReach(SimulationSession session)
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
               singularity.set(computeFullJacobianSingularityMetric(poseExtraData));
               writeVoxelJointData(poseExtraData, robotArmJoints);
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

   public static void writeVoxelJointData(VoxelExtraData voxelExtraData, OneDoFJointBasics[] robotArmJoints)
   {
      DMatrixRMaj jointPositions = toVectorMatrix(voxelExtraData.getJointPositions());
      DMatrixRMaj jointTorques = toVectorMatrix(voxelExtraData.getJointTorques());

      int positionIndex = 0;
      int torqueIndex = 0;

      for (JointBasics joint : robotArmJoints)
      {
         positionIndex = joint.setJointConfiguration(positionIndex, jointPositions);
         torqueIndex = joint.setJointTau(torqueIndex, jointTorques);
         joint.updateFrame();
      }
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

   private class VisualizationControlsStageController
   {
      private static final String Reach = "Reach";
      private static final String NeighborhoodDexterity6 = "Neighborhood Dexterity 6";
      private static final String NeighborhoodDexterity18 = "Neighborhood Dexterity 18";
      private static final String NeighborhoodDexterity26 = "Neighborhood Dexterity 26";
      private static final String FullSingularity = "Full Singularity";
      private static final String LinearSingularity = "Linear Singularity";
      private static final String AngularSingularity = "Angular Singularity";
      private static final String RangeOfMotion = "Range of Motion";
      private static final String TauCapability = "Tau Capability";

      private static final String PositionTarget = "Position";
      private static final String RayTarget = "Ray";
      private static final String PoseTarget = "Pose";

      @FXML
      private VBox mainPane;
      @FXML
      private VBox extraControlsContainer;
      @FXML
      private ComboBox<String> visualizationTypeComboBox;
      @FXML
      private ComboBox<String> visualizationTargetComboBox;
      @FXML
      private ToggleButton normalizeDataToggleButton, heatmapToggleButton;
      @FXML
      private Spinner<Double> xMinSpinner, xMaxSpinner;
      @FXML
      private Spinner<Double> yMinSpinner, yMaxSpinner;
      @FXML
      private Spinner<Double> zMinSpinner, zMaxSpinner;
      @FXML
      private Spinner<Double> scaleSpinner;
      @FXML
      private Spinner<Double> payloadSpinner;
      @FXML
      private Label jointLabel0, jointLabel1, jointLabel2, jointLabel3, jointLabel4, jointLabel5, jointLabel6;
      @FXML
      private TextField jointQMinText0, jointQMinText1, jointQMinText2, jointQMinText3, jointQMinText4, jointQMinText5, jointQMinText6;
      @FXML
      private TextField jointQMaxText0, jointQMaxText1, jointQMaxText2, jointQMaxText3, jointQMaxText4, jointQMaxText5, jointQMaxText6;
      @FXML
      private TextField jointTauMaxText0, jointTauMaxText1, jointTauMaxText2, jointTauMaxText3, jointTauMaxText4, jointTauMaxText5, jointTauMaxText6;

      private List<VisualDefinition> previousVisuals;
      private List<VisualDefinition> previousReachabilityMapBBXVisuals;

      public void initialize()
      {
         visualizationTypeComboBox.setItems(FXCollections.observableArrayList(Reach,
                                                                              NeighborhoodDexterity6,
                                                                              NeighborhoodDexterity18,
                                                                              NeighborhoodDexterity26,
                                                                              FullSingularity,
                                                                              LinearSingularity,
                                                                              AngularSingularity,
                                                                              RangeOfMotion,
                                                                              TauCapability));
         visualizationTypeComboBox.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
         {
            // Make it obvious that the payload feature is only available with TauCapability
            payloadSpinner.setDisable(!TauCapability.equals(newValue));
         });
         visualizationTargetComboBox.setItems(FXCollections.observableArrayList(PositionTarget, RayTarget, PoseTarget));
         visualizationTargetComboBox.getSelectionModel().select(RayTarget);

         FramePoint3D minPoint = reachabilityMap.getMinPoint();
         FramePoint3D maxPoint = reachabilityMap.getMaxPoint();

         xMinSpinner.setValueFactory(new DoubleSpinnerValueFactory(minPoint.getX(), maxPoint.getX(), minPoint.getX(), reachabilityMap.getVoxelSize()));
         xMaxSpinner.setValueFactory(new DoubleSpinnerValueFactory(minPoint.getX(), maxPoint.getX(), maxPoint.getX(), reachabilityMap.getVoxelSize()));
         yMinSpinner.setValueFactory(new DoubleSpinnerValueFactory(minPoint.getY(), maxPoint.getY(), minPoint.getY(), reachabilityMap.getVoxelSize()));
         yMaxSpinner.setValueFactory(new DoubleSpinnerValueFactory(minPoint.getY(), maxPoint.getY(), maxPoint.getY(), reachabilityMap.getVoxelSize()));
         zMinSpinner.setValueFactory(new DoubleSpinnerValueFactory(minPoint.getZ(), maxPoint.getZ(), minPoint.getZ(), reachabilityMap.getVoxelSize()));
         zMaxSpinner.setValueFactory(new DoubleSpinnerValueFactory(minPoint.getZ(), maxPoint.getZ(), maxPoint.getZ(), reachabilityMap.getVoxelSize()));
         scaleSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.05, 1.0, 0.20, 0.05));
         payloadSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 150.0, 0.0, 0.25));

         Label[] jointLabels = {jointLabel0, jointLabel1, jointLabel2, jointLabel3, jointLabel4, jointLabel5, jointLabel6};
         TextField[] jointQMinTexts = {jointQMinText0, jointQMinText1, jointQMinText2, jointQMinText3, jointQMinText4, jointQMinText5, jointQMinText6};
         TextField[] jointQMaxTexts = {jointQMaxText0, jointQMaxText1, jointQMaxText2, jointQMaxText3, jointQMaxText4, jointQMaxText5, jointQMaxText6};
         TextField[] jointTauMaxTexts = {jointTauMaxText0,
                                         jointTauMaxText1,
                                         jointTauMaxText2,
                                         jointTauMaxText3,
                                         jointTauMaxText4,
                                         jointTauMaxText5,
                                         jointTauMaxText6};

         ensureJointsCopyExist();

         for (int i = 0; i < Math.min(7, jointsCopy.length); i++)
         {
            OneDoFJointBasics joint = jointsCopy[i];
            jointLabels[i].setText(joint.getName());

            TextFormatter<Double> qminFormatter = new TextFormatter<>(new DoubleStringConverter());
            TextFormatter<Double> qmaxFormatter = new TextFormatter<>(new DoubleStringConverter());
            TextFormatter<Double> taumaxFormatter = new TextFormatter<>(new DoubleStringConverter());
            jointQMinTexts[i].setTextFormatter(qminFormatter);
            jointQMaxTexts[i].setTextFormatter(qmaxFormatter);
            jointTauMaxTexts[i].setTextFormatter(taumaxFormatter);

            qminFormatter.setValue(joint.getJointLimitLower());
            qmaxFormatter.setValue(joint.getJointLimitUpper());
            taumaxFormatter.setValue(joint.getEffortLimitUpper());

            qminFormatter.valueProperty().addListener((o, oldValue, newValue) -> joint.setJointLimitLower(newValue));
            qmaxFormatter.valueProperty().addListener((o, oldValue, newValue) -> joint.setJointLimitUpper(newValue));
            taumaxFormatter.valueProperty().addListener((o, oldValue, newValue) -> joint.setEffortLimit(newValue));
         }
      }

      @FXML
      private void refreshVisualization()
      {
         String selectedType = visualizationTypeComboBox.getSelectionModel().getSelectedItem();
         String selectedTarget = visualizationTargetComboBox.getSelectionModel().getSelectedItem();
         boolean normalize = normalizeDataToggleButton.isSelected();
         BoundingBox3D bbx = new BoundingBox3D(xMinSpinner.getValue(),
                                               yMinSpinner.getValue(),
                                               zMinSpinner.getValue(),
                                               xMaxSpinner.getValue(),
                                               yMaxSpinner.getValue(),
                                               zMaxSpinner.getValue());

         List<VisualDefinition> visuals = null;

         if (selectedType != null)
         {
            if (!heatmapToggleButton.isSelected())
            {
               if (selectedType.equals(Reach))
                  visuals = generateReachVisuals(selectedTarget, normalize, bbx);
               else if (selectedType.equals(NeighborhoodDexterity6))
                  visuals = generateMetricVisual(normalize, bbx, Voxel3DData::computeD06);
               else if (selectedType.equals(NeighborhoodDexterity18))
                  visuals = generateMetricVisual(normalize, bbx, Voxel3DData::computeD018);
               else if (selectedType.equals(NeighborhoodDexterity26))
                  visuals = generateMetricVisual(normalize, bbx, Voxel3DData::computeD026);
               else if (selectedType.equals(FullSingularity))
                  visuals = generateMetricVisual(selectedTarget, normalize, bbx, ReachabilityMapVisualizer.this::computeFullJacobianSingularityMetric);
               else if (selectedType.equals(LinearSingularity))
                  visuals = generateMetricVisual(selectedTarget, normalize, bbx, ReachabilityMapVisualizer.this::computeLinearJacobianSingularityMetric);
               else if (selectedType.equals(AngularSingularity))
                  visuals = generateMetricVisual(selectedTarget, normalize, bbx, ReachabilityMapVisualizer.this::computeAngularJacobianSingularityMetric);
               else if (selectedType.equals(RangeOfMotion))
                  visuals = generateMetricVisual(selectedTarget, normalize, bbx, ReachabilityMapVisualizer.this::computeRoMMetric);
               else if (selectedType.equals(TauCapability))
                  visuals = generateMetricVisual(selectedTarget, normalize, bbx, extraData -> computeTauCapabilityMetric(extraData, payloadSpinner.getValue()));
            }
            else
            {
               RayIndexBasedVoxelQualityMetric qualityMetric = null;
               if (selectedType.equals(Reach))
                  qualityMetric = getReachHeatMapMetric(selectedTarget);
               else if (selectedType.equals(FullSingularity))
                  qualityMetric = getVoxelExtraDataBasedMetric(selectedTarget, ReachabilityMapVisualizer.this::computeFullJacobianSingularityMetric);
               else if (selectedType.equals(LinearSingularity))
                  qualityMetric = getVoxelExtraDataBasedMetric(selectedTarget, ReachabilityMapVisualizer.this::computeLinearJacobianSingularityMetric);
               else if (selectedType.equals(AngularSingularity))
                  qualityMetric = getVoxelExtraDataBasedMetric(selectedTarget, ReachabilityMapVisualizer.this::computeAngularJacobianSingularityMetric);
               else if (selectedType.equals(RangeOfMotion))
                  qualityMetric = getVoxelExtraDataBasedMetric(selectedTarget, ReachabilityMapVisualizer.this::computeRoMMetric);
               else if (selectedType.equals(TauCapability))
                  qualityMetric = getVoxelExtraDataBasedMetric(selectedTarget, extraData -> computeTauCapabilityMetric(extraData, payloadSpinner.getValue()));

               visuals = generateMetricRayBasedHeatMapVisuals(normalize, qualityMetric, bbx);
            }
         }

         if (previousVisuals != null)
            guiControls.removeStaticVisuals(previousVisuals);
         guiControls.addStaticVisuals(visuals);

         if (previousReachabilityMapBBXVisuals != null)
            guiControls.removeStaticVisuals(previousReachabilityMapBBXVisuals);
         List<VisualDefinition> reachabilityMapBBXVisuals = generateMapBBX(bbx);
         guiControls.addStaticVisuals(reachabilityMapBBXVisuals);
         previousReachabilityMapBBXVisuals = reachabilityMapBBXVisuals;

         previousVisuals = visuals;
      }

      private List<VisualDefinition> generateMapBBX(BoundingBox3D bbx)
      {
         ReferenceFrame referenceFrame = reachabilityMap.getReferenceFrame();
         return ReachabilityMapTools.createBoundingBoxVisuals(new FramePoint3D(referenceFrame, bbx.getMinPoint()),
                                                              new FramePoint3D(referenceFrame, bbx.getMaxPoint()));
      }

      public List<VisualDefinition> generateReachVisuals(String target, boolean normalizeMetric, BoundingBox3D bbx)
      {
         switch (target)
         {
            case PositionTarget:
               return generateMetricVisual(normalizeMetric, bbx, voxel -> 1.0);
            case RayTarget:
               return generateMetricVisual(normalizeMetric, bbx, voxel ->
               {
                  int n = 0;
                  for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
                  {
                     if (voxel.isRayReachable(rayIndex) && isWithinLimits(voxel.getRayExtraData(rayIndex)))
                        n++;
                  }
                  return (double) n / (double) voxel.getNumberOfRays();
               });
            case PoseTarget:
               return generateMetricVisual(normalizeMetric, bbx, voxel ->
               {
                  int n = 0;
                  for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
                  {
                     for (int rotationIndex = 0; rotationIndex < voxel.getNumberOfRotationsAroundRay(); rotationIndex++)
                     {
                        if (voxel.isPoseReachable(rayIndex, rotationIndex) && isWithinLimits(voxel.getPoseExtraData(rayIndex, rotationIndex)))
                           n++;
                     }
                  }
                  return (double) n / (double) voxel.getNumberOfRays() / (double) voxel.getNumberOfRotationsAroundRay();
               });
         }
         return null;
      }

      private List<VisualDefinition> generateMetricVisual(String target,
                                                          boolean normalizeMetric,
                                                          BoundingBox3D bbx,
                                                          ToDoubleFunction<VoxelExtraData> metricFunction)
      {
         switch (target)
         {
            case PositionTarget:
               return generateMetricVisual(normalizeMetric, bbx, voxel -> metricFunction.applyAsDouble(voxel.getPositionExtraData()));
            case RayTarget:
               return generateMetricVisual(normalizeMetric, bbx, voxel -> evaluateRayVoxelExtraData(voxel, metricFunction));
            case PoseTarget:
               return generateMetricVisual(normalizeMetric, bbx, voxel -> evaluatePoseVoxelExtraData(voxel, metricFunction));
         }
         return null;
      }

      private List<VisualDefinition> generateMetricVisual(boolean normalizeMetric, BoundingBox3D bbx, ToDoubleFunction<Voxel3DData> metricFunction)
      {
         List<VisualDefinition> visuals = new ArrayList<>();
         TDoubleArrayList metricValues = new TDoubleArrayList();
         List<Voxel3DData> evaluatedVoxels = new ArrayList<>();

         for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
         {
            Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);
            if (voxel != null && bbx.isInsideInclusive(voxel.getPosition()))
            {
               double metricValue = metricFunction.applyAsDouble(voxel);

               if (metricValue > 1.0e-3)
               {
                  metricValues.add(metricValue);
                  evaluatedVoxels.add(voxel);
               }
            }
         }

         double min = metricValues.min();
         double max = metricValues.max();
         double range = max - min;

         for (int i = 0; i < evaluatedVoxels.size(); i++)
         {
            double value = normalizeMetric ? (metricValues.get(i) - min) / range : metricValues.get(i);
            visuals.add(ReachabilityMapTools.createMetricVisual(evaluatedVoxels.get(i), scaleSpinner.getValue(), value));
         }

         return visuals;
      }

      public RayIndexBasedVoxelQualityMetric getReachHeatMapMetric(String target)
      {
         if (RayTarget.equals(target))
         {
            return (voxel, rayIndex) -> voxel.isRayReachable(rayIndex) ? 1.0 : 0.0;
         }

         if (PoseTarget.equals(target))
         {
            return (voxel, rayIndex) ->
            {
               double r = (double) voxel.getNumberOfReachableRotationsAroundRay(rayIndex);
               double R = (double) voxel.getNumberOfRotationsAroundRay();
               return r / R;
            };
         }

         return null;
      }

      public RayIndexBasedVoxelQualityMetric getVoxelExtraDataBasedMetric(String target, ToDoubleFunction<VoxelExtraData> function)
      {
         switch (target)
         {
            case RayTarget:
               return (voxel, rayIndex) -> function.applyAsDouble(voxel.getRayExtraData(rayIndex));
            case PoseTarget:
               return (voxel, rayIndex) ->
               {
                  double s = 0.0;
                  for (int rotationIndex = 0; rotationIndex < voxel.getNumberOfRotationsAroundRay(); rotationIndex++)
                  {
                     s += function.applyAsDouble(voxel.getPoseExtraData(rayIndex, rotationIndex));
                  }
                  return s /= voxel.getNumberOfRotationsAroundRay();
               };
         }
         return null;
      }

      public List<VisualDefinition> generateMetricRayBasedHeatMapVisuals(boolean normalizeMetric, RayIndexBasedVoxelQualityMetric metric, BoundingBox3D bbx)
      {
         if (metric == null)
            return null;

         TriangleMesh3DBuilder vizMeshBuilder = new TriangleMesh3DBuilder();
         Point2D unreachableTexture = new Point2D(0.0, 0.5);
         Point2D reachableTexture = new Point2D(1.0, 0.5);

         List<double[]> savedValues = new ArrayList<>();
         List<Voxel3DData> savedVoxels = new ArrayList<>();

         double min = Double.POSITIVE_INFINITY;
         double max = Double.NEGATIVE_INFINITY;

         for (int voxelIndex = 0; voxelIndex < reachabilityMap.getNumberOfVoxels(); voxelIndex++)
         {
            Voxel3DData voxel = reachabilityMap.getVoxel(voxelIndex);
            if (voxel == null)
               continue;

            boolean insideBBX = bbx.isInsideInclusive(voxel.getPosition());

            double[] voxelValues = new double[voxel.getNumberOfRays()];
            double voxelMin = Double.POSITIVE_INFINITY;
            double voxelMax = Double.NEGATIVE_INFINITY;

            for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
            {
               double value = metric.evaluate(voxel, rayIndex);
               voxelValues[rayIndex] = value;
               voxelMin = Math.min(voxelMin, value);
               voxelMax = Math.max(voxelMax, value);
            }

            if (voxelMax > 1.0e-3)
            { // Otherwise the voxel is pretty much not reachable
               min = Math.min(min, voxelMin);
               max = Math.max(max, voxelMax);
               if (insideBBX)
               {
                  savedVoxels.add(voxel);
                  savedValues.add(voxelValues);
               }
            }
         }

         double offset = min;
         double range = max - min;

         for (int i = 0; i < savedVoxels.size(); i++)
         {
            Voxel3DData voxel = savedVoxels.get(i);
            double[] voxelValues = savedValues.get(i);

            ReachabilityMapTools.createVoxelRayHeatmap(voxel,
                                                       scaleSpinner.getValue(),
                                                       rayIndex -> normalizeMetric ? (voxelValues[rayIndex] - offset) / range : voxelValues[rayIndex],
                                                       reachableTexture,
                                                       unreachableTexture,
                                                       vizMeshBuilder);
         }

         TextureDefinition diffuseMap = ReachabilityMapTools.generateReachabilityGradient(0.0, 0.7);
         return Collections.singletonList(new VisualDefinition(vizMeshBuilder.generateTriangleMesh3D(), new MaterialDefinition(diffuseMap)));
      }
   }

   public double evaluateRayVoxelExtraData(Voxel3DData voxel, ToDoubleFunction<VoxelExtraData> jointPositionFunction)
   {
      double s = 0.0;

      for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
      {
         if (voxel.isRayReachable(rayIndex))
         {
            if (isWithinLimits(voxel.getRayExtraData(rayIndex)))
            {
               s += jointPositionFunction.applyAsDouble(voxel.getRayExtraData(rayIndex));
            }
         }
      }

      return s / voxel.getNumberOfReachableRays();
   }

   public double evaluatePoseVoxelExtraData(Voxel3DData voxel, ToDoubleFunction<VoxelExtraData> jointPositionFunction)
   {
      double s = 0.0;

      for (int rayIndex = 0; rayIndex < voxel.getNumberOfRays(); rayIndex++)
      {
         for (int rotationIndex = 0; rotationIndex < voxel.getNumberOfRotationsAroundRay(); rotationIndex++)
         {
            if (voxel.isPoseReachable(rayIndex, rotationIndex) && isWithinLimits(voxel.getPoseExtraData(rayIndex, rotationIndex)))
            {
               s += jointPositionFunction.applyAsDouble(voxel.getPoseExtraData(rayIndex, rotationIndex));
            }
         }
      }

      return s / voxel.getNumberOfReachablePoses();
   }

   public double computeRoMMetric(VoxelExtraData extraData)
   {
      if (extraData == null || !isWithinLimits(extraData))
         return 0.0;
      ensureJointsCopyExist();

      double l = 0.0;

      for (int i = 0; i < jointsCopy.length; i++)
      {
         double min = jointsCopy[i].getJointLimitLower();
         double max = jointsCopy[i].getJointLimitUpper();
         double midRange = 0.5 * (min + max);
         double rom = max - min;
         double q = MathTools.clamp(extraData.getJointPositions()[i], min, max);
         double ratio = (q - midRange) / rom;
         l += ratio * ratio;
      }

      return 1.0 - 4.0 / jointsCopy.length * l;
   }

   private GravityCoriolisExternalWrenchMatrixCalculator jointTorquesCalculator;

   public double computeTauCapabilityMetric(VoxelExtraData extraData, double payloadInKg)
   {
      if (extraData == null || !isWithinLimits(extraData))
         return 0.0;

      if (payloadInKg == 0.0 && extraData.getJointTorques() != null)
         return computeTauCapabilityMetric(extraData.getJointTorques());

      ensureJointsCopyExist();

      double gravity = -9.81;
      if (jointTorquesCalculator == null)
      {
         jointTorquesCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(MultiBodySystemTools.getRootBody(jointsCopy[0].getPredecessor()));
         jointTorquesCalculator.setGravitionalAcceleration(gravity);
      }

      for (int i = 0; i < jointsCopy.length; i++)
      {
         jointsCopy[i].setQ(extraData.getJointPositions()[i]);
         jointsCopy[i].updateFrame();
      }

      Wrench payloadWrench = new Wrench(endEffectorCopy.getBodyFixedFrame(), controlFrameCopy);
      payloadWrench.getLinearPart().setMatchingFrame(ReferenceFrame.getWorldFrame(), new Vector3D(0.0, 0.0, gravity * payloadInKg));
      jointTorquesCalculator.getExternalWrench(endEffectorCopy).setMatchingFrame(payloadWrench);
      jointTorquesCalculator.compute();

      float[] jointTorques = new float[jointsCopy.length];
      for (int i = 0; i < jointsCopy.length; i++)
      {
         OneDoFJointBasics joint = jointsCopy[i];
         double tau = jointTorquesCalculator.getComputedJointTau(joint).get(0);

         if (!MathTools.intervalContains(tau, joint.getEffortLimitLower(), joint.getEffortLimitUpper()))
            return 0.0;

         jointTorques[i] = (float) tau;
      }
      return computeTauCapabilityMetric(jointTorques);
   }

   public double computeTauCapabilityMetric(float[] jointTorques)
   {
      ensureJointsCopyExist();

      double l = 0.0;

      for (int i = 0; i < jointsCopy.length; i++)
      {
         double min = jointsCopy[i].getEffortLimitLower();
         double max = jointsCopy[i].getEffortLimitUpper();
         double midRange = 0.5 * (min + max);
         double rom = max - min;
         double tau = MathTools.clamp(jointTorques[i], min, max);
         double ratio = (tau - midRange) / rom;
         l += ratio * ratio;
      }

      return 1.0 - 4.0 / jointsCopy.length * l;
   }

   public double computeFullJacobianSingularityMetric(VoxelExtraData extraData)
   {
      if (extraData == null || !isWithinLimits(extraData))
         return 0.0;

      DMatrixRMaj jacobian = computeJacobian(extraData.getJointPositions());
      DMatrixRMaj outer = new DMatrixRMaj(6, 6);
      CommonOps_DDRM.multOuter(jacobian, outer);

      if (!eig.decompose(outer))
         return 0.0;

      double min = eig.getEigenvalue(0).getReal();
      double max = eig.getEigenvalue(0).getReal();

      for (int i = 1; i < eig.getNumberOfEigenvalues(); i++)
      {
         min = Math.min(min, eig.getEigenvalue(i).getReal());
         max = Math.max(max, eig.getEigenvalue(i).getReal());
      }

      return min / max;
   }

   public double computeLinearJacobianSingularityMetric(VoxelExtraData extraData)
   {
      if (extraData == null || !isWithinLimits(extraData))
         return 0.0;
      DMatrixRMaj jacobian = computeJacobian(extraData.getJointPositions());
      DMatrixRMaj linearPart = new DMatrixRMaj(3, jacobian.numCols);
      CommonOps_DDRM.extract(jacobian, 3, 6, 0, jacobian.numCols, linearPart, 0, 0);
      DMatrixRMaj outer = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.multOuter(linearPart, outer);

      if (!eig.decompose(outer))
         return 0.0;

      double min = eig.getEigenvalue(0).getReal();
      double max = eig.getEigenvalue(0).getReal();

      for (int i = 1; i < eig.getNumberOfEigenvalues(); i++)
      {
         min = Math.min(min, eig.getEigenvalue(i).getReal());
         max = Math.max(max, eig.getEigenvalue(i).getReal());
      }

      return min / max;
   }

   public double computeAngularJacobianSingularityMetric(VoxelExtraData extraData)
   {
      if (extraData == null || !isWithinLimits(extraData))
         return 0.0;

      DMatrixRMaj jacobian = computeJacobian(extraData.getJointPositions());
      DMatrixRMaj angularPart = new DMatrixRMaj(3, jacobian.numCols);
      CommonOps_DDRM.extract(jacobian, 0, 3, 0, jacobian.numCols, angularPart, 0, 0);
      DMatrixRMaj outer = new DMatrixRMaj(3, 3);
      CommonOps_DDRM.multOuter(angularPart, outer);

      if (!eig.decompose(outer))
         return 0.0;

      double min = eig.getEigenvalue(0).getReal();
      double max = eig.getEigenvalue(0).getReal();

      for (int i = 1; i < eig.getNumberOfEigenvalues(); i++)
      {
         min = Math.min(min, eig.getEigenvalue(i).getReal());
         max = Math.max(max, eig.getEigenvalue(i).getReal());
      }

      return min / max;
   }

   private final EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(false);

   private GeometricJacobianCalculator jacobianCalculator;
   private OneDoFJointBasics[] jointsCopy;
   private ReferenceFrame controlFrameCopy;
   private RigidBodyBasics endEffectorCopy;

   private DMatrixRMaj computeJacobian(float[] jointPositions)
   {
      ensureJointsCopyExist();

      for (int i = 0; i < jointsCopy.length; i++)
      {
         jointsCopy[i].setQ(jointPositions[i]);
         jointsCopy[i].updateFrame();
      }

      if (jacobianCalculator == null)
         jacobianCalculator = new GeometricJacobianCalculator();
      jacobianCalculator.setKinematicChain(jointsCopy);
      jacobianCalculator.setJacobianFrame(controlFrameCopy);
      return jacobianCalculator.getJacobianMatrix();
   }

   public boolean isWithinLimits(VoxelExtraData extraData)
   {
      ensureJointsCopyExist();

      for (int i = 0; i < jointsCopy.length; i++)
      {
         double q = extraData.getJointPositions()[i];

         OneDoFJointBasics joint = jointsCopy[i];
         if (!MathTools.intervalContains(q, joint.getJointLimitLower() - 1.0e-7, joint.getJointLimitUpper() + 1.0e-7))
            return false;
      }

      return true;
   }

   public void ensureJointsCopyExist()
   {
      if (jointsCopy == null)
      { // Initializing the copy of the joints that we'll use to evaluate the Jacobian.
         RigidBodyBasics rootBodyCopy = robotInformation.getRobotDefinition().newInstance(ReferenceFrame.getWorldFrame());
         RigidBodyBasics baseCopy = MultiBodySystemTools.findRigidBody(rootBodyCopy, robotInformation.getBaseName());
         endEffectorCopy = MultiBodySystemTools.findRigidBody(rootBodyCopy, robotInformation.getEndEffectorName());
         jointsCopy = MultiBodySystemTools.createOneDoFJointPath(baseCopy, endEffectorCopy);
         controlFrameCopy = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("controlFrameCopy",
                                                                                              endEffectorCopy.getParentJoint().getFrameAfterJoint(),
                                                                                              new RigidBodyTransform(robotInformation.getControlFramePoseInParentJoint()
                                                                                                                                     .getOrientation(),
                                                                                                                     robotInformation.getControlFramePoseInParentJoint()
                                                                                                                                     .getPosition()));
      }
   }

   private static interface RayIndexBasedVoxelQualityMetric
   {
      double evaluate(Voxel3DData voxel, int rayIndex);
   }
}
