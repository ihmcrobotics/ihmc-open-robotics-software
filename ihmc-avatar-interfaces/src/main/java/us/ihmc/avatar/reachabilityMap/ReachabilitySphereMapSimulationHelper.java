package us.ihmc.avatar.reachabilityMap;

import java.io.IOException;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class ReachabilitySphereMapSimulationHelper
{
   private final ReachabilitySphereMapCalculator calculator;
   private final SimulationSession session;

   public enum VisualizationType
   {
      PositionReach, RayReach, PoseReach, WristManipulability, Unreachable;
   };

   private final Map<VisualizationType, ObservableList<VisualDefinition>> voxelVisualization = new EnumMap<>(VisualizationType.class);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final AtomicReference<VisualizationType> previousVisualizationType = new AtomicReference<>(VisualizationType.RayReach);
   private final YoEnum<VisualizationType> currentVisualizationType = new YoEnum<>("currentVisualizationType", registry, VisualizationType.class);

   public ReachabilitySphereMapSimulationHelper(RobotDefinition robotDefinition, String baseName, String endEffectorName)
   {
      robotDefinition.ignoreAllJoints();

      ReferenceFrame robotRootFrame = Robot.createRobotRootFrame(robotDefinition, ReferenceFrame.getWorldFrame()); // This allows to visualize YoGraphics in frames other than world
      RigidBodyBasics rootBody = robotDefinition.newInstance(robotRootFrame);
      RigidBodyBasics base = MultiBodySystemTools.findRigidBody(rootBody, baseName);
      RigidBodyBasics endEffector = MultiBodySystemTools.findRigidBody(rootBody, endEffectorName);
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);

      session = new SimulationSession("Reachability Analysis - " + robotDefinition.getName());
      session.initializeBufferSize(16000);
      session.getRootRegistry().addChild(registry);
      Robot robot = session.addRobot(robotDefinition);
      calculator = new ReachabilitySphereMapCalculator(armJoints, robot.getControllerOutput());
      calculator.setVoxel3DGrid(Voxel3DGrid.newVoxel3DGrid(25, 0.05, 50, 1));
      calculator.setRobotCollisionModel(robotDefinition);
      robot.addController(calculator);

      previousVisualizationType.set(VisualizationType.RayReach);
      currentVisualizationType.set(VisualizationType.RayReach);
   }

   /**
    * Specifies while axis is orthogonal to the palm.
    */
   public void setPalmOrthogonalAxis(Axis3D axis)
   {
      calculator.setPalmOrthogonalAxis(axis);
   }

   public void enableJointTorqueAnalysis(boolean considerJointTorqueLimits)
   {
      calculator.enableJointTorqueAnalysis(considerJointTorqueLimits);
   }

   public void setEvaluateDReachability(boolean enable)
   {
      calculator.setEvaluateDReachability(enable);
   }

   public void setEvaluateD0Reachability(boolean enable)
   {
      calculator.setEvaluateD0Reachability(enable);
   }

   public void setGridPose(RigidBodyTransformReadOnly pose)
   {
      calculator.getGridFramePose().set(pose);
   }

   public void setGridPosition(double x, double y, double z)
   {
      calculator.getGridFramePose().set(x, y, z, 0, 0, 0);
   }

   public void setGridParameters(int gridSizeInNumberOfVoxels, double voxelSize, int numberOfRays, int numberOfRotationsAroundRay)
   {
      Voxel3DGrid voxel3DGrid = Voxel3DGrid.newVoxel3DGrid(gridSizeInNumberOfVoxels, voxelSize, numberOfRays, numberOfRotationsAroundRay);
      calculator.setVoxel3DGrid(voxel3DGrid);
   }

   public void setControlFramePoseInBody(RigidBodyTransformReadOnly controlFramePoseInParentJointFrame)
   {
      calculator.setControlFramePoseInBody(controlFramePoseInParentJointFrame);
   }

   public void setControlFramePoseInParentJoint(RigidBodyTransformReadOnly controlFramePoseInParentJointFrame)
   {
      calculator.setControlFramePoseInParentJoint(controlFramePoseInParentJointFrame);
   }

   private SessionVisualizerControls guiControls;

   public boolean start()
   {
      session.addYoGraphicDefinition(calculator.getYoGraphicVisuals());
      guiControls = SessionVisualizer.startSessionVisualizer(session);
      guiControls.waitUntilFullyUp();
      session.stopSessionThread(); // We'll run the simulation in the current thread.
      guiControls.setCameraFocusPosition(calculator.getGridFramePose().getX(), calculator.getGridFramePose().getY(), calculator.getGridFramePose().getZ());
      guiControls.setCameraOrientation(Math.toRadians(15.0), Math.toRadians(170.0), 0.0);
      setupVisualization();

      SimulationSessionControls simControls = session.getSimulationSessionControls();
      simControls.addExternalTerminalCondition(calculator::isDone);
      simControls.simulateNow();
      session.startSessionThread(); // Give controls back to the GUI.
      return calculator.isDone();
   }

   private void setupVisualization()
   {
      guiControls.addStaticVisuals(ReachabilityMapTools.createReachibilityColorScaleVisuals());
      guiControls.addStaticVisuals(ReachabilityMapTools.createBoundingBoxVisuals(calculator.getVoxel3DGrid()));

      SphereVoxelShape sphereVoxelShape = calculator.getVoxel3DGrid().getSphereVoxelShape();

      for (VisualizationType visualizationType : VisualizationType.values())
      {
         ObservableList<VisualDefinition> visualList = FXCollections.observableArrayList();

         visualList.addListener(new ListChangeListener<VisualDefinition>()
         {
            @Override
            public void onChanged(Change<? extends VisualDefinition> change)
            {
               if (currentVisualizationType.getValue() != visualizationType)
                  return;

               while (change.next())
               {
                  if (change.wasAdded())
                     guiControls.addStaticVisuals(change.getAddedSubList());
                  if (change.wasRemoved())
                     guiControls.removeStaticVisuals(change.getAddedSubList());
               }
            }
         });
         voxelVisualization.put(visualizationType, visualList);
      }

      calculator.setVoxelUnreachableListener(voxel ->
      {
         voxelVisualization.get(VisualizationType.Unreachable).add(sphereVoxelShape.createDReachabilityVisual(voxel.getPosition(), 0.1, -1));
      });

      calculator.setVoxelCompletedListener(voxel ->
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
      });

      currentVisualizationType.addListener(v ->
      {
         guiControls.removeStaticVisuals(voxelVisualization.get(previousVisualizationType.get()));
         guiControls.addStaticVisuals(voxelVisualization.get(currentVisualizationType.getValue()));
         previousVisualizationType.set(currentVisualizationType.getValue());
      });
   }

   public void exportDataToFile(String robotName, Class<?> classForFilePath) throws IOException
   {
      ReachabilityMapSpreadsheetExporterV0.exportVoxel3DGridToFile(robotName,
                                                        classForFilePath,
                                                        calculator.getRobotArmJoints(),
                                                        calculator.getControlFramePose(),
                                                        calculator.getVoxel3DGrid());
   }

   public ReachabilitySphereMapCalculator getCalculator()
   {
      return calculator;
   }

   public Voxel3DGrid getVoxel3DGrid()
   {
      return calculator.getVoxel3DGrid();
   }
}
