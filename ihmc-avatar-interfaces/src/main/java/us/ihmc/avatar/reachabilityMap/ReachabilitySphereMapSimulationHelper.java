package us.ihmc.avatar.reachabilityMap;

import java.io.IOException;

import us.ihmc.avatar.reachabilityMap.example.RobotParameters.RobotArmJointParameters;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;

public class ReachabilitySphereMapSimulationHelper
{
   private static final boolean VISUALIZE_UNREACHABLE_VOXELS = false;

   private final ReachabilitySphereMapCalculator calculator;
   private final SimulationSession session;

   private Voxel3DGrid voxel3DGrid;

   public ReachabilitySphereMapSimulationHelper(RobotDefinition robotDefinition, String baseName, String endEffectorName)
   {
      robotDefinition.ignoreAllJoints();

      RigidBodyBasics rootBody = robotDefinition.newInstance(ReferenceFrame.getWorldFrame());
      RigidBodyBasics base = MultiBodySystemTools.findRigidBody(rootBody, baseName);
      RigidBodyBasics endEffector = MultiBodySystemTools.findRigidBody(rootBody, endEffectorName);
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);

      session = new SimulationSession("Reachability Analysis - " + robotDefinition.getName());
      session.initializeBufferSize(16000);
      Robot robot = session.addRobot(robotDefinition);
      voxel3DGrid = Voxel3DGrid.newVoxel3DGrid(25, 0.05, 50, 1);
      calculator = new ReachabilitySphereMapCalculator(armJoints, robot.getControllerOutput(), voxel3DGrid);
      calculator.setRobotCollisionModel(robotDefinition);
      robot.addController(calculator);
      session.addYoGraphicDefinition(calculator.getYoGraphicVisuals());
   }

   public void setAngularSelection(boolean selectX, boolean selectY, boolean selectZ)
   {
      calculator.setAngularSelection(selectX, selectY, selectZ);
   }

   public void enableJointTorqueAnalysis(boolean considerJointTorqueLimits)
   {
      calculator.enableJointTorqueAnalysis(considerJointTorqueLimits);
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

   public boolean start()
   {
      SessionVisualizerControls guiControls = SessionVisualizer.startSessionVisualizer(session);
      guiControls.setCameraFocusPosition(calculator.getGridFramePose().getX(), calculator.getGridFramePose().getY(), calculator.getGridFramePose().getZ());
      guiControls.setCameraOrientation(Math.toRadians(15.0), Math.toRadians(170.0), 0.0);
      VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
      visualDefinitionFactory.appendTranslation(RobotArmJointParameters.getRootJoint().getJointOffset());
      visualDefinitionFactory.addCoordinateSystem(1.0);
      guiControls.addStaticVisuals(visualDefinitionFactory.getVisualDefinitions());
      guiControls.addStaticVisuals(ReachabilityMapTools.createReachibilityColorScaleVisuals());
      guiControls.addStaticVisuals(ReachabilityMapTools.createBoundingBoxVisuals(calculator.getVoxel3DGrid()));

      SphereVoxelShape sphereVoxelShape = voxel3DGrid.getSphereVoxelShape();

      if (VISUALIZE_UNREACHABLE_VOXELS)
      {
         calculator.setVoxelUnreachableListener(voxel ->
         {
            LogTools.info("Unreachable voxel, key: {}, position: {}", voxel.getKey(), voxel.getPosition());
            guiControls.addStaticVisual(sphereVoxelShape.createVisual(voxel.getPosition(), 0.1, -1));
         });
      }

      calculator.setVoxelCompletedListener(voxel ->
      {
         double reachabilityValue = voxel.getD();
         if (reachabilityValue > 1e-3)
            guiControls.addStaticVisual(sphereVoxelShape.createVisual(voxel.getPosition(), 0.25, reachabilityValue));

      });

      SimulationSessionControls simControls = session.getSimulationSessionControls();
      simControls.addExternalTerminalCondition(calculator::isDone);
      simControls.simulateNow();
      return calculator.isDone();
   }

   public void exportDataToFile(String robotName, Class<?> classForFilePath) throws IOException
   {
      ReachabilityMapFileWriter.exportVoxel3DGridToFile(robotName, classForFilePath, calculator.getRobotArmJoints(), voxel3DGrid);

   }

   public ReachabilitySphereMapCalculator getCalculator()
   {
      return calculator;
   }

   public Voxel3DGrid getVoxel3DGrid()
   {
      return voxel3DGrid;
   }
}
