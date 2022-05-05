package us.ihmc.avatar.reachabilityMap;

import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3DDefinition;
import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicPoint3DDefinition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.IntStream;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.session.tools.SCS1GraphicConversionTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReachabilitySphereMapCalculator implements Controller
{
   private static final boolean VISUALIZE_ALL_SOLVERS = false;

   private final ControllerOutput controllerOutput;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private Voxel3DGrid voxel3DGrid;

   private Consumer<Voxel3DData> voxelUnreachableListener = null;
   private Consumer<Voxel3DData> voxelCompletedListener = null;

   private final FramePoint3D[] desiredPosition;
   private final FrameQuaternion[] orientation;
   private final FrameVector3D[] translationFromVoxelOrigin;
   private final ReachabilityMapSolver[] solvers;

   private final YoFramePose3D gridFramePose = new YoFramePose3D("gridFramePose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D currentEvaluationPose = new YoFramePose3D("currentEvaluationPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final List<YoGraphicGroupDefinition> solverYoGraphicGroupDefinitions = new ArrayList<>();

   public ReachabilitySphereMapCalculator(OneDoFJointBasics[] robotArmJoints, ControllerOutput controllerOutput, Voxel3DGrid voxel3DGrid)
   {
      this(robotArmJoints, controllerOutput, voxel3DGrid, 20);
   }

   public ReachabilitySphereMapCalculator(OneDoFJointBasics[] robotArmJoints, ControllerOutput controllerOutput, Voxel3DGrid voxel3DGrid, int numberOfThreads)
   {
      this.controllerOutput = controllerOutput;
      this.voxel3DGrid = voxel3DGrid;

      solvers = new ReachabilityMapSolver[numberOfThreads];

      desiredPosition = new FramePoint3D[solvers.length];
      orientation = new FrameQuaternion[solvers.length];
      translationFromVoxelOrigin = new FrameVector3D[solvers.length];

      OneDoFJointBasics firstJoint = robotArmJoints[0];
      for (int i = 0; i < numberOfThreads; i++)
      {
         YoRegistry solverRegistry = new YoRegistry("solverRegistry" + i);
         registry.addChild(solverRegistry);

         OneDoFJointBasics[] solverJoints;
         String cloneSuffix;

         if (i == 0)
         {
            solverJoints = robotArmJoints;
            cloneSuffix = "";
         }
         else
         {
            RigidBodyBasics originalRootBody = MultiBodySystemTools.getRootBody(firstJoint.getPredecessor());
            cloneSuffix = "-solver" + i;
            RigidBodyBasics solverRootBody = MultiBodySystemFactories.cloneMultiBodySystem(originalRootBody, ReferenceFrame.getWorldFrame(), cloneSuffix);
            solverJoints = Arrays.stream(robotArmJoints)
                                 .map(originalJoint -> MultiBodySystemTools.findJoint(solverRootBody, originalJoint.getName() + cloneSuffix))
                                 .toArray(OneDoFJointBasics[]::new);
         }

         YoGraphicsListRegistry solverGraphicsRegistry = new YoGraphicsListRegistry();
         solvers[i] = new ReachabilityMapSolver(cloneSuffix, solverJoints, solverGraphicsRegistry, solverRegistry);

         if (i == 0 || VISUALIZE_ALL_SOLVERS)
         {
            YoGraphicGroupDefinition solverYoGraphicGroup = new YoGraphicGroupDefinition("solver" + i);
            solverYoGraphicGroup.setChildren(SCS1GraphicConversionTools.toYoGraphicDefinitions(solverGraphicsRegistry));
            solverYoGraphicGroupDefinitions.add(solverYoGraphicGroup);
         }
         desiredPosition[i] = new FramePoint3D();
         orientation[i] = new FrameQuaternion();
         translationFromVoxelOrigin[i] = new FrameVector3D();
      }

      gridFramePose.attachVariableChangedListener(v -> this.voxel3DGrid.setGridPose(gridFramePose));
      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), firstJoint.getFrameBeforeJoint().getTransformToRoot());
      gridFramePose.appendTranslation(getGridSizeInMeters() / 2.5, 0.0, 0.0);
      setGridFramePose(gridFramePose);
   }

   public void setRobotCollisionModel(RobotDefinition robotDefinition)
   {
      for (ReachabilityMapSolver solver : solvers)
      {
         solver.setCollisionModel(robotDefinition);
      }
   }

   public void setVoxel3DGrid(Voxel3DGrid voxel3DGrid)
   {
      if (this.voxel3DGrid != null)
         voxel3DGrid.getReferenceFrame().setPoseAndUpdate(this.voxel3DGrid.getReferenceFrame().getTransformToParent());
      this.voxel3DGrid = voxel3DGrid;
   }

   public void setVoxelUnreachableListener(Consumer<Voxel3DData> voxelUnreachableListener)
   {
      this.voxelUnreachableListener = voxelUnreachableListener;
   }

   public void setVoxelCompletedListener(Consumer<Voxel3DData> voxelCompletedListener)
   {
      this.voxelCompletedListener = voxelCompletedListener;
   }

   public YoGraphicDefinition getYoGraphicVisuals()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition("ReachabilityCalculatorVisuals");
      List<YoGraphicDefinition> yoGraphics = new ArrayList<>();
      yoGraphics.add(newYoGraphicCoordinateSystem3DDefinition("gridFramePose", gridFramePose, 0.5, ColorDefinitions.Blue()));
      yoGraphics.add(newYoGraphicCoordinateSystem3DDefinition("currentEvaluationPose", currentEvaluationPose, 0.15, ColorDefinitions.HotPink()));
      yoGraphics.add(newYoGraphicPoint3DDefinition("currentEvaluationPosition", currentEvaluationPose.getPosition(), 0.0125, ColorDefinitions.DeepPink()));
      yoGraphics.add(newYoGraphicCoordinateSystem3DDefinition("controlFrame",
                                                              solvers[0].getControlFramePoseInEndEffector(),
                                                              0.05,
                                                              ColorDefinitions.parse("#A1887F")));
      yoGraphics.addAll(solverYoGraphicGroupDefinitions);
      group.setChildren(yoGraphics);
      return group;
   }

   public OneDoFJointBasics[] getRobotArmJoints()
   {
      return solvers[0].getRobotArmJoints();
   }

   /**
    * Sets the transform of the end-effector's frame of interest with respect to the end-effector's
    * body-fixed frame.
    * <p>
    * It is recommended to align the x-axis of the control frame with the vector that is orthogonal to
    * the palm.
    * </p>
    * 
    * @param controlFramePose the transform from the control frame with respect to the end-effector's
    *                         body-fixed frame.
    */
   public void setControlFramePoseInBody(RigidBodyTransformReadOnly controlFramePose)
   {
      for (ReachabilityMapSolver solver : solvers)
      {
         solver.setControlFramePoseInBody(controlFramePose);
      }
   }

   /**
    * Sets the transform of the end-effector's frame of interest with respect to the end-effector's
    * parent joint frame.
    * <p>
    * It is recommended to align the x-axis of the control frame with the vector that is orthogonal to
    * the palm.
    * </p>
    * 
    * @param controlFramePose the transform from the control frame with respect to the end-effector's
    *                         parent joint frame.
    */
   public void setControlFramePoseInParentJoint(RigidBodyTransformReadOnly controlFramePose)
   {
      for (ReachabilityMapSolver solver : solvers)
      {
         solver.setControlFramePoseInParentJoint(controlFramePose);
      }
   }

   /**
    * By default all the axes are selected. For faster exploration, deselect the axis that is
    * orthogonal to the palm.
    * 
    * @param selectX whether to control the orientation around the x-axis of the control frame.
    * @param selectY whether to control the orientation around the y-axis of the control frame.
    * @param selectZ whether to control the orientation around the z-axis of the control frame.
    */
   public void setAngularSelection(boolean selectX, boolean selectY, boolean selectZ)
   {
      for (ReachabilityMapSolver solver : solvers)
      {
         solver.setAngularSelection(selectX, selectY, selectZ);
      }
   }

   public void enableJointTorqueAnalysis(boolean considerJointTorqueLimits)
   {
      for (ReachabilityMapSolver solver : solvers)
      {
         solver.enableJointTorqueAnalysis(considerJointTorqueLimits);
      }
   }

   /**
    * Sets the center and orientation of the grid.
    * 
    * @param pose the pose of the grid.
    */
   public void setGridFramePose(FramePose3DReadOnly pose)
   {
      gridFramePose.setMatchingFrame(pose);
   }

   private final YoInteger currentVoxelIndex = new YoInteger("currentVoxelIndex", registry);

   @Override
   public void doControl()
   {
      if (isDone.getValue())
         return;

      IntStream.range(0, solvers.length).parallel().forEach(solverIndex ->
      {
         int voxelIndex = currentVoxelIndex.getValue() + solverIndex;
         if (voxelIndex >= voxel3DGrid.getNumberOfVoxels())
            return;

         Voxel3DData voxel = voxel3DGrid.getOrCreateVoxel(voxelIndex);

         if (solverIndex == 0)
         {
            currentEvaluationPose.getPosition().setMatchingFrame(voxel.getPosition());
            currentEvaluationPose.getOrientation().setToZero();
         }

         if (solvers[solverIndex].solveFor(voxel.getPosition()))
         {
            computeVoxel(voxel, solverIndex);

            if (voxelCompletedListener != null)
               voxelCompletedListener.accept(voxel);
         }
         else
         {
            if (voxelUnreachableListener != null)
               voxelUnreachableListener.accept(voxel);

            voxel3DGrid.destroy(voxel);
            return;
         }
      });

      currentVoxelIndex.add(solvers.length);

      if (currentVoxelIndex.getValue() >= voxel3DGrid.getNumberOfVoxels())
         isDone.set(true);
   }

   private void computeVoxel(Voxel3DData voxel, int solverIndex)
   {
      SphereVoxelShape sphereVoxelShape = voxel3DGrid.getSphereVoxelShape();

      for (int rayIndex = 0; rayIndex < sphereVoxelShape.getNumberOfRays(); rayIndex++)
      {
         for (int rotationAroundRayIndex = 0; rotationAroundRayIndex < sphereVoxelShape.getNumberOfRotationsAroundRay(); rotationAroundRayIndex++)
         {
            desiredPosition[solverIndex].setIncludingFrame(voxel.getPosition());
            sphereVoxelShape.getPose(translationFromVoxelOrigin[solverIndex], orientation[solverIndex], rayIndex, rotationAroundRayIndex);
            desiredPosition[solverIndex].add(translationFromVoxelOrigin[solverIndex]);

            desiredPosition[solverIndex].changeFrame(ReferenceFrame.getWorldFrame());
            orientation[solverIndex].changeFrame(ReferenceFrame.getWorldFrame());
            if (solverIndex == 0)
               currentEvaluationPose.set(desiredPosition[solverIndex], orientation[solverIndex]);

            boolean success = solvers[solverIndex].solveFor(desiredPosition[solverIndex], orientation[solverIndex]);

            if (success)
            {
               voxel.registerReachablePose(rayIndex, rotationAroundRayIndex);

               if (solverIndex == 0)
               {
                  for (OneDoFJointBasics joint : solvers[solverIndex].getRobotArmJoints())
                  {
                     controllerOutput.getOneDoFJointOutput(joint).setConfiguration(joint);
                     controllerOutput.getOneDoFJointOutput(joint).setEffort(joint);
                  }
               }
               break;
            }
         }
      }
   }

   /**
    * Starts the exploration of the reachable space of the arm. This can take a looooooong time.
    */
   public void buildReachabilitySpace()
   {
      while (!isDone())
         doControl();
   }

   public double getGridSizeInMeters()
   {
      return voxel3DGrid.getGridSizeMeters();
   }

   public boolean isDone()
   {
      return isDone.getValue();
   }

   public Voxel3DGrid getVoxel3DGrid()
   {
      return voxel3DGrid;
   }

   public YoFramePose3D getGridFramePose()
   {
      return gridFramePose;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
