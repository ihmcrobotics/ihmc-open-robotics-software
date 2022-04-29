package us.ihmc.avatar.reachabilityMap;

import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3DDefinition;
import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicPoint3DDefinition;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DKey;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReachabilitySphereMapCalculator implements Controller
{
   private final ControllerOutput controllerOutput;

   public enum CalculatorState
   {
      VOXEL_REACH, RAY_REACH, DONE;
   };

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final Voxel3DGrid voxel3DGrid;

   private Consumer<VisualDefinition> staticVisualConsumer;

   private boolean showUnreachableVoxels = false;

   private final ReachabilityMapSolver solver;

   private final YoFramePose3D gridFramePose = new YoFramePose3D("gridFramePose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D currentEvaluationPose = new YoFramePose3D("currentEvaluationPose", ReferenceFrame.getWorldFrame(), registry);
   private final StateMachine<CalculatorState, State> stateMachine;

   public ReachabilitySphereMapCalculator(OneDoFJointBasics[] robotArmJoints, ControllerOutput controllerOutput, Voxel3DGrid voxel3DGrid)
   {
      this.controllerOutput = controllerOutput;
      this.voxel3DGrid = voxel3DGrid;

      solver = new ReachabilityMapSolver(robotArmJoints, new YoGraphicsListRegistry(), registry);
      gridFramePose.attachVariableChangedListener(v -> voxel3DGrid.setGridPose(gridFramePose));
      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), robotArmJoints[0].getFrameBeforeJoint().getTransformToWorldFrame());
      gridFramePose.appendTranslation(getGridSizeInMeters() / 2.5, 0.0, 0.0);
      setGridFramePose(gridFramePose);
      stateMachine = setupStateMachine();
   }

   private StateMachine<CalculatorState, State> setupStateMachine()
   {
      StateMachineFactory<CalculatorState, State> factory = new StateMachineFactory<>(CalculatorState.class);
      factory.setNamePrefix("reachabilityCalculatorStateMachine").setRegistry(registry);
      factory.addStateAndDoneTransition(CalculatorState.VOXEL_REACH, new VoxelReachState(), CalculatorState.RAY_REACH);
      factory.addStateAndDoneTransition(CalculatorState.RAY_REACH, new RayReachState(), CalculatorState.DONE);
      factory.addState(CalculatorState.DONE, new DoneState());
      return factory.build(CalculatorState.VOXEL_REACH);
   }

   public void setStaticVisualConsumer(Consumer<VisualDefinition> staticVisualConsumer)
   {
      this.staticVisualConsumer = staticVisualConsumer;
   }

   public YoGraphicDefinition getYoGraphicVisuals()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition("ReachabilityCalculatorVisuals");
      List<YoGraphicDefinition> yoGraphics = new ArrayList<>();
      yoGraphics.add(newYoGraphicCoordinateSystem3DDefinition("gridFramePose", gridFramePose, 0.5, ColorDefinitions.Blue()));
      yoGraphics.add(newYoGraphicCoordinateSystem3DDefinition("currentEvaluationPose", currentEvaluationPose, 0.15, ColorDefinitions.HotPink()));
      yoGraphics.add(newYoGraphicPoint3DDefinition("currentEvaluationPosition", currentEvaluationPose.getPosition(), 0.0125, ColorDefinitions.DeepPink()));
      group.setChildren(yoGraphics);
      return group;
   }

   public OneDoFJointBasics[] getRobotArmJoints()
   {
      return solver.getRobotArmJoints();
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
   public void setControlFramePose(RigidBodyTransform controlFramePose)
   {
      solver.setControlFramePose(controlFramePose);
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
      solver.setAngularSelection(selectX, selectY, selectZ);
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

   @Override
   public void doControl()
   {
      stateMachine.doActionAndTransition();
      //      computeNext();
   }

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoInteger currentVoxelIndex = new YoInteger("currentVoxelIndex", registry);
   private final YoInteger currentRayIndex = new YoInteger("currentRayIndex", registry);

   /**
    * Starts the exploration of the reachable space of the arm. This can take a looooooong time.
    */
   public void buildReachabilitySpace()
   {
      while (!isDone.getValue())
         doControl();
   }

   private class VoxelReachState implements State
   {
      @Override
      public void onEntry()
      {
         currentVoxelIndex.set(0);
      }

      @Override
      public void doAction(double timeInState)
      {
         int voxelIndex = currentVoxelIndex.getValue();

         for (; voxelIndex < voxel3DGrid.getNumberOfVoxels(); voxelIndex++)
         {
            Voxel3DData voxel = voxel3DGrid.getOrCreateVoxel(voxelIndex);
            Voxel3DKey key = voxel.getKey();

            if (isPositionReachable(voxel))
            {
               for (OneDoFJointBasics joint : solver.getRobotArmJoints())
               {
                  controllerOutput.getOneDoFJointOutput(joint).setConfiguration(joint);
               }

               break;
            }
            else
            {
               if (showUnreachableVoxels)
               {
                  LogTools.info("Unreachable voxel, key: {}, position: {}", key, voxel.getPosition());
                  SphereVoxelShape sphereVoxelShape = voxel3DGrid.getSphereVoxelShape();
                  staticVisualConsumer.accept(sphereVoxelShape.createVisual(voxel.getPosition(), 0.1, -1));
               }

               voxel3DGrid.destroy(voxel);
               continue;
            }
         }

         currentVoxelIndex.set(voxelIndex + 1);
      }

      @Override
      public void onExit(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return currentVoxelIndex.getValue() >= voxel3DGrid.getNumberOfVoxels();
      }
   }

   private class RayReachState implements State
   {
      @Override
      public void onEntry()
      {
         currentVoxelIndex.set(0);
         currentRayIndex.set(0);
      }

      private final FramePoint3D desiredPosition = new FramePoint3D();
      private final FrameVector3D translationFromVoxelOrigin = new FrameVector3D();
      private final FrameQuaternion orientation = new FrameQuaternion();

      @Override
      public void doAction(double timeInState)
      {
         if (isDone.getValue())
            return;

         SphereVoxelShape sphereVoxelShape = voxel3DGrid.getSphereVoxelShape();
         int voxelIndex = currentVoxelIndex.getValue();

         for (; voxelIndex < voxel3DGrid.getNumberOfVoxels(); voxelIndex++)
         {
            Voxel3DData voxel = voxel3DGrid.getVoxel(voxelIndex);

            if (voxel == null)
               continue;

            boolean isDoneWithVoxel = computeNextRay(voxel);

            if (isDoneWithVoxel)
            {
               double reachabilityValue = voxel.getD();
               if (reachabilityValue > 1e-3)
                  staticVisualConsumer.accept(sphereVoxelShape.createVisual(voxel.getPosition(), 0.25, reachabilityValue));
            }
            else
            {
               break;
            }
         }

         currentVoxelIndex.set(voxelIndex);
      }

      private boolean computeNextRay(Voxel3DData voxel)
      {
         SphereVoxelShape sphereVoxelShape = voxel3DGrid.getSphereVoxelShape();
         int rayIndex = currentRayIndex.getValue();

         if (rayIndex == sphereVoxelShape.getNumberOfRays())
            rayIndex = 0;

         for (; rayIndex < sphereVoxelShape.getNumberOfRays(); rayIndex++)
         {
            for (int rotationAroundRayIndex = 0; rotationAroundRayIndex < sphereVoxelShape.getNumberOfRotationsAroundRay(); rotationAroundRayIndex++)
            {
               desiredPosition.setIncludingFrame(voxel.getPosition());
               sphereVoxelShape.getPose(translationFromVoxelOrigin, orientation, rayIndex, rotationAroundRayIndex);
               desiredPosition.add(translationFromVoxelOrigin);

               desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());
               orientation.changeFrame(ReferenceFrame.getWorldFrame());
               currentEvaluationPose.set(desiredPosition, orientation);

               boolean success = solver.solveFor(desiredPosition, orientation);

               if (success)
               {
                  voxel.registerReachablePose(rayIndex, rotationAroundRayIndex);

                  for (OneDoFJointBasics joint : solver.getRobotArmJoints())
                  {
                     controllerOutput.getOneDoFJointOutput(joint).setConfiguration(joint);
                  }

                  currentRayIndex.set(rayIndex + 1);
                  return currentRayIndex.getValue() == sphereVoxelShape.getNumberOfRays();
               }
            }
         }

         return true;
      }

      @Override
      public void onExit(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return currentVoxelIndex.getValue() >= voxel3DGrid.getNumberOfVoxels()
               && currentRayIndex.getValue() >= voxel3DGrid.getSphereVoxelShape().getNumberOfRays();
      }
   }

   private class DoneState implements State
   {
      @Override
      public void onEntry()
      {
         LogTools.info("Done!!");
      }

      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onExit(double timeInState)
      {
      }
   }

   private boolean isPositionReachable(Voxel3DData voxel)
   {
      currentEvaluationPose.getPosition().setMatchingFrame(voxel.getPosition());
      currentEvaluationPose.getOrientation().setToZero();
      return solver.solveFor(voxel.getPosition());
   }

   public double getGridSizeInMeters()
   {
      return voxel3DGrid.getGridSizeMeters();
   }

   public boolean isDone()
   {
      return stateMachine.isCurrentStateTerminal();
      //      return isDone.getValue();
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
