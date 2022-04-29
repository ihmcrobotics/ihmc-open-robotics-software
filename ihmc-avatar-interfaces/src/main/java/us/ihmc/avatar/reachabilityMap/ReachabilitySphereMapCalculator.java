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

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final Voxel3DGrid voxel3DGrid;

   private Consumer<VisualDefinition> staticVisualConsumer;

   private boolean showUnreachableVoxels = false;
   private final FramePoint3D desiredPosition = new FramePoint3D();

   private final ReachabilityMapSolver solver;

   private final YoFramePose3D gridFramePose = new YoFramePose3D("gridFramePose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D currentEvaluationPose = new YoFramePose3D("currentEvaluationPose", ReferenceFrame.getWorldFrame(), registry);

   public ReachabilitySphereMapCalculator(OneDoFJointBasics[] robotArmJoints, ControllerOutput controllerOutput, Voxel3DGrid voxel3DGrid)
   {
      this.controllerOutput = controllerOutput;
      this.voxel3DGrid = voxel3DGrid;

      solver = new ReachabilityMapSolver(robotArmJoints, new YoGraphicsListRegistry(), registry);
      gridFramePose.attachVariableChangedListener(v -> voxel3DGrid.setGridPose(gridFramePose));
      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), robotArmJoints[0].getFrameBeforeJoint().getTransformToWorldFrame());
      gridFramePose.appendTranslation(getGridSizeInMeters() / 2.5, 0.0, 0.0);
      setGridFramePose(gridFramePose);
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
      computeNext();
   }

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoInteger currentVoxelIndex = new YoInteger("currentVoxelIndex", registry);
   private final YoBoolean currentVoxelReachComputed = new YoBoolean("currentVoxelReachComputed", registry);
   private final YoInteger currentRayIndex = new YoInteger("currentRayIndex", registry);

   private final FrameVector3D translationFromVoxelOrigin = new FrameVector3D();
   private final FrameQuaternion orientation = new FrameQuaternion();

   public void computeNext()
   {
      if (isDone.getValue())
         return;

      initialize();

      SphereVoxelShape sphereVoxelShape = voxel3DGrid.getSphereVoxelShape();
      int voxelIndex = currentVoxelIndex.getValue();
      int rayIndex = currentRayIndex.getValue();

      boolean hasReachNext = false;

      for (; voxelIndex < voxel3DGrid.getNumberOfVoxels(); voxelIndex++)
      {
         Voxel3DData voxel = voxel3DGrid.getOrCreateVoxel(voxelIndex);
         Voxel3DKey key = voxel.getKey();

         if (!currentVoxelReachComputed.getValue())
         {
            if (isPositionReachable(voxel))
            {
               currentVoxelReachComputed.set(true);
            }
            else
            {
               if (showUnreachableVoxels)
               {
                  LogTools.info("Unreachable voxel, key: {}, position: {}", key, voxel.getPosition());
                  staticVisualConsumer.accept(sphereVoxelShape.createVisual(voxel.getPosition(), 0.1, -1));
               }

               voxel3DGrid.destroy(voxel);
               currentVoxelReachComputed.set(false);
               continue;
            }
         }

         for (; rayIndex < sphereVoxelShape.getNumberOfRays() && !hasReachNext; rayIndex++)
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

                  hasReachNext = true;
                  break;
               }
            }
         }

         if (hasReachNext)
            break;

         double reachabilityValue = voxel.getD();

         if (reachabilityValue > 1e-3)
            staticVisualConsumer.accept(sphereVoxelShape.createVisual(voxel.getPosition(), 0.25, reachabilityValue));

         rayIndex = 0;
         currentVoxelReachComputed.set(false);
      }

      if (!hasReachNext)
      {
         voxelIndex = 0;
         isDone.set(true);
         System.out.println("Done!");
      }

      currentVoxelIndex.set(voxelIndex);
      currentRayIndex.set(rayIndex);
   }

   /**
    * Starts the exploration of the reachable space of the arm. This can take a looooooong time.
    */
   public void buildReachabilitySpace()
   {
      while (!isDone.getValue())
         computeNext();
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
