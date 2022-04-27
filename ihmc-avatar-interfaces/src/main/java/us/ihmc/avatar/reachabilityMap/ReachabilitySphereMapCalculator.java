package us.ihmc.avatar.reachabilityMap;

import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3DDefinition;
import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicPoint3DDefinition;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
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
   private Voxel3DGrid voxel3dGrid;
   private SphereVoxelShape sphereVoxelShape;

   private Consumer<VisualDefinition> staticVisualConsumer;

   private boolean showUnreachableVoxels = false;
   private int gridSizeInNumberOfVoxels = 25;
   private double voxelSize = 0.05;
   private int numberOfRays = 50;
   private int numberOfRotationsAroundRay = 1;
   private final FramePoint3D voxelLocation = new FramePoint3D();
   private final FramePoint3D modifiableVoxelLocation = new FramePoint3D();

   private final ReachabilityMapSolver solver;
   private ReachabilityMapFileWriter reachabilityMapFileWriter;

   private final YoFramePose3D gridFramePose = new YoFramePose3D("gridFramePose", ReferenceFrame.getWorldFrame(), registry);
   private final ReferenceFrame gridFrame = new ReferenceFrame("gridFrame", ReferenceFrame.getWorldFrame())
   {
      {
         gridFramePose.attachVariableChangedListener(v -> update());
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         gridFramePose.get(transformToParent);
      }
   };
   private final YoFramePose3D currentEvaluationPose = new YoFramePose3D("currentEvaluationPose", ReferenceFrame.getWorldFrame(), registry);

   public ReachabilitySphereMapCalculator(OneDoFJointBasics[] robotArmJoints, ControllerOutput controllerOutput)
   {
      this.controllerOutput = controllerOutput;
      solver = new ReachabilityMapSolver(robotArmJoints, new YoGraphicsListRegistry(), registry);

      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), robotArmJoints[0].getFrameBeforeJoint().getTransformToWorldFrame());
      gridFramePose.appendTranslation(getGridSizeInMeters() / 2.5, 0.0, 0.0);
      setGridFramePose(gridFramePose);
   }

   public void setStaticVisualConsumer(Consumer<VisualDefinition> staticVisualConsumer)
   {
      ReachabilityMapTools.createReachibilityColorScaleVisuals().forEach(staticVisualConsumer);
      this.staticVisualConsumer = staticVisualConsumer;
   }

   public YoGraphicDefinition getYoGraphicVisuals()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition("ReachabilityCalcualtorVisuals");
      List<YoGraphicDefinition> yoGraphics = new ArrayList<>();
      yoGraphics.add(newYoGraphicCoordinateSystem3DDefinition("gridFramePose", gridFramePose, 0.5, ColorDefinitions.Blue()));
      yoGraphics.add(newYoGraphicCoordinateSystem3DDefinition("currentEvaluationPose", currentEvaluationPose, 0.15, ColorDefinitions.HotPink()));
      yoGraphics.add(newYoGraphicPoint3DDefinition("currentEvaluationPosition", currentEvaluationPose.getPosition(), 0.0125, ColorDefinitions.DeepPink()));
      group.setChildren(yoGraphics);
      return group;
   }

   /**
    * Configure the space and resolution for the exploration of the arm reachability.
    * 
    * @param gridSizeInNumberOfVoxels   the region explored is a cube of size
    *                                   {@code gridSizeInNumberOfVoxels * vozelSize}.
    * @param voxelSize                  directly relates to the resolution of the exploration.
    * @param numberOfRays               sets for each voxel the number of directions that are to be
    *                                   explored with the x-axis.
    * @param numberOfRotationsAroundRay sets the number of rotations to explore about each ray. The
    *                                   total number of orientations explored at each voxel is
    *                                   {@code numberOfRays * numberOfRotationsAroundRay}.
    */
   public void setGridParameters(int gridSizeInNumberOfVoxels, double voxelSize, int numberOfRays, int numberOfRotationsAroundRay)
   {
      this.gridSizeInNumberOfVoxels = gridSizeInNumberOfVoxels;
      this.voxelSize = voxelSize;
      this.numberOfRays = numberOfRays;
      this.numberOfRotationsAroundRay = numberOfRotationsAroundRay;
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

   /**
    * Sets up the calculator so it exports the result in an Excel file. That file can later be loaded
    * using {@link ReachabilityMapFileLoader}.
    * 
    * @param robotName        the robot name.
    * @param classForFilePath this can be the class of the caller of this method.
    */
   public void setupCalculatorToRecordInFile(String robotName, Class<?> classForFilePath) throws IOException
   {
      if (robotName == null || robotName.isEmpty())
      {
         System.err.println("Invalid robot name (either null or empty)");
         return;
      }
      reachabilityMapFileWriter = new ReachabilityMapFileWriter(robotName, classForFilePath);
   }

   private final YoBoolean isInitialized = new YoBoolean("isInitialized", registry);

   @Override
   public void initialize()
   {
      if (isInitialized.getValue())
         return;

      isInitialized.set(true);

      sphereVoxelShape = new SphereVoxelShape(gridFrame, voxelSize, numberOfRays, numberOfRotationsAroundRay, SphereVoxelType.graspOrigin);
      voxel3dGrid = new Voxel3DGrid(gridFrame, sphereVoxelShape, gridSizeInNumberOfVoxels, voxelSize);
      if (reachabilityMapFileWriter != null)
         reachabilityMapFileWriter.initialize(solver.getRobotArmJoints(), voxel3dGrid);
      ReachabilityMapTools.createBoundingBoxVisuals(voxel3dGrid.getMinPoint(), voxel3dGrid.getMaxPoint()).forEach(staticVisualConsumer);
   }

   @Override
   public void doControl()
   {
      computeNext();
   }

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoInteger currentXIndex = new YoInteger("currentXIndex", registry);
   private final YoInteger currentYIndex = new YoInteger("currentYIndex", registry);
   private final YoInteger currentZIndex = new YoInteger("currentZIndex", registry);
   private final YoBoolean currentVoxelReachComputed = new YoBoolean("currentVoxelReachComputed", registry);
   private final YoInteger currentRayIndex = new YoInteger("currentRayIndex", registry);

   private final FrameVector3D translationFromVoxelOrigin = new FrameVector3D();
   private final FrameQuaternion orientation = new FrameQuaternion();

   public void computeNext()
   {
      if (isDone.getValue())
         return;

      initialize();

      int xIndex = currentXIndex.getValue();
      int yIndex = currentYIndex.getValue();
      int zIndex = currentZIndex.getValue();
      int rayIndex = currentRayIndex.getValue();

      boolean hasReachNext = false;

      for (; xIndex < gridSizeInNumberOfVoxels; xIndex++)
      {
         for (; yIndex < gridSizeInNumberOfVoxels; yIndex++)
         {
            for (; zIndex < gridSizeInNumberOfVoxels; zIndex++)
            {
               if (!currentVoxelReachComputed.getValue())
               {
                  if (isPositionReachable(xIndex, yIndex, zIndex))
                  {
                     currentVoxelReachComputed.set(true);
                  }
                  else
                  {
                     if (showUnreachableVoxels)
                     {
                        voxel3dGrid.getVoxel(voxelLocation, xIndex, yIndex, zIndex);
                        LogTools.info("Unreachable voxel: ({}, {}, {})", xIndex, yIndex, zIndex);
                        staticVisualConsumer.accept(sphereVoxelShape.createVisual(voxelLocation, 0.1, -1));
                     }
                     currentVoxelReachComputed.set(false);
                     continue;
                  }
               }

               voxel3dGrid.getVoxel(voxelLocation, xIndex, yIndex, zIndex);

               for (; rayIndex < numberOfRays && !hasReachNext; rayIndex++)
               {

                  for (int rotationAroundRayIndex = 0; rotationAroundRayIndex < numberOfRotationsAroundRay; rotationAroundRayIndex++)
                  {
                     modifiableVoxelLocation.setIncludingFrame(voxelLocation);
                     sphereVoxelShape.getPose(translationFromVoxelOrigin, orientation, rayIndex, rotationAroundRayIndex);
                     modifiableVoxelLocation.add(translationFromVoxelOrigin);

                     modifiableVoxelLocation.changeFrame(ReferenceFrame.getWorldFrame());
                     orientation.changeFrame(ReferenceFrame.getWorldFrame());
                     currentEvaluationPose.set(modifiableVoxelLocation, orientation);

                     boolean success = solver.solveFor(modifiableVoxelLocation, orientation);

                     if (success)
                     {
                        voxel3dGrid.registerReachablePose(xIndex, yIndex, zIndex, rayIndex, rotationAroundRayIndex);
                        if (reachabilityMapFileWriter != null)
                           reachabilityMapFileWriter.registerReachablePose(xIndex, yIndex, zIndex, rayIndex, rotationAroundRayIndex);

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

               double reachabilityValue = voxel3dGrid.getD(xIndex, yIndex, zIndex);

               if (reachabilityValue > 1e-3)
                  staticVisualConsumer.accept(sphereVoxelShape.createVisual(voxelLocation, 0.25, reachabilityValue));

               rayIndex = 0;
               currentVoxelReachComputed.set(false);
            }

            if (hasReachNext)
               break;

            zIndex = 0;
            currentVoxelReachComputed.set(false);
         }
         if (hasReachNext)
            break;

         yIndex = 0;
         currentVoxelReachComputed.set(false);
      }

      if (!hasReachNext)
      {
         xIndex = 0;
         isDone.set(true);

         if (reachabilityMapFileWriter != null)
            reachabilityMapFileWriter.exportAndClose();
         System.out.println("Done!");
      }

      currentXIndex.set(xIndex);
      currentYIndex.set(yIndex);
      currentZIndex.set(zIndex);
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

   private boolean isPositionReachable(int xIndex, int yIndex, int zIndex)
   {
      voxel3dGrid.getVoxel(voxelLocation, xIndex, yIndex, zIndex);
      modifiableVoxelLocation.setIncludingFrame(voxelLocation);
      modifiableVoxelLocation.changeFrame(ReferenceFrame.getWorldFrame());
      currentEvaluationPose.getPosition().set(modifiableVoxelLocation);
      currentEvaluationPose.getOrientation().setToZero();

      return solver.solveFor(voxelLocation);
   }

   public double getGridSizeInMeters()
   {
      return gridSizeInNumberOfVoxels * voxelSize;
   }

   public boolean isDone()
   {
      return isDone.getValue();
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
