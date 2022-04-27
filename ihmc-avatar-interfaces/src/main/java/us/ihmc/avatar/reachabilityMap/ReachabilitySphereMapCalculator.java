package us.ihmc.avatar.reachabilityMap;

import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3DDefinition;
import static us.ihmc.avatar.scs2.YoGraphicDefinitionFactory.newYoGraphicPoint3DDefinition;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
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
   private Voxel3DGrid voxel3DGrid;
   private SphereVoxelShape sphereVoxelShape;

   private Consumer<VisualDefinition> staticVisualConsumer;

   private boolean showUnreachableVoxels = false;
   private int gridSizeInNumberOfVoxels = 25;
   private double voxelSize = 0.05;
   private int numberOfRays = 50;
   private int numberOfRotationsAroundRay = 1;
   private final FramePoint3D desiredPosition = new FramePoint3D();

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
      voxel3DGrid = new Voxel3DGrid(gridFrame, sphereVoxelShape, gridSizeInNumberOfVoxels, voxelSize);
      if (reachabilityMapFileWriter != null)
         reachabilityMapFileWriter.initialize(solver.getRobotArmJoints(), voxel3DGrid);
      ReachabilityMapTools.createBoundingBoxVisuals(voxel3DGrid.getMinPoint(), voxel3DGrid.getMaxPoint()).forEach(staticVisualConsumer);
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
               Voxel3DData voxel = voxel3DGrid.getOrCreateVoxel(xIndex, yIndex, zIndex);

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
                        LogTools.info("Unreachable voxel: ({}), position: {}", EuclidCoreIOTools.getStringOf(", ", xIndex, yIndex, zIndex), voxel.getPosition());
                        staticVisualConsumer.accept(sphereVoxelShape.createVisual(voxel.getPosition(), 0.1, -1));
                     }

                     voxel3DGrid.destroy(voxel);
                     currentVoxelReachComputed.set(false);
                     continue;
                  }
               }

               for (; rayIndex < numberOfRays && !hasReachNext; rayIndex++)
               {

                  for (int rotationAroundRayIndex = 0; rotationAroundRayIndex < numberOfRotationsAroundRay; rotationAroundRayIndex++)
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

               double reachabilityValue = voxel.getD();

               if (reachabilityValue > 1e-3)
                  staticVisualConsumer.accept(sphereVoxelShape.createVisual(voxel.getPosition(), 0.25, reachabilityValue));

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

   private boolean isPositionReachable(Voxel3DData voxel)
   {
      currentEvaluationPose.getPosition().setMatchingFrame(voxel.getPosition());
      currentEvaluationPose.getOrientation().setToZero();
      return solver.solveFor(voxel.getPosition());
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
