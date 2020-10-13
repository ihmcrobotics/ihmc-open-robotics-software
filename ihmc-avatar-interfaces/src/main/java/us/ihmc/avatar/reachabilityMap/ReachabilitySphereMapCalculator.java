package us.ihmc.avatar.reachabilityMap;

import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ReachabilitySphereMapCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private Voxel3DGrid voxel3dGrid;
   private SphereVoxelShape sphereVoxelShape;

   private final SimulationConstructionSet scs;
   private int gridSizeInNumberOfVoxels = 25;
   private double voxelSize = 0.05;
   private int numberOfRays = 50;
   private int numberOfRotationsAroundRay = 1;
   private final FramePoint3D voxelLocation = new FramePoint3D();
   private final FramePoint3D modifiableVoxelLocation = new FramePoint3D();
   private final ArrayList<ReachabilityMapListener> reachabilityMapListeners = new ArrayList<>();

   private final ReachabilityMapSolver solver;
   private ReachabilityMapFileWriter reachabilityMapFileWriter;

   private final PoseReferenceFrame gridFrame = new PoseReferenceFrame("gridFrame", ReferenceFrame.getWorldFrame());
   private final YoGraphicReferenceFrame gridFrameViz = new YoGraphicReferenceFrame(gridFrame, registry, true, 0.5, YoAppearance.Blue());
   private final YoGraphicCoordinateSystem currentEvaluationPose = new YoGraphicCoordinateSystem("currentEvaluationPose", "", registry, true, 0.15,
                                                                                                 YoAppearance.HotPink());
   private final YoGraphicPosition currentEvaluationPosition = new YoGraphicPosition("currentEvaluationPosition", "", registry, 0.0125,
                                                                                     YoAppearance.DeepPink());

   public ReachabilitySphereMapCalculator(OneDoFJointBasics[] robotArmJoints, SimulationConstructionSet scs)
   {
      this.scs = scs;
      solver = new ReachabilityMapSolver(robotArmJoints, null, registry);

      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), robotArmJoints[0].getFrameBeforeJoint().getTransformToWorldFrame());
      gridFramePose.appendTranslation(getGridSizeInMeters() / 2.5, 0.0, 0.0);
      setGridFramePose(gridFramePose);

      scs.addStaticLinkGraphics(ReachabilityMapTools.createReachibilityColorScale());

      scs.addYoGraphic(gridFrameViz);
      scs.addYoGraphic(currentEvaluationPose);
      scs.addYoGraphic(currentEvaluationPosition);
      scs.addYoRegistry(registry);
   }

   /**
    * Configure the space and resolution for the exploration of the arm reachability.
    * 
    * @param gridSizeInNumberOfVoxels the region explored is a cube of size
    *           {@code gridSizeInNumberOfVoxels * vozelSize}.
    * @param voxelSize directly relates to the resolution of the exploration.
    * @param numberOfRays sets for each voxel the number of directions that are to be explored with
    *           the x-axis.
    * @param numberOfRotationsAroundRay sets the number of rotations to explore about each ray. The
    *           total number of orientations explored at each voxel is
    *           {@code numberOfRays * numberOfRotationsAroundRay}.
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
    * It is recommended to align the x-axis of the control frame with the vector that is orthogonal
    * to the palm.
    * </p>
    * 
    * @param controlFramePose the transform from the control frame with respect to the
    *           end-effector's body-fixed frame.
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
    * @param pose the pose of the grid expressed in world.
    */
   public void setGridFramePose(FramePose3D pose)
   {
      pose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      gridFrame.setPoseAndUpdate(pose);
   }

   /**
    * Sets up the calculator so it exports the result in an Excel file. That file can later be
    * loaded using {@link ReachabilityMapFileLoader}.
    * 
    * @param robotName the robot name.
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

   /**
    * Attaches a listener that is to be notified every time a pose is successfully reached. Can be
    * used to visualize to robot doing something, decrease boredom when watching progress.
    * 
    * @param listener the listener to attach.
    */
   public void attachReachabilityMapListener(ReachabilityMapListener listener)
   {
      reachabilityMapListeners.add(listener);
   }

   private void initialize()
   {
      sphereVoxelShape = new SphereVoxelShape(gridFrame, voxelSize, numberOfRays, numberOfRotationsAroundRay, SphereVoxelType.graspOrigin);
      voxel3dGrid = new Voxel3DGrid(gridFrame, sphereVoxelShape, gridSizeInNumberOfVoxels, voxelSize);
      if (reachabilityMapFileWriter != null)
         reachabilityMapFileWriter.initialize(solver.getRobotArmJoints(), voxel3dGrid);
      gridFrameViz.update();
      scs.addStaticLinkGraphics(ReachabilityMapTools.createBoundingBoxGraphics(voxel3dGrid.getMinPoint(), voxel3dGrid.getMaxPoint()));
   }

   /**
    * Starts the exploration of the reachable space of the arm. This can take a looooooong time.
    */
   public void buildReachabilitySpace()
   {
      initialize();

      FrameVector3D translationFromVoxelOrigin = new FrameVector3D();
      FrameQuaternion orientation = new FrameQuaternion();

      for (int xIndex = 0; xIndex < gridSizeInNumberOfVoxels; xIndex++)
      {
         for (int yIndex = 0; yIndex < gridSizeInNumberOfVoxels; yIndex++)
         {
            for (int zIndex = 0; zIndex < gridSizeInNumberOfVoxels; zIndex++)
            {
               if (!isPositionReachable(xIndex, yIndex, zIndex))
                  continue;

               for (int rayIndex = 0; rayIndex < numberOfRays; rayIndex++)
               {
                  voxel3dGrid.getVoxel(voxelLocation, xIndex, yIndex, zIndex);

                  for (int rotationAroundRayIndex = 0; rotationAroundRayIndex < numberOfRotationsAroundRay; rotationAroundRayIndex++)
                  {
                     modifiableVoxelLocation.setIncludingFrame(voxelLocation);
                     sphereVoxelShape.getPose(translationFromVoxelOrigin, orientation, rayIndex, rotationAroundRayIndex);
                     modifiableVoxelLocation.add(translationFromVoxelOrigin);

                     modifiableVoxelLocation.changeFrame(ReferenceFrame.getWorldFrame());
                     orientation.changeFrame(ReferenceFrame.getWorldFrame());
                     currentEvaluationPose.setPose(new FramePose3D(modifiableVoxelLocation, orientation));
                     currentEvaluationPose.update();

                     boolean success = solver.solveFor(modifiableVoxelLocation, orientation);

                     if (success)
                     {
                        voxel3dGrid.registerReachablePose(xIndex, yIndex, zIndex, rayIndex, rotationAroundRayIndex);
                        if (reachabilityMapFileWriter != null)
                           reachabilityMapFileWriter.registerReachablePose(xIndex, yIndex, zIndex, rayIndex, rotationAroundRayIndex);

                        for (int i = 0; i < reachabilityMapListeners.size(); i++)
                        {
                           reachabilityMapListeners.get(i).hasReachedNewConfiguration();
                        }

                        scs.tickAndUpdate();
                        break;
                     }
                  }
               }

               double reachabilityValue = voxel3dGrid.getD(xIndex, yIndex, zIndex);

               if (reachabilityValue > 1e-3)
               {
                  Graphics3DObject voxelViz = sphereVoxelShape.createVisualization(voxelLocation, 0.25, reachabilityValue);
                  scs.addStaticLinkGraphics(voxelViz);
               }
            }
         }
      }

      if (reachabilityMapFileWriter != null)
         reachabilityMapFileWriter.exportAndClose();
      System.out.println("Done!");
   }

   private boolean isPositionReachable(int xIndex, int yIndex, int zIndex)
   {
      voxel3dGrid.getVoxel(voxelLocation, xIndex, yIndex, zIndex);
      modifiableVoxelLocation.setIncludingFrame(voxelLocation);
      modifiableVoxelLocation.changeFrame(ReferenceFrame.getWorldFrame());
      currentEvaluationPosition.setPosition(modifiableVoxelLocation);
      currentEvaluationPosition.update();

      return solver.solveFor(voxelLocation);
   }

   public double getGridSizeInMeters()
   {
      return gridSizeInNumberOfVoxels * voxelSize;
   }
}
