package us.ihmc.avatar.reachabilityMap;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.IntStream;

import us.ihmc.avatar.reachabilityMap.Voxel3DGrid.Voxel3DData;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
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
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * <p>
 * This class computes the reachability sphere for a robot's end effector, giving a number of reachability measures for that envelope. This consists of a
 * discretized space, where the score is computed at each discrete point in that space.
 * </p>
 * <p>
 * The robot itself is contained in {@link ReachabilityMapRobotInformation}. This consists of a {@link RobotDefinition} in the
 * {@link ReachabilityMapRobotInformation#robotDefinition}, name for the "base link" contained in {@link ReachabilityMapRobotInformation#baseName}, and name for
 * the "end effector" contained in {@link ReachabilityMapRobotInformation#endEffectorName}. This defines what the reachability will be calculated for. This
 * reachability score will respect the joint limits defined in the {@link ReachabilityMapRobotInformation#robotDefinition}. Additionally, it will consider any
 * collision models defined in {@link ReachabilityMapRobotInformation#robotDefinition}. A user can also add additional collisions that the robot should consider
 * using {@link #addRobotCollisionModel(RobotDefinition)}, including potential external objects.
 * </p>
 * <p>
 * The discretization of the environment is defined in the {@link #voxel3DGrid}, which can be set using {@link #setVoxel3DGrid(Voxel3DGrid)}. This consists of a
 * 3D grid in the world, with each edge of a length of {@link Voxel3DGrid#getGridSizeMeters()}, with {@link Voxel3DGrid#getGridSizeVoxels()} voxels along each
 * axis. Then, at each point, there are a number of different orientation axes defined in the object {@link Voxel3DGrid#getSphereVoxelShape()}, which has the
 * field {@link SphereVoxelShape#getNumberOfRays()}, which dictate the alignment of the hand for grasping an object. This is then further discretized with
 * rotations about each ray, which can be seen at {@link SphereVoxelShape#getNumberOfRotationsAroundRay()}.
 * </p>
 * <p>
 * The score of reachability is broken into two separate metrics: R and R2. Each of these is computed for each voxel in the grid, and then stored in the
 * {@link #voxel3DGrid} object, which can be accessed using {@link #getVoxel3DGrid()} and then {@link Voxel3DGrid#getVoxel(int)} and then
 * {@link Voxel3DData#getR()} or {@link Voxel3DData#getR2()}.
 * </p>
 * <p>
 * The R metric is the ratio of the axis-aligned pose of the end effector compared to the total number of rays to be considered. That is, it checks to see if
 * the end effector can reach the position of the voxel while staying axially aligned with a ray along pointed towards that voxel. The end-effector is allowed
 * to freely rotate around the ray, but must be aligned with it. This is useful for grasping objects that are aligned with the robot's end effector. The ratio
 * comes in by determining the ratio of rays that can be achieved to the total number of rays that were attempted.
 * </p>
 * <p>
 * The R2 metric is similar to the R metric, but removes the freedom of the end-effector to rotate around the ray. Instead, it checks to see if the end-effector
 * can fully achieve the pose, both position and orientation, of the voxel.
 * </p>
 */
public class ReachabilitySphereMapCalculator implements Controller
{
   private static final boolean VISUALIZE_ALL_SOLVERS = false;

   private final ControllerOutput controllerOutput;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private Voxel3DGrid voxel3DGrid;

   private Consumer<Voxel3DData> voxelUnreachableListener = null;
   private Consumer<Voxel3DData> voxelCompletedListener = null;

   private final FramePose3D[] solverInputs;
   private final ReachabilityMapSolver[] solvers;
   private final boolean[] solverResults;
   private final Voxel3DData[] solverVoxels;

   private final YoFramePose3D gridFramePose = new YoFramePose3D("gridFramePose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D evaluatedPose = new YoFramePose3D("evaluatedPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final YoBoolean evaluateRReachability = new YoBoolean("evaluateRReachability", registry);
   private final YoBoolean evaluateR2Reachability = new YoBoolean("evaluateR2Reachability", registry);

   private final List<YoGraphicGroupDefinition> solverYoGraphicGroupDefinitions = new ArrayList<>();

   public ReachabilitySphereMapCalculator(ReachabilityMapRobotInformation robotInformation, ControllerOutput controllerOutput, int numberOfThreads)
   {
      this.controllerOutput = controllerOutput;

      solvers = new ReachabilityMapSolver[numberOfThreads];
      solverResults = new boolean[numberOfThreads];
      solverVoxels = new Voxel3DData[numberOfThreads];
      solverInputs = new FramePose3D[numberOfThreads];

      RobotDefinition robotDefinition = robotInformation.getRobotDefinition();
      ReferenceFrame robotRootFrame = Robot.createRobotRootFrame(robotDefinition, ReferenceFrame.getWorldFrame()); // This allows to visualize YoGraphics in frames other than world
      RigidBodyBasics rootBody = robotDefinition.newInstance(robotRootFrame);
      RigidBodyBasics base = MultiBodySystemTools.findRigidBody(rootBody, robotInformation.getBaseName());
      RigidBodyBasics endEffector = MultiBodySystemTools.findRigidBody(rootBody, robotInformation.getEndEffectorName());
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(base, endEffector);

      OneDoFJointBasics firstJoint = armJoints[0];
      for (int i = 0; i < numberOfThreads; i++)
      {
         YoRegistry solverRegistry = new YoRegistry("solverRegistry" + i);
         registry.addChild(solverRegistry);

         OneDoFJointBasics[] solverJoints;
         String cloneSuffix;

         if (i == 0)
         {
            solverJoints = armJoints;
            cloneSuffix = "";
         }
         else
         {
            RigidBodyBasics originalRootBody = MultiBodySystemTools.getRootBody(firstJoint.getPredecessor());
            cloneSuffix = "-solver" + i;
            RigidBodyBasics solverRootBody = MultiBodySystemFactories.cloneMultiBodySystem(originalRootBody, ReferenceFrame.getWorldFrame(), cloneSuffix);
            solverJoints = Arrays.stream(armJoints)
                                 .map(originalJoint -> MultiBodySystemTools.findJoint(solverRootBody, originalJoint.getName() + cloneSuffix))
                                 .toArray(OneDoFJointBasics[]::new);
         }

         YoGraphicsListRegistry solverGraphicsRegistry = new YoGraphicsListRegistry();
         solvers[i] = new ReachabilityMapSolver(cloneSuffix, solverJoints, solverGraphicsRegistry, solverRegistry);

         if (i == 0 || VISUALIZE_ALL_SOLVERS)
         {
            YoGraphicGroupDefinition solverYoGraphicGroup = new YoGraphicGroupDefinition("solver" + i);
            solverYoGraphicGroup.setChildren(YoGraphicConversionTools.toYoGraphicDefinitions(solverGraphicsRegistry));
            solverYoGraphicGroupDefinitions.add(solverYoGraphicGroup);
         }
         solverInputs[i] = new FramePose3D();
      }

      for (ReachabilityMapSolver solver : solvers)
      {
         solver.setControlFramePoseInParentJoint(robotInformation.getControlFramePoseInParentJoint());
         solver.setRayAxis(robotInformation.getOrthogonalToPalm());
      }

      gridFramePose.attachVariableChangedListener(v ->
      {
         if (voxel3DGrid != null)
            voxel3DGrid.setGridPose(gridFramePose);
      });
      FramePose3D gridFramePose = new FramePose3D(ReferenceFrame.getWorldFrame(), firstJoint.getFrameBeforeJoint().getTransformToRoot());
      setGridFramePose(gridFramePose);
   }

   /**
    * Adds a collision model for the underlying inverse kinematics solver to use. Using this, the robot will not return motions that result in self
    * collisions. To use this, the collisions must be defined as part of the robot definition. Note that this does not override the underlying robot model, but
    * simply copies the collisions from the argument into the internal model.
    *
    * <p>
    *    Note that this does not remove the previously added collision models, and simply adds to them.
    * </p>
    *
    * <p>
    * If the rigid body contained in the argument {@param robotDefinition} is not part of the robot model, its collision model is added as a static collidable
    * to the environment. This allows for adding external collidables (e.g. other robots) that must be avoided to the solver.
    * </p>
    * @param robotDefinition container for the collisions to use.
    */
   public void addRobotCollisionModel(RobotDefinition robotDefinition)
   {
      for (ReachabilityMapSolver solver : solvers)
      {
         solver.addCollisionModel(robotDefinition);
      }
   }

   /**
    * Please use {@link #addRobotCollisionModel(RobotDefinition)} instead. The method name was changed for clarity, but the old method was retained for backward
    * compatibility.
    */
   @Deprecated
   public void setRobotCollisionModel(RobotDefinition robotDefinition)
   {
      addRobotCollisionModel(robotDefinition);
   }

   public void setVoxel3DGrid(Voxel3DGrid voxel3DGrid)
   {
      this.voxel3DGrid = voxel3DGrid;
      voxel3DGrid.setGridPose(gridFramePose);
   }

   /**
    * Sets whether to evaluate R Reachability. R reachability evaluates whether the position is reachable by the end effector while aligning the end effector
    * along a ray. The R factor is the ratio of achievable rays and positions to the total number of rays.
    * @param enable if true, R reachability is evaluated, otherwise it is not.
    */
   public void setEvaluateRReachability(boolean enable)
   {
      evaluateRReachability.set(enable);
   }

   public void setEvaluateR2Reachability(boolean enable)
   {
      evaluateR2Reachability.set(enable);
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
      yoGraphics.add(newYoGraphicCoordinateSystem3D("gridFramePose", gridFramePose, 0.5, ColorDefinitions.Blue()));
      yoGraphics.add(newYoGraphicCoordinateSystem3D("evaluatedPose", evaluatedPose, 0.15, ColorDefinitions.HotPink()));
      yoGraphics.add(newYoGraphicCoordinateSystem3D("controlFrame", solvers[0].getControlFramePoseInEndEffector(), 0.05, ColorDefinitions.parse("#A1887F")));
      yoGraphics.addAll(solverYoGraphicGroupDefinitions);
      group.setChildren(yoGraphics);
      return group;
   }

   public OneDoFJointBasics[] getRobotArmJoints()
   {
      return solvers[0].getRobotArmJoints();
   }

   public FramePose3DReadOnly getControlFramePose()
   {
      return solvers[0].getControlFramePoseInEndEffector();
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

      Arrays.fill(solverResults, false);

      // This is solving in parallel, on a multi-threaded approach. Each solverIndex is equivalent to stating the thread index. It performs a single
      // batch-update on a new set of voxels, and packs whether the solve was successful into the solverResults array.
      IntStream.range(0, solvers.length).parallel().forEach(solverIndex ->
      {
         int voxelIndex = currentVoxelIndex.getValue() + solverIndex;
         if (voxelIndex >= voxel3DGrid.getNumberOfVoxels())
         {
            // We are out of voxels to test. There's no need to proceed.
            solverVoxels[solverIndex] = null;
            solverResults[solverIndex] = false;
            return;
         }

         Voxel3DData voxel = voxel3DGrid.getOrCreateVoxel(voxelIndex);
         solverVoxels[solverIndex] = voxel;

         // Set the solver input pose to the position of the voxel, with zero orientation. The orientation of the input is not currently used.
         FramePose3D solverInput = solverInputs[solverIndex];
         solverInput.setToZero(ReferenceFrame.getWorldFrame());
         solverInput.getPosition().setMatchingFrame(voxel.getPosition());

         if (solverIndex == 0)
         {
            // We are visualizing the solver on thread 0. Update the pose that's evaluated.
            evaluatedPose.set(solverInput);
         }

         // Compute the solution configuration to reach the desired voxel position. Success is false if the point is unreachable. This must respect the joint
         // limits and collisions that have been previously defined.
         boolean success = solvers[solverIndex].solveFor(solverInput.getPosition());
         solverResults[solverIndex] = success;

         if (success)
         {
            //
            voxel.registerReachablePosition(solverInput.getPosition(), solvers[solverIndex].getRobotArmJoints());
            if (solverIndex == 0)
               writeSolverSolutionToRobotForViz(solvers[solverIndex], controllerOutput); // Update visualization
            computeVoxel(voxel, solverIndex);
         }
      });

      for (int solverIndex = 0; solverIndex < solverResults.length; solverIndex++)
      {
         boolean solverResult = solverResults[solverIndex];

         if (solverResult)
         {
            if (voxelCompletedListener != null)
               voxelCompletedListener.accept(solverVoxels[solverIndex]);
         }
         else if (solverVoxels[solverIndex] != null)
         {
            if (voxelUnreachableListener != null)
               voxelUnreachableListener.accept(solverVoxels[solverIndex]);

            voxel3DGrid.destroy(solverVoxels[solverIndex]);
         }

         solverVoxels[solverIndex] = null;
      }

      currentVoxelIndex.add(solvers.length);

      if (currentVoxelIndex.getValue() >= voxel3DGrid.getNumberOfVoxels())
         isDone.set(true);
   }

   private void computeVoxel(Voxel3DData voxel, int solverIndex)
   {
      FramePose3D solverInput = solverInputs[solverIndex];
      ReachabilityMapSolver solver = solvers[solverIndex];

      SphereVoxelShape sphereVoxelShape = voxel3DGrid.getSphereVoxelShape();

      for (int rayIndex = 0; rayIndex < sphereVoxelShape.getNumberOfRays(); rayIndex++)
      {
         if (evaluateRReachability.getValue())
         {
            sphereVoxelShape.getPose(solverInput, rayIndex, 0);
            solverInput.getPosition().add(voxel.getPosition());
            solverInput.changeFrame(ReferenceFrame.getWorldFrame());

            if (solver.solveForRay(solverInput))
            {
               voxel.registerReachableRay(rayIndex, solverInput, solver.getRobotArmJoints());

               if (solverIndex == 0)
                  writeSolverSolutionToRobotForViz(solver, controllerOutput);
            }
            else
            {
               continue;
            }
         }

         if (evaluateR2Reachability.getValue())
         {
            for (int rotationAroundRayIndex = 0; rotationAroundRayIndex < sphereVoxelShape.getNumberOfRotationsAroundRay(); rotationAroundRayIndex++)
            {
               sphereVoxelShape.getPose(solverInput, rayIndex, rotationAroundRayIndex);
               solverInput.getPosition().add(voxel.getPosition());
               solverInput.changeFrame(ReferenceFrame.getWorldFrame());

               if (solverIndex == 0)
                  evaluatedPose.set(solverInput);

               if (solver.solveForPose(solverInput))
               {
                  voxel.registerReachablePose(rayIndex, rotationAroundRayIndex, solverInput, solver.getRobotArmJoints());

                  if (solverIndex == 0)
                     writeSolverSolutionToRobotForViz(solver, controllerOutput);
               }
            }
         }
      }
   }

   /**
    * Sets the solution configuration of the solver to the robot to allow it to update the visualizer.
    */
   public static void writeSolverSolutionToRobotForViz(ReachabilityMapSolver solver, ControllerOutput controllerOutput)
   {
      for (OneDoFJointBasics joint : solver.getRobotArmJoints())
      {
         controllerOutput.getOneDoFJointOutput(joint).setConfiguration(joint);
         controllerOutput.getOneDoFJointOutput(joint).setEffort(joint);
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
