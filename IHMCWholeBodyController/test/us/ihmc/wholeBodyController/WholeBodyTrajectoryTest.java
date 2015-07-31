package us.ihmc.wholeBodyController;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.dataProcessors.RobotAllJointsDataChecker;
import us.ihmc.utilities.MemoryTools;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.math.trajectories.TrajectoryND;
import us.ihmc.utilities.math.trajectories.TrajectoryND.WaypointND;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeOption;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ComputeResult;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicShape;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

public abstract class WholeBodyTrajectoryTest
{
   public abstract WholeBodyTrajectoryTestHelper getWholeBodyTrajectoryTestHelper();

   private SimulationTestingParameters simulationTestingParameters;
   private SimulationConstructionSet scs;
   private FullRobotModelVisualizer modelVisualizer;
   
   private final static int MAX_NUMBER_OF_RESEEDS = 6;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }

      modelVisualizer = null;
      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Ignore
   @EstimatedDuration(duration = 4.1)
   @Test(timeout = 22445)
   public void testTrajectory() throws Exception
   {
      WholeBodyTrajectoryTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyTrajectoryTestHelper();

      SDFFullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualRobotModel();
      SDFFullRobotModel desiredRobotModel = wholeBodyTrajectoryTestHelper.getRobotModel().createFullRobotModel();
      WholeBodyIkSolver wbSolver = wholeBodyTrajectoryTestHelper.getWholeBodyIkSolver();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);
      scs.startOnAThread();

      scs.maximizeMainWindow();
      modelVisualizer.update(0);


      wbSolver.setVerbosityLevel((scs != null) ? 1 : 0);

      wbSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.LEFT, ControlledDoF.DOF_NONE);
      wbSolver.getConfiguration().setNumberOfControlledDoF(RobotSide.RIGHT, ControlledDoF.DOF_3P);

      // ReferenceFrame soleFrame = actualRobotModel.getSoleFrame(RobotSide.RIGHT);

      ArrayList<Point3d> targetListRight = new ArrayList<Point3d>();

      // FramePose targetL = new FramePose(soleFrame, new Point3d( 0.5, 0.4, 0.8 ), new Quat4d() );

      targetListRight.add(new Point3d(0.4, -0.6, 1.4));
      targetListRight.add(new Point3d(0.4, -0.6, 0.8));
      targetListRight.add(new Point3d(0.5, 0.6, 1.0));
      targetListRight.add(new Point3d(0.5, -0.0, 1.0));
      targetListRight.add(new Point3d(0.5, -0.4, 1.4));
      targetListRight.add(new Point3d(0.5, -0.2, 0.6));

      double maximumJointVelocity = 1.5;
      double maximumJointAcceleration = 15.0;


      for (Point3d rightTarget : targetListRight)
      {
         for (int a = 0; a < 50; a++)
         {
            modelVisualizer.update(0);
         }


         FramePose targetR = new FramePose(ReferenceFrame.getWorldFrame(), rightTarget, new Quat4d());


         visualizePoint(0.04, YoAppearance.Green(), targetR);


         // wbSolver.setGripperPalmTarget(actualRobotModel, RobotSide.LEFT,  targetL );
         wbSolver.setGripperPalmTarget( RobotSide.RIGHT, targetR);

         ComputeResult ret = wbSolver.compute(actualRobotModel, desiredRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);


         if (ret == ComputeResult.SUCCEEDED)
         {
            WholeBodyTrajectory trajectoryGenerator = new WholeBodyTrajectory(actualRobotModel, maximumJointVelocity, maximumJointAcceleration, 5.0);    // 0.10);
            TrajectoryND trajectory = trajectoryGenerator.createTaskSpaceTrajectory(wbSolver, actualRobotModel, desiredRobotModel);

            ImmutablePair<Boolean, WaypointND> result = trajectory.getNextInterpolatedPoints(0.01);

            while (result.getLeft().booleanValue() == false)
            {
               HashMap<String, Double> angles = new HashMap<String, Double>();
               HashMap<String, Double> velocities = new HashMap<String, Double>();
               int index = 0;
               for (OneDoFJoint joint : actualRobotModel.getOneDoFJoints())
               {
                  if (wbSolver.hasJoint(joint.getName()))
                  {
                     angles.put(joint.getName(), result.getRight().position[index]);
                     velocities.put(joint.getName(), result.getRight().velocity[index]);
                     index++;
                  }
               }

               actualRobotModel.updateJointsStateButKeepOneFootFixed(angles, velocities, RobotSide.RIGHT);
               result = trajectory.getNextInterpolatedPoints(0.01);


               modelVisualizer.update(0);

            }

         }
         else
         {
            fail("no solution found\n");
         }


         for (int a = 0; a < 50; a++)
         {
            modelVisualizer.update(0);
         }

      }

      // value test
      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();

      Robot robot = scs.getRobots()[0];
      RobotAllJointsDataChecker robotAllJointsDataChecker = new RobotAllJointsDataChecker(scs, robot);
      robotAllJointsDataChecker.setMaximumDerivativeForAllJoints(1.01 * maximumJointVelocity);
      robotAllJointsDataChecker.setMaximumSecondDerivativeForAllJoints(1.01 * maximumJointAcceleration);

      scs.applyDataProcessingFunction(robotAllJointsDataChecker);
   }

   //@Ignore
   @EstimatedDuration(duration = 4.1)
   @Test(timeout = 22445)
   public void testPointToPointRight() throws Exception
   {
      WholeBodyTrajectoryTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyTrajectoryTestHelper();

      SDFFullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualRobotModel();
      SDFFullRobotModel desiredRobotModel = wholeBodyTrajectoryTestHelper.getRobotModel().createFullRobotModel();
      WholeBodyIkSolver wbSolver = wholeBodyTrajectoryTestHelper.getWholeBodyIkSolver();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);


      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);
      scs.startOnAThread();

      scs.maximizeMainWindow();
      modelVisualizer.update(0);

      wbSolver.setVerbosityLevel(0);
      wbSolver.getConfiguration().setMaxNumberOfAutomaticReseeds(MAX_NUMBER_OF_RESEEDS);

      RobotSide robotSide = RobotSide.RIGHT;

      wbSolver.getConfiguration().setNumberOfControlledDoF(robotSide.getOppositeSide(), ControlledDoF.DOF_NONE);
      wbSolver.getConfiguration().setNumberOfControlledDoF(robotSide, ControlledDoF.DOF_3P);

      ArrayList<Point3d> targetList = new ArrayList<Point3d>();

      // FramePose targetL = new FramePose(soleFrame, new Point3d( 0.5, 0.4, 0.8 ), new Quat4d() );

      // For now, use a set of hard corded points. It would be better to have random points to chos from
      double sideMultiplier = 1.0;
      robotSide.negateIfLeftSide(sideMultiplier);

      targetList.add(new Point3d(0.4, -0.6 * sideMultiplier, 1.4));
      targetList.add(new Point3d(0.4, -0.6 * sideMultiplier, 0.8));
      targetList.add(new Point3d(0.5, 0.6 * sideMultiplier, 1.0));
      targetList.add(new Point3d(0.5, -0.0 * sideMultiplier, 1.0));
      targetList.add(new Point3d(0.5, -0.4 * sideMultiplier, 1.4));
      targetList.add(new Point3d(0.5, -0.2 * sideMultiplier, 0.6));

      double maximumJointVelocity = 1.5;
      double maximumJointAcceleration = 15.0;

      for (Point3d target : targetList)
      {
         modelVisualizer.update(0);

         FramePose poseFrameTarget = new FramePose(ReferenceFrame.getWorldFrame(), target, new Quat4d());

         visualizePoint(0.04, YoAppearance.Green(), poseFrameTarget);

         wbSolver.setGripperPalmTarget( robotSide, poseFrameTarget);

         ComputeResult ret = wbSolver.compute(actualRobotModel, desiredRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);

         if (ret == ComputeResult.SUCCEEDED)
         {
            WholeBodyTrajectory trajectoryGenerator = new WholeBodyTrajectory(actualRobotModel, maximumJointVelocity, maximumJointAcceleration, 0.10);
            TrajectoryND trajectory = trajectoryGenerator.createTaskSpaceTrajectory(wbSolver, actualRobotModel, desiredRobotModel);

            ImmutablePair<Boolean, WaypointND> result = trajectory.getNextInterpolatedPoints(0.01);

            while (result.getLeft().booleanValue() == false)
            {
               HashMap<String, Double> angles = new HashMap<String, Double>();
               HashMap<String, Double> velocities = new HashMap<String, Double>();
               int index = 0;
               for (OneDoFJoint joint : actualRobotModel.getOneDoFJoints())
               {
                  if (wbSolver.hasJoint(joint.getName()))
                  {
                     angles.put(joint.getName(), result.getRight().position[index]);
                     velocities.put(joint.getName(), result.getRight().velocity[index]);
                     index++;
                  }
               }

               actualRobotModel.updateJointsStateButKeepOneFootFixed(angles, velocities, RobotSide.RIGHT);
               result = trajectory.getNextInterpolatedPoints(0.01);

               modelVisualizer.update(0);
            }
         }
         else
         {
            fail("no solution found\n");
         }

         modelVisualizer.update(0);
      }

      // value test
      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();

      Robot robot = scs.getRobots()[0];
      RobotAllJointsDataChecker robotAllJointsDataChecker = new RobotAllJointsDataChecker(scs, robot);
      robotAllJointsDataChecker.setMaximumDerivativeForAllJoints(1.2 * maximumJointVelocity);
      robotAllJointsDataChecker.setMaximumSecondDerivativeForAllJoints(1.2 * maximumJointAcceleration);

      scs.applyDataProcessingFunction(robotAllJointsDataChecker);

      assertFalse(robotAllJointsDataChecker.hasDerivativeComparisonErrorOccurredAnyJoint());
      assertFalse(robotAllJointsDataChecker.hasMaxDerivativeExeededAnyJoint());
     // assertFalse(robotAllJointsDataChecker.hasMaxSecondDerivativeExeededAnyJoint());
      assertFalse(robotAllJointsDataChecker.hasMaxValueExeededAnyJoint());
      assertFalse(robotAllJointsDataChecker.hasMinValueExeededAnyJoint());
   }

   //@Ignore
   @EstimatedDuration(duration = 4.1)
   @Test(timeout = 22445)
   public void testPointToPointLeft() throws Exception
   {
      WholeBodyTrajectoryTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyTrajectoryTestHelper();

      SDFFullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualRobotModel();
      SDFFullRobotModel desiredRobotModel = wholeBodyTrajectoryTestHelper.getRobotModel().createFullRobotModel();
      WholeBodyIkSolver wbSolver = wholeBodyTrajectoryTestHelper.getWholeBodyIkSolver();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);
      scs.startOnAThread();

      scs.maximizeMainWindow();
      modelVisualizer.update(0);

      wbSolver.setVerbosityLevel(0);
      wbSolver.getConfiguration().setMaxNumberOfAutomaticReseeds(MAX_NUMBER_OF_RESEEDS);

      RobotSide robotSide = RobotSide.LEFT;
      
      wbSolver.getConfiguration().setNumberOfControlledDoF(robotSide.getOppositeSide(), ControlledDoF.DOF_NONE);
      wbSolver.getConfiguration().setNumberOfControlledDoF(robotSide, ControlledDoF.DOF_3P);

      ArrayList<Point3d> targetList = new ArrayList<Point3d>();

      // FramePose targetL = new FramePose(soleFrame, new Point3d( 0.5, 0.4, 0.8 ), new Quat4d() );

      // For now, use a set of hard corded points. It would be better to have random points to choose from
      double sideMultiplier = 1.0;
      robotSide.negateIfLeftSide(sideMultiplier);

      targetList.add(new Point3d(0.4, -0.6 * sideMultiplier, 1.4));
      targetList.add(new Point3d(0.4, -0.6 * sideMultiplier, 0.8));
      targetList.add(new Point3d(0.5,  0.6 * sideMultiplier, 1.0));
      targetList.add(new Point3d(0.5, -0.0 * sideMultiplier, 1.0));
      targetList.add(new Point3d(0.5, -0.4 * sideMultiplier, 1.4));
      targetList.add(new Point3d(0.5, -0.2 * sideMultiplier, 0.6));

      double maximumJointVelocity = 1.5;
      double maximumJointAcceleration = 15.0;

      for (Point3d target : targetList)
      {
         modelVisualizer.update(0);

         FramePose poseFrameTarget = new FramePose(ReferenceFrame.getWorldFrame(), target, new Quat4d());

         visualizePoint(0.04, YoAppearance.Green(), poseFrameTarget);

         wbSolver.setGripperPalmTarget(robotSide, poseFrameTarget);

         ComputeResult ret = wbSolver.compute(actualRobotModel, desiredRobotModel, ComputeOption.USE_ACTUAL_MODEL_JOINTS);

         if (ret == ComputeResult.SUCCEEDED)
         {
            WholeBodyTrajectory trajectoryGenerator = new WholeBodyTrajectory(actualRobotModel, maximumJointVelocity, maximumJointAcceleration, 0.1);
            TrajectoryND trajectory = trajectoryGenerator.createTaskSpaceTrajectory(wbSolver, actualRobotModel, desiredRobotModel);

            ImmutablePair<Boolean, WaypointND> result = trajectory.getNextInterpolatedPoints(0.01);

            while (result.getLeft().booleanValue() == false)
            {
               HashMap<String, Double> angles = new HashMap<String, Double>();
               HashMap<String, Double> velocities = new HashMap<String, Double>();
               int index = 0;
               for (OneDoFJoint joint : actualRobotModel.getOneDoFJoints())
               {
                  if (wbSolver.hasJoint(joint.getName()))
                  {
                     angles.put(joint.getName(), result.getRight().position[index]);
                     velocities.put(joint.getName(), result.getRight().velocity[index]);
                     index++;
                  }
               }

               actualRobotModel.updateJointsStateButKeepOneFootFixed(angles, velocities, RobotSide.RIGHT);
               result = trajectory.getNextInterpolatedPoints(0.01);

               modelVisualizer.update(0);
            }
         }
         else {
            fail("no solution found\n");
         }

         modelVisualizer.update(0);
      }

      // value test
      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();

      Robot robot = scs.getRobots()[0];
      RobotAllJointsDataChecker robotAllJointsDataChecker = new RobotAllJointsDataChecker(scs, robot);
      robotAllJointsDataChecker.setMaximumDerivativeForAllJoints(1.2 * maximumJointVelocity);
      robotAllJointsDataChecker.setMaximumSecondDerivativeForAllJoints(1.2 * maximumJointAcceleration);

      scs.applyDataProcessingFunction(robotAllJointsDataChecker);

      assertFalse(robotAllJointsDataChecker.hasDerivativeComparisonErrorOccurredAnyJoint());
      assertFalse(robotAllJointsDataChecker.hasMaxDerivativeExeededAnyJoint());
    //  assertFalse(robotAllJointsDataChecker.hasMaxSecondDerivativeExeededAnyJoint());
      assertFalse(robotAllJointsDataChecker.hasMaxValueExeededAnyJoint());
      assertFalse(robotAllJointsDataChecker.hasMinValueExeededAnyJoint());


   }



   static int count = 0;

   private void visualizePoint(double radius, AppearanceDefinition color, FramePose spherePose)
   {
      FramePose pose = new FramePose(spherePose);
      count++;
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(radius, color);

      YoVariableRegistry registry = new YoVariableRegistry("WholeBodyIkSolverTestFactory_Registry");

      YoFramePoint framePoint = new YoFramePoint("point" + count, ReferenceFrame.getWorldFrame(), registry);
      YoFrameOrientation frameOrientation = new YoFrameOrientation("orientation" + count, ReferenceFrame.getWorldFrame(), registry);
      YoGraphicShape yoGraphicsShape = new YoGraphicShape("target" + count, linkGraphics, framePoint, frameOrientation, 1.0);

      RigidBodyTransform transform = new RigidBodyTransform();
      pose.changeFrame(ReferenceFrame.getWorldFrame());
      pose.getRigidBodyTransform(transform);

      yoGraphicsShape.setTransformToWorld(transform);

      scs.addYoGraphic(yoGraphicsShape);
   }

}
