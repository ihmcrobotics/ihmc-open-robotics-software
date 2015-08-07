package us.ihmc.wholeBodyController;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.utilities.MemoryTools;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;

public abstract class WholeBodyIkSolverTest
{
   private SimulationTestingParameters simulationTestingParameters;
   private SimulationConstructionSet scs;
   private final boolean USE_RANDOM_START_LOCATIONS = true;

   public abstract WholeBodyIkSolverTestHelper getWholeBodyIkSolverTestHelper();

   public abstract ArrayList<ImmutablePair<FramePose, FramePose>> generatePointsForRegression(RobotSide robotSide, int numberOfPoints);
   public abstract ArrayList<ImmutablePair<FramePose, FramePose>> generatePointsForRegression(int numberOfPoints);
   
   public abstract ArrayList<SideDependentList<RigidBodyTransform>>  getHandToRootArray();

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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @EstimatedDuration(duration = 5.0)
   @Test(timeout = 20000)
   public void testLeftHandIn3P3RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();

      // modelVisualizer.update(0);

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getHandToRootArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_NONE, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   @EstimatedDuration(duration = 5.0)
   @Test(timeout = 20000)
   public void testRightHandIn3PModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray =  wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getHandToRootArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

   @EstimatedDuration(duration = 6.0)
   @Test(timeout = 20000)
   public void testRightHandIn3PMode()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createHalfCylinderOfTargetPoints(RobotSide.RIGHT);
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   @EstimatedDuration(duration = 5.0)
   @Test(timeout = 20000)
   public void testLeftHandIn3PModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray =  wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getHandToRootArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

   @EstimatedDuration(duration = 6.0)
   @Test(timeout = 20000)
   public void testLeftHandIn3PMode()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createHalfCylinderOfTargetPoints(RobotSide.LEFT);
      
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }




   @EstimatedDuration(duration = 5.0)
   @Test(timeout = 20000)
   public void testRightHandIn3P2RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();
      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray =  wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getHandToRootArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P2R, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

// PASS
   @EstimatedDuration(duration = 5.0)
   @Test(timeout = 20000)
   public void testLeftHandIn3P2RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();
      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray =  wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getHandToRootArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_NONE, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

// PASS
   @EstimatedDuration(duration = 5.0)
   @Test(timeout = 20000)
   public void testRightHandIn3P3RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();
      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray =  wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getHandToRootArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P3R, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


// PASS 
   @EstimatedDuration(duration = 5.0)
   @Test(timeout = 20000)
   public void testBothHandsIn3PModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();
      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray =  wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getHandToRootArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_3P, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   // May FAIL
   @EstimatedDuration(duration = 7.0)
   @Test(timeout = 30000)
   public void testBothHandsIn3P2RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray =  wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getHandToRootArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(
            ControlledDoF.DOF_3P2R, ControlledDoF.DOF_3P2R, 
            handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }



   @EstimatedDuration(duration = 6.0)
   @Test(timeout = 30000)
   public void testBothHandsIn3P3RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray =  wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getHandToRootArray());
      
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_3P3R, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testLeftHandIn3PModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();
      
      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray = this.generatePointsForRegression(RobotSide.LEFT,
               wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());
         
      if( handTargetArray == null)
      {
         return; // skip test
      }

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

//    ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.generatePointsForRegression(RobotSide.LEFT);

      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testRightHandIn3PModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray = this.generatePointsForRegression(RobotSide.RIGHT,
            wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());
      
      if( handTargetArray == null)
      {
         return; // skip test
      }
      
      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();
      
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }



   @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testBothHandsIn3PModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray = this.generatePointsForRegression(wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());
      
      if( handTargetArray == null)
      {
         return; // skip test
      }
      
      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();
   
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_3P, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testBothtHandsIn3P2RModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray = generatePointsForRegression(wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());

      if( handTargetArray == null)
      {
         return; // skip test
      }

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_3P2R, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

   @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testBothHandsIn3P3RModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray = generatePointsForRegression(wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());

      if( handTargetArray == null)
      {
         return; // skip test
      }
      
      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_3P3R, handTargetArray, true, USE_RANDOM_START_LOCATIONS);
   }

   @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testWorkingSpaceFor5DoF()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      ArrayList<ImmutablePair<FramePose, FramePose>> handTargetArray = new  ArrayList<ImmutablePair<FramePose, FramePose>> ();

      Point3d point = new Point3d();
      Quat4d  orientation = new Quat4d( 0, 0, -0.7, 0.7); 
      orientation.normalize();
      
      for (double x = 0.3; x<= 0.8; x += 0.1 )
      {
         for (double y = -0.5; y<= 0.5; y += 0.1 )
         {
            for (double z = 0.5; z<= 1.9; z += 0.1 )
            {
               FramePose handTarget = new FramePose();
               point.set( x,y,z);
               handTarget.setPoseIncludingFrame( ReferenceFrame.getWorldFrame(), point, orientation);
               handTargetArray.add( new ImmutablePair<FramePose, FramePose>(handTarget, null));
            }
         }
      }

      
      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_NONE, handTargetArray, true, false);
   }
   

}
