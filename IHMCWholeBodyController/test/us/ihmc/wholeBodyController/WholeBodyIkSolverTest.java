package us.ihmc.wholeBodyController;

import java.util.ArrayList;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.FullRobotModelVisualizer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;

public abstract class WholeBodyIkSolverTest
{
   private SimulationTestingParameters simulationTestingParameters;
   private SimulationConstructionSet scs;
   private final boolean USE_RANDOM_START_LOCATIONS = true;

   public abstract WholeBodyIkSolverTestHelper getWholeBodyIkSolverTestHelper();

   public abstract ArrayList<Pair<FramePose, FramePose>> generatePointsForRegression(RobotSide robotSide, int numberOfPoints);

   public abstract ArrayList<Pair<FramePose, FramePose>> generatePointsForRegression(int numberOfPoints);

   public abstract ArrayList<Matrix4d> getRightHandToWorldArray();

   public abstract ArrayList<Matrix4d> getLeftHandToWorldArray();

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

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.LEFT,
                                                                 getLeftHandToWorldArray());
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

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.RIGHT,
                                                                 getRightHandToWorldArray());
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

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createHalfCylinderOfTargetPoints(RobotSide.RIGHT);
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

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.LEFT,
                                                                 getLeftHandToWorldArray());
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

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createHalfCylinderOfTargetPoints(RobotSide.LEFT);
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }



   @Ignore    // 3/6/15: test is failing
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
      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.RIGHT,
                                                                 getRightHandToWorldArray());
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
      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.LEFT,
                                                                 getLeftHandToWorldArray());
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
      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.RIGHT,
                                                                 getRightHandToWorldArray());
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
      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getLeftHandToWorldArray(),
                                                                 getRightHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_3P, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   // May FAIL
   @EstimatedDuration(duration = 7.0)
   @Test(timeout = 20000)
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

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getLeftHandToWorldArray(),
                                                                 getRightHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_3P2R, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   @Ignore    // 3/6/15: test is failing
   @EstimatedDuration(duration = 6.0)
   @Test(timeout = 20000)
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

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(getLeftHandToWorldArray(),
                                                                 getRightHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_3P3R, handTargetArray, false, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   // @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testLeftHandIn3PModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

//    ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.generatePointsForRegression(RobotSide.LEFT);
      ArrayList<Pair<FramePose, FramePose>> handTargetArray = this.generatePointsForRegression(RobotSide.LEFT,
                                                                 wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   // @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testRightHandIn3PModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = this.generatePointsForRegression(RobotSide.RIGHT,
                                                                 wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }




   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testBothHandsIn3PModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = this.generatePointsForRegression(wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_3P, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   // @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testBothtHandsIn3P2RModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = generatePointsForRegression(wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());

      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_3P2R, handTargetArray, true, USE_RANDOM_START_LOCATIONS);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

   // @Ignore
   @EstimatedDuration(duration = 45.0)
   @Test(timeout = 200000)
   public void testBothHandsIn3P3RModeRegression()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper();

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = generatePointsForRegression(wholeBodyTrajectoryTestHelper.getNumberOfRegressionPoses());

      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_3P3R, handTargetArray, true, USE_RANDOM_START_LOCATIONS);
   }


}
