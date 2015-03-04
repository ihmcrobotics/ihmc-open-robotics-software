package us.ihmc.wholeBodyController;

import java.util.ArrayList;

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
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;

public abstract class WholeBodyIkSolverTest
{
   private SimulationTestingParameters simulationTestingParameters;
   private SimulationConstructionSet scs;

   public abstract WholeBodyIkSolverTestHelper getWholeBodyIkSolverTestHelper(boolean generateRandomHandPoses);

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

   @AverageDuration(duration = 0.7)
   @Test(timeout = 12128)
   public void testLeftHandIn3P3RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

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
                                                                 wholeBodyTrajectoryTestHelper.getLeftHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_NONE, handTargetArray, false);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   @AverageDuration(duration = 0.6)
   @Test(timeout = 11715)
   public void testRightHandIn3PModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.RIGHT,
                                                                 wholeBodyTrajectoryTestHelper.getRightHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, handTargetArray, true);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

   @AverageDuration(duration = 2.3)
   @Test(timeout = 16751)
   public void testRightHandIn3PMode()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createHalfCylinderOfTargetPoints(RobotSide.RIGHT);
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, handTargetArray, true);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


   @AverageDuration(duration = 0.5)
   @Test(timeout = 11563)
   public void testLeftHandIn3PModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.LEFT,
                                                                 wholeBodyTrajectoryTestHelper.getLeftHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray, true);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

   @AverageDuration(duration = 2.5)
   @Test(timeout = 17630)
   public void testLeftHandIn3PMode()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();

      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createHalfCylinderOfTargetPoints(RobotSide.LEFT);
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray, true);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


// PASS
   @AverageDuration(duration = 0.7)
   @Test(timeout = 11977)
   public void testRightHandIn3P2RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();
      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.RIGHT,
                                                                 wholeBodyTrajectoryTestHelper.getRightHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P2R, handTargetArray, false);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

// PASS
   @AverageDuration(duration = 0.6)
   @Test(timeout = 11707)
   public void testLeftHandIn3P2RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();
      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.LEFT,
                                                                 wholeBodyTrajectoryTestHelper.getLeftHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_NONE, handTargetArray, false);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }

// PASS
   @AverageDuration(duration = 0.5)
   @Test(timeout = 11462)
   public void testRightHandIn3P3RModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();
      ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayListForOneHand(RobotSide.RIGHT,
                                                                 wholeBodyTrajectoryTestHelper.getRightHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P3R, handTargetArray, false);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }


// PASS 
   @AverageDuration(duration = 0.9)
   @Test(timeout = 12820)
   public void testBothHandsIn3PModeManual()
   {
      WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

      FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

      Vector3d rootPosition = new Vector3d(0, 0, 0.93);
      actualRobotModel.getRootJoint().setPosition(rootPosition);

      scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
      FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

      wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

      scs.startOnAThread();

      scs.maximizeMainWindow();
      ArrayList<Pair<FramePose, FramePose>> handTargetArray =
         wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(wholeBodyTrajectoryTestHelper.getLeftHandToWorldArray(),
            wholeBodyTrajectoryTestHelper.getRightHandToWorldArray());
      wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_3P, handTargetArray, true);

      scs.cropBuffer();
      scs.gotoInPointNow();
      scs.stepForwardNow(1);
      scs.setInPoint();
      scs.cropBuffer();
   }
   
   
 // May FAIL
 @AverageDuration(duration = 1.8)
 @Test(timeout = 15368)
 public void testBothHandsIn3P2RModeManual()
 {
    WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

    FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

    Vector3d rootPosition = new Vector3d(0, 0, 0.93);
    actualRobotModel.getRootJoint().setPosition(rootPosition);

    scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
    FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

    wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

    scs.startOnAThread();
    
    ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(wholeBodyTrajectoryTestHelper.getLeftHandToWorldArray(),
          wholeBodyTrajectoryTestHelper.getRightHandToWorldArray());
    wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_3P2R, handTargetArray, false);
    
    scs.cropBuffer();
    scs.gotoInPointNow();
    scs.stepForwardNow(1);
    scs.setInPoint();
    scs.cropBuffer();
 }

 //    FAILS
 @Ignore
 @AverageDuration
 @Test(timeout = 15000)
 public void testBothHandsIn3P3RModeManual()
 {
    WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(false);

    FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

    Vector3d rootPosition = new Vector3d(0, 0, 0.93);
    actualRobotModel.getRootJoint().setPosition(rootPosition);

    scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
    FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

    wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

    scs.startOnAThread();
    
    ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.createManualFramePosePairArrayList(wholeBodyTrajectoryTestHelper.getLeftHandToWorldArray(),
          wholeBodyTrajectoryTestHelper.getRightHandToWorldArray());
    wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_3P3R, handTargetArray, false);
    
    scs.cropBuffer();
    scs.gotoInPointNow();
    scs.stepForwardNow(1);
    scs.setInPoint();
    scs.cropBuffer();
 }
 
 
 //@Ignore
 @AverageDuration
 @Test(timeout = 120000)
 public void testLeftHandIn3PModeRegression()
 {
    WholeBodyIkSolverTestHelper wholeBodyTrajectoryTestHelper = getWholeBodyIkSolverTestHelper(true);

    FullRobotModel actualRobotModel = wholeBodyTrajectoryTestHelper.getActualFullRobotModel();

    Vector3d rootPosition = new Vector3d(0, 0, 0.93);
    actualRobotModel.getRootJoint().setPosition(rootPosition);

    scs = new SimulationConstructionSet(wholeBodyTrajectoryTestHelper.getRobot(), simulationTestingParameters);
    FullRobotModelVisualizer modelVisualizer = new FullRobotModelVisualizer(scs, actualRobotModel, 0.01);

    wholeBodyTrajectoryTestHelper.addGraphics(scs, modelVisualizer);

    scs.startOnAThread();
    
    ArrayList<Pair<FramePose, FramePose>> handTargetArray = wholeBodyTrajectoryTestHelper.generatePointsForRegression();
    wholeBodyTrajectoryTestHelper.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_NONE, handTargetArray, true , false);
    
    scs.cropBuffer();
    scs.gotoInPointNow();
    scs.stepForwardNow(1);
    scs.setInPoint();
    scs.cropBuffer();
 }
// 
// 
// @AverageDuration
// @Test(timeout = 120000)
// public void testRighttHandIn3PModeRegression()
// {
//    ArrayList<Pair<FramePose, FramePose>> handTargetArray = generatePointsForRegression(5);
//    wholeBodyTest.executeHandTargetTest(ControlledDoF.DOF_NONE, ControlledDoF.DOF_3P, handTargetArray, true, false);
// }
//
//
// @AverageDuration
// @Test(timeout = 120000)
// public void testBothtHandsIn3PModeRegression()
// {
//    ArrayList<Pair<FramePose, FramePose>> handTargetArray = generatePointsForRegression(5);
//    wholeBodyTest.executeHandTargetTest(ControlledDoF.DOF_3P, ControlledDoF.DOF_3P, handTargetArray, true, false);
// }
// 
// @Ignore
// @AverageDuration
// @Test(timeout = 120000)
// public void testBothtHandsIn3P2RModeRegression()
// {
//    ArrayList<Pair<FramePose, FramePose>> handTargetArray = generatePointsForRegression(5);
//    wholeBodyTest.executeHandTargetTest(ControlledDoF.DOF_3P2R, ControlledDoF.DOF_3P2R, handTargetArray, true, false);
// }
// 
// @Ignore
// @AverageDuration
// @Test(timeout = 120000)
// public void testBothtHandsIn3P3RModeRegression()
// {
//    ArrayList<Pair<FramePose, FramePose>> handTargetArray = generatePointsForRegression(5);
//    wholeBodyTest.executeHandTargetTest(ControlledDoF.DOF_3P3R, ControlledDoF.DOF_3P3R, handTargetArray, true, false);
// }


}
