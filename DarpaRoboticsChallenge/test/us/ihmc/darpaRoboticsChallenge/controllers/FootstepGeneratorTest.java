package us.ihmc.darpaRoboticsChallenge.controllers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepGeneratorVisualizer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepValidityMetric;
import us.ihmc.commonWalkingControlModules.desiredFootStep.SemiCircularStepValidityMetric;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;
import us.ihmc.darpaRoboticsChallenge.userInterface.DRCOperatorUserInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.overheadPath.OverheadPath;
import us.ihmc.utilities.math.overheadPath.TurnThenStraightOverheadPath;

import com.yobotics.simulationconstructionset.Robot;

public class FootstepGeneratorTest
{
   private static final double eps = 1e-7;
   private final static boolean VISUALIZE = false;
   private final static boolean SIDESTEP = false;
   private final static boolean DEBUG_S_PARAM = false;
   private final static boolean DEBUG = false;
   public static final ReferenceFrame WORLD_FRAME = ReferenceFrames.getWorldFrame();
   List<Footstep> footSteps = new ArrayList<Footstep>();
   FullRobotModel fullRobotModel;
   ReferenceFrames referenceFrames;
   SideDependentList<ContactablePlaneBody> bipedFeet;



   @Test
   public void testStraightLinePath()
   {
      Point3d destination = new Point3d(5.0, 0.0, 0.0);
      testPathToDestination(destination);
   }

   @Test
   public void testAngledPaths()
   {
      double maxAngle = Math.PI;
      double angle = 0;
      int n = 10;
      for (int i = -n; i <= n; i++)
      {
         angle = (maxAngle * i) / n;
         Point3d destination = new Point3d(5.0 * Math.cos(angle), 5.0 * Math.sin(angle), 0.0);
         testPathToDestination(destination);
      }
   }

   private void testPathToDestination(Point3d destination)
   {
      setupRobotParameters();
      OverheadPath pathToDestination = new TurnThenStraightOverheadPath(new FramePose2d(WORLD_FRAME),
                                          new FramePoint2d(WORLD_FRAME, destination.x, destination.y), SIDESTEP ? Math.PI / 2 : 0.0);
      if (DEBUG_S_PARAM)
         System.out.println("FootstepGeneratorTest: startPose=" + pathToDestination.getPoseAtS(0.0).toString());
      if (DEBUG_S_PARAM)
         System.out.println("FootstepGeneratorTest: start++Pose=" + pathToDestination.getPoseAtS(0.25).toString());
      if (DEBUG_S_PARAM)
         System.out.println("FootstepGeneratorTest: intPose=" + pathToDestination.getPoseAtS(0.5).toString());
      if (DEBUG_S_PARAM)
         System.out.println("FootstepGeneratorTest: int++Pose=" + pathToDestination.getPoseAtS(0.75).toString());
      if (DEBUG_S_PARAM)
         System.out.println("FootstepGeneratorTest: endPose=" + pathToDestination.getPoseAtS(1.0).toString());
      generateFootstepsUsingPath(pathToDestination);
      FootstepValidityMetric footstepValidityMetric = new SemiCircularStepValidityMetric(fullRobotModel.getFoot(RobotSide.LEFT), 0.08, 0.8, 0.8);
      assertAllStepsLevelAndZeroHeight();
      assertAllStepsValid(footstepValidityMetric);
      assertLastStepIsPointingCorrectly(footSteps.get(footSteps.size() - 1), destination);
      if (VISUALIZE)
         FootstepGeneratorVisualizer.visualizeFootsteps(new Robot("null"), footSteps);
      if (VISUALIZE)
         ThreadTools.sleepForever();
   }

   public void generateFootstepsUsingPath(OverheadPath pathToDestination)
   {
      FootstepGenerator footstepGenerator = new FootstepGenerator(bipedFeet);
      footstepGenerator.setFootstepPath(pathToDestination);
      footstepGenerator.setStepAngle(Math.PI / 6);
      footstepGenerator.setStepLength(0.4);
      footstepGenerator.setStepWidth(0.2);
      Footstep swingStart = FootstepUtils.getCurrentFootstep(RobotSide.LEFT, referenceFrames, bipedFeet);
      Footstep stanceStart = FootstepUtils.getCurrentFootstep(RobotSide.RIGHT, referenceFrames, bipedFeet);

//    footstepGenerator.setSwingStart(swingStart);
      footstepGenerator.setStanceStart(stanceStart);
      footSteps = footstepGenerator.generateDesiredFootstepList();
   }

   private static void assertLastStepIsPointingCorrectly(Footstep footstep, Point3d destination)
   {
      Vector3d footstepOrientation = SIDESTEP ? new Vector3d(0.0, -1.0, 0.0) : new Vector3d(1.0, 0.0, 0.0);
      footstep.getPose().getOrientationMatrix3d().transform(footstepOrientation);
      footstepOrientation.normalize();
      Vector3d pathOrientation = new Vector3d(destination);
      pathOrientation.normalize();
      assertEquals(pathOrientation.getX(), footstepOrientation.getX(), 1e-1);
      assertEquals(pathOrientation.getY(), footstepOrientation.getY(), 1e-1);
      assertEquals(pathOrientation.getZ(), footstepOrientation.getZ(), 1e-1);


   }

   private void assertAllStepsValid(FootstepValidityMetric footstepValidityMetric)
   {
      ArrayList<Footstep> testableFootstepQueue = prependStanceToFootstepQueue();
      for (int i = 0; i < testableFootstepQueue.size() - 2; i++)
      {
         Footstep swingStart = testableFootstepQueue.get(i);
         Footstep stance = testableFootstepQueue.get(i + 1);
         Footstep swingEnd = testableFootstepQueue.get(i + 2);
         assertTrue(footstepValidityMetric.isValid(swingStart, stance, swingEnd));
      }
   }

   private void assertAllStepsLevelAndZeroHeight()
   {
      for (Footstep footstep : footSteps)
      {
         assertEquals(DRCRobotParameters.DRC_ROBOT_ANKLE_HEIGHT, footstep.getPose().getZ(), eps);
      }
   }

   private ArrayList<Footstep> prependStanceToFootstepQueue()
   {
      SideDependentList<Footstep> currentFootLocations = new SideDependentList<Footstep>();
      for (RobotSide side : RobotSide.values)
      {
         currentFootLocations.put(side, FootstepUtils.generateStandingFootstep(side, fullRobotModel, referenceFrames, bipedFeet));
      }

      boolean firstStepIsLeft = footSteps.get(0).getBody() == fullRobotModel.getFoot(RobotSide.LEFT);
      RobotSide firstStepSide = firstStepIsLeft ? RobotSide.LEFT : RobotSide.RIGHT;
      ArrayList<Footstep> footstepQueue = new ArrayList<Footstep>();
      footstepQueue.add(currentFootLocations.get(firstStepSide));
      footstepQueue.add(currentFootLocations.get(firstStepSide.getOppositeSide()));
      footstepQueue.addAll(footSteps);

      return footstepQueue;
   }

   private void setupRobotParameters()
   {
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.getDefaultRobotModel());
      DRCRobotSDFLoader drcRobotSDFLoader = new DRCRobotSDFLoader();
      JaxbSDFLoader jaxbSDFLoader = drcRobotSDFLoader.loadDRCRobot(jointMap);
      fullRobotModel = jaxbSDFLoader.getFullRobotModel();
      referenceFrames = jaxbSDFLoader.getReferenceFrames();
      bipedFeet = DRCOperatorUserInterface.setupBipedFeet(referenceFrames, fullRobotModel);

   }


}
