package us.ihmc.darpaRoboticsChallenge.footStepGenerator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.DummySteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTools;
import us.ihmc.commonWalkingControlModules.trajectories.HeightCalculatorParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingTrajectoryHeightCalculator;
import us.ihmc.darpaRoboticsChallenge.footstepGenerator.BasicFootstepValidityMetric;
import us.ihmc.darpaRoboticsChallenge.footstepGenerator.PathToFootstepGenerator;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepVisualizer;
import us.ihmc.humanoidRobotics.footstep.footsepGenerator.FootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footsepGenerator.InterpolatedFootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footsepGenerator.overheadPath.TurnInPlaceFootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footsepGenerator.overheadPath.TurnStraightTurnFootstepOverheadPath;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.AtlasFootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.ConvexHullFootstepSnapper;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnapper;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.SimpleFootstepValueFunction;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeHeightMapGeneratorTools;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeHeightMapInterface;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

/**
 * Created by agrabertilton on 2/19/15.
 */
public class PathToFootstepGeneratorTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   @DeployableTestMethod
   @Test (timeout=300000)
   public void testStraightLinePathToFootstepGeneratorNoHeightMap()
   {
      boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();

      double horizontalOffset = 0.25/2.0;
      RobotSide startSide = RobotSide.LEFT;
      FramePose2d startPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.0, 0.0), 0.0);
      FramePose2d endPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(2.0, 0.0), 0.0);

      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      CombinedTerrainObject3D groundProfile = createFlatTerrainProfile();
      HeightMapWithPoints heightMap = createTerrainHeightMap(groundProfile);
      SwingTrajectoryHeightCalculator heightCalculator = createSwingHeightCalculator();
      SteppingParameters steppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric validityMetric = new BasicFootstepValidityMetric(steppingParameters);

      PathToFootstepGenerator generator = new PathToFootstepGenerator(heightCalculator, footstepSnapper, validityMetric, null);

      FootstepOverheadPath overheadPath = new InterpolatedFootstepOverheadPath(startPose, endPose);
      SideDependentList<FootstepData> originalFeet = createStartFeet(startPose, horizontalOffset, footstepSnapper, heightMap);
      generator.initialize(originalFeet, overheadPath);

      List<FootstepData> generatedFootsteps = generator.getStepsAlongPath(startSide);

      SideDependentList<FootstepData> previousFeet = new SideDependentList<>(originalFeet);
      FootstepData initialSwingFootstep;
      FootstepData stanceFootstep;
      for (FootstepData footstepData : generatedFootsteps){
         initialSwingFootstep = previousFeet.get(footstepData.getRobotSide());
         stanceFootstep = previousFeet.get(footstepData.getRobotSide().getOppositeSide());
         assertTrue(validityMetric.footstepValid(stanceFootstep, footstepData));
         assertTrue(validityMetric.footstepValid(initialSwingFootstep, stanceFootstep, footstepData));
         previousFeet.put(footstepData.getRobotSide(), footstepData);
      }

      for (FootstepData lastFootstep : previousFeet){
         testLastFootstep(endPose, lastFootstep, horizontalOffset);
      }

      if (VISUALIZE)
      {
         visualizeFootsteps(groundProfile, originalFeet, generatedFootsteps, steppingParameters);
      }
   }

   @DeployableTestMethod
   @Test (timeout=300000)
   public void testStraightLinePathToFootstepGenerator()
   {
      boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();

      double horizontalOffset = 0.25/2.0;
      RobotSide startSide = RobotSide.LEFT;
      FramePose2d startPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.0, 0.0), 0.0);
      FramePose2d endPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(2.0, 0.0), 0.0);

      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      CombinedTerrainObject3D groundProfile = createTerrainProfile();
      HeightMapWithPoints heightMap = createTerrainHeightMap(groundProfile);
      SwingTrajectoryHeightCalculator heightCalculator = createSwingHeightCalculator();
      SteppingParameters steppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric validityMetric = new BasicFootstepValidityMetric(steppingParameters);

      PathToFootstepGenerator generator = new PathToFootstepGenerator(heightCalculator, footstepSnapper, validityMetric, heightMap);

      FootstepOverheadPath overheadPath = new InterpolatedFootstepOverheadPath(startPose, endPose);
      SideDependentList<FootstepData> originalFeet = createStartFeet(startPose, horizontalOffset, footstepSnapper, heightMap);
      generator.initialize(originalFeet, overheadPath);

      List<FootstepData> generatedFootsteps = generator.getStepsAlongPath(startSide);

      SideDependentList<FootstepData> previousFeet = new SideDependentList<>(originalFeet);
      FootstepData initialSwingFootstep;
      FootstepData stanceFootstep;
      for (FootstepData footstepData : generatedFootsteps){
         initialSwingFootstep = previousFeet.get(footstepData.getRobotSide());
         stanceFootstep = previousFeet.get(footstepData.getRobotSide().getOppositeSide());
         assertTrue(validityMetric.footstepValid(stanceFootstep, footstepData));
         assertTrue(validityMetric.footstepValid(initialSwingFootstep, stanceFootstep, footstepData));
         previousFeet.put(footstepData.getRobotSide(), footstepData);
      }

      for (FootstepData lastFootstep : previousFeet){
         testLastFootstep(endPose, lastFootstep, horizontalOffset);
      }

      if (VISUALIZE)
      {
         visualizeFootsteps(groundProfile, originalFeet, generatedFootsteps, steppingParameters);
      }
   }

   @DeployableTestMethod
   @Test (timeout=300000)
   public void testStraightLinePathToFootstepGeneratorOnRandomTerrain()
   {
      boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();

      Random random = new Random(8678); //(34567);  //(8872); //(8678); //(7264L);
      double horizontalOffset = 0.25/2.0;
      RobotSide startSide = RobotSide.LEFT;
      FramePose2d startPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.0, 0.0), 0.0);
      FramePose2d endPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(2.0, 0.0), 0.0);

      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      CombinedTerrainObject3D groundProfile = createRandomTerrainProfile(random);
      HeightMapWithPoints heightMap = createTerrainHeightMap(groundProfile);
      SwingTrajectoryHeightCalculator heightCalculator = createSwingHeightCalculator();
      SteppingParameters steppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric validityMetric = new BasicFootstepValidityMetric(steppingParameters);

      PathToFootstepGenerator generator = new PathToFootstepGenerator(heightCalculator, footstepSnapper, validityMetric, heightMap);

      FootstepOverheadPath overheadPath = new InterpolatedFootstepOverheadPath(startPose, endPose);
      SideDependentList<FootstepData> originalFeet = createStartFeet(startPose, horizontalOffset, footstepSnapper, heightMap);
      generator.initialize(originalFeet, overheadPath);

      List<FootstepData> generatedFootsteps = generator.getStepsAlongPath(startSide);

      SideDependentList<FootstepData> previousFeet = new SideDependentList<>(originalFeet);
      FootstepData initialSwingFootstep;
      FootstepData stanceFootstep;
      for (FootstepData footstepData : generatedFootsteps){
         initialSwingFootstep = previousFeet.get(footstepData.getRobotSide());
         stanceFootstep = previousFeet.get(footstepData.getRobotSide().getOppositeSide());
         assertTrue(validityMetric.footstepValid(stanceFootstep, footstepData));
         assertTrue(validityMetric.footstepValid(initialSwingFootstep, stanceFootstep, footstepData));
         previousFeet.put(footstepData.getRobotSide(), footstepData);
      }

      for (FootstepData lastFootstep : previousFeet){
         testLastFootstep(endPose, lastFootstep, horizontalOffset);
      }

      if (VISUALIZE)
      {
         visualizeFootsteps(groundProfile, originalFeet, generatedFootsteps, steppingParameters);
      }
   }

   @DeployableTestMethod
   @Test (timeout=300000)
   public void testTurnStraightTurnOnRandomTerrain()
   {
      boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();

      Random random = new Random(8678); //(34567);  //(8872); //(8678); //(7264L);
      double horizontalOffset = 0.25/2.0;
      RobotSide startSide = RobotSide.LEFT;
      FramePose2d startPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(-0.2, 0.0), Math.PI * (2.0 *random.nextDouble() - 1.0));
      FramePose2d endPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(2.0, 0.0), Math.PI * (2.0 *random.nextDouble() - 1.0));

      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      CombinedTerrainObject3D groundProfile = createRandomTerrainProfile(random);
      HeightMapWithPoints heightMap = createTerrainHeightMap(groundProfile);
      SwingTrajectoryHeightCalculator heightCalculator = createSwingHeightCalculator();
      SteppingParameters steppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric validityMetric = new BasicFootstepValidityMetric(steppingParameters);

      PathToFootstepGenerator generator = new PathToFootstepGenerator(heightCalculator, footstepSnapper, validityMetric, heightMap);

      FootstepOverheadPath overheadPath = new TurnStraightTurnFootstepOverheadPath(startPose, endPose);
      SideDependentList<FootstepData> originalFeet = createStartFeet(startPose, horizontalOffset, footstepSnapper, heightMap);
      generator.initialize(originalFeet, overheadPath);

      List<FootstepData> generatedFootsteps = generator.getStepsAlongPath(startSide);

      SideDependentList<FootstepData> previousFeet = new SideDependentList<>(originalFeet);
      FootstepData initialSwingFootstep;
      FootstepData stanceFootstep;
      for (FootstepData footstepData : generatedFootsteps){
         initialSwingFootstep = previousFeet.get(footstepData.getRobotSide());
         stanceFootstep = previousFeet.get(footstepData.getRobotSide().getOppositeSide());
         assertTrue(validityMetric.footstepValid(stanceFootstep, footstepData));
         assertTrue(validityMetric.footstepValid(initialSwingFootstep, stanceFootstep, footstepData));
         previousFeet.put(footstepData.getRobotSide(), footstepData);
      }

      for (FootstepData lastFootstep : previousFeet){
         testLastFootstep(endPose, lastFootstep, horizontalOffset);
      }

      if (VISUALIZE)
      {
         visualizeFootsteps(groundProfile, originalFeet, generatedFootsteps, steppingParameters);
      }
   }

   @DeployableTestMethod
   @Test (timeout=300000)
   public void testTurnStraightTurnOnFlatGround()
   {
      boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();

      double horizontalOffset = 0.25/2.0;
      RobotSide startSide = RobotSide.LEFT;
      FramePose2d startPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.0, 0.0), -Math.PI / 4.0);
      FramePose2d endPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(2.0, 0.0), Math.PI/4.0);
      FootstepOverheadPath overheadPath = new TurnStraightTurnFootstepOverheadPath(startPose, endPose);

      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      CombinedTerrainObject3D groundProfile = createFlatTerrainProfile();
      HeightMapWithPoints heightMap = createTerrainHeightMap(groundProfile);
      SwingTrajectoryHeightCalculator heightCalculator = createSwingHeightCalculator();
      SteppingParameters steppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric validityMetric = new BasicFootstepValidityMetric(steppingParameters);

      PathToFootstepGenerator generator = new PathToFootstepGenerator(heightCalculator, footstepSnapper, validityMetric, heightMap);


      SideDependentList<FootstepData> originalFeet = createStartFeet(startPose, horizontalOffset, footstepSnapper, heightMap);
      generator.initialize(originalFeet, overheadPath);

      List<FootstepData> generatedFootsteps = generator.getStepsAlongPath(startSide);

      SideDependentList<FootstepData> previousFeet = new SideDependentList<>(originalFeet);
      FootstepData initialSwingFootstep;
      FootstepData stanceFootstep;
      for (FootstepData footstepData : generatedFootsteps){
         initialSwingFootstep = previousFeet.get(footstepData.getRobotSide());
         stanceFootstep = previousFeet.get(footstepData.getRobotSide().getOppositeSide());
         assertTrue(validityMetric.footstepValid(stanceFootstep, footstepData));
         assertTrue(validityMetric.footstepValid(initialSwingFootstep, stanceFootstep, footstepData));
         previousFeet.put(footstepData.getRobotSide(), footstepData);
      }

      for (FootstepData lastFootstep : previousFeet){
         testLastFootstep(endPose, lastFootstep, horizontalOffset);
      }

      if (VISUALIZE)
      {
         visualizeFootsteps(groundProfile, originalFeet, generatedFootsteps, steppingParameters);
      }
   }

   @DeployableTestMethod
   @Test (timeout=300000)
   public void testTurnInPlacePathOnFlatGround()
   {
      boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();

      double horizontalOffset = 0.25/2.0;
      RobotSide startSide = RobotSide.LEFT;
      Point2d centerPoint = new Point2d(0.0, 0.0);
      double startYaw = 0.0;
      double endYaw = Math.PI;

      FramePose2d startPose = new FramePose2d(ReferenceFrame.getWorldFrame(), centerPoint, startYaw);
      FramePose2d endPose = new FramePose2d(ReferenceFrame.getWorldFrame(), centerPoint, endYaw);


      FootstepOverheadPath overheadPath = new TurnInPlaceFootstepOverheadPath(startPose, endPose);

      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      CombinedTerrainObject3D groundProfile = createFlatTerrainProfile();
      HeightMapWithPoints heightMap = createTerrainHeightMap(groundProfile);
      SwingTrajectoryHeightCalculator heightCalculator = createSwingHeightCalculator();
      SteppingParameters steppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric validityMetric = new BasicFootstepValidityMetric(steppingParameters);

      PathToFootstepGenerator generator = new PathToFootstepGenerator(heightCalculator, footstepSnapper, validityMetric, heightMap);


      SideDependentList<FootstepData> originalFeet = createStartFeet(startPose, horizontalOffset, footstepSnapper, heightMap);
      generator.initialize(originalFeet, overheadPath);

      List<FootstepData> generatedFootsteps = generator.getStepsAlongPath(startSide);

      SideDependentList<FootstepData> previousFeet = new SideDependentList<>(originalFeet);
      FootstepData initialSwingFootstep;
      FootstepData stanceFootstep;
      for (FootstepData footstepData : generatedFootsteps){
         initialSwingFootstep = previousFeet.get(footstepData.getRobotSide());
         stanceFootstep = previousFeet.get(footstepData.getRobotSide().getOppositeSide());
         assertTrue(validityMetric.footstepValid(stanceFootstep, footstepData));
         assertTrue(validityMetric.footstepValid(initialSwingFootstep, stanceFootstep, footstepData));
         previousFeet.put(footstepData.getRobotSide(), footstepData);
      }

      for (FootstepData lastFootstep : previousFeet){
         testLastFootstep(endPose, lastFootstep, horizontalOffset);
      }

      if (VISUALIZE)
      {
         visualizeFootsteps(groundProfile, originalFeet, generatedFootsteps, steppingParameters);
      }
   }

   @DeployableTestMethod
   @Test (timeout=300000)
   public void testTurnSideStepTurnOnFlatGround()
   {
      boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();
      //Purple is Left, Green is Right

      double horizontalOffset = 0.25/2.0;
      RobotSide startSide = RobotSide.LEFT;
      FramePose2d startPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.0, 0.0), -Math.PI / 4.0);
      FramePose2d endPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(2.0, 0.0), Math.PI/4.0);
      FootstepOverheadPath overheadPath = new TurnStraightTurnFootstepOverheadPath(startPose, endPose, Math.PI/2.0);

      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      CombinedTerrainObject3D groundProfile = createFlatTerrainProfile();
      HeightMapWithPoints heightMap = createTerrainHeightMap(groundProfile);
      SwingTrajectoryHeightCalculator heightCalculator = createSwingHeightCalculator();
      SteppingParameters steppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric validityMetric = new BasicFootstepValidityMetric(steppingParameters);

      PathToFootstepGenerator generator = new PathToFootstepGenerator(heightCalculator, footstepSnapper, validityMetric, heightMap);


      SideDependentList<FootstepData> originalFeet = createStartFeet(startPose, horizontalOffset, footstepSnapper, heightMap);
      generator.initialize(originalFeet, overheadPath);

      List<FootstepData> generatedFootsteps = generator.getStepsAlongPath(startSide);

      SideDependentList<FootstepData> previousFeet = new SideDependentList<>(originalFeet);
      FootstepData initialSwingFootstep;
      FootstepData stanceFootstep;
      for (FootstepData footstepData : generatedFootsteps){
         initialSwingFootstep = previousFeet.get(footstepData.getRobotSide());
         stanceFootstep = previousFeet.get(footstepData.getRobotSide().getOppositeSide());
         assertTrue(validityMetric.footstepValid(stanceFootstep, footstepData));
         assertTrue(validityMetric.footstepValid(initialSwingFootstep, stanceFootstep, footstepData));
         previousFeet.put(footstepData.getRobotSide(), footstepData);
      }

      for (FootstepData lastFootstep : previousFeet){
         testLastFootstep(endPose, lastFootstep, horizontalOffset);
      }

      if (VISUALIZE)
      {
         visualizeFootsteps(groundProfile, originalFeet, generatedFootsteps, steppingParameters);
      }
   }

   @DeployableTestMethod
   @Test (timeout=300000)
   public void testTurnSideStepOtherWayTurnOnFlatGround()
   {
      boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();
      //Purple is Left, Green is Right

      double horizontalOffset = 0.25/2.0;
      RobotSide startSide = RobotSide.LEFT;
      FramePose2d startPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.0, 0.0), -Math.PI / 4.0);
      FramePose2d endPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(2.0, 0.0), Math.PI/4.0);
      FootstepOverheadPath overheadPath = new TurnStraightTurnFootstepOverheadPath(startPose, endPose, -Math.PI/2.0);

      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      CombinedTerrainObject3D groundProfile = createFlatTerrainProfile();
      HeightMapWithPoints heightMap = createTerrainHeightMap(groundProfile);
      SwingTrajectoryHeightCalculator heightCalculator = createSwingHeightCalculator();
      SteppingParameters steppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric validityMetric = new BasicFootstepValidityMetric(steppingParameters);

      PathToFootstepGenerator generator = new PathToFootstepGenerator(heightCalculator, footstepSnapper, validityMetric, heightMap);


      SideDependentList<FootstepData> originalFeet = createStartFeet(startPose, horizontalOffset, footstepSnapper, heightMap);
      generator.initialize(originalFeet, overheadPath);

      List<FootstepData> generatedFootsteps = generator.getStepsAlongPath(startSide);

      SideDependentList<FootstepData> previousFeet = new SideDependentList<>(originalFeet);
      FootstepData initialSwingFootstep;
      FootstepData stanceFootstep;
      for (FootstepData footstepData : generatedFootsteps){
         initialSwingFootstep = previousFeet.get(footstepData.getRobotSide());
         stanceFootstep = previousFeet.get(footstepData.getRobotSide().getOppositeSide());
         assertTrue(validityMetric.footstepValid(stanceFootstep, footstepData));
         assertTrue(validityMetric.footstepValid(initialSwingFootstep, stanceFootstep, footstepData));
         previousFeet.put(footstepData.getRobotSide(), footstepData);
      }

      for (FootstepData lastFootstep : previousFeet){
         testLastFootstep(endPose, lastFootstep, horizontalOffset);
      }

      if (VISUALIZE)
      {
         visualizeFootsteps(groundProfile, originalFeet, generatedFootsteps, steppingParameters);
      }
   }

   @DeployableTestMethod
   @Test (timeout=300000)
   public void testTurnBackwardsTurnOnFlatGround()
   {
      boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();
      //Purple is Left, Green is Right

      double horizontalOffset = 0.25/2.0;
      RobotSide startSide = RobotSide.LEFT;
      FramePose2d startPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(0.0, 0.0), -Math.PI / 4.0);
      FramePose2d endPose = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(2.0, 0.0), Math.PI/4.0);
      FootstepOverheadPath overheadPath = new TurnStraightTurnFootstepOverheadPath(startPose, endPose, Math.PI);

      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      CombinedTerrainObject3D groundProfile = createFlatTerrainProfile();
      HeightMapWithPoints heightMap = createTerrainHeightMap(groundProfile);
      SwingTrajectoryHeightCalculator heightCalculator = createSwingHeightCalculator();
      SteppingParameters steppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric validityMetric = new BasicFootstepValidityMetric(steppingParameters);

      PathToFootstepGenerator generator = new PathToFootstepGenerator(heightCalculator, footstepSnapper, validityMetric, heightMap);


      SideDependentList<FootstepData> originalFeet = createStartFeet(startPose, horizontalOffset, footstepSnapper, heightMap);
      generator.initialize(originalFeet, overheadPath);

      List<FootstepData> generatedFootsteps = generator.getStepsAlongPath(startSide);

      SideDependentList<FootstepData> previousFeet = new SideDependentList<>(originalFeet);
      FootstepData initialSwingFootstep;
      FootstepData stanceFootstep;
      for (FootstepData footstepData : generatedFootsteps){
         initialSwingFootstep = previousFeet.get(footstepData.getRobotSide());
         stanceFootstep = previousFeet.get(footstepData.getRobotSide().getOppositeSide());
         assertTrue(validityMetric.footstepValid(stanceFootstep, footstepData));
         assertTrue(validityMetric.footstepValid(initialSwingFootstep, stanceFootstep, footstepData));
         previousFeet.put(footstepData.getRobotSide(), footstepData);
      }

      for (FootstepData lastFootstep : previousFeet){
         testLastFootstep(endPose, lastFootstep, horizontalOffset);
      }

      if (VISUALIZE)
      {
         visualizeFootsteps(groundProfile, originalFeet, generatedFootsteps, steppingParameters);
      }
   }



   private void visualizeFootsteps(CombinedTerrainObject3D groundProfile, SideDependentList<FootstepData> originalFeet, List<FootstepData> generatedFootsteps,  SteppingParameters steppingParameters){
      int maxNumberOfFootstepsPerSide = 50;
      int maxContactPointsPerFoot = 4;
      String description = "Basic Test";
      FootSpoof footSpoof = new FootSpoof("footSpoof", 0.0, 0.0, 0.0, steppingParameters.getFootForwardOffset(), steppingParameters.getFootBackwardOffset(), steppingParameters.getToeWidth()/2.0, 0.0);
      FootstepVisualizer footstepVisualizer = new FootstepVisualizer(null, groundProfile.getLinkGraphics(), maxNumberOfFootstepsPerSide, maxContactPointsPerFoot, description);
      footstepVisualizer.startVisualizer();

      SimulationConstructionSet scs = footstepVisualizer.getSimulationConstructionSet();    // new SimulationConstructionSet(robot);
      for (RobotSide side : RobotSide.values()){
         FootstepData footstepData = originalFeet.get(side);
         Footstep footstep = FootstepTools.generateFootstepFromFootstepData(footstepData, footSpoof);
         FramePose footSolePose = new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation());
         footstep.setSolePose(footSolePose);
         footstepVisualizer.visualizeFootstep(footSpoof, footstep);
      }

      for (FootstepData footstepData : generatedFootsteps)
      {
         Footstep footstep = FootstepTools.generateFootstepFromFootstepData(footstepData, footSpoof);
         FramePose footSolePose = new FramePose(ReferenceFrame.getWorldFrame(), footstepData.getLocation(), footstepData.getOrientation());
         footstep.setSolePose(footSolePose);
         footstepVisualizer.visualizeFootstep(footSpoof, footstep);
      }
      ThreadTools.sleepForever();
   }

   private SideDependentList<FootstepData> createStartFeet(FramePose2d centerPoint, double horizontalDistance, FootstepSnapper snapper, HeightMapWithPoints heightMap){
      SideDependentList<FootstepData> originalFeet = new SideDependentList<>();
      double yaw = centerPoint.getYaw();
      Vector3d verticalVector = new Vector3d(0.0, 0.0, 1.0);

      for (RobotSide side : RobotSide.values())
      {
         Point3d position = new Point3d();
         Quat4d orientation = new Quat4d();
         double sign = (side == RobotSide.LEFT) ? 1 : -1;
         //sign * horizontalDistance
         position.x = centerPoint.getX() + -1.0 * Math.sin(yaw) * sign * horizontalDistance;
         position.y = centerPoint.getY() + 1.0 * Math.cos(yaw) * sign * horizontalDistance;
         RotationFunctions.getQuaternionFromYawAndZNormal(yaw, verticalVector, orientation);
         FootstepData footstep = new FootstepData(side, position, orientation);
         snapper.snapFootstep(footstep, heightMap);
         originalFeet.put(side, footstep);
      }
      return originalFeet;
   }

   private void testLastFootstep(FramePose2d endPoint, FootstepData endFootstep, double horizontalDistance){
      double yaw = endPoint.getYaw();
      double sign = (endFootstep.getRobotSide() == RobotSide.LEFT) ? 1 : -1;
      //sign * horizontalDistance
      double endXToCheckAgainst = endPoint.getX() + -1.0 * Math.sin(yaw) * sign * horizontalDistance;
      double endYToCheckAgainst = endPoint.getY() + 1.0 * Math.cos(yaw) * sign * horizontalDistance;

      Point3d footstepPosition = endFootstep.getLocation();
      assertEquals(endXToCheckAgainst, footstepPosition.getX(), 1e-10);
      assertEquals(endYToCheckAgainst, footstepPosition.getY(), 1e-10);
      double angleError = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(yaw, RotationFunctions.getYawFromQuaternion(endFootstep.getOrientation())));
      assertTrue(angleError < 1e-10);
   }

   private ConvexHullFootstepSnapper createConvexHullFootstepSnapper()
   {
      FootstepSnappingParameters snappingParameters = new AtlasFootstepSnappingParameters();
      ConvexHullFootstepSnapper footstepSnapper = new ConvexHullFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);

      return footstepSnapper;
   }

   private SwingTrajectoryHeightCalculator createSwingHeightCalculator()
   {
      double verticalBuffer = 0.05;    // 5cm
      double pathWidth = 0.12;    // 12cm
      SteppingParameters steppingParameters = new DummySteppingParameters();
      HeightCalculatorParameters calculatorParameters = new HeightCalculatorParameters(verticalBuffer, pathWidth);

      SwingTrajectoryHeightCalculator generator = new SwingTrajectoryHeightCalculator(calculatorParameters, steppingParameters);
      return generator;
   }

   private QuadTreeHeightMapInterface createTerrainHeightMap(CombinedTerrainObject3D groundProfile)
   {
      double centerX = 0;
      double centerY = 0;
      double halfWidth = 10.0;
      double resolution = 0.01;
      BoundingBox2d rangeOfPointsToTest = new BoundingBox2d(centerX - halfWidth, centerY - halfWidth, centerX + halfWidth, centerY + halfWidth);
      QuadTreeHeightMapInterface groundMap = QuadTreeHeightMapGeneratorTools.createHeightMap(groundProfile, rangeOfPointsToTest, resolution);

      return groundMap;
   }

   private CombinedTerrainObject3D createFlatTerrainProfile(){
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("Flat Terrain");
      AppearanceDefinition color = YoAppearance.DarkGray();
      combinedTerrainObject.addBox(-100.0, -100.0, 100.0, 100.0, 0.001, color);
      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D createTerrainProfile()
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("Difficult Terrain");

      AppearanceDefinition color = YoAppearance.DarkGray();
      combinedTerrainObject.addBox(-100.0, -100.0, 100.0, 100.0, 0.001, color);

      combinedTerrainObject.addBox(0.15, -0.5, 0.4, 0.5, 0.1);
      combinedTerrainObject.addBox(0.45, -0.5, 0.8, 0.5, 0.15);
      combinedTerrainObject.addBox(0.8, -0.5, 1.05, 0.5, 0.2);
      combinedTerrainObject.addBox(1.1, -0.5, 1.15, 0.5, 0.1);
      combinedTerrainObject.addBox(1.3, -0.5, 1.5, 0.5, 0.15);
      combinedTerrainObject.addBox(1.5, -0.5, 2.1, 0.5, 0.2);
      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D createRandomTerrainProfile(Random random)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("RandomTerrain");

      AppearanceDefinition color = YoAppearance.DarkGray();
      combinedTerrainObject.addBox(-100.0, -100.0, 100.0, 100.0, 0.001, color);

      double currentDistance = 0.15;
      double nextDistance = currentDistance;
      double endDistance = 1.8;
      double maxHeight = 0.25;
      double minPlatformLength = 0.05;
      double maxPlatformLength = 0.4;
      while (currentDistance < endDistance){
         nextDistance = currentDistance + Math.max(minPlatformLength, random.nextDouble() * maxPlatformLength);
         nextDistance = Math.min(nextDistance, endDistance);
         combinedTerrainObject.addBox(currentDistance, -0.5, nextDistance, 0.5, random.nextDouble() * maxHeight);
         currentDistance = nextDistance;
      }
      combinedTerrainObject.addBox(endDistance, -0.5, endDistance + 0.4, 0.5, random.nextDouble() * maxHeight);
      return combinedTerrainObject;
   }
}
