package us.ihmc.atlas.rrtWalkingpathtest;

import static us.ihmc.robotics.Assert.assertTrue;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.walkingpath.footstep.SkeletonPathFootStep;
import us.ihmc.manipulation.planning.walkingpath.footstep.SkeletonPathFootStepPlanner;
import us.ihmc.manipulation.planning.walkingpath.rrtplanner.RRT2DNodeWalkingPath;
import us.ihmc.manipulation.planning.walkingpath.rrtplanner.RRT2DPlannerWalkingPath;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class AvatarWalkingPathGeneratorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   Point3D goalState;
   Point3D initialState;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public class WalkingPathTestEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D EnvSet;

      public WalkingPathTestEnvironment()
      {
         EnvSet = DefaultCommonAvatarEnvironment.setUpGround("Ground");

         addBoxEnvironment(RRT2DNodeWalkingPath.boxes[0], YoAppearance.Grey());

         for (int i = 1; i < RRT2DNodeWalkingPath.boxes.length; i++)
         {
            addBoxEnvironment(RRT2DNodeWalkingPath.boxes[i], YoAppearance.Red());
         }

         EnvSet.addSphere(goalState.getX(), goalState.getY(), goalState.getZ(), 0.1, YoAppearance.Blue());

         RigidBodyTransform location = new RigidBodyTransform();
         location.getTranslation().set(initialState);

         // EnvSet.addCylinder(location, 1.0, 0.4, YoAppearance.Wheat());
      }

      public void addBoxEnvironment(RRT2DNodeWalkingPath.BoxInfo box, AppearanceDefinition app)
      {
         EnvSet.addBox(box.centerX - box.sizeX / 2, box.centerY - box.sizeY / 2, box.centerX + box.sizeX / 2, box.centerY + box.sizeY / 2, box.sizeZ, app);
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return EnvSet;
      }

      @Override
      public List<? extends Robot> getEnvironmentRobots()
      {
         return null;
      }

      @Override
      public void createAndSetContactControllerToARobot()
      {

      }

      @Override
      public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
      {

      }

      @Override
      public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
      {
      }
   }

   private void setUpSimulationTestHelper(CommonAvatarEnvironmentInterface environment, DRCObstacleCourseStartingLocation startingLocation)
   {

      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
      }

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(startingLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

   }

   @Test
   public void testOne() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      // ******************************** //
      // Input ************************** //
      // ******************************** //
      initialState = new Point3D(0.0, 0.0, 0.0);
      goalState = new Point3D(8.0, -4.0, 0.0);

      RRTNode startNode = new RRT2DNodeWalkingPath(initialState.getX(), initialState.getY());
      RRTNode goalNode = new RRT2DNodeWalkingPath(goalState.getX(), goalState.getY());

      RRT2DPlannerWalkingPath rrtPlanner = new RRT2DPlannerWalkingPath(startNode, goalNode, 0.4);

      RRTNode upperBoundNode = new RRT2DNodeWalkingPath(12.0, 3.0);
      RRTNode lowerBoundNode = new RRT2DNodeWalkingPath(-2.0, -7.0);

      rrtPlanner.getRRTTree().setUpperBound(upperBoundNode);
      rrtPlanner.getRRTTree().setLowerBound(lowerBoundNode);

      // ******************************** //

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface = new WalkingPathTestEnvironment();
      setUpSimulationTestHelper(commonAvatarEnvironmentInterface, selectedLocation);

      setupCamera();

      boolean success = simulationTestHelper.simulateAndWait(1.0);
      assertTrue(success);

      // ******************************** //
      // RRT Planning ******************* //
      // ******************************** //

      rrtPlanner.expandTreeGoal(2000);
      LogTools.info("path has " + rrtPlanner.getOptimalPath().size() + " nodes");

      simulationTestHelper.addStaticVisuals(getPrintNodePath(rrtPlanner.getOptimalPath(), ColorDefinitions.Red()));
      success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      rrtPlanner.updateOptimalPath(50, 200);
      simulationTestHelper.addStaticVisuals(getPrintNodePath(rrtPlanner.getOptimalPath(), ColorDefinitions.Aqua()));
      success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      LogTools.info("shortcut optimized path has " + rrtPlanner.getOptimalPath().size() + " nodes");

      // footstep
      rrtPlanner.createFootStepPlanner(0.5, 0.35);

      SkeletonPathFootStepPlanner footStepPlanner = rrtPlanner.getFootStepPlanner();

      simulationTestHelper.addStaticVisuals(getPrintSegments(footStepPlanner.pathSegmentsLeftSide.get()));
      simulationTestHelper.addStaticVisuals(getPrintSegments(footStepPlanner.pathSegmentsRightSide.get()));

      footStepPlanner.setZeroStep(RobotSide.LEFT);
      footStepPlanner.createFootSteps();

      simulationTestHelper.addStaticVisuals(getPrintFootSteps(footStepPlanner.footSteps));
      simulationTestHelper.addStaticVisuals(getPrintSphere(footStepPlanner.tempPoint, ColorDefinitions.Black()));

      // ******************************** //
      // Foot Step Planning ************* //
      // ******************************** //
      //
      //      double swingTime = 2.0;
      //      double transferTime = 0.8;
      //      int steps = 3;
      //
      //      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime);
      //      FootstepDataMessage footstepData;
      //      RobotSide robotSide;
      //      double footstepY;
      //      double footstepX;
      //      Point3D location;
      //      Quaternion orientation;
      //
      //      robotSide = RobotSide.LEFT;
      //      footstepX = 0.2;
      //      footstepY = 0.14;
      //      location = new Point3D(footstepX, footstepY, 0.0);
      //      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      //      footstepData = new FootstepDataMessage(robotSide, location, orientation);
      //      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      //      footsteps.add(footstepData);
      //
      //      robotSide = RobotSide.RIGHT;
      //      footstepX = 0.4;
      //      footstepY = -0.14;
      //      location = new Point3D(footstepX, footstepY, 0.0);
      //      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      //      footstepData = new FootstepDataMessage(robotSide, location, orientation);
      //      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      //      footsteps.add(footstepData);
      //
      //      robotSide = RobotSide.LEFT;
      //      footstepX = 0.4;
      //      footstepY = 0.14;
      //      location = new Point3D(footstepX, footstepY, 0.0);
      //      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      //      footstepData = new FootstepDataMessage(robotSide, location, orientation);
      //      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      //      footsteps.add(footstepData);
      //
      //      drcSimulationTestHelper.send(footsteps);
      //      double simulationTime = (swingTime + transferTime) * steps + 1.0;

      //      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      // ******************************** //

      LogTools.info("END!!");

      success = simulationTestHelper.simulateAndWait(1.0);
      assertTrue(success);

   }

   private void setupCamera()
   {
      Point3D cameraFix = new Point3D(2.0, 0.0, 1.1);
      Point3D cameraPosition = new Point3D(-5.0, 10.0, 15.0);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }
   // ******************************** //
   // Print
   // ******************************** //

   private List<VisualDefinition> getPrintNodePath(List<RRTNode> nodePath, ColorDefinition app)
   {
      List<VisualDefinition> ret = new ArrayList<>();

      for (int i = 0; i < nodePath.size() - 1; i++)
      {
         ret.addAll(getPrintNodeConnection(nodePath.get(i), nodePath.get(i + 1), app));
      }

      return ret;
   }

   private List<VisualDefinition> getPrintNodeConnection(RRTNode nodeOne, RRTNode nodeTwo)
   {
      return getPrintNodeConnection(nodeOne, nodeTwo, ColorDefinitions.DarkGray());
   }

   private List<VisualDefinition> getPrintNodeConnection(RRTNode nodeOne, RRTNode nodeTwo, ColorDefinition color)
   {
      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      MaterialDefinition materialDefinition = new MaterialDefinition(color);

      Point3D translationNodeOne = new Point3D(nodeOne.getNodeData(0), nodeOne.getNodeData(1), 0);
      factory.appendTranslation(translationNodeOne);
      factory.addSphere(0.05, materialDefinition);

      Point3D translationNodeTwo = new Point3D(nodeTwo.getNodeData(0), nodeTwo.getNodeData(1), 0);
      factory.identity();
      factory.appendTranslation(translationNodeTwo);
      factory.addSphere(0.05, materialDefinition);

      Point3D translationLine = new Point3D((nodeOne.getNodeData(0) + nodeTwo.getNodeData(0)) / 2, (nodeOne.getNodeData(1) + nodeTwo.getNodeData(1)) / 2, 0);
      AxisAngle rotationLine = new AxisAngle(-(nodeOne.getNodeData(1) - nodeTwo.getNodeData(1)),
                                             (nodeOne.getNodeData(0) - nodeTwo.getNodeData(0)),
                                             0,
                                             Math.PI / 2);
      factory.identity();
      factory.appendTranslation(translationLine);
      factory.appendRotation(rotationLine);
      factory.addCapsule(0.02, nodeOne.getDistance(nodeTwo), materialDefinition);

      return factory.getVisualDefinitions();
   }

   private List<VisualDefinition> getPrintFootStep(FootstepDataListMessage footsteps)
   {
      VisualDefinitionFactory factory = new VisualDefinitionFactory();

      for (int i = 0; i < footsteps.getFootstepDataList().size(); i++)
      {
         factory.identity();
         factory.appendTranslation(footsteps.getFootstepDataList().get(i).getLocation());
         if (footsteps.getFootstepDataList().get(i).getRobotSide() == RobotSide.RIGHT.toByte())
         {
            factory.addSphere(0.05, new MaterialDefinition(ColorDefinitions.Blue()));
         }
         else
         {
            factory.addSphere(0.05, new MaterialDefinition(ColorDefinitions.Green()));
         }
      }
      return factory.getVisualDefinitions();
   }

   private List<VisualDefinition> getPrintFootStep(SkeletonPathFootStep footstep)
   {
      double sizeX = 0.3;
      double sizeY = 0.15;
      double sizeZ = 0.01;

      Point3D location = new Point3D(footstep.getLocation().getX(), footstep.getLocation().getY(), 0);
      AxisAngle rotation = new AxisAngle();
      rotation.appendYawRotation(footstep.getYawAngle());

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.appendTranslation(location);
      factory.appendRotation(rotation);
      if (footstep.getRobotSide() == RobotSide.RIGHT)
      {
         factory.addBox(sizeX, sizeY, sizeZ, new MaterialDefinition(ColorDefinitions.Green()));
         //PrintTools.info("R "+footstep.getLocation().getX()+" "+footstep.getLocation().getY());
      }

      else
      {
         factory.addBox(sizeX, sizeY, sizeZ, new MaterialDefinition(ColorDefinitions.Blue()));
         //PrintTools.info("L "+footstep.getLocation().getX()+" "+footstep.getLocation().getY());
      }

      return factory.getVisualDefinitions();
   }

   private List<VisualDefinition> getPrintFootSteps(List<SkeletonPathFootStep> footSteps)
   {
      List<VisualDefinition> ret = new ArrayList<>();
      for (int i = 0; i < footSteps.size(); i++)
      {
         ret.addAll(getPrintFootStep(footSteps.get(i)));
         //         PrintTools.info(" "+footSteps.get(i).getLocation().getX()+" "+footSteps.get(i).getLocation().getY());
      }

      return ret;
   }

   private List<VisualDefinition> getPrintSphere(Point2D aPoint, ColorDefinition app)
   {
      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.appendTranslation(aPoint.getX(), aPoint.getY(), 0);
      factory.addSphere(0.05, new MaterialDefinition(app));
      return factory.getVisualDefinitions();
   }

   private List<VisualDefinition> getPrintSegment(Line2D pathSegment)
   {
      VisualDefinitionFactory factory = new VisualDefinitionFactory();

      Point3D translationNodeOne = new Point3D(pathSegment.getX1(), pathSegment.getY1(), 0);
      factory.appendTranslation(translationNodeOne);
      factory.addSphere(0.05, new MaterialDefinition(ColorDefinitions.DarkGray()));

      Point3D translationNodeTwo = new Point3D(pathSegment.getX2(), pathSegment.getY2(), 0);
      factory.identity();
      factory.appendTranslation(translationNodeTwo);
      factory.addSphere(0.05, new MaterialDefinition(ColorDefinitions.DarkGray()));

      Point3D translationLine = new Point3D((pathSegment.getX1() + pathSegment.getX2()) / 2, (pathSegment.getY1() + pathSegment.getY2()) / 2, 0);
      AxisAngle rotationLine = new AxisAngle(-(pathSegment.getY1() - pathSegment.getY2()), (pathSegment.getX1() - pathSegment.getX2()), 0, Math.PI / 2);
      factory.identity();
      factory.appendTranslation(translationLine);
      factory.appendRotation(rotationLine);
      factory.addCapsule(0.02, pathSegment.getP1().distance(pathSegment.getP2()), new MaterialDefinition(ColorDefinitions.Gray()));

      return factory.getVisualDefinitions();
   }

   private List<VisualDefinition> getPrintSegments(ArrayList<Line2D> pathSegments)
   {
      List<VisualDefinition> ret = new ArrayList<>();

      for (int i = 0; i < pathSegments.size(); i++)
      {
         ret.addAll(getPrintSegment(pathSegments.get(i)));
      }

      return ret;
   }

}
