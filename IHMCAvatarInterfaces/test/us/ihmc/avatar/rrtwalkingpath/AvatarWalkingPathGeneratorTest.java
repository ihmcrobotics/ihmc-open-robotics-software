package us.ihmc.avatar.rrtwalkingpath;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.walkingpath.RRT2DNodeWalkingPath;
import us.ihmc.manipulation.planning.walkingpath.RRT2DPlannerWalkingPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarWalkingPathGeneratorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private CommunicationBridge communicationBridge;
   private DoubleYoVariable yoTime;

   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidRobotDataReceiver robotDataReceiver;
   private HumanoidFloatingRootJointRobot robot;
   private static final boolean DEBUG = false;

   SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
   CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();

   SimpleCollisionShapeFactory shapeFactory;

   BoxInfo boxA, boxB, boxC, boxD, boxE;
   
   Vector3d goalState;
   Vector3d initialState;
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      fullRobotModel = getRobotModel().createFullRobotModel();
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()) // set
      //if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
         communicationBridge = null;
         yoTime = null;
         fullRobotModel = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public class WalkingPathTestEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D EnvSet;

      public WalkingPathTestEnvironment()
      {
         EnvSet = DefaultCommonAvatarEnvironment.setUpGround("Ground");

         addBoxEnvironment(boxA);
         addBoxEnvironment(boxB);
         addBoxEnvironment(boxC);
         addBoxEnvironment(boxD);
         addBoxEnvironment(boxE);
         
         EnvSet.addBox(goalState.x + 0.5, goalState.y -0.6, goalState.x + 0.6, goalState.y +0.6, 2, YoAppearance.Gray());
         
         EnvSet.addSphere(goalState.x, goalState.y, goalState.z, 0.1, YoAppearance.Blue());
      }
      
      public void addBoxEnvironment(BoxInfo box)
      {
         EnvSet.addBox(box.centerX - box.sizeX/2, box.centerY - box.sizeY/2,
                       box.centerX + box.sizeX/2, box.centerY + box.sizeY/2, box.sizeZ, YoAppearance.Red());
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         // TODO Auto-generated method stub
         return EnvSet;
      }

      @Override
      public List<? extends Robot> getEnvironmentRobots()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public void createAndSetContactControllerToARobot()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
      {
         // TODO Auto-generated method stub
      }
   }

   private void setUpSimulationTestHelper(CommonAvatarEnvironmentInterface environment, DRCObstacleCourseStartingLocation startingLocation)
   {

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
      }

      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getSimpleRobotName(), startingLocation, simulationTestingParameters, getRobotModel());

   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testOne() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      // ******************************** //
      // Input ************************** //
      // ******************************** //
      initialState = new Vector3d(0.0, 0.0, 0.0);
      goalState = new Vector3d(8.0, -4.0, 0.0);
      
      setUpEnvironment();
      
      RRTNode startNode = new RRT2DNodeWalkingPath(0.0, 0.0);
      RRTNode goalNode = new RRT2DNodeWalkingPath(3.0, 2.0);
      RRT2DPlannerWalkingPath rrtPlanner = new RRT2DPlannerWalkingPath(startNode, goalNode, 0.3);

      RRTNode upperBoundNode = new RRT2DNodeWalkingPath(12.0, 3.0);
      RRTNode lowerBoundNode = new RRT2DNodeWalkingPath(-2.0, -7.0);
      
      rrtPlanner.rrtTree.setUpperBound(upperBoundNode);
      rrtPlanner.rrtTree.setLowerBound(lowerBoundNode);

      // ******************************** //

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;      
      CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface = new WalkingPathTestEnvironment();
      setUpSimulationTestHelper(commonAvatarEnvironmentInterface, selectedLocation);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCamera(simulationConstructionSet);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);


      // ******************************** //
      // RRT Planning ******************* //
      // ******************************** //
//      RRTNode nodeOne = new RRT2DNodeWalkingPath(1.0, 1.0);
//      RRTNode nodeTwo = new RRT2DNodeWalkingPath(1.2, 1.2);
      RRTNode nodeOne = new RRT2DNodeWalkingPath(0,0);
      RRTNode nodeTwo = new RRT2DNodeWalkingPath(0,0);
      
      for (int i = 0; i < 500; i++)
      {  
         boolean isConnected = rrtPlanner.expandTreeGoal(nodeOne, nodeTwo);
         simulationConstructionSet.addStaticLinkGraphics(getNodeConnection(nodeOne, nodeTwo));         
         
         if (isConnected == true)
         {            
            PrintTools.info("is Reached! "+i);
            break;
         }
      }
      
      
      
      // ******************************** //
      // Foot Step Planning ************* //
      // ******************************** //
      
      double swingTime = 2.0;
      double transferTime = 0.8;
      int steps = 3;

      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime);
      FootstepDataMessage footstepData;
      RobotSide robotSide;
      double footstepY;
      double footstepX;
      Point3D location;
      Quaternion orientation;

      robotSide = RobotSide.LEFT;
      footstepX = 0.2;
      footstepY = 0.14;
      location = new Point3D(footstepX, footstepY, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepData = new FootstepDataMessage(robotSide, location, orientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);

      robotSide = RobotSide.RIGHT;
      footstepX = 0.4;
      footstepY = -0.14;
      location = new Point3D(footstepX, footstepY, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepData = new FootstepDataMessage(robotSide, location, orientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);

      robotSide = RobotSide.LEFT;
      footstepX = 0.4;
      footstepY = 0.14;
      location = new Point3D(footstepX, footstepY, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepData = new FootstepDataMessage(robotSide, location, orientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);

      drcSimulationTestHelper.send(footsteps);
      double simulationTime = (swingTime + transferTime) * steps + 1.0;

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      // ******************************** //

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

   }

   private void setupCamera(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(2.0, 0.0, 1.1);
      Point3D cameraPosition = new Point3D(-5.0, 10.0, 15.0);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
   // ******************************** //

   class BoxInfo
   {
      double centerX;
      double centerY;
      double sizeX;
      double sizeY;
      double sizeZ;
      Point3D center;
      public BoxInfo(Point3D center, double[] size)
      {
         this.center = center;
         centerX = center.getX();
         centerY = center.getY();
         
         sizeX = size[0];
         sizeY = size[1];
         sizeZ = size[2];
      }
   }
      
   
   private void setUpEnvironment()
   {      
      boxA = new BoxInfo(new Point3D(1.5, -0.5, 0), new double[]{0.2, 1.0, 0.5});
      boxB = new BoxInfo(new Point3D(1.0, -2.0, 0), new double[]{0.5, 0.5, 0.5});
      boxC = new BoxInfo(new Point3D(4.0,  2.0, 0), new double[]{1.0, 1.0, 0.5});
      boxD = new BoxInfo(new Point3D(4.5, -1.5, 0), new double[]{1.8, 1.0, 0.5});
      boxE = new BoxInfo(new Point3D(3.5, -5.5, 0), new double[]{1.5, 1.5, 0.5});      
      
      shapeFactory = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();
      
      shapeFactory.addShape(shapeFactory.createBox(boxA.sizeX/2, boxA.sizeY/2, boxA.sizeZ/2));
      shapeFactory.addShape(shapeFactory.createBox(boxB.sizeX/2, boxB.sizeY/2, boxB.sizeZ/2));
      shapeFactory.addShape(shapeFactory.createBox(boxC.sizeX/2, boxC.sizeY/2, boxC.sizeZ/2));
      shapeFactory.addShape(shapeFactory.createBox(boxD.sizeX/2, boxD.sizeY/2, boxD.sizeZ/2));
      shapeFactory.addShape(shapeFactory.createBox(boxE.sizeX/2, boxE.sizeY/2, boxE.sizeZ/2));
      
      RigidBodyTransform transform;
      transform = new RigidBodyTransform();
      transform.setTranslation(boxA.center);
      collisionDetector.getCollisionObjects().get(0).setTransformToWorld(transform);
      
      transform = new RigidBodyTransform();
      transform.setTranslation(boxB.center);
      collisionDetector.getCollisionObjects().get(1).setTransformToWorld(transform);
      
      transform = new RigidBodyTransform();
      transform.setTranslation(boxC.center);
      collisionDetector.getCollisionObjects().get(2).setTransformToWorld(transform);
      
      transform = new RigidBodyTransform();
      transform.setTranslation(boxD.center);
      collisionDetector.getCollisionObjects().get(3).setTransformToWorld(transform);
      
      transform = new RigidBodyTransform();
      transform.setTranslation(boxE.center);
      collisionDetector.getCollisionObjects().get(4).setTransformToWorld(transform);
   }

   public ArrayList<Graphics3DObject> getNodeConnection(RRTNode nodeOne, RRTNode nodeTwo)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject nodeOneSphere = new Graphics3DObject();
      Graphics3DObject nodeTwoSphere = new Graphics3DObject();
      
      Graphics3DObject lineCapsule = new Graphics3DObject();
      
      Point3D translationNodeOne = new Point3D(nodeOne.getNodeData(0), nodeOne.getNodeData(1), 0);
      nodeOneSphere.translate(translationNodeOne);
      nodeOneSphere.addSphere(0.05, YoAppearance.DarkGray());
      
      Point3D translationNodeTwo = new Point3D(nodeTwo.getNodeData(0), nodeTwo.getNodeData(1), 0);
      nodeTwoSphere.translate(translationNodeTwo);
      nodeTwoSphere.addSphere(0.05, YoAppearance.DarkGray());
      
      Point3D translationLine = new Point3D((nodeOne.getNodeData(0)+nodeTwo.getNodeData(0))/2, (nodeOne.getNodeData(1)+nodeTwo.getNodeData(1))/2, 0);
      AxisAngle rotationLine = new AxisAngle(-(nodeOne.getNodeData(1)-nodeTwo.getNodeData(1)), (nodeOne.getNodeData(0)-nodeTwo.getNodeData(0)), 0, Math.PI/2);
      lineCapsule.translate(translationLine);
      lineCapsule.rotate(rotationLine);
      lineCapsule.addCapsule(0.03, nodeOne.getDistance(nodeTwo), YoAppearance.Gray());
      
      
      
      
      ret.add(nodeOneSphere);
      ret.add(nodeTwoSphere);
      ret.add(lineCapsule);

      return ret;
   }
   
}
