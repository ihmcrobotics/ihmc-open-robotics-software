package us.ihmc.avatar.rrtWalkingpath;

import static org.junit.Assert.assertTrue;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

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
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.walkingpath.footstep.SkeletonPathFootStep;
import us.ihmc.manipulation.planning.walkingpath.footstep.SkeletonPathFootStepPlanner;
import us.ihmc.manipulation.planning.walkingpath.rrtplanner.RRT2DNodeWalkingPath;
import us.ihmc.manipulation.planning.walkingpath.rrtplanner.RRT2DPlannerWalkingPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
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
   
   Point3D goalState;
   Point3D initialState;
   
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

         addBoxEnvironment(RRT2DNodeWalkingPath.boxes[0], YoAppearance.Grey());
         
         for(int i=1;i<RRT2DNodeWalkingPath.boxes.length;i++)
         {
            addBoxEnvironment(RRT2DNodeWalkingPath.boxes[i], YoAppearance.Red());
         }
                  
         EnvSet.addSphere(goalState.getX(), goalState.getY(), goalState.getZ(), 0.1, YoAppearance.Blue());
         
         RigidBodyTransform location = new RigidBodyTransform();
         location.setTranslation(initialState);
         
         // EnvSet.addCylinder(location, 1.0, 0.4, YoAppearance.Wheat());
      }
      
      public void addBoxEnvironment(RRT2DNodeWalkingPath.BoxInfo box, AppearanceDefinition app)
      {
         EnvSet.addBox(box.centerX - box.sizeX/2, box.centerY - box.sizeY/2,
                       box.centerX + box.sizeX/2, box.centerY + box.sizeY/2, box.sizeZ, app);
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

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCamera(simulationConstructionSet);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      // ******************************** //
      // RRT Planning ******************* //
      // ******************************** //
           
      
      rrtPlanner.expandTreeGoal(2000);      
      PrintTools.info("path has "+rrtPlanner.getOptimalPath().size()+" nodes");
            
      simulationConstructionSet.addStaticLinkGraphics(getPrintNodePath(rrtPlanner.getOptimalPath(), YoAppearance.Red()));
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      
      rrtPlanner.updateOptimalPath(50, 200);
      simulationConstructionSet.addStaticLinkGraphics(getPrintNodePath(rrtPlanner.getOptimalPath(), YoAppearance.Aqua()));
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      
      PrintTools.info("shortcut optimized path has "+rrtPlanner.getOptimalPath().size()+" nodes");
      
            
      // footstep
      rrtPlanner.createFootStepPlanner(0.5, 0.35);
      
      SkeletonPathFootStepPlanner footStepPlanner = rrtPlanner.getFootStepPlanner();
      
      simulationConstructionSet.addStaticLinkGraphics(getPrintSegments(footStepPlanner.pathSegmentsLeftSide.get()));
      simulationConstructionSet.addStaticLinkGraphics(getPrintSegments(footStepPlanner.pathSegmentsRightSide.get()));
      
      footStepPlanner.setZeroStep(RobotSide.LEFT);
      footStepPlanner.createFootSteps();
            
      simulationConstructionSet.addStaticLinkGraphics(getPrintFootSteps(footStepPlanner.footSteps));
      simulationConstructionSet.addStaticLinkGraphics(getPrintSphere(footStepPlanner.tempPoint, YoAppearance.Black()));
      
      
      
      
      
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

      
      PrintTools.info("END!!");
      
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
   // Print
   // ******************************** //

   
      
   private ArrayList<Graphics3DObject> getPrintNodePath(ArrayList<RRTNode> nodePath, AppearanceDefinition app)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();
      
      for(int i =0;i<nodePath.size()-1;i++)
      {
         ret.addAll(getPrintNodeConnection(nodePath.get(i), nodePath.get(i+1), app));         
      }
      
      return ret;
   }


   private ArrayList<Graphics3DObject> getPrintNodeConnection(RRTNode nodeOne, RRTNode nodeTwo)
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
      lineCapsule.addCapsule(0.02, nodeOne.getDistance(nodeTwo), YoAppearance.Gray());
      
      ret.add(nodeOneSphere);
      ret.add(nodeTwoSphere);
      ret.add(lineCapsule);

      return ret;
   }
   
   private ArrayList<Graphics3DObject> getPrintNodeConnection(RRTNode nodeOne, RRTNode nodeTwo, AppearanceDefinition app)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject nodeOneSphere = new Graphics3DObject();
      Graphics3DObject nodeTwoSphere = new Graphics3DObject();
      
      Graphics3DObject lineCapsule = new Graphics3DObject();
      
      Point3D translationNodeOne = new Point3D(nodeOne.getNodeData(0), nodeOne.getNodeData(1), 0);
      nodeOneSphere.translate(translationNodeOne);
      nodeOneSphere.addSphere(0.05, app);
      
      Point3D translationNodeTwo = new Point3D(nodeTwo.getNodeData(0), nodeTwo.getNodeData(1), 0);
      nodeTwoSphere.translate(translationNodeTwo);
      nodeTwoSphere.addSphere(0.05, app);
      
      Point3D translationLine = new Point3D((nodeOne.getNodeData(0)+nodeTwo.getNodeData(0))/2, (nodeOne.getNodeData(1)+nodeTwo.getNodeData(1))/2, 0);
      AxisAngle rotationLine = new AxisAngle(-(nodeOne.getNodeData(1)-nodeTwo.getNodeData(1)), (nodeOne.getNodeData(0)-nodeTwo.getNodeData(0)), 0, Math.PI/2);
      lineCapsule.translate(translationLine);
      lineCapsule.rotate(rotationLine);
      lineCapsule.addCapsule(0.02, nodeOne.getDistance(nodeTwo), app);
      
      ret.add(nodeOneSphere);
      ret.add(nodeTwoSphere);
      ret.add(lineCapsule);

      return ret;
   }
   
   private ArrayList<Graphics3DObject> getPrintFootStep(FootstepDataListMessage footsteps)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      for(int i =0;i<footsteps.size();i++)
      {
         Graphics3DObject singleStep = new Graphics3DObject();
         
         singleStep.translate(footsteps.get(i).location);
         if(footsteps.get(i).robotSide == RobotSide.RIGHT)
         {
            singleStep.addSphere(0.05, YoAppearance.Blue());
         }
         else
         {
            singleStep.addSphere(0.05, YoAppearance.Green());
         }
         
         ret.add(singleStep);
      }
      return ret;
   }
   
   private Graphics3DObject getPrintFootStep(SkeletonPathFootStep footstep)
   {
      Graphics3DObject ret = new Graphics3DObject();
      
      double sizeX = 0.3;
      double sizeY = 0.15;
      double sizeZ = 0.01;      
      
      Point3D location = new Point3D(footstep.getLocation().getX(), footstep.getLocation().getY(), 0);
      AxisAngle rotation = new AxisAngle();
      rotation.appendYawRotation(footstep.getYawAngle());
      
      ret.translate(location);
      ret.rotate(rotation);
      if(footstep.getRobotSide() == RobotSide.RIGHT)
      {
         ret.addCube(sizeX, sizeY, sizeZ, YoAppearance.Green());
         //PrintTools.info("R "+footstep.getLocation().getX()+" "+footstep.getLocation().getY());
      }
      
      else
      {
         ret.addCube(sizeX, sizeY, sizeZ, YoAppearance.Blue());
         //PrintTools.info("L "+footstep.getLocation().getX()+" "+footstep.getLocation().getY());
      }
         
      return ret;
   }
   
   private ArrayList<Graphics3DObject> getPrintFootSteps(ArrayList<SkeletonPathFootStep> footSteps)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();
      
      for(int i=0;i<footSteps.size();i++)
      {
         ret.add(getPrintFootStep(footSteps.get(i)));
//         PrintTools.info(" "+footSteps.get(i).getLocation().getX()+" "+footSteps.get(i).getLocation().getY());
      }
         
      return ret;
   }
   
   private Graphics3DObject getPrintSphere(Point2D aPoint, AppearanceDefinition app)
   {
      Graphics3DObject aSphere = new Graphics3DObject();
      aSphere.translate(aPoint.getX(), aPoint.getY(), 0);
      aSphere.addSphere(0.05, app);
      return aSphere;
   }
   
   private ArrayList<Graphics3DObject> getPrintSegment(Line2D pathSegment)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject nodeOneSphere = new Graphics3DObject();
      Graphics3DObject nodeTwoSphere = new Graphics3DObject();
      
      Graphics3DObject lineCapsule = new Graphics3DObject();
      
      Point3D translationNodeOne = new Point3D(pathSegment.getX1(), pathSegment.getY1(), 0);
      nodeOneSphere.translate(translationNodeOne);
      nodeOneSphere.addSphere(0.05, YoAppearance.DarkGray());
      
      Point3D translationNodeTwo = new Point3D(pathSegment.getX2(), pathSegment.getY2(), 0);
      nodeTwoSphere.translate(translationNodeTwo);
      nodeTwoSphere.addSphere(0.05, YoAppearance.DarkGray());
      
      Point3D translationLine = new Point3D((pathSegment.getX1()+pathSegment.getX2())/2, (pathSegment.getY1()+pathSegment.getY2())/2, 0);
      AxisAngle rotationLine = new AxisAngle(-(pathSegment.getY1()-pathSegment.getY2()), (pathSegment.getX1()-pathSegment.getX2()), 0, Math.PI/2);
      lineCapsule.translate(translationLine);
      lineCapsule.rotate(rotationLine);
      lineCapsule.addCapsule(0.02, pathSegment.getP1().distance(pathSegment.getP2()), YoAppearance.Gray());
      
      ret.add(nodeOneSphere);
      ret.add(nodeTwoSphere);
      ret.add(lineCapsule);

      return ret;
   }
   
   private ArrayList<Graphics3DObject> getPrintSegments(ArrayList<Line2D> pathSegments)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      for(int i =0;i<pathSegments.size();i++)
      {
         ret.addAll(getPrintSegment(pathSegments.get(i)));
      }

      return ret;
   }

}
