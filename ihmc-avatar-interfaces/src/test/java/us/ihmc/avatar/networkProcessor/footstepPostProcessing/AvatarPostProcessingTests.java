package us.ihmc.avatar.networkProcessor.footstepPostProcessing;

import controller_msgs.ArmTrajectoryMessage;
import controller_msgs.FootstepDataListMessage;
import controller_msgs.FootstepDataMessage;
import controller_msgs.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.TrajectoryPoint1DMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import perception_msgs.TerrainMapMessage;
import toolbox_msgs.FootstepPlanningRequestPacket;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.BlockEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.LittleWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

public abstract class AvatarPostProcessingTests implements MultiRobotTestInterface
{
   private static final boolean keepSCSUp = false;

   protected SimulationTestingParameters simulationTestingParameters;
   protected SCS2AvatarTestingSimulation simulationTestHelper;

   private FootstepPlanningModule footstepPlanningModule;
   private DefaultFootstepPlannerParametersBasics footstepPlannerParameters;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(keepSCSUp && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());

      DRCRobotModel robotModel = getRobotModel();
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();

      footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(getRobotModel());

      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      footstepPlanningModule.closeAndDispose();
      footstepPlanningModule = null;

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   protected abstract String getLeftAnkleXName();

   protected abstract String getLeftAnkleYName();

   protected abstract String getRightAnkleXName();

   protected abstract String getRightAnkleYName();

   @Test
   public void testWalkingOffOfMediumPlatform()
   {
      double height = 0.3;
      OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup();
      startingLocation.addAdditionalOffset(new Vector3D(0.0, 0.0, height));

      BlockEnvironment blockEnvironment = new BlockEnvironment(1.0, 1.0, height);
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             blockEnvironment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(startingLocation);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      footstepPlannerParameters.setMaxStepZ(height + 0.05);
      footstepPlannerParameters.setIdealFootstepLength(0.28);
      footstepPlannerParameters.setMinFootholdPercent(0.95);

      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      PoseReferenceFrame startingFrame = new PoseReferenceFrame("startingFrame", ReferenceFrame.getWorldFrame());
      startingFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), startingLocation.getAdditionalOffset()));
      startingFrame.setOrientationAndUpdate(new Quaternion(startingLocation.getYaw(), 0.0, 0.0));

      FramePose3D goalPose = new FramePose3D(startingFrame);
      goalPose.getPosition().set(1.0, 0.0, -height);
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket request = getRequest(simulationTestHelper.getControllerFullRobotModel(),
                                                         blockEnvironment.getPlanarRegionsList(),
                                                         goalPose,
                                                         footstepPlannerParameters);
      //      request.setRequestedPathHeading(Math.toRadians(30.0));

      request.setRequestedSwingPlanner(SwingPlannerType.MULTI_WAYPOINT_POSITION.toByte());

      runTest(request);
   }

   @Test
   public void testSwingOverPlanarRegions()
   {
      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), environment, simulationTestingParameters);
      simulationTestHelper.start();

      footstepPlannerParameters.setMaxStepReach(0.6);
      footstepPlannerParameters.setMinStepWidth(0.05);
      footstepPlannerParameters.setMaxStepWidth(0.35);
      footstepPlannerParameters.setBodyBoxBaseZ(0.4);
      footstepPlannerParameters.setCheckForBodyBoxCollisions(false);
      footstepPlannerParameters.setCheckForPathCollisions(false);
      footstepPlannerParameters.setMinFootholdPercent(0.99);
      footstepPlannerParameters.setMaxStepZ(0.32);
      footstepPlannerParameters.setMinDistanceFromCliffBottoms(-1.0);
      footstepPlannerParameters.setMinDistanceFromCliffTops(-1.0);

      simulationTestHelper.simulateNow(0.5);

      FramePose3D goalPose = new FramePose3D();
      goalPose.getPosition().set(2.0, 0.0, 0.0);
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket requestPacket = getRequest(simulationTestHelper.getControllerFullRobotModel(),
                                                               environment.getPlanarRegionsList(),
                                                               goalPose,
                                                               footstepPlannerParameters);
      requestPacket.setTimeout(10.0);
      requestPacket.setRequestedSwingPlanner(SwingPlannerType.MULTI_WAYPOINT_POSITION.toByte());

      runTest(requestPacket);
   }

   @Test
   public void testWalkingOnStraightForwardLines()
   {
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), emptyEnvironment, simulationTestingParameters);
      simulationTestHelper.start();

//      ((YoBoolean) simulationTestHelper.findVariable("doPartialFootholdDetection")).set(false);
      ((YoDouble) simulationTestHelper.findVariable("fractionLoadIfFootHasFullSupport")).set(0.6);
      ((YoDouble) simulationTestHelper.findVariable("fractionTimeOnFootIfFootHasFullSupport")).set(0.6);
      ((YoDouble) simulationTestHelper.findVariable("fractionLoadIfOtherFootHasNoWidth")).set(0.7);
      ((YoDouble) simulationTestHelper.findVariable("fractionTimeOnFootIfOtherFootHasNoWidth")).set(0.7);
      //      SplitFractionCalculatorParametersBasics parameters = footstepPlanningModule.getSplitFractionParameters();
      //      parameters.setFractionLoadIfFootHasFullSupport(0.6);
      //      parameters.setFractionTimeOnFootIfFootHasFullSupport(0.6);
      //      parameters.setFractionLoadIfOtherFootHasNoWidth(0.7);
      //      parameters.setFractionTimeOnFootIfOtherFootHasNoWidth(0.7);

      // increase ankle damping to match the real robot better
      YoDouble damping_l_akx = (YoDouble) simulationTestHelper.findVariable("damping_" + getLeftAnkleXName());
      YoDouble damping_l_aky = (YoDouble) simulationTestHelper.findVariable("damping_" + getLeftAnkleYName());
      YoDouble damping_r_akx = (YoDouble) simulationTestHelper.findVariable("damping_" + getRightAnkleXName());
      YoDouble damping_r_aky = (YoDouble) simulationTestHelper.findVariable("damping_" + getRightAnkleYName());
      damping_l_akx.set(1.0);
      damping_l_aky.set(0.1);
      damping_r_akx.set(1.0);
      damping_r_aky.set(0.1);

      SideDependentList<YoEnum<ConstraintType>> footStates = new SideDependentList<>();
      // get foot states
      for (RobotSide robotSide : RobotSide.values)
      {
         String variableName = robotSide.getCamelCaseNameForStartOfExpression() + "FootCurrentState";
         @SuppressWarnings("unchecked")
         YoEnum<ConstraintType> footState = (YoEnum<ConstraintType>) simulationTestHelper.findVariable(variableName);
         footStates.put(robotSide, footState);
      }

      // setup camera
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(-10.0, 0.0, 1.0);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);

      final ContactPointController contactPointController = new ContactPointController(footStates);
      simulationTestHelper.addRobotControllerOnControllerThread(contactPointController);
      //      setupSupportViz();

      SteppingParameters steppingParameters = getRobotModel().getWalkingControllerParameters().getSteppingParameters();
      double footForwardOffset = steppingParameters.getFootForwardOffset();
      double footBackwardOffset = steppingParameters.getFootBackwardOffset();
      double footWidth = steppingParameters.getFootWidth();
      double toeWidth = steppingParameters.getToeWidth();

      ArrayList<Point2D> soleVertices = new ArrayList<Point2D>();
      soleVertices.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      soleVertices.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      ConvexPolygon2D defaultSolePolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(soleVertices));
      defaultSolePolygon.update();

      armsUp();

      double swingDuration = 0.6;
      double transferDuration = 0.5;
      FootstepPlan footstepPlan = new FootstepPlan();

      int numberOfSteps = 2;

      for (int i = 0; i < numberOfSteps; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         ArrayList<Point2D> newContactPoints = generateContactPointsForRotatedLineOfContact(0.0, 0.0, 0.0);

         PlannedFootstep footstep = new PlannedFootstep(robotSide);

         ReferenceFrame soleFrame = simulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
         FramePoint3D placeToStepInWorld = new FramePoint3D(soleFrame, 0.0, 0.0, 0.0);
         placeToStepInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         placeToStepInWorld.setX(0.3 * i);

         footstep.getFootstepPose().getPosition().set(placeToStepInWorld);
         newContactPoints.forEach(footstep.getFoothold()::addVertex);
         footstep.getFoothold().update();

         footstepPlan.addFootstep(footstep);
      }

      RobotSide robotSide = numberOfSteps % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;

      PlannedFootstep footstep = new PlannedFootstep(robotSide);

      ReferenceFrame soleFrame = simulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
      FramePoint3D placeToStepInWorld = new FramePoint3D(soleFrame, 0.0, 0.0, 0.0);
      placeToStepInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      placeToStepInWorld.setX(0.3 * (numberOfSteps - 1));

      footstep.getFootstepPose().getPosition().set(placeToStepInWorld);
      footstepPlan.addFootstep(footstep);

      FootstepPlannerRequest request = new FootstepPlannerRequest();

      for (RobotSide side : RobotSide.values)
      {
         FramePose3D footPose = new FramePose3D(simulationTestHelper.getControllerFullRobotModel().getSoleFrame(side));
         footPose.changeFrame(ReferenceFrame.getWorldFrame());
         request.getStartFootPoses().get(side).set(footPose);
      }

      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan,
                                                                                                                    swingDuration,
                                                                                                                    transferDuration);
      List<FootstepDataMessage> footsteps = new ArrayList<>();
      for (int i = 0; i < footstepDataListMessage.getFootstepDataList().size(); i++)
         footsteps.add(footstepDataListMessage.getFootstepDataList().get(i));

      int stepCounter = 0;
      for (RobotSide robotSide1 : RobotSide.values)
      {
         footStates.get(robotSide1).addListener(v ->
         {
            if (footStates.get(robotSide1).getEnumValue() == ConstraintType.SWING)
            {
               if (stepCounter == footsteps.size())
               {
                  return;
               }

               IDLObjectSequence<EuclidPoint3DMessage> contactPointsSeq = footsteps.remove(stepCounter).getPredictedContactPoints2d();
               List<Point3D> contactPoints3D = new ArrayList<>();
               for (int j = 0; j < contactPointsSeq.size(); j++)
                  contactPoints3D.add(new Point3D(contactPointsSeq.get(j).getPoint()));
               if (contactPoints3D.size() < 1)
               {
                  contactPointController.setNewContacts(defaultSolePolygon.getVertexBufferView(), robotSide1, true);
               }
               else
               {
                  List<Point2D> newContactPoints = contactPoints3D.stream().map(Point2D::new).collect(Collectors.toList());
                  contactPointController.setNewContacts(newContactPoints, robotSide1, true);
               }

            }
         });
      }

      simulationTestHelper.publishToController(footstepDataListMessage);

      boolean success = simulationTestHelper.simulateNow((swingDuration + transferDuration) * numberOfSteps + 3.0);
      assertTrue(success);
   }


   protected abstract double[] getStraightArmConfig(RobotSide robotSide);

   private void armsUp()
   {
      simulationTestHelper.simulateNow(0.1);

      // bring the arms in a stretched position
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
         armTrajectoryMessage.setRobotSide(robotSide.toByte());
         double[] armConfig = getStraightArmConfig(robotSide);
         for (int i = 0; i < armConfig.length; i++)
         {
            TrajectoryPoint1DMessage trajectoryPoint = new TrajectoryPoint1DMessage();
            trajectoryPoint.setPosition(armConfig[i]);
            trajectoryPoint.setTime(0.5);
            OneDoFJointTrajectoryMessage jointTrajectory = new OneDoFJointTrajectoryMessage();
            jointTrajectory.getTrajectoryPoints().add().set(trajectoryPoint);
            armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add().set(jointTrajectory);
         }
         simulationTestHelper.publishToController(armTrajectoryMessage);
      }

      simulationTestHelper.simulateNow(0.6);
   }

   private static FootstepPlanningRequestPacket getRequest(FullHumanoidRobotModel fullRobotModel,
                                                           PlanarRegionsList planarRegionsList,
                                                           FramePose3D goalPose,
                                                           DefaultFootstepPlannerParametersBasics footstepPlannerParameters)
   {
      FramePose3D leftFoot = new FramePose3D(fullRobotModel.getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFoot = new FramePose3D(fullRobotModel.getSoleFrame(RobotSide.RIGHT));
      leftFoot.changeFrame(ReferenceFrame.getWorldFrame());
      rightFoot.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket request = new FootstepPlanningRequestPacket();
      request.setRequestedInitialStanceSide(FootstepPlanningRequestPacket.ROBOT_SIDE_LEFT);
      request.getStartLeftFootPose().set(leftFoot);
      request.getStartRightFootPose().set(rightFoot);

      TerrainMapMessage terrainMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);
      request.getTerrainMapMessage().set(terrainMapMessage);

      SideDependentList<Pose3D> goalSteps = PlannerTools.createSquaredUpFootsteps(goalPose, footstepPlannerParameters.getIdealFootstepWidth());
      request.getGoalLeftFootPose().set(goalSteps.get(RobotSide.LEFT));
      request.getGoalRightFootPose().set(goalSteps.get(RobotSide.RIGHT));

      request.setPlanBodyPath(false);

      double timeout = 12.0;
      request.setTimeout(timeout);

      return request;
   }

   private void runTest(FootstepPlanningRequestPacket requestPacket)
   {
      YoRegistry registry = new YoRegistry("TestRegistry");
      YoFramePoint3D goalPosition = new YoFramePoint3D("goalPosition", ReferenceFrame.getWorldFrame(), registry);

      Pose3D goalMidFootPose = new Pose3D();
      goalMidFootPose.interpolate(requestPacket.getGoalLeftFootPose().getPose(), requestPacket.getGoalRightFootPose().getPose(), 0.5);
      goalPosition.set(goalMidFootPose.getPosition());
      simulationTestHelper.getRootRegistry().addChild(registry);
      simulationTestHelper.addYoGraphicDefinition(YoGraphicDefinitionFactory.newYoGraphicPoint3D("goalGraphic", goalPosition, 0.025, ColorDefinitions.Green()));

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setFromPacket(requestPacket);

      footstepPlanningModule.getFootstepPlannerParameters().set(footstepPlannerParameters);
      FootstepPlannerOutput plannerOutput = footstepPlanningModule.handleRequest(request);

      LogTools.info("output. " + plannerOutput.getFootstepPlanningResult());

      if (!plannerOutput.getFootstepPlanningResult().validForExecution())
      {
         fail("Invalid footstep plan: " + plannerOutput.getFootstepPlanningResult());
      }

      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 0.4, 0.8);
      for (FootstepDataMessage footstepDataMessage : footstepDataListMessage.getFootstepDataList())
      {
         footstepDataMessage.setSwingDuration(0.8);
         footstepDataMessage.setTransferDuration(0.4);
      }

      simulationTestHelper.publishToController(footstepDataListMessage);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();

      double stepTime = footstepDataListMessage.getDefaultSwingDuration() + footstepDataListMessage.getDefaultTransferDuration();
      if (stepTime < 0.5)
      {
         stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      }
      double simulationTime =  walkingControllerParameters.getDefaultInitialTransferTime() + walkingControllerParameters.getDefaultFinalTransferTime()
                               + stepTime * footstepDataListMessage.getFootstepDataList().size();

      boolean success = simulationTestHelper.simulateNow(simulationTime);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(goalMidFootPose.getPosition());
      center.addZ(0.7);

      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   private class ContactPointController implements RobotController
   {
      private List<? extends Point2DReadOnly> newContactPoints = null;
      private RobotSide robotSide = null;

      private AtomicBoolean setNewContactPoints = new AtomicBoolean(false);
      private boolean setOnStep = false;

      private SideDependentList<YoEnum<ConstraintType>> footStates;

      public ContactPointController(SideDependentList<YoEnum<ConstraintType>> footStates)
      {
         this.footStates = footStates;
      }

      /**
       * Changes the foot contact points of the robot. The contact points can be changed immediately or
       * when the foot is in swing.
       *
       * @param newContactPoints
       * @param robotSide
       * @param setOnStep
       */
      public void setNewContacts(List<? extends Point2DReadOnly> newContactPoints, RobotSide robotSide, boolean setOnStep)
      {
         if (setNewContactPoints.get())
         {
            System.err.println("New contact points are already waiting to be set.");
            return;
         }

         this.newContactPoints = newContactPoints;
         this.robotSide = robotSide;
         this.setOnStep = setOnStep;

         setNewContactPoints.set(true);
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return null;
      }

      @Override
      public String getName()
      {
         return null;
      }

      @Override
      public String getDescription()
      {
         return null;
      }

      @Override
      public void doControl()
      {
         if (setNewContactPoints.get())
         {
            if (!setOnStep)
               setNewContacts();
            else if (footStates.get(robotSide).getEnumValue() == ConstraintType.SWING)
               setNewContacts();
         }
      }

      private void setNewContacts()
      {
         String footJointName = simulationTestHelper.getControllerFullRobotModel().getFoot(robotSide).getParentJoint().getName();
         Robot robot = simulationTestHelper.getRobot();

         int pointIndex = 0;
         List<GroundContactPoint> allGroundContactPoints = new ArrayList<>();
         for (SimJointBasics joint : robot.getAllJoints())
         {
            allGroundContactPoints.addAll(joint.getAuxialiryData().getGroundContactPoints());
         }

         for (GroundContactPoint point : allGroundContactPoints)
         {
            if (point.getParentJoint().getName().equals(footJointName))
            {
               Point2DReadOnly newContactPoint = newContactPoints.get(pointIndex);

               point.getInContact().set(false);
               point.getOffset().getPosition().setX(newContactPoint.getX());
               point.getOffset().getPosition().setY(newContactPoint.getY());
               pointIndex++;
            }
         }

         //         if (footContactsInAnkleFrame != null)
         //         {
         //            footContactsInAnkleFrame.set(robotSide, newContactPoints);
         //         }

         setNewContactPoints.set(false);
      }

   }

   private ArrayList<Point2D> generateContactPointsForRotatedLineOfContact(double angle, double xLine, double yLine)
   {
      double lineWidth = 0.01;

      // build default foot polygon:
      SteppingParameters steppingParameters = getRobotModel().getWalkingControllerParameters().getSteppingParameters();
      double footForwardOffset = steppingParameters.getFootLength() / 2.0;
      double footBackwardOffset = footForwardOffset;
      double footWidth = steppingParameters.getFootWidth();
      double toeWidth = steppingParameters.getToeWidth();

      ArrayList<Point2D> soleVertices = new ArrayList<Point2D>();
      soleVertices.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      soleVertices.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      soleVertices.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      ConvexPolygon2D solePolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(soleVertices));
      solePolygon.update();

      // shrink polygon and project line origin inside
      ConvexPolygon2D shrunkSolePolygon = new ConvexPolygon2D();
      ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
      shrinker.scaleConvexPolygon(solePolygon, lineWidth / 2.0 + (footWidth - toeWidth) / 2.0, shrunkSolePolygon);

      Point2D lineOrigin = new Point2D(xLine, yLine);
      shrunkSolePolygon.orthogonalProjection(lineOrigin);

      // transform line and compute intersections with default foot polygon
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(angle);
      transform.getTranslation().set(lineOrigin.getX(), lineOrigin.getY(), 0.0);

      Line2D line = new Line2D(new Point2D(0.0, 0.0), new Vector2D(1.0, 0.0));
      line.applyTransform(transform);

      line.shiftToLeft(lineWidth / 2.0);
      Point2DBasics[] leftIntersections = solePolygon.intersectionWith(line);
      line.shiftToRight(lineWidth);
      Point2DBasics[] rightIntersections = solePolygon.intersectionWith(line);

      ArrayList<Point2D> ret = new ArrayList<Point2D>();
      ret.add(new Point2D(leftIntersections[0]));
      ret.add(new Point2D(leftIntersections[1]));
      ret.add(new Point2D(rightIntersections[0]));
      ret.add(new Point2D(rightIntersections[1]));
      return ret;
   }
}
