package us.ihmc.atlas.commonWalkingControlModules;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.dynamicReachability.DynamicReachabilityCalculator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlanner;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class AtlasLearningDynamicReachabilitySafetyFactor
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private DoubleYoVariable yoInitialTransferDuration;
   private DoubleYoVariable yoTransferDuration;
   private DoubleYoVariable yoSwingDuration;
   private static final double initialTransferDuration = 1.0;
   private static final double transferDuration = 0.8;
   private static final double swingDuration = 1.2;

   private SideDependentList<ContactableFoot> contactableFeet;

   private final SideDependentList<FramePose> footPosesAtTouchdown = new SideDependentList<FramePose>(new FramePose(), new FramePose());
   private final SideDependentList<YoFramePose> currentFootPoses = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   private DoubleYoVariable yoTime;

   private final ArrayList<Footstep> plannedFootsteps = new ArrayList<>();

   private YoFramePose yoNextFootstepPlan;
   private YoFramePose yoNextNextFootstepPlan;
   private YoFramePose yoNextNextNextFootstepPlan;

   private YoFramePose yoNextFootstepPose;
   private YoFramePose yoNextNextFootstepPose;
   private YoFramePose yoNextNextNextFootstepPose;
   private YoFrameConvexPolygon2d yoNextFootstepPolygon;
   private YoFrameConvexPolygon2d yoNextNextFootstepPolygon;
   private YoFrameConvexPolygon2d yoNextNextNextFootstepPolygon;

   private BipedSupportPolygons bipedSupportPolygons;
   private FootstepTestHelper footstepTestHelper;

   private ICPPlanner icpPlanner;
   private DynamicReachabilityCalculator dynamicReachabilityCalculator;

   private FullHumanoidRobotModel fullRobotModel;
   private CommonHumanoidReferenceFrames referenceFrames;
   private DRCRobotModel robotModel;

   public AtlasLearningDynamicReachabilitySafetyFactor()
   {
      Robot robot = new DummyRobot();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry robotRegistry = robot.getRobotsYoVariableRegistry();

      yoTime = robot.getYoTime();
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(DRCRobotModel.RobotTarget.SCS, getJointMap(), getContactPointParameters())
            {
               @Override
               public boolean editStepTimingForReachability()
               {
                  return true;
               }
            };
         }
      };

      fullRobotModel = robotModel.createFullRobotModel();
      RobotContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      ContactableBodiesFactory contactableBodiesFactory = contactPointParameters.getContactableBodiesFactory();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      contactableFeet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, referenceFrames);
      CapturePointPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();


      setupFeetFrames(robotRegistry, yoGraphicsListRegistry);
      createAdjustmentGraphics(robotRegistry, yoGraphicsListRegistry);

      yoInitialTransferDuration = new DoubleYoVariable("initialTransferDuration", robotRegistry);
      yoSwingDuration = new DoubleYoVariable("swingDuration", robotRegistry);
      yoTransferDuration = new DoubleYoVariable("transferDuration", robotRegistry);
      yoInitialTransferDuration.set(initialTransferDuration);
      yoSwingDuration.set(swingDuration);
      yoTransferDuration.set(transferDuration);


      icpPlanner = new ICPPlanner(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, robotRegistry, yoGraphicsListRegistry);
      icpPlanner.setOmega0(3.0);
      icpPlanner.setFinalTransferDuration(1.0);

      dynamicReachabilityCalculator = new DynamicReachabilityCalculator(icpPlanner, fullRobotModel, referenceFrames.getCenterOfMassFrame(), robotRegistry,
            yoGraphicsListRegistry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      Thread myThread = new Thread(scs);
      myThread.start();
   }


   private final RigidBodyTransform rootToWorld = new RigidBodyTransform();
   private final Vector3D positionInWorld = new Vector3D();
   private final Vector3D offset = new Vector3D();
   private final Quaternion rotation = new Quaternion();
   private static final double groundZ = 0.0;
   private static final double initialYaw = 0.0;
   private void initializeAtlasPose()
   {
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      DRCRobotJointMap jointMap = robotModel.getJointMap();

      // Avoid singularities at startup
      for (RobotSide robotSide : RobotSide.values)
      {
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW)).setQ(0.0); //leg_hpz
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL)).setQ(robotSide.negateIfRightSide(0.062)); //leg_hpx
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH)).setQ(-0.233); //leg_hpy
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH)).setQ(0.518); //leg_kny
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH)).setQ(-0.276); //leg_aky
         robot.getOneDegreeOfFreedomJoint(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL)).setQ(robotSide.negateIfRightSide(-0.062)); //leg_akx

         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.SHOULDER_YAW, robotSide.negateIfRightSide(0.785398)); //arm_shz
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.SHOULDER_ROLL, robotSide.negateIfRightSide(-0.1)); //arm_shx
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.ELBOW_PITCH, 3.00); //arm_ely
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.ELBOW_ROLL, robotSide.negateIfRightSide(1.8)); //arm_elx
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.FIRST_WRIST_PITCH, -0.30); //arm_wry
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.WRIST_ROLL, robotSide.negateIfRightSide(0.70)); //arm_wrx
         setArmJointPosition(robot, jointMap, robotSide, ArmJointName.SECOND_WRIST_PITCH, 0.15); //arm_wry2
      }

      robot.update();
      robot.getRootJointToWorldTransform(rootToWorld);
      rootToWorld.get(rotation, positionInWorld);

      GroundContactPoint gc1 = robot.getFootGroundContactPoints(RobotSide.LEFT).get(0);
      double pelvisToFoot = positionInWorld.getZ() - gc1.getPositionPoint().getZ();

      // Hardcoded for gazebo integration
      //      double pelvisToFoot = 0.887;

      positionInWorld.setZ(groundZ + pelvisToFoot);
      positionInWorld.add(offset);

      robot.setPositionInWorld(positionInWorld);

      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), rotation);
      double[] yawPitchRoll = frameOrientation.getYawPitchRoll();
      yawPitchRoll[0] = initialYaw;
      frameOrientation.setYawPitchRoll(yawPitchRoll);

      robot.setOrientation(frameOrientation.getQuaternionCopy());
      robot.update();
   }

   private void setArmJointPosition(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap, RobotSide robotSide, ArmJointName armJointName, double q)
   {
      String armJointString = jointMap.getArmJointName(robotSide, armJointName);
      if (armJointString == null)
         return;
      OneDegreeOfFreedomJoint joint = robot.getOneDegreeOfFreedomJoint(armJointString);
      if (joint == null)
         return;
      joint.setQ(q);
   }

   private final FramePose footstepPose = new FramePose();
   private double initialTime;
   private final FootstepTiming firstTiming = new FootstepTiming();
   private final FootstepTiming secondTiming = new FootstepTiming();
   private final FootstepTiming thirdTiming = new FootstepTiming();
   private void initializeAdjustmentTest()
   {
      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();

      int index = 0;
      for (Footstep footstep : footstepTestHelper.createFootsteps(0.25, 0.4, 4))
      {
         footstep.getPose(footstepPose);
         plannedFootsteps.get(index).setPose(footstepPose);

         index++;
      }

      for (RobotSide robotSide : RobotSide.values)
         contactStates.get(robotSide).setFullyConstrained();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      icpPlanner.clearPlan();
      firstTiming.setTimings(yoSwingDuration.getDoubleValue(), yoInitialTransferDuration.getDoubleValue());
      secondTiming.setTimings(yoSwingDuration.getDoubleValue(), yoTransferDuration.getDoubleValue());
      thirdTiming.setTimings(yoSwingDuration.getDoubleValue(), yoTransferDuration.getDoubleValue());
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(0), firstTiming);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(1), secondTiming);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(2), thirdTiming);

      RobotSide supportSide = plannedFootsteps.get(0).getRobotSide().getOppositeSide();

      icpPlanner.setSupportLeg(supportSide);
      icpPlanner.initializeForTransfer(yoTime.getDoubleValue());
      icpPlanner.compute(yoTime.getDoubleValue());

      initialTime = yoTime.getDoubleValue();
      updateAdjustmentViz();
   }

   private boolean firstTick = true;

   public void updateAdjustmentGraphic()
   {
      if (firstTick)
      {
         initializeAtlasPose();
         initializeAdjustmentTest();
         firstTick = false;
      }

      icpPlanner.clearPlan();
      firstTiming.setTimings(swingDuration, initialTransferDuration);
      secondTiming.setTimings(swingDuration, transferDuration);
      thirdTiming.setTimings(swingDuration, transferDuration);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(0), firstTiming);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(1), secondTiming);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(2), thirdTiming);

      icpPlanner.initializeForTransfer(initialTime);
      icpPlanner.compute(initialTime);
      dynamicReachabilityCalculator.setInTransfer();

      updateAdjustmentViz();
   }

   public void updateDynamicReachabilityCalculation()
   {
      dynamicReachabilityCalculator.setUpcomingFootstep(plannedFootsteps.get(0));
      dynamicReachabilityCalculator.verifyAndEnsureReachability();
   }


   private final FramePose nextPlannedFootstep = new FramePose();
   private final FramePose nextNextPlannedFootstep = new FramePose();
   private final FramePose nextNextNextPlannedFootstep = new FramePose();

   private final FrameConvexPolygon2d footstepPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempFootstepPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();

   private void updateAdjustmentViz()
   {
      nextPlannedFootstep.setToZero(plannedFootsteps.get(0).getSoleReferenceFrame());
      nextNextPlannedFootstep.setToZero(plannedFootsteps.get(0).getSoleReferenceFrame());
      nextNextNextPlannedFootstep.setToZero(plannedFootsteps.get(0).getSoleReferenceFrame());
      yoNextFootstepPlan.setAndMatchFrame(nextPlannedFootstep);
      yoNextNextFootstepPlan.setAndMatchFrame(nextNextPlannedFootstep);
      yoNextNextNextFootstepPlan.setAndMatchFrame(nextNextNextPlannedFootstep);

      if (plannedFootsteps.get(0) == null)
      {
         yoNextFootstepPose.setToNaN();
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextFootstepPolygon.hide();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (plannedFootsteps.get(0).getPredictedContactPoints() == null)
         plannedFootsteps.get(0).setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(plannedFootsteps.get(0).getRobotSide()).getContactPoints2d());

      double polygonShrinkAmount = 0.005;

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(plannedFootsteps.get(0).getSoleReferenceFrame(), plannedFootsteps.get(0).getPredictedContactPoints());
      convexPolygonShrinker.shrinkConstantDistanceInto(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextFootstepPose = new FramePose(plannedFootsteps.get(0).getSoleReferenceFrame());
      yoNextFootstepPose.setAndMatchFrame(nextFootstepPose);

      if (plannedFootsteps.get(1) == null)
      {
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (plannedFootsteps.get(1).getPredictedContactPoints() == null)
         plannedFootsteps.get(1).setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(plannedFootsteps.get(1).getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(plannedFootsteps.get(1).getSoleReferenceFrame(), plannedFootsteps.get(1).getPredictedContactPoints());
      convexPolygonShrinker.shrinkConstantDistanceInto(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextNextFootstepPose = new FramePose(plannedFootsteps.get(1).getSoleReferenceFrame());
      yoNextNextFootstepPose.setAndMatchFrame(nextNextFootstepPose);

      if (plannedFootsteps.get(2) == null)
      {
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (plannedFootsteps.get(2).getPredictedContactPoints() == null)
         plannedFootsteps.get(2).setPredictedContactPointsFromFramePoint2ds(contactableFeet.get(plannedFootsteps.get(2).getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(plannedFootsteps.get(2).getSoleReferenceFrame(), plannedFootsteps.get(2).getPredictedContactPoints());
      convexPolygonShrinker.shrinkConstantDistanceInto(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextNextNextFootstepPose = new FramePose(plannedFootsteps.get(2).getSoleReferenceFrame());
      yoNextNextNextFootstepPose.setAndMatchFrame(nextNextNextFootstepPose);

      //// FIXME: 3/30/17
      /*
      RobotSide supportSide = plannedFootsteps.get(0).getRobotSide().getOppositeSide();
      ContactableFoot contactableFoot = contactableFeet.get(supportSide.getOppositeSide());
      FramePose nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
      nextSupportPose.setToZero(plannedFootsteps.get(0).getSoleReferenceFrame());
      nextSupportPose.changeFrame(ReferenceFrame.getWorldFrame());
      footSpoof.setSoleFrame(nextSupportPose);
      */
   }

   private void setupFeetFrames(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {

      SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         // // TODO: 3/30/17  is this strictly necessary?
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FramePose startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.25 / 2.0));

         currentFootPoses.put(robotSide, new YoFramePose(sidePrefix + "FootPose", worldFrame, registry));

         ContactableFoot contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2d> contactFramePoints = contactableFoot.getContactPoints2d();

         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", contactableFoot.getRigidBody(), soleFrame, contactFramePoints, 0.5, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);

         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();
      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, yoGraphicsListRegistry);

      footstepTestHelper = new FootstepTestHelper(contactableFeet, ankleFrames);

      RobotSide currentSide = RobotSide.LEFT;
      for (int i = 0; i < 6; i++)
      {
         currentSide = currentSide.getOppositeSide();
         plannedFootsteps.add(new Footstep(contactableFeet.get(currentSide).getRigidBody(), currentSide, contactableFeet.get(currentSide).getSoleFrame()));
      }
   }

   private void createAdjustmentGraphics(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      yoNextFootstepPlan = new YoFramePose("nextFootstepPlan", worldFrame, registry);
      yoNextNextFootstepPlan = new YoFramePose("nextNextFootstepPlan", worldFrame, registry);
      yoNextNextNextFootstepPlan = new YoFramePose("nextNextNextFootstepPlan", worldFrame, registry);

      yoNextFootstepPose = new YoFramePose("nextFootstepPose", worldFrame, registry);
      yoNextNextFootstepPose = new YoFramePose("nextNextFootstepPose", worldFrame, registry);
      yoNextNextNextFootstepPose = new YoFramePose("nextNextNextFootstepPose", worldFrame, registry);

      yoNextFootstepPolygon = new YoFrameConvexPolygon2d("nextFootstep", "", worldFrame, 4, registry);
      yoNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextFootstep", "", worldFrame, 4, registry);
      yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextNextFootstep", "", worldFrame, 4, registry);

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<>();
      for (FramePoint2d point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(point.getPointCopy());
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));

      YoGraphicShape nextFootstepViz = new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0);
      YoGraphicShape nextNextFootstepViz = new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0);
      YoGraphicShape nextNextNextFootstepViz = new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0);

      YoArtifactPolygon nextFootstepArtifact = new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false);
      YoArtifactPolygon nextNextFootstepArtifact = new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false);
      YoArtifactPolygon nextNextNextFootstepArtifact = new YoArtifactPolygon("nextNextNextFootstep", yoNextNextNextFootstepPolygon, Color.blue, false);

      yoGraphicsListRegistry.registerYoGraphic("dummy", nextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic("dummy", nextNextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic("dummy", nextNextNextFootstepViz);

      yoGraphicsListRegistry.registerArtifact("dummy", nextFootstepArtifact);
      yoGraphicsListRegistry.registerArtifact("dummy", nextNextFootstepArtifact);
      yoGraphicsListRegistry.registerArtifact("dummy", nextNextNextFootstepArtifact);
   }

   private class DummyRobot extends Robot
   {
      public DummyRobot()
      {
         super("dummyRobot");
      }

      @Override
      public void update()
      {
         super.update();
         updateAdjustmentGraphic();
         updateDynamicReachabilityCalculation();
      }

   }

   public static void main(String[] args)
   {
      AtlasLearningDynamicReachabilitySafetyFactor test = new AtlasLearningDynamicReachabilitySafetyFactor();
   }
}
