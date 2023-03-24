package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.euclid.YoOrientation2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RaymanController implements Controller
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private static final double DESIRED_CENTER_OF_MASS_HEIGHT = 1.0;

   private final YoRegistry registry = new YoRegistry("Controller");
   private final YoBoolean walk = new YoBoolean("walk", registry);
   private final YoDouble transferDuration = new YoDouble("transferDuration", registry);
   private final YoDouble swingDuration = new YoDouble("swingDuration", registry);
   // calcculate the desired step length based on desired top velocity by calculating orbital energy.
   private YoDouble desiredStepLength = new YoDouble("desiredStepLength", registry);
   private final WholeBodyControllerCore wholeBodyControllerCore;
   private final ControllerInput controllerInput;
   private WholeBodyControlCoreToolbox toolbox;

   private final StateMachine<WalkingStateEnum, State> stateMachine;
   private final RaymanWalker robotJae;

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   private final DefaultYoPIDSE3Gains gains = new DefaultYoPIDSE3Gains("gains", GainCoupling.XYZ, false, registry);

   BipedSupportPolygons bipedSupportPolygons;
   SideDependentList<PlaneContactStateCommand> planeContactStateCommands;
   private final ReferenceFrame midFeetFrame;
   private final SideDependentList<ReferenceFrame> soleFrames, soleZUpFrames;

   private final YoGraphicDefinition graphicsGroup;
   private final YoFramePoint3D measuredCoM = new YoFramePoint3D("measuredCoM", WORLD_FRAME, registry);
   private final YoFramePoint3D measuredCoMVelocity = new YoFramePoint3D("measuredCoMVelocity", WORLD_FRAME, registry);
   private final YoFramePoint3D desiredCoM = new YoFramePoint3D("desiredCenterOfMassPoint", WORLD_FRAME, registry);
   private final YoFramePoint3D desireCMP = new YoFramePoint3D("desiredCMP", WORLD_FRAME, registry);
   private final SideDependentList<YoFramePoint3D> soleFramePoints = new SideDependentList<>();

   private final YoFramePoint3D measuredICP = new YoFramePoint3D("measuredCapturePoint", WORLD_FRAME, registry);
   private static final double OMEGA = Math.sqrt(9.81 / DESIRED_CENTER_OF_MASS_HEIGHT);

   private final YoDouble desiredComVelocityX = new YoDouble("desiredComVelocityX", registry);

   private final YoFrameYawPitchRoll ankleDesiredYawPitchRoll;
   private final YoFrameVector3D ankleDesiredAngularVelocity;
   private final FramePose3D zeroPose3D = new FramePose3D();
   private final FrameVector3D zeroVector = new FrameVector3D();

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final YoFramePoint3D desiredNextFootstepPositionForVisual = new YoFramePoint3D("desiredFootstepPosition", WORLD_FRAME, registry);
   private final YoFramePoint3D desiredCurrentFootPosition = new YoFramePoint3D("desiredCurrentFootPosition", WORLD_FRAME, registry);

   private final ArrayList<Point3D> leftHipTrajectory = new ArrayList<>();
   private final BagOfBalls leftHipTrajectoryBalls = new BagOfBalls(500, 0.05, "leftHipTrajectory", YoAppearance.Black(), registry, yoGraphicsListRegistry);

   // this is icp0 or final icp in the backward icp equation
   private final YoFramePoint3D referenceICP = new YoFramePoint3D("referenceICP", WORLD_FRAME, registry);
   private final YoFramePoint3D referenceCOM = new YoFramePoint3D("referenceCOM", WORLD_FRAME, registry);

   private final YoFramePoint3D guiDesiredICP = new YoFramePoint3D("guiDesiredICP", WORLD_FRAME, registry);
   private final YoFrameVector3D guiDesiredICPVelocity = new YoFrameVector3D("guiDesiredICPVelocity", WORLD_FRAME, registry);

   // dcm keypoints for generating dcm trajectories for the mass to follow
   private final YoFramePoint3D dcmStart = new YoFramePoint3D("dcmStart", WORLD_FRAME, registry);
   private final YoFramePoint3D dcmFinal = new YoFramePoint3D("dcmFinal", WORLD_FRAME, registry);
   private final YoPolynomial dcmTrajectory = new YoPolynomial("dcmTrajectory", 6, registry);
   private final YoFramePoint3D dcmWantedNow = new YoFramePoint3D("dcmWantedNow", WORLD_FRAME, registry);

   // walk mode
   private enum WalkMode
   {
      ASIMO, LIP
   }
   private final YoEnum<WalkMode> walkModeYoEnum = new YoEnum<>("walkMode", registry, WalkMode.class);

   private enum WalkingStateEnum
   {
      STANDING, TRANSFER_TO_LEFT, TRANSFER_TO_RIGHT, LEFT_SUPPORT, RIGHT_SUPPORT
   };

   // planning
   private final YoInteger numStepsToPlan = new YoInteger("numStepsToPlan", registry);
   private final YoDouble strideLength = new YoDouble("strideLength", registry);
   private final YoDouble stepWidth = new YoDouble("stepWidth", registry);
   private final YoOrientation2D stepDirection = new YoOrientation2D("stepDirection", registry);
   ArrayList<Footstep> plannedFootsteps = new ArrayList<>();
   private final YoFramePoint3D nextICP = new YoFramePoint3D("nextICP", WORLD_FRAME, registry);

   private NFootstepListVisualizer visualizerPlannedFootSteps;
   SideDependentList<ContactablePlaneBody> contactableFeet;

   private final YoPolynomial doubleSupportICPTrajectory = new YoPolynomial("doubleSupportICPTrajectory", 4, registry);
   private final YoBoolean dsICPTraj = new YoBoolean("dsICPTraj", registry);

   public RaymanController(ControllerInput controllerInput,
                           ControllerOutput controllerOutput,
                           double controlDT,
                           double gravityZ,
                           RaymanDefinition robotDefinition)
   {
      this.controllerInput = controllerInput;
      this.robotJae = new RaymanWalker(controllerInput, controllerOutput, robotDefinition);

      // Create an empty graphics registry (is needed due to SCS 1)

      wholeBodyControllerCore = createControllerCore(controlDT, gravityZ, yoGraphicsListRegistry);

      soleFrames = new SideDependentList<>(side -> robotJae.getFootContactableBody(side).getSoleFrame());
      soleZUpFrames = new SideDependentList<>(side -> new ZUpFrame(soleFrames.get(side), soleFrames.get(side).getName() + "ZUp"));
      midFeetFrame = new MidFrameZUpFrame("midFeetZUpFrame", WORLD_FRAME, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));

      bipedSupportPolygons = new BipedSupportPolygons(midFeetFrame, soleZUpFrames, soleFrames, registry, yoGraphicsListRegistry);

      stateMachine = createStateMachine();
      transferDuration.set(3.0);
      swingDuration.set(1.2);

      //TODO: this should be calculated
      desiredStepLength.set(0.15);
      desiredComVelocityX.set(0.5);

      soleFramePoints.put(RobotSide.LEFT, new YoFramePoint3D(RobotSide.LEFT.getCamelCaseName() + "framePoint", ReferenceFrame.getWorldFrame(), registry));
      soleFramePoints.put(RobotSide.RIGHT, new YoFramePoint3D(RobotSide.RIGHT.getCamelCaseName() + "framePoint", ReferenceFrame.getWorldFrame(), registry));
      //  create our graphics here
      this.graphicsGroup = createVisualization();

      ankleDesiredYawPitchRoll = new YoFrameYawPitchRoll("ankleDesiredYawPitchRoll", ReferenceFrame.getWorldFrame(), registry);
      ankleDesiredAngularVelocity = new YoFrameVector3D("ankleDesiredAngularVelocity", ReferenceFrame.getWorldFrame(), registry);

      // Set walk mode here
      walkModeYoEnum.set(WalkMode.LIP);

      // planning param
      numStepsToPlan.set(5);
      strideLength.set(0.3);
      stepWidth.set(0.6);

      // planning visual
      contactableFeet = new SideDependentList<>(robotJae::getFootContactableBody);
      visualizerPlannedFootSteps = new NFootstepListVisualizer(contactableFeet, yoGraphicsListRegistry, registry);

      dsICPTraj.set(false);
   }

   public YoGraphicDefinition createVisualization()
   {
      // define a group of YoGraphic definitions
      YoGraphicGroupDefinition graphicsGroup = new YoGraphicGroupDefinition("Controller");
      //      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desiredCapturePoint", desiredCapturePointPosition, 0.02, ColorDefinitions.Red()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("measuredICP", measuredICP,
                                                                            0.1,
                                                                            ColorDefinitions.BurlyWood()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("measuredCoM", measuredCoM, 0.1, ColorDefinitions.Black()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desiredCenterOfMass", desiredCoM, 0.1, ColorDefinitions.GreenYellow()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("leftSoleFramePoint",
                                                                            soleFramePoints.get(RobotSide.LEFT),
                                                                            0.05,
                                                                            ColorDefinitions.Red()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("rightSoleFramePoint",
                                                                            soleFramePoints.get(RobotSide.RIGHT),
                                                                            0.05,
                                                                            ColorDefinitions.Green()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desiredNextFootstepPosition", desiredNextFootstepPositionForVisual,
                                                                            0.05,
                                                                            ColorDefinitions.Gold()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desiredCurrentFootPosition",
                                                                            desiredCurrentFootPosition,
                                                                            0.1,
                                                                            ColorDefinitions.Blue()));

      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPointcloud3D("leftHipTrajectory", leftHipTrajectoryBalls.getPositions(), 0.1, ColorDefinitions.Blue()));

      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("dcmWantedNow", dcmWantedNow, 0.12, ColorDefinitions.LightGoldenrodYellow()));

      //      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desiredCentroidalMomentPivotPoint", desiredCentroidalMomentPivotPosition, 0.02, ColorDefinitions.Black()));
      return graphicsGroup;
   }

   private WholeBodyControllerCore createControllerCore(double controlDT, double gravityZ, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      // This time the robot has a floating joint.
      FloatingJointBasics rootJoint = robotJae.getRootJoint();
      RigidBodyBasics elevator = robotJae.getElevator();

      // These are all the joints of the robot arm.
      JointBasics[] jointsArray = MultiBodySystemTools.collectSubtreeJoints(elevator);
      OneDoFJoint[] controlledJoints = MultiBodySystemTools.filterJoints(jointsArray, OneDoFJoint.class);

      // This class contains basic optimization settings required for QP formulation.
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new RaymanOptimizationSettings();
      // This is the toolbox for the controller core with everything it needs to run properly.
      toolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                gravityZ,
                                                rootJoint,
                                                controlledJoints,
                                                robotJae.getCenterOfMassFrame(),
                                                controllerCoreOptimizationSettings,
                                                yoGraphicsListRegistry,
                                                registry);

      // The controller core needs all the possibly contacting bodies of the robot to create all the modules needed for later.
      toolbox.setupForInverseDynamicsSolver(Arrays.asList(robotJae.getFootContactableBody(RobotSide.LEFT), robotJae.getFootContactableBody(RobotSide.RIGHT)));


      /*
       * We register all the commands that we will use in this controller, i.e. commands for the
       * feet that we'll for the swing, an orientation command for the pelvis to keep it level to the
       * ground, and the command for controlling the center of mass.
       */
      FeedbackControllerTemplate template = new FeedbackControllerTemplate();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = robotJae.getFoot(robotSide);
         template.enableSpatialFeedbackController(foot);
         template.enableOrientationFeedbackController(foot);
      }

      template.enableOrientationFeedbackController(robotJae.getPelvis());
      template.enableCenterOfMassFeedbackController();

      // Finally we can create the controller core.
      return new WholeBodyControllerCore(toolbox, template, registry);
   }

   private StateMachine<WalkingStateEnum, State> createStateMachine()
   {
      // The creation of a state machine is done using a factory:
      StateMachineFactory<WalkingStateEnum, State> factory = new StateMachineFactory<>(WalkingStateEnum.class);
      // The namePrefix will be used to create internally some YoVariables.
      factory.setNamePrefix("stateMachine");
      // The registry to which the YoVariables will be registered with.
      factory.setRegistry(registry);
      // We will need a clock to have access to the time spent in each state for computing trajectories.
      factory.buildYoClock(controllerInput::getTime);

      /*
       * Then we get to the point where actually create the state and the transitions. In this example, we
       * will only use what is called here "done transitions". These transitions get triggered as a state
       * reports that it is done, when triggered the state machine goes to the next state.
       */
      // Here we setup the STANDING state. When done, the state machine will transition to the TRANSFER_TO_LEFT state.
      factory.addStateAndDoneTransition(WalkingStateEnum.STANDING, new StandingState(), WalkingStateEnum.TRANSFER_TO_LEFT);

      // Here using the transitions, we create a cycle that will result in having the robot walking indefinitely.
      factory.addStateAndDoneTransition(WalkingStateEnum.TRANSFER_TO_LEFT, new TransferState(RobotSide.LEFT), WalkingStateEnum.LEFT_SUPPORT);
      factory.addStateAndDoneTransition(WalkingStateEnum.LEFT_SUPPORT, new SingleSupportState(RobotSide.LEFT), WalkingStateEnum.TRANSFER_TO_RIGHT);
      factory.addStateAndDoneTransition(WalkingStateEnum.TRANSFER_TO_RIGHT, new TransferState(RobotSide.RIGHT), WalkingStateEnum.RIGHT_SUPPORT);
      factory.addStateAndDoneTransition(WalkingStateEnum.RIGHT_SUPPORT, new SingleSupportState(RobotSide.RIGHT), WalkingStateEnum.TRANSFER_TO_LEFT);

      // Finally we can build the state machine which will start with the STANDING state.
      return factory.build(WalkingStateEnum.STANDING);
   }

   @Override
   public void initialize()
   {
      // We initialize the gains. As in the previous examples, the values here are rather arbitrary.
      gains.setPositionProportionalGains(100.0);
      gains.setPositionDerivativeGains(10.0);
      gains.setOrientationProportionalGains(100.0);
      gains.setOrientationDerivativeGains(10.0);
   }

   /**
    * This time, the {@code doControl} method is rather empty as most of the controller is actually
    * implemented in the different states declared further down.
    */
   @Override
   public void doControl()
   {
      // We update the configuration state of our inverse dynamics robot model from the latest state of the simulated robot.
      robotJae.updateInverseDynamicsRobotState();

      /*
       * Here we request the state machine to call the doAction() method of the active state and to check
       * if the transition to the next state should be engaged. See further below for the implementation
       * of the doAction() method for each state.
       */
      stateMachine.doActionAndTransition();

      /*
       * As the pelvis is to kept level to the ground independently to the active state, we can simply
       * setup the command here. MAIN BODY IS PELVIS IN RAYMAN.
       */
      OrientationFeedbackControlCommand pelvisOrientationCommand = new OrientationFeedbackControlCommand();
      pelvisOrientationCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS); // sets control mode to
      // inverse dynamics
      pelvisOrientationCommand.set(robotJae.getElevator(), robotJae.getPelvis());
      pelvisOrientationCommand.setGains(gains.getOrientationGains());
      pelvisOrientationCommand.setInverseDynamics(zeroPose3D.getOrientation(), zeroVector, zeroVector);
      pelvisOrientationCommand.setWeightForSolver(1.0);
      controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationCommand);

      // Submit all the objectives to be achieved to the controller core.
      // Magic happens here.
      wholeBodyControllerCore.compute(controllerCoreCommand);

      // And collect the output to update the simulated robot.
      JointDesiredOutputListReadOnly outputForLowLevelController = wholeBodyControllerCore.getOutputForLowLevelController();

      for (int i = 0; i < outputForLowLevelController.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly oneDoFJoint = outputForLowLevelController.getOneDoFJoint(i);
         JointDesiredOutputReadOnly jointDesiredOutput = outputForLowLevelController.getJointDesiredOutput(i);
         robotJae.setDesiredEffort(oneDoFJoint.getName(), jointDesiredOutput.getDesiredTorque());
      }

      for (RobotSide side : RobotSide.values())
      {
         soleFramePoints.get(side).set(robotJae.getSoleFrame(side).getTransformToWorldFrame().getTranslation());
      }
   }

   /**
    * A {@code CenterOfMassCommand} is created using the new calculated {@code centerOfMassPosition}
    * and adds the command to the controller core.
    *
    * @param centerOfMassPosition refers to newly calculated position of the center of mass of the
    *                             robot.
    */
   public void sendCenterOfMassCommand(FramePoint3DReadOnly centerOfMassPosition)
   {
      CenterOfMassFeedbackControlCommand centerOfMassCommand = new CenterOfMassFeedbackControlCommand();
      centerOfMassCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS); // sets control mode to inverse dynamics
      FrameVector3D feedForwardLinearVelocity = new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.0);
      FrameVector3D feedForwardLinearAcceleration = new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.0);
      centerOfMassCommand.setInverseDynamics(centerOfMassPosition, feedForwardLinearVelocity, feedForwardLinearAcceleration);
      centerOfMassCommand.setGains(gains.getPositionGains());
      centerOfMassCommand.setWeightForSolver(1.0);
      controllerCoreCommand.addFeedbackControlCommand(centerOfMassCommand);
   }

   /**
    * During the standing state, the center of mass is kept at a constant position right between the
    * feet and the feet are both used for support.
    */
   private class StandingState implements State
   {

      @Override
      public void onEntry()
      {
      }

      @Override
      public void doAction(double timeInState)
      {
         addHipTrajectoryPoint();

         // Here we get the position of both feet to compute the middle.
         FramePoint3D leftSolePosition = new FramePoint3D(robotJae.getSoleFrame(RobotSide.LEFT));
         leftSolePosition.changeFrame(WORLD_FRAME);
         FramePoint3D rightSolePosition = new FramePoint3D(robotJae.getSoleFrame(RobotSide.RIGHT));
         rightSolePosition.changeFrame(WORLD_FRAME);
         // The desired center of mass position is set to be right in between the feet.
         desiredCoM.interpolate(leftSolePosition, rightSolePosition, 0.5);
         // We set the desired height.
         desiredCoM.setZ(DESIRED_CENTER_OF_MASS_HEIGHT);

         // So now, we just have pack the command for the controller core.
         if (walkModeYoEnum.getValue() == WalkMode.ASIMO)
            sendCenterOfMassCommand(desiredCoM);

         FramePoint3D measuredCoMPosition = new FramePoint3D(WORLD_FRAME, toolbox.getCenterOfMassFrame().getTransformToWorldFrame().getTranslation());
         measuredCoM.set(measuredCoMPosition);

         // Now it is the turn of the feet.
         planeContactStateCommands = new SideDependentList<>(side -> createPlaneContactStateCommand(side, true));

         updateICP();
         if (walkModeYoEnum.getValue() == WalkMode.LIP)
         {
//            guiDesiredICP.setZ(0.0);
            sendICPCommand(guiDesiredICP, guiDesiredICPVelocity);
            dcmWantedNow.set(guiDesiredICP);
            dcmStart.set(guiDesiredICP);
            dcmFinal.set(guiDesiredICP);
         }

         measuredCoM.set(measuredCoMPosition);

         for (RobotSide robotSide : RobotSide.values)
         {
            // As their are in support, they should not be accelerating. So we make a zero-acceleration command.
            SpatialAccelerationCommand footZeroAcceleration = createFootZeroAccelerationCommand(robotSide);
            controllerCoreCommand.addInverseDynamicsCommand(footZeroAcceleration);
            // This is the command that we can use to request a contactable body to be used for support or not.
            controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommands.get(robotSide));
         }
         bipedSupportPolygons.updateUsingContactStateCommand(planeContactStateCommands);
      }

      @Override
      public void onExit(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         /*
          * As soon as the walk variable is set to true via the simulation GUI, the state machine will leave
          * this state causing the robot to start walking.
          */
         return walk.getValue();
      }
   }

   /**
    * During the transfer state, the robot is still in double support but the center of mass is now
    * moving to be above the leading foot.
    */
   private class TransferState implements State
   {
      private final RobotSide transferToSide;
      private final FramePoint3D initialCenterOfMassPosition = new FramePoint3D();
      private final FramePoint3D finalCenterOfMassPosition = new FramePoint3D();
      /**
       * We use a 5th order polynomial to smooth out the velocity of the center of mass at the start and
       * end of this state. This allows the robot to walk slightly faster without falling.
       */
      private final YoPolynomial centerOfMassTrajectory;
      private boolean initialized = false;

      public TransferState(RobotSide transferToSide)
      {
         this.transferToSide = transferToSide;
         centerOfMassTrajectory = new YoPolynomial(transferToSide.getCamelCaseName() + "CenterOfMassTrajectory", 6, registry);
      }

      @Override
      public void onEntry()
      {
         /*
          * This method is called once when this state becomes active. this is the perfect time to initialize
          * the endpoints for the center of mass. It should start from wherever it is at the start of this
          * state and end above the leading foot.
          */
         initialCenterOfMassPosition.setToZero(robotJae.getCenterOfMassFrame());
         initialCenterOfMassPosition.changeFrame(WORLD_FRAME);
         finalCenterOfMassPosition.setToZero(robotJae.getSoleFrame(transferToSide));
         finalCenterOfMassPosition.changeFrame(WORLD_FRAME);
//         double y1 = robotJae.getSoleFrame(transferToSide).getTransformToWorldFrame().getTranslationY();
//         double y2 = robotJae.getSoleFrame(transferToSide.getOppositeSide()).getTransformToWorldFrame().getTranslationY();
//         double y = (y1 + y2) / 2;
//         finalCenterOfMassPosition.setX(y);
         // The trajectory is setup such that it will always from 0.0 to 1.0 within the given transferDuration.
         centerOfMassTrajectory.setQuintic(0, transferDuration.getValue(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

         // plan n steps here to create dcm key points but we will only use the first planned step and the first dcm keypoint.

         // more like future swing side since currently we are in transfer state.
         RobotSide swingSide = transferToSide.getOppositeSide();
         plannedFootsteps.clear();
         plannedFootsteps = planSteps(swingSide,
                                      robotJae.getSoleFrame(swingSide),
                                      robotJae.getMidFeetFrame(),
                                      strideLength.getDoubleValue(),
                                      stepWidth.getDoubleValue(),
                                      stepDirection.getYaw(),
                                      numStepsToPlan.getIntegerValue());
         visualizerPlannedFootSteps.update(plannedFootsteps);

         // visual of planned steps

         ArrayList<Double> stepTimes = new ArrayList<>();
         for (int i = 0; i < plannedFootsteps.size(); ++i)
         {
            stepTimes.add(swingDuration.getDoubleValue());
         }
         nextICP.set(calculateNextICPFromPlannedSteps(plannedFootsteps, stepTimes));

         // define doubles support trajectory
         FramePoint3D icp_ini = new FramePoint3D(measuredICP);
         FramePoint3D vrp_i = new FramePoint3D();
         FramePose3D plannedFootPose = plannedFootsteps.get(0).getFootstepPose();
         vrp_i.changeFrame(plannedFootPose.getReferenceFrame());
         vrp_i.set(plannedFootPose.getPosition());
         vrp_i.changeFrame(WORLD_FRAME);
         FramePoint3D vrp_i_1 = new FramePoint3D();
         vrp_i_1.set(WORLD_FRAME, robotJae.getSoleFrame(transferToSide).getTransformToWorldFrame().getTranslation());

         ArrayList<FramePoint3D> dsIcpPoints = calculateDoubleSupportIcpPoints(0.5, transferDuration.getDoubleValue(), icp_ini, vrp_i, vrp_i_1);


         // set trajectory for dcm
         dcmStart.set(dsIcpPoints.get(0));
         dcmFinal.set(dsIcpPoints.get(1));
         dcmTrajectory.setQuintic(0, transferDuration.getValue(), 0.0, 0.0, 0.0, 0.4, 0.0, 0.0);
      }

      @Override
      public void doAction(double timeInState)
      {
         // update icp
         updateICP();

         // left hip trajectory for debugging
         addHipTrajectoryPoint();

         // The trajectory generator is updated to get the current progression percentage alpha which is in [0, 1].
         centerOfMassTrajectory.compute(MathTools.clamp(timeInState, 0.0, transferDuration.getValue()));
         double alpha = centerOfMassTrajectory.getValue();

         // The alpha parameter is now used to interpolate between the initial and final center of mass positions.
         desiredCoM.interpolate(initialCenterOfMassPosition, finalCenterOfMassPosition, alpha);
         desiredCoM.setZ(DESIRED_CENTER_OF_MASS_HEIGHT);

         FramePoint3D measuredCoMPosition = new FramePoint3D(WORLD_FRAME, toolbox.getCenterOfMassFrame().getTransformToWorldFrame().getTranslation());
         measuredCoM.set(measuredCoMPosition);

         if (walkModeYoEnum.getValue() == WalkMode.ASIMO)
         {
            // And now we pack the command for the controller core.
            sendCenterOfMassCommand(desiredCoM);
         }

         else if (walkModeYoEnum.getValue() == WalkMode.LIP)
         {
            dcmTrajectory.compute(MathTools.clamp(timeInState, 0.0, transferDuration.getValue()));
            double dcmTrajAlpha = dcmTrajectory.getValue();
            double alphaDot = dcmTrajectory.getVelocity();
            FrameVector3D dcmVelocityFromTrajectory = new FrameVector3D();
            dcmVelocityFromTrajectory.sub(dcmFinal, dcmStart);
            dcmVelocityFromTrajectory.scale(alphaDot);

            dcmWantedNow.setToZero();
            dcmWantedNow.interpolate(dcmStart, dcmFinal, dcmTrajAlpha);
            dcmWantedNow.setZ(DESIRED_CENTER_OF_MASS_HEIGHT);

            if (dsICPTraj.getValue())
            {
               // TODO: testing if putting zerovelocity works
               dcmVelocityFromTrajectory.setToZero();
               sendICPCommand(dcmWantedNow, dcmVelocityFromTrajectory);
            }
            else
            {
               sendCenterOfMassCommand(desiredCoM);
            }
         }

         // As for the standing state, we request both feet to be in support.
         planeContactStateCommands = new SideDependentList<>(side -> createPlaneContactStateCommand(side, true));
         for (RobotSide robotSide : RobotSide.values)
         {
            SpatialAccelerationCommand footZeroAcceleration = createFootZeroAccelerationCommand(robotSide);
            controllerCoreCommand.addInverseDynamicsCommand(footZeroAcceleration);
            controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommands.get(robotSide));
         }
         bipedSupportPolygons.updateUsingContactStateCommand(planeContactStateCommands);
      }

      @Override
      public void onExit(double timeInState)
      {

      }

      @Override
      public boolean isDone(double timeInState)
      {
         // As soon as the time reaches the given transferDuration, the state machine will enter the next single support state.
         return timeInState >= transferDuration.getValue();
      }
   }

   /**
    * During the single support state, only one foot is in support while the other is swinging to the
    * next footstep. The center of mass stays still.
    */
   private class SingleSupportState implements State
   {
      private final RobotSide supportSide;
      private final RobotSide swingSide;
      /**
       * We will use a trajectory generator for the swing.
       */
      private final TwoWaypointSwingGenerator swingPositionTrajectory;
//      private final MultipleWaypointsTrajectoryGenerator multiWaypointSwingGenerator;
      private final YoFramePoint3D initialPosition;
      /**
       * YoVariable for the next footstep position so we can monitor it during the simulation.
       */
      private final YoFramePoint3D desiredFootstepPosition;
      private final YoFrameVector3D touchdownVelocity;
      private final FramePose3D swingControlFramePose = new FramePose3D();

      private final YoFramePoint3D currentDesiredFootstepPosition;

      public SingleSupportState(RobotSide supportSide)
      {
         this.supportSide = supportSide;
         swingSide = supportSide.getOppositeSide();
         initialPosition = new YoFramePoint3D(swingSide.getCamelCaseName() + "SwingStart", WORLD_FRAME, registry);
         desiredFootstepPosition = new YoFramePoint3D(swingSide.getCamelCaseName() + "DesiredFootstep", WORLD_FRAME, registry);
         currentDesiredFootstepPosition = new YoFramePoint3D(swingSide.getCamelCaseName() + "CurrentDesiredFootstep", WORLD_FRAME, registry);
         touchdownVelocity = new YoFrameVector3D(swingSide.getCamelCaseName() + "TouchdownVelocity", WORLD_FRAME, registry);
         touchdownVelocity.set(0.0, 0.0, -0.1);

         double groundClearance = 0.25;
         // The trajectory generator such that the swing starts from the current foot position and ends at the given footstepPosition.
         swingPositionTrajectory = new TwoWaypointSwingGenerator(swingSide.getCamelCaseName() + "Swing", 0.15, 0.4, groundClearance, 0.0, registry, null);
//         multiWaypointSwingGenerator = new MultipleWaypointsTrajectoryGenerator(swingSide.getCamelCaseName() + "MultiWaypointSwing", 5, registry);
         // Here we define the control frame pose that is needed to specify the point on
         // the foot we want to control to the controller core.
         swingControlFramePose.setToZero(robotJae.getSoleFrame(swingSide));
         swingControlFramePose.changeFrame(robotJae.getFoot(swingSide).getBodyFixedFrame());
      }

      @Override
      public void onEntry()
      {
         // determine step length based on orbital energy
         FrameVector3D measuredCoMVelocity = new FrameVector3D(WORLD_FRAME, toolbox.getCentroidalMomentumRateCalculator().getCenterOfMassVelocity());
         FramePoint3D measuredCPPosition = new FramePoint3D(WORLD_FRAME);
         FramePoint3D measuredCoMPosition = new FramePoint3D(WORLD_FRAME, toolbox.getCenterOfMassFrame().getTransformToWorldFrame().getTranslation());
         FramePoint3D supportFootPosition = new FramePoint3D(robotJae.getSoleFrame(supportSide));
         supportFootPosition.changeFrame(WORLD_FRAME);

         double currentEnergy = calculateOrbitalEnergy(measuredCoMPosition.getX(), supportFootPosition.getX(), measuredCoMVelocity.getX(), OMEGA);

         double desiredEnergy = 0.5 * Math.pow(desiredComVelocityX.getValue(), 2);
         double a = 1;
         double b = -2 * (measuredCPPosition.getX() - supportFootPosition.getX());
         double c = 2 * (currentEnergy - desiredEnergy) / Math.pow(OMEGA, 2);
         double[] sol = solveQuadratic(a, b, c);
         //         desiredStepLength.set(sol[0]);
         desiredStepLength.set(0.3);

         // Here we compute the position for the next footstep.
         initialPosition.setFromReferenceFrame(robotJae.getSoleFrame(swingSide));
         // case 1 : adding in x direction for next foot step
//         desiredFootstepPosition.setFromReferenceFrame(robotJae.getSoleFrame(swingSide));
//         desiredFootstepPosition.setX(supportFootPosition.getX() + desiredStepLength.getValue());

         // case 2 : from plan
         FramePose3D plannedStepPose = plannedFootsteps.get(0).getFootstepPose();
         plannedStepPose.changeFrame(WORLD_FRAME);
         desiredFootstepPosition.set(WORLD_FRAME, plannedStepPose.getPosition());

         desiredNextFootstepPositionForVisual.set(desiredFootstepPosition);

         // TODO: for now let's make next desired footstep location as reference icp.
         referenceICP.set(desiredFootstepPosition);
         referenceCOM.set(measuredCoMPosition);

         // normal
         swingPositionTrajectory.setInitialConditions(initialPosition, new FrameVector3D());
         swingPositionTrajectory.setFinalConditions(desiredFootstepPosition, touchdownVelocity);
         swingPositionTrajectory.setTrajectoryType(TrajectoryType.DEFAULT);
         swingPositionTrajectory.setStepTime(swingDuration.getValue());
         swingPositionTrajectory.initialize();

         // multi
//         multiWaypointSwingGenerator.
         while (swingPositionTrajectory.doOptimizationUpdate())
            ;

         FramePoint3D icp_i = new FramePoint3D(measuredICP);
         FramePoint3D vrp_i = new FramePoint3D();
         vrp_i.setFromReferenceFrame(robotJae.getSoleFrame(supportSide));
         ArrayList<FramePoint3D> ssIcpPoint_ini_pos_and_vel = calculateICPAtTime(0, icp_i, vrp_i, swingDuration.getDoubleValue());
         ArrayList<FramePoint3D> ssIcpPoint_end_pos_and_vel = calculateICPAtTime(swingDuration.getDoubleValue(), icp_i, vrp_i, swingDuration.getDoubleValue());

         // set dcm
         dcmStart.set(ssIcpPoint_ini_pos_and_vel.get(0));
         dcmStart.setZ(DESIRED_CENTER_OF_MASS_HEIGHT);
         dcmFinal.set(ssIcpPoint_end_pos_and_vel.get(0));
         dcmFinal.setZ(DESIRED_CENTER_OF_MASS_HEIGHT);
         dcmTrajectory.setQuintic(0, swingDuration.getValue(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
      }

      @Override
      public void doAction(double timeInState)
      {
         addHipTrajectoryPoint();

         // During this state, the center of mass is kept right above the support foot.

         if (walkModeYoEnum.getValue() == WalkMode.ASIMO)
         {
            // Mode 1 : ASIMO-like movement of the COM
            FramePoint3D centerOfMassPosition = new FramePoint3D(robotJae.getSoleFrame(supportSide));
            centerOfMassPosition.changeFrame(WORLD_FRAME);
            centerOfMassPosition.setZ(DESIRED_CENTER_OF_MASS_HEIGHT);
            desiredCoM.set(centerOfMassPosition);
            // We pack the center of mass command for the controller core.
            sendCenterOfMassCommand(desiredCoM);
         }

         if (walkModeYoEnum.getValue() == WalkMode.LIP)
         {
            // Mode 2 : LIPM-like movement of the COM
//            FramePoint3D comFromTrajectory = evaluateCOMFromMassTrajectory(referenceICP,
//                                                                           soleFrames.get(supportSide).getTransformToWorldFrame().getTranslation(),
//                                                                           referenceCOM,
//                                                                           timeInState);
//            desiredCoM.set(comFromTrajectory);

            dcmTrajectory.compute(MathTools.clamp(timeInState, 0.0, swingDuration.getValue()));
            double alpha = dcmTrajectory.getValue();
            double alphaDot = dcmTrajectory.getVelocity();
            FrameVector3D velocity = new FrameVector3D();
            velocity.sub(dcmFinal, dcmStart);
            velocity.scale(alphaDot);

            dcmWantedNow.setToZero();
            dcmWantedNow.interpolate(dcmStart, dcmFinal, alpha);
            dcmWantedNow.setZ(DESIRED_CENTER_OF_MASS_HEIGHT);

            //TODO: testing if sending zero velocity works
            velocity.setToZero();
            sendICPCommand(dcmWantedNow, velocity);
         }


         FramePoint3D measuredCoMPosition = new FramePoint3D(WORLD_FRAME, toolbox.getCenterOfMassFrame().getTransformToWorldFrame().getTranslation());
         measuredCoM.set(measuredCoMPosition);




         // As in the standing state, the support is specified with the contact state
         // command and zero acceleration command.
         SpatialAccelerationCommand footZeroAcceleration = createFootZeroAccelerationCommand(supportSide);
         controllerCoreCommand.addInverseDynamicsCommand(footZeroAcceleration);
         planeContactStateCommands = new SideDependentList<>();
         planeContactStateCommands.put(supportSide, createPlaneContactStateCommand(supportSide, true));
         planeContactStateCommands.put(swingSide, createPlaneContactStateCommand(swingSide, false));

         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommands.get(swingSide));
         // We need to specify that the swing foot is not in contact anymore.
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommands.get(supportSide));

         /*
          * Using the swing trajectory generator, we can compute the current desired position, velocity and
          * acceleration for the swing foot.
          */
         FramePoint3D position = new FramePoint3D();
         FrameVector3D velocity = new FrameVector3D();
         FrameVector3D acceleration = new FrameVector3D();
         swingPositionTrajectory.compute(timeInState);
         swingPositionTrajectory.getLinearData(position, velocity, acceleration);

         // store currently desired footstep position to Yo
         currentDesiredFootstepPosition.set(position);

         FrameVector3D takeOffVelocity = new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.01);
         if (timeInState <= swingDuration.getValue() * 0.1)
         {
            velocity.add(takeOffVelocity);
         }

         // Finally, we pack the swing foot command for the controller core.
         SpatialFeedbackControlCommand swingFootCommand = new SpatialFeedbackControlCommand();

         // sets control mode to inverse dynamics
         swingFootCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
         swingFootCommand.set(robotJae.getElevator(), robotJae.getFoot(swingSide));
         swingFootCommand.setInverseDynamics(position, velocity, acceleration);
         swingFootCommand.setInverseDynamics(zeroPose3D.getOrientation(), zeroVector, zeroVector);
         swingFootCommand.setControlFrameFixedInEndEffector(swingControlFramePose);
         swingFootCommand.setGains(gains);
         swingFootCommand.setWeightForSolver(1.0);
         controllerCoreCommand.addFeedbackControlCommand(swingFootCommand);

         desiredCurrentFootPosition.set(position);
         updateICP();

         bipedSupportPolygons.updateUsingContactStateCommand(planeContactStateCommands);
      }

      public double calculateOrbitalEnergy(double pos, double support, double velocity, double omega)
      {
         return 0.5 * Math.pow(velocity, 2) - 0.5 * Math.pow(omega, 2) * Math.pow(pos - support, 2);
      }

      public double[] solveQuadratic(double a, double b, double c)
      {
         double[] sol = new double[2];
         double b2_4ac_term = Math.sqrt(Math.pow(b, 2) - 4 * a * c);
         sol[0] = (-b + b2_4ac_term) / (2 * a);
         sol[1] = (-b - b2_4ac_term) / (2 * a);
         return sol;
      }

      @Override
      public void onExit(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         // As soon as the time reaches the given swingDuration, the state machine will
         // enter the next transfer state.
         return timeInState >= swingDuration.getValue();
      }
   }

   /**
    * Creates a spatial acceleration command to request a zero acceleration of the right or left foot.
    *
    * @param robotSide refers to which foot the command is to be created for.
    * @return the zero acceleration command.
    */
   public SpatialAccelerationCommand createFootZeroAccelerationCommand(RobotSide robotSide)
   {
      SpatialAccelerationCommand footZeroAcceleration = new SpatialAccelerationCommand();
      footZeroAcceleration.set(robotJae.getElevator(), robotJae.getFoot(robotSide));
      footZeroAcceleration.setWeight(1.0);
      return footZeroAcceleration;
   }

   /**
    * Creates the command for the controller core to indicate whether a foot is in support or not.
    *
    * @param robotSide refers to which foot the command is to be created for.
    * @param inSupport whether the foot should be used to support the robot weight or not.
    * @return the command for the controller core.
    */
   public PlaneContactStateCommand createPlaneContactStateCommand(RobotSide robotSide, boolean inSupport)
   {
      ContactablePlaneBody footContactableBody = robotJae.getFootContactableBody(robotSide);

      PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
      planeContactStateCommand.setContactingRigidBody(footContactableBody.getRigidBody());

      if (inSupport)
      {
         List<FramePoint2D> contactPoints2d = footContactableBody.getContactPoints2d();
         for (int i = 0; i < contactPoints2d.size(); i++)
         {
            FramePoint2D contactPoint = contactPoints2d.get(i);
            planeContactStateCommand.addPointInContact(contactPoint);
         }
      }

      planeContactStateCommand.setCoefficientOfFriction(0.8);
      planeContactStateCommand.setContactNormal(new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 1.0));
      return planeContactStateCommand;
   }

   public void sendICPCommand(FramePoint3DReadOnly trajectoryICP, FrameVector3DReadOnly trajectoryICPVelocity)
   {
      measuredCoM.set(WORLD_FRAME, toolbox.getCenterOfMassFrame().getTransformToWorldFrame().getTranslation());
      measuredCoMVelocity.set(WORLD_FRAME, toolbox.getCentroidalMomentumRateCalculator().getCenterOfMassVelocity());
      updateICP();
      desireCMP.set(calculateDesiredCMP(measuredICP, trajectoryICP, trajectoryICPVelocity));

      FramePoint3D desiredCoMAcceleration = calculateDesiredCoMAcceleration(measuredCoM, desireCMP);
      double kp = 15.0;
      double kd = GainCalculator.computeDerivativeGain(kp, 1.0);
      desiredCoMAcceleration.setZ(kp * (DESIRED_CENTER_OF_MASS_HEIGHT - measuredCoM.getZ()) - kd * measuredCoMVelocity.getZ());

      FrameVector3D desiredLinearMomentumRate = new FrameVector3D(WORLD_FRAME, desiredCoMAcceleration);
      desiredLinearMomentumRate.scale(toolbox.getTotalRobotMass());
      MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
      momentumRateCommand.setWeight(1.0);
      momentumRateCommand.setLinearMomentumRate(desiredLinearMomentumRate);
      momentumRateCommand.setSelectionMatrixForLinearControl();

      // send command to controller
      controllerCoreCommand.addInverseDynamicsCommand(momentumRateCommand);
   }

   // updates and writes to measuredICP
   public void updateICP()
   {
      FramePoint3D measuredCoMPosition = new FramePoint3D(WORLD_FRAME, toolbox.getCenterOfMassFrame().getTransformToWorldFrame().getTranslation());
      FrameVector3D measuredCoMVelocity = new FrameVector3D(WORLD_FRAME, toolbox.getCentroidalMomentumRateCalculator().getCenterOfMassVelocity());
      measuredICP.setToZero();
      measuredICP.scaleAdd(1.0 / OMEGA, measuredCoMVelocity, measuredCoMPosition);
      measuredICP.setZ(0.0);
   }

   public static FramePoint3D calculateDesiredCMP(FramePoint3DReadOnly measuredICP, FramePoint3DReadOnly trajectoryICP, FrameVector3DReadOnly trajectoryICPVelocity)
   {
      FramePoint3D cmp = new FramePoint3D();
      double kp = 1.5;
      FramePoint3D ICPError = new FramePoint3D();
      ICPError.sub(measuredICP, trajectoryICP);
      ICPError.scale(kp);

      cmp.add(measuredICP);
      cmp.add(ICPError);
      cmp.scaleAdd(-1.0 / OMEGA, trajectoryICPVelocity, cmp);

      return cmp;
   }

   public static FramePoint3D calculateDesiredCoMAcceleration(FramePoint3DReadOnly measuredCoM, FramePoint3DReadOnly desiredCMP)
   {
      FramePoint3D desiredComAcceleration = new FramePoint3D();
      desiredComAcceleration.sub(measuredCoM, desiredCMP);
      desiredComAcceleration.scale(OMEGA * OMEGA);
      return desiredComAcceleration;
   }

   public void addHipTrajectoryPoint()
   {
      ReferenceFrame leftHipFrame = robotJae.getHipBodyReferenceFrame(RobotSide.LEFT);
      leftHipTrajectoryBalls.setBall(new Point3D(leftHipFrame.getTransformToWorldFrame().getTranslation()));

      if (leftHipTrajectory.size() == 500)
      {
         leftHipTrajectory.remove(0);
      }
      leftHipTrajectory.add(new Point3D(leftHipFrame.getTransformToWorldFrame().getTranslation()));

      leftHipTrajectoryBalls.reset();

      for (int i = 0; i < leftHipTrajectory.size(); ++i)
      {
         leftHipTrajectoryBalls.setBall((Point3DReadOnly) leftHipTrajectory.get(i), YoAppearance.Black(), i);
      }

//      LogTools.info("left hip location: {}", leftHipFrame.getTransformToWorldFrame().getTranslation());
   }

   @Override
   public String getName()
   {
      return "RobotWalkerFourController";
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public YoGraphicDefinition getYoGraphicDefinition()
   {
      return graphicsGroup;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   private FramePoint3D evaluateICP(Tuple3DReadOnly icp0, Tuple3DReadOnly support, double t)
   {
      // eqn: backwards eta(t) = exp(-w*t) * (eta0 - u) + u
      double exp_wt = Math.exp(-OMEGA * t);
      FramePoint3D icp = new FramePoint3D();
      icp.sub(icp0, support);
      icp.scale(exp_wt);
      icp.add(support);

      return icp;
   }

   private static FramePoint3D calculateNextICPFromPlannedSteps(ArrayList<Footstep> plannedSteps, ArrayList<Double> stepTimes)
   {
      FramePoint3D nextICP = new FramePoint3D();
      int n = plannedSteps.size();
      nextICP.set(WORLD_FRAME, plannedSteps.get(n-1).getFootstepPose().getPosition());

      for (int i = n - 2; i >= 0; --i)
      {
         FramePoint3DReadOnly vrp = new FramePoint3D(WORLD_FRAME, plannedSteps.get(i).getFootstepPose().getPosition());
         double exp_wt = Math.exp(-OMEGA * stepTimes.get(i));
         nextICP.sub(vrp);
         nextICP.scale(exp_wt);
         nextICP.add(vrp);
      }
      return nextICP;
   }

   // returns two key ICP points( ini and end ) in double support phase
   private static ArrayList<FramePoint3D> calculateDoubleSupportIcpPoints(double alpha, double duration, FramePoint3D ici_ini, FramePoint3D vrp_i, FramePoint3D vrp_i_1)
   {
      ArrayList<FramePoint3D> doubleSupportICPs = new ArrayList<>();

      // icp_ini_DS_i
      double exp_wt_ini = Math.exp(-OMEGA * alpha * duration);
      FramePoint3D icp_ini_minus_vrp_i_1 = new FramePoint3D();

      icp_ini_minus_vrp_i_1.sub(ici_ini, vrp_i_1);
      FramePoint3D icpDS_initial = new FramePoint3D();
      icpDS_initial.add(icp_ini_minus_vrp_i_1);
      icpDS_initial.scale(exp_wt_ini);
      icpDS_initial.add(vrp_i_1);

      // icp_end_DS_i
      double exp_wt_end = Math.exp(OMEGA * (1 - alpha) * duration);
      FramePoint3D icp_ini_minus_vrp_i = new FramePoint3D();
      icp_ini_minus_vrp_i.sub(ici_ini, vrp_i);
      FramePoint3D icpDS_end = new FramePoint3D();
      icpDS_end.add(icp_ini_minus_vrp_i);
      icpDS_end.scale(exp_wt_end);
      icpDS_end.add(vrp_i);

      doubleSupportICPs.add(icpDS_initial);
      doubleSupportICPs.add(icpDS_end);

      return doubleSupportICPs;
   }

   // returns the key icp point and velocity at time t
   private static ArrayList<FramePoint3D> calculateICPAtTime(double t, FramePoint3D icp_i, FramePoint3D vrp_i, double duration)
   {
      ArrayList<FramePoint3D> icpPointAndVelocity = new ArrayList<>();
      double exp_w_t_minus_duration = Math.exp(OMEGA * (t - duration));

      // icp point at t
      FramePoint3D icp = new FramePoint3D();
      icp.sub(icp_i, vrp_i);
      icp.scale(exp_w_t_minus_duration);
      icp.add(vrp_i);

      // icp vel at t
      FramePoint3D icpVel = new FramePoint3D();
      icpVel.sub(icp_i, vrp_i);
      icpVel.scale(OMEGA * exp_w_t_minus_duration);

      icpPointAndVelocity.add(icp);
      icpPointAndVelocity.add(icpVel);

      return icpPointAndVelocity;
   }

   private FramePoint3D evaluateCOMFromMassTrajectory(Tuple3DReadOnly icp0, Tuple3DReadOnly support, Tuple3DReadOnly com0, double t)
   {
      // icp at time t
      Point3DBasics icp = evaluateICP(icp0, support, t);

      // eqn: forward com(t) = exp(-w*t) * (com0 - icp(t)) + icp(t)
      FramePoint3D com = new FramePoint3D();
      double exp_wt = Math.exp(-OMEGA * t);
      com.sub(com0, icp);
      com.scale(exp_wt);
      com.add(icp);

      return com;
   }

   public static ArrayList<Footstep> planSteps(RobotSide swingSide,
                                               ReferenceFrame swingFootFrame,
                                               ReferenceFrame midFeetFrame,
                                               double strideLength,
                                               double stepWidth,
                                               double directionYaw,
                                               int numStepsToPlan)
   {
      ArrayList<Footstep> steps = new ArrayList<>();
      RobotSide side = swingSide;

      FramePose3D pose3D = new FramePose3D(midFeetFrame);
      for (int i = 0; i < numStepsToPlan; ++i)
      {
         Footstep step = new Footstep(side, pose3D);
         step.getFootstepPose().setRotationYawAndZeroTranslation(directionYaw);
         if (side == RobotSide.LEFT)
         {
            step.getFootstepPose().appendTranslation((i + 1) * strideLength, stepWidth / 2, 0.0);
         }
         else
         {
            step.getFootstepPose().appendTranslation((i + 1) * strideLength, -stepWidth / 2, 0.0);
         }
         steps.add(step);
         side = side.getOppositeSide();
      }

      return steps;
   }

   private void setupDoubleSupportICPTrajectory(double Ts, FramePoint3D icp_ini, FramePoint3D icp_ini_vel, FramePoint3D icp_end, FramePoint3D icp_end_vel)
   {
//      doubleSupportICPTrajectory.
   }
}