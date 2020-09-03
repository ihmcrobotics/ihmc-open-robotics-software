package us.ihmc.exampleSimulations.simple3DWalker;

import static java.lang.Math.sqrt;

import java.util.ArrayList;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class SimpleWalkerController implements RobotController
{
   private final double MAX_HIP_ANGLE = 0.8;
   private double HIP_DEFUALT_P_GAIN = 100.0;
   private double HIP_DEFUALT_D_GAIN = 10.0;

   private double KNEE_DEFUALT_P_GAIN = 10000.0;
   private double KNEE_DEFUALT_D_GAIN = 1000.0;

   private double KNEE_SOFT_P_GAIN = 30.0;
   private double KNEE_SOFT_D_GAIN = 10.0;

   private double deltaT;

   private SimpleWalkerRobot robot;
   private YoRegistry registry = new YoRegistry("Controller");
   private SideDependentList<PIDController> kneeControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> kneeControllersSoft = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> hipPitchControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> hipRollControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> hipYawControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> anklePitchControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> ankleRollControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> ankleYawControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> angularMomentumPitchControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> angularMomentumRollControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> impactControllers = new SideDependentList<PIDController>();

   private YoDouble desiredKneeExtension = new YoDouble("desiredKneeExtension", registry);
   private YoDouble desiredPitch = new YoDouble("desiredPitch", registry);
   private YoDouble desiredRoll = new YoDouble("desiredRoll", registry);
   private YoDouble desiredHeight = new YoDouble("desiredHeight", registry);
   private YoDouble swingTime = new YoDouble("swingTime", registry);
   private YoDouble desiredSwingLegHipPitchAngle = new YoDouble("desiredSwingLegHipPitchAngle", registry);
   private YoDouble desiredSwingLegHipRollAngle = new YoDouble("desiredSwingLegHipRollAngle", registry);
   private YoDouble scaleForVelToAngle = new YoDouble("scaleForVelToAngle", registry);
   private YoDouble desiredKneeStance = new YoDouble("desiredKneeStance", registry);
   private YoDouble pitchAngleForCapture = new YoDouble("pitchAngleForCapture", registry);
   private YoDouble rollAngleForCapture = new YoDouble("rollAngleForCapture", registry);
   private YoDouble feedForwardAngle = new YoDouble("feedForwardAngle", registry);
   private YoDouble velocityErrorPitchAngle = new YoDouble("velocityErrorPitchAngle", registry);
   private YoDouble velocityErrorRollAngle = new YoDouble("velocityErrorRollAngle", registry);
   private YoDouble feedForwardGain = new YoDouble("feedForwardGain", registry);
   private YoDouble lastStepHipPitchAngle = new YoDouble("lastStepHipPitchAngle", registry);
   private YoDouble lastStepHipRollAngle = new YoDouble("lastStepHipRollAngle", registry);
   private YoDouble stepToStepHipAngleDelta = new YoDouble("stepToStepHipAngleDelta", registry);

   private YoDouble swingTimeForThisStep = new YoDouble("swingTimeForThisStep", registry);
   private YoBoolean initalizedKneeExtension = new YoBoolean("initalizedKneeExtension", registry);
   private YoBoolean initalizedKneeExtensionSupport = new YoBoolean("initalizedKneeExtensionSupport", registry);
   private YoBoolean initalizedKneeDoubleExtension = new YoBoolean("initalizedKneeDoubleExtension", registry);

   private YoDouble kneeMoveStartTime = new YoDouble("kneeMoveStartTime", registry);
   private YoDouble startingHipPitchAngle = new YoDouble("startingHipPitchAngle", registry);
   private YoDouble startingHipRollAngle = new YoDouble("startingHipRollAngle", registry);
   private YoDouble yoTimeInState = new YoDouble("timeInState", registry);
   private YoDouble maxVelocityErrorAngle = new YoDouble("maxVelocityErrorAngle", registry);

   private YoDouble ankleControlTorquePitch = new YoDouble("ankleControlTorquePitch", registry);
   private YoDouble ankleControlTorqueRoll = new YoDouble("ankleControlTorqueRoll", registry);
   private YoDouble coPLocationX = new YoDouble("CoPX", registry);
   private YoDouble coPLocationY = new YoDouble("CoPY", registry);

   private YoDouble desiredBodyVelocityX = new YoDouble("desiredBodyVelocityX", registry);
   private YoDouble alphaFilterVariableX = new YoDouble("alphaFilterVariableX", registry);
   private AlphaFilteredYoVariable filteredDesiredVelocityX = new AlphaFilteredYoVariable("filteredDesiredVelocityX", registry, alphaFilterVariableX,
                                                                                          desiredBodyVelocityX);

   private YoDouble desiredBodyVelocityY = new YoDouble("desiredBodyVelocityY", registry);
   private YoDouble alphaFilterVariableY = new YoDouble("alphaFilterVariableY", registry);
   private AlphaFilteredYoVariable filteredDesiredVelocityY = new AlphaFilteredYoVariable("filteredDesiredVelocityY", registry, alphaFilterVariableY,
                                                                                          desiredBodyVelocityY);

   private YoMinimumJerkTrajectory trajectorySwingHipPitch;
   private YoMinimumJerkTrajectory trajectorySwingHipRoll;
   private YoMinimumJerkTrajectory trajectorySwingKnee;
   private YoMinimumJerkTrajectory trajectorySupportKnee;

   private final YoFramePoint3D centerOfMassPosition;
   private final YoFramePoint2D centerOfMassPosition2D;
   private final YoFrameVector3D centerOfMassVelocity;
   private final YoFrameVector3D centerOfMassAcceleration;

   private final YoFramePoint2D desiredICP2D;

   private final YoFramePoint3D currentICP;
   private final YoFramePoint2D currentICP2D;

   private final YoFramePoint2D desiredCoP2D;

   private final YoFramePoint3D currentCOP;
   private final YoFramePoint2D currentCOP2D;

   private final YoFramePoint3D currentLFoot;
   private final YoFramePoint2D currentLFoot2D;

   private final YoFramePoint3D currentRFoot;
   private final YoFramePoint2D currentRFoot2D;

   private final SideDependentList<YoFramePoint2D> currentFeet2D = new SideDependentList<>();

   private final YoDouble omega;

   private YoEnum<RobotSide> swingLeg = new YoEnum<RobotSide>("swingLeg", registry, RobotSide.class);

   private StateMachine<ControllerState, State> stateMachine;

   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private YoGraphicPosition desiredCoPGraphic;
   private Artifact desiredCoPGraphicArtifact;

   private YoGraphicPosition currentCoPGraphic;
   private Artifact currentCoPGraphicArtifact;

   private YoGraphicPosition currentCoMGraphic;
   private Artifact currentCoMGraphicArtifact;

   private YoGraphicPosition desiredICPGraphic;
   private Artifact desiredICPGraphicArtifact;

   private YoGraphicPosition currentICPGraphic;
   private Artifact currentICPGraphicArtifact;

   private YoGraphicPosition currentLFootGraphic;
   private Artifact currentLFootGraphicArtifact;

   private YoGraphicPosition currentRFootGraphic;
   private Artifact currentRFootGraphicArtifact;

   YoFrameConvexPolygon2D footPolygon;
   YoGraphicPolygon foot;
   Artifact footArtifact;

   private final boolean withInertiaControl;
   private final boolean withImpactControl;
   private final boolean withTwan;
   private final boolean withHeightOnly;

   private YoDouble legLength = new YoDouble("legLengt", registry);

   private YoDouble aTwan = new YoDouble("aTwan", registry);
   private YoDouble bTwan = new YoDouble("bTwan", registry);
   private YoDouble uTwan = new YoDouble("uTwan", registry);
   private double zfTwan = 1.1;
   private YoBoolean useHeight = new YoBoolean("useHeight", registry);
   private YoDouble secondCriterium = new YoDouble( "secondCriterium", registry);
   private YoDouble x0Twan = new YoDouble("x0Twan", registry);
   private YoDouble z0Twan = new YoDouble("z0Twan", registry);
   private YoDouble dx0Twan = new YoDouble("dx0Twan", registry);
   private YoDouble dz0Twan = new YoDouble("dz0Twan", registry);

   private ArrayList<Double> footStepPlan = new ArrayList<>();
   private YoDouble nextFootStepX = new YoDouble("nextFootStepX",registry);
   private YoDouble currentFootStepX = new YoDouble("currentFootStepX",registry);
   private YoInteger footStepCounter = new YoInteger("footStepCounter",registry);
   private SimpleWalkerICPPlanner icpPlanner;
   private final SimpleWalkerHeightStopMPC heightStopMPC;


   public SimpleWalkerController(SimpleWalkerRobot robot, double deltaT, boolean withInertiaControl, boolean withImpactControl, boolean withTwan, boolean withHeightOnly)
   {
      this.robot = robot;
      this.deltaT = deltaT;
      this.withInertiaControl= withInertiaControl;
      this.withImpactControl = withImpactControl;
      this.withTwan = withTwan;
      this.withHeightOnly = withHeightOnly;

      heightStopMPC = new SimpleWalkerHeightStopMPC(1.15,robot.nominalHeight,40,registry);

      centerOfMassPosition = new YoFramePoint3D("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
      centerOfMassPosition2D = new YoFramePoint2D("centerOfMass2D", ReferenceFrame.getWorldFrame(), registry);
      centerOfMassVelocity  = new YoFrameVector3D("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
      centerOfMassAcceleration  = new YoFrameVector3D("centerOfMassAcceleration", ReferenceFrame.getWorldFrame(), registry);
      omega = new YoDouble("omega", registry);

      desiredICP2D = new YoFramePoint2D("desiredICP2D", ReferenceFrame.getWorldFrame(), registry);

      currentICP = new YoFramePoint3D("currentICP", ReferenceFrame.getWorldFrame(), registry);
      currentICP2D = new YoFramePoint2D("currentICP2D", ReferenceFrame.getWorldFrame(), registry);

      desiredCoP2D = new YoFramePoint2D("desiredCoP2D", ReferenceFrame.getWorldFrame(), registry);

      currentCOP = new YoFramePoint3D("currentCOP", ReferenceFrame.getWorldFrame(), registry);
      currentCOP2D = new YoFramePoint2D("currentCOP2D", ReferenceFrame.getWorldFrame(),registry);

      currentLFoot = new YoFramePoint3D("currentLFoot", ReferenceFrame.getWorldFrame(), registry);
      currentLFoot2D = new YoFramePoint2D("currentLFoot2D", ReferenceFrame.getWorldFrame(),registry);

      currentRFoot = new YoFramePoint3D("currentRFoot", ReferenceFrame.getWorldFrame(), registry);
      currentRFoot2D = new YoFramePoint2D("currentRFoot2D", ReferenceFrame.getWorldFrame(),registry);

      YoFrameVector3D reactionForce = new YoFrameVector3D("reactionForce", ReferenceFrame.getWorldFrame(), registry);

      desiredCoPGraphic = new YoGraphicPosition("CoPrgraph",desiredCoP2D,0.02,YoAppearance.Green(),GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistry.registerYoGraphic("CoPrgraphrRegistry", desiredCoPGraphic);
      desiredCoPGraphicArtifact= desiredCoPGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("CoPrArtifact",desiredCoPGraphicArtifact);

      currentCoPGraphic = new YoGraphicPosition("CoPgraph",currentCOP2D,0.02,YoAppearance.Green());
      yoGraphicsListRegistry.registerYoGraphic("CoPgraphRegistry", currentCoPGraphic);
      currentCoPGraphicArtifact = currentCoPGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("CoPArtifact", currentCoPGraphicArtifact);

      currentCoMGraphic = new YoGraphicPosition("CoMgraph",centerOfMassPosition2D,0.02 ,YoAppearance.Black());
      yoGraphicsListRegistry.registerYoGraphic("CoMgraphRegistry", currentCoMGraphic);
      currentCoMGraphicArtifact = currentCoMGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("CoPArtifact", currentCoMGraphicArtifact);

      desiredICPGraphic = new YoGraphicPosition("ICPrgraph",desiredICP2D,0.02,YoAppearance.Tomato(),GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistry.registerYoGraphic("ICPrgraphRegistry", desiredICPGraphic);
      desiredICPGraphicArtifact= desiredICPGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("ICPrArtifact",desiredICPGraphicArtifact);

      currentICPGraphic = new YoGraphicPosition("ICPgraph",currentICP2D,0.02 ,YoAppearance.Tomato());
      yoGraphicsListRegistry.registerYoGraphic("ICPgraphRegistry", currentICPGraphic);
      currentICPGraphicArtifact = currentICPGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("ICPArtifact", currentICPGraphicArtifact);

      currentLFootGraphic = new YoGraphicPosition("LFootgraph",currentLFoot2D,0.15 ,YoAppearance.Blue(), GraphicType.BALL);
      yoGraphicsListRegistry.registerYoGraphic("LFootgraphRegistry", currentLFootGraphic);
      currentLFootGraphicArtifact = currentLFootGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("LFootArtifact", currentLFootGraphicArtifact);

      currentRFootGraphic = new YoGraphicPosition("RFootgraph", currentRFoot2D, 0.15 , YoAppearance.Red(), GraphicType.BALL);
      yoGraphicsListRegistry.registerYoGraphic("RFootgraphRegistry", currentRFootGraphic);
      currentRFootGraphicArtifact = currentRFootGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("RFootArtifact", currentRFootGraphicArtifact);

      currentFeet2D.put(RobotSide.LEFT, currentLFoot2D);
      currentFeet2D.put(RobotSide.RIGHT, currentRFoot2D);




      for (RobotSide robotSide : RobotSide.values)
      {
         for(GroundContactPoint gCpoint : robot.getgCpoints().get(robotSide))
         {
            YoFrameVector3D reactionForcePoint = gCpoint.getYoForce();
            reactionForce.add(reactionForcePoint);
         }


         YoGraphicVector reactionForceViz = new YoGraphicVector("reactionForceViz", currentCOP, reactionForce, YoAppearance.Red());
         reactionForceViz.setVisible(true);
         reactionForceViz.setDrawArrowhead(true);

         PIDController pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_Knee", registry);
         pidController.setProportionalGain(KNEE_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(KNEE_DEFUALT_D_GAIN);
         kneeControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_KneeSoft", registry);
         pidController.setProportionalGain(KNEE_SOFT_P_GAIN);
         pidController.setDerivativeGain(KNEE_SOFT_D_GAIN);
         kneeControllersSoft.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_HipPitch", registry);
         pidController.setProportionalGain(HIP_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(HIP_DEFUALT_D_GAIN);
         hipPitchControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_HipRoll", registry);
         pidController.setProportionalGain(HIP_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(HIP_DEFUALT_D_GAIN);
         hipRollControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_HipYaw", registry);
         pidController.setProportionalGain(HIP_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(HIP_DEFUALT_D_GAIN);
         hipYawControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AnklePitch", registry);
         pidController.setProportionalGain(1);
         pidController.setDerivativeGain(1);
         anklePitchControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AnkleRoll", registry);
         pidController.setProportionalGain(1);
         pidController.setDerivativeGain(1);
         ankleRollControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AnkleYaw", registry);
         pidController.setProportionalGain(0.1);
         pidController.setIntegralGain(0.1);
         pidController.setDerivativeGain(0.03);
         ankleYawControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AngularMomentumPitch", registry);
         pidController.setProportionalGain(5);
         pidController.setDerivativeGain(1);
         angularMomentumPitchControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AngularMomentumRoll", registry);
         pidController.setProportionalGain(5);
         pidController.setDerivativeGain(1);
         angularMomentumRollControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_Impact", registry);
         pidController.setProportionalGain(-300);
         pidController.setDerivativeGain(10);
         impactControllers.put(robotSide, pidController);

      }

      trajectorySwingHipPitch = new YoMinimumJerkTrajectory("trajectorySwingHipPitch", registry);
      trajectorySwingHipRoll = new YoMinimumJerkTrajectory("trajectorySwingHipRoll", registry);
      trajectorySwingKnee = new YoMinimumJerkTrajectory("trajectorySwingKnee", registry);
      trajectorySupportKnee = new YoMinimumJerkTrajectory("trajectorySupportKnee", registry);

      desiredHeight.set(robot.nominalHeight);
      desiredKneeStance.set(robot.lowerLinkLength / 2.0);
      swingTime.set(0.8);
      scaleForVelToAngle.set(0.0);
      feedForwardGain.set(0.1);
      stepToStepHipAngleDelta.set(0.3);
      maxVelocityErrorAngle.set(1);
      alphaFilterVariableX.set(0.99);
      alphaFilterVariableY.set(0.99);

      stateMachine = initializeStateMachine();

      footStepCounter.set(0);
      createEvenStepPlan(8, 0.3);
      YoDouble numberOfSteps = new YoDouble("totalNSteps",registry);
      numberOfSteps.set(footStepPlan.size());
      icpPlanner = new SimpleWalkerICPPlanner(footStepPlan, swingTime.getDoubleValue(), sqrt(9.81/robot.nominalHeight));


   }

   @Override
   public void initialize()
   {

   }

   private StateMachine<ControllerState, State> initializeStateMachine()
   {
      StateMachineFactory<ControllerState, State> factory = new StateMachineFactory<>(ControllerState.class);
      factory.setNamePrefix("controllerState").setRegistry(registry).buildYoClock(robot.getYoTime());
      factory.addStateAndDoneTransition(ControllerState.START, new

            StartState(), ControllerState.RIGHT_SUPPORT);
      factory.addStateAndDoneTransition(ControllerState.RIGHT_SUPPORT, new SingleSupportState(RobotSide.RIGHT), ControllerState.LEFT_SUPPORT);
      factory.addStateAndDoneTransition(ControllerState.LEFT_SUPPORT, new SingleSupportState(RobotSide.LEFT), ControllerState.RIGHT_SUPPORT);

      return factory.build(ControllerState.START);
   }

   private class StartState implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return true;
      }

      @Override
      public void onEntry()
      {
      }

      @Override
      public void onExit()
      {
      }
   }

   private class SingleSupportState implements State
   {
      private RobotSide supportLeg;

      public SingleSupportState(RobotSide supportLeg)
      {
         this.supportLeg = supportLeg;
      }

      @Override
      public void doAction(double timeInState)
      {
         yoTimeInState.set(timeInState);

         /**
          * Part with all trajectories.
          */
         //Swing Leg



         if ((timeInState > swingTimeForThisStep.getDoubleValue() / 2.0) && !initalizedKneeExtension.getBooleanValue())
         {
            double currentKneePosition = robot.getKneePosition(swingLeg.getEnumValue());
            trajectorySwingKnee
                  .setParams(currentKneePosition, 0.0, 0.0, desiredKneeStance.getDoubleValue(), 0.0, 0.0, 0.0, swingTimeForThisStep.getDoubleValue() / 2.0);
            initalizedKneeExtension.set(true);
            kneeMoveStartTime.set(timeInState);
         }

         else if ((timeInState > swingTimeForThisStep.getDoubleValue() && !initalizedKneeDoubleExtension.getBooleanValue()))
         {
            double currentKneePosition = robot.getKneePosition(swingLeg.getEnumValue());
            trajectorySwingKnee.setParams(currentKneePosition, 0.0, 0.0, desiredKneeStance.getDoubleValue() + 0.5, 0.0, 0.0, 0.0, 0.125);
            initalizedKneeDoubleExtension.set(true);
            kneeMoveStartTime.set(timeInState);
         }

         trajectorySwingKnee.computeTrajectory(timeInState - kneeMoveStartTime.getDoubleValue());
         double desiredKneePosition = trajectorySwingKnee.getPosition();
         double desiredKneeVelocity = trajectorySwingKnee.getVelocity();
         controlKneeToPosition(swingLeg.getEnumValue(), desiredKneePosition, desiredKneeVelocity);


         desiredSwingLegHipPitchAngle.set(getDesiredHipPitchAngle(nextFootStepX.getDoubleValue()));
         trajectorySwingHipPitch.setParams(startingHipPitchAngle.getDoubleValue(), 0.0, 0.0, desiredSwingLegHipPitchAngle.getDoubleValue(), 0.0, 0.0, 0.0,
                                           swingTimeForThisStep.getDoubleValue());

         desiredSwingLegHipRollAngle.set(getDesiredHipRollAngle());
         trajectorySwingHipRoll.setParams(startingHipRollAngle.getDoubleValue(), 0.0, 0.0, desiredSwingLegHipRollAngle.getDoubleValue(), 0.0, 0.0, 0.0,

                                          swingTimeForThisStep.getDoubleValue());

         /**
          * Control part.
          */
         // pitch

         trajectorySwingHipPitch.computeTrajectory(timeInState);
         double desiredHipPitchAngle = trajectorySwingHipPitch.getPosition();
         double desiredHipPitchAngleRate = trajectorySwingHipPitch.getVelocity();
         double currentHipPitchAngle = robot.getHipPitchPosition(swingLeg.getEnumValue());
         double currentHipPitchAngleRate = robot.getHipPitchVelocity(swingLeg.getEnumValue());

         PIDController pidControllerPitch = hipPitchControllers.get(swingLeg.getEnumValue());
         double controlEffortPitch = pidControllerPitch.compute(currentHipPitchAngle, desiredHipPitchAngle, currentHipPitchAngleRate, desiredHipPitchAngleRate, deltaT);
         robot.setHipPitchTorque(swingLeg.getEnumValue(), controlEffortPitch);

         // roll
         trajectorySwingHipRoll.computeTrajectory(timeInState);
         double desiredHipRollAngle = trajectorySwingHipRoll.getPosition();
         double desiredHipRollAngleRate = trajectorySwingHipRoll.getVelocity();
         double currentHipRollAngle = robot.getHipRollPosition(swingLeg.getEnumValue());
         double currentHipRollAngleRate = robot.getHipRollVelocity(swingLeg.getEnumValue());

         PIDController pidControllerRoll = hipRollControllers.get(swingLeg.getEnumValue());
         double controlEffortRoll = pidControllerRoll.compute(currentHipRollAngle, desiredHipRollAngle, currentHipRollAngleRate, desiredHipRollAngleRate, deltaT);
         robot.setHipRollTorque(swingLeg.getEnumValue(), controlEffortRoll);

         //foot
         if (robot.withFeet())
         {
            PIDController pidControllerAnkleSwingPitch = anklePitchControllers.get(swingLeg.getEnumValue());
            PIDController pidControllerAnkleSwingRoll = ankleRollControllers.get(swingLeg.getEnumValue());

            double controlEffortAnkleSwingRoll = pidControllerAnkleSwingRoll
                  .compute(robot.getAnkleRollPosition(swingLeg.getEnumValue()), 0.0, 0.0, 0.0, deltaT);
            robot.setAnkleRollTorque(swingLeg.getEnumValue(), -controlEffortAnkleSwingRoll);


            double tau;
            if(footStepCounter.getIntegerValue()>8)
            {
               tau = -200*robot.getAnklePitchPosition(supportLeg) - 60*robot.getAnklePitchVelocity(supportLeg);
            }
            else
            {
               double ICPd = icpPlanner.getICPReference(footStepCounter.getIntegerValue(), timeInState);
               desiredICP2D.set(ICPd,0);
               double kp = 5;
               double CoPd = currentFootStepX.getDoubleValue() + kp * (currentICP2D.getX() - ICPd);

               //CoPd= MathTools.clamp(CoPd,robot.getAnklePositionInWorldX(supportLeg)-0.15,robot.getAnklePositionInWorldX(supportLeg)+0.15);
               desiredCoP2D.set(CoPd,0);
               tau = -45* (currentCOP2D.getX() - CoPd) + 9.81 * (robot.getBodyPositionX() - currentCOP.getX());
            }
            robot.setAnklePitchTorque(supportLeg, tau);

            double controlEffortAnkleSwingPitch = pidControllerAnkleSwingPitch.compute(robot.getAnklePitchPosition(swingLeg.getEnumValue()), -desiredSwingLegHipPitchAngle.getDoubleValue(), robot.getAnklePitchVelocity(swingLeg.getEnumValue()), 0.0, deltaT);
            robot.setAnklePitchTorque(swingLeg.getEnumValue(), controlEffortAnkleSwingPitch);

            addCoPRollControl(supportLeg);
            //addCoPPitchControl(supportLeg);
         }

         if (robot.withYaw())
         {
            PIDController pidControllerAnkleSwingYaw = ankleYawControllers.get(swingLeg.getEnumValue());
            double controlEffortAnkleSwingYaw = pidControllerAnkleSwingYaw
                  .compute(robot.getAnkleYawPosition(swingLeg.getEnumValue()), 0.0, robot.getAnkleYawVelocity(swingLeg.getEnumValue()), 0.0, deltaT);
            robot.setAnkleYawTorque(swingLeg.getEnumValue(), controlEffortAnkleSwingYaw);
            addAnkleYawTorque(supportLeg);
            controlHipToMaintainYaw(supportLeg);
         }

         //Stance leg
         controlHipToMaintainPitch(supportLeg);
         controlHipToMaintainRoll(supportLeg);

         //add swing leg torque to stand leg
         addOppositeLegHipPitchTorque(supportLeg);
         addOppositeLegHipRollTorque(supportLeg);

         if (withInertiaControl)
         {
            addInertiaControlPitch(supportLeg);
            addInertiaControlRoll(supportLeg);
         }


         if (withImpactControl)
         {

            if ((timeInState < 0.15 * swingTimeForThisStep.getDoubleValue()) && desiredBodyVelocityX.getDoubleValue() < (robot.getBodyVelocityX() - 0.1))
            {
               // double tauKneeImpact = impactControllers.get(supportLeg).compute(robot.getBodyVelocityX(), desiredBodyVelocityX.getDoubleValue(), robot.getKneeVelocity(supportLeg), 0.0, deltaT);
               robot.setKneeTorque(supportLeg, 250);
            }
            else
            {
               controlKneeToPosition(supportLeg, desiredKneeStance.getDoubleValue(), 0.0);
            }
         }

         else if (withTwan)
         {
            legLength.set(robot.getLegLenght(supportLeg));
            double CoPforMPC = MathTools.clamp(currentCOP2D.getX(),robot.getAnklePositionInWorldX(supportLeg)-0.14,robot.getAnklePositionInWorldX(supportLeg)+0.14);
            x0Twan.set(centerOfMassPosition2D.getX()-CoPforMPC);
            if ((x0Twan.getDoubleValue()<-0.03)&&(timeInState>0.02)&&((desiredICP2D.getX()-currentICP2D.getX())<-0.04))
            {
               heightStopMPC.computeInvOutLoop(x0Twan.getDoubleValue(),robot.getBodyVelocityY(),robot.getBodyHeight(),robot.getBodyHeightVelocity());
               double zdes = heightStopMPC.getDesiredHeight();
               double dzdes = heightStopMPC.getDesiredHeighRate();
               controlKneeToBodyHeight(supportLeg,zdes,dzdes);
            }
            else
            {
               controlKneeToMaintainBodyHeight(supportLeg);
            }
         }


         else
         {
            //controlKneeToPosition(supportLeg, desiredKneeStance.getDoubleValue(), 0.0);
            controlKneeToMaintainBodyHeight(supportLeg);
         }




         if (supportLeg == RobotSide.LEFT)
         {
            double lFootX = robot.getBodyPositionX() - robot.getHipPitchPosition(supportLeg);
            double lFootY = robot.getBodyPositionY() + robot.getHipRollPosition(supportLeg);
            currentLFoot2D.set(lFootX,lFootY);

            double rFootX = robot.getBodyPositionX() - robot.getHipPitchPosition(swingLeg.getEnumValue());
            double rFootY = robot.getBodyPositionY() + robot.getHipRollPosition(swingLeg.getEnumValue());
            currentRFoot2D.set(rFootX,rFootY);
         }
         else
         {
            double lFootX = robot.getBodyPositionX() - robot.getHipPitchPosition(swingLeg.getEnumValue());
            double lFootY = robot.getBodyPositionY() + robot.getHipRollPosition(swingLeg.getEnumValue());
            currentLFoot2D.set(lFootX,lFootY);

            double rFootX = robot.getBodyPositionX() - robot.getHipPitchPosition(supportLeg);
            double rFootY = robot.getBodyPositionY() + robot.getHipRollPosition(supportLeg);
            currentRFoot2D.set(rFootX,rFootY);
         }




      }

      @Override
      public boolean isDone(double timeInState)
      {
         return initalizedKneeExtension.getBooleanValue() && robot.isFootOnGround(swingLeg.getEnumValue());
      }

      @Override
      public void onEntry()
      {
         swingLeg.set(supportLeg.getOppositeSide());
         swingTimeForThisStep.set(swingTime.getDoubleValue());
         initalizedKneeExtension.set(false);
         initalizedKneeExtensionSupport.set(false);
         initalizedKneeDoubleExtension.set(false);
         kneeMoveStartTime.set(0.0);

         startingHipPitchAngle.set(robot.getHipPitchPosition(swingLeg.getEnumValue()));
         startingHipRollAngle.set(robot.getHipRollPosition(swingLeg.getEnumValue()));

         double currentKneePosition = robot.getKneePosition(swingLeg.getEnumValue());
         double desiredRetractedPosition = 0.1;
         trajectorySwingKnee.setParams(currentKneePosition, 0.0, 0.0, desiredRetractedPosition, 0.0, 0.0, 0.0, swingTimeForThisStep.getDoubleValue() / 2.0);

         //retract knee
         robot.setKneeTorque(swingLeg.getEnumValue(), -10.0);

         createEvenStepPlan(6, 0.2);


         if(footStepCounter.getIntegerValue()<8)
         {
            nextFootStepX.set(footStepPlan.get(footStepCounter.getIntegerValue() + 1));
            currentFootStepX.set(footStepPlan.get(footStepCounter.getIntegerValue()));
         }
         else
         {
            nextFootStepX.set(footStepPlan.get(8));
            currentFootStepX.set(footStepPlan.get(8));
         }
      }

      @Override
      public void onExit()
      {
         lastStepHipPitchAngle.set(desiredSwingLegHipPitchAngle.getDoubleValue());
         lastStepHipRollAngle.set(desiredSwingLegHipRollAngle.getDoubleValue());
         useHeight.set(false);
         footStepCounter.set(footStepCounter.getIntegerValue()+1);



      }
   }

   private double getDesiredHipPitchAngle(double nextFootStepX)
   {
      /*
      double legLength = robot.upperLinkLength + desiredKneeStance.getDoubleValue();
      pitchAngleForCapture.set(HipAngleCapturePointCalculator3D.getHipPitchAngle(robot.getBodyVelocityX(), legLength));
      pitchAngleForCapture.set(-pitchAngleForCapture.getDoubleValue() * Math.signum(robot.getBodyVelocityX()));

      //only use a fraction of it
      //angleForCapture.set(0.8 *angleForCapture.getDoubleValue()
      // );

      //limit this angle
      pitchAngleForCapture.set(MathTools.clamp(pitchAngleForCapture.getDoubleValue(), MAX_HIP_ANGLE));

      //angle is opposite sign of desired velocity
      double velocityError = (filteredDesiredVelocityX.getDoubleValue() - robot.getBodyVelocityX());
      velocityErrorPitchAngle.set(velocityError * scaleForVelToAngle.getDoubleValue());
      velocityErrorPitchAngle.set(MathTools.clamp(velocityErrorPitchAngle.getDoubleValue(), maxVelocityErrorAngle.getDoubleValue()));

      feedForwardAngle.set(filteredDesiredVelocityX.getDoubleValue() * feedForwardGain.getDoubleValue());


      double angle = pitchAngleForCapture.getDoubleValue()+ feedForwardAngle.getDoubleValue() + velocityErrorPitchAngle.getDoubleValue();

      //angle = MathTools.clamp(angle, pitchAngleForCapture.getDoubleValue() - stepToStepHipAngleDelta.getDoubleValue(),
       //                       pitchAngleForCapture.getDoubleValue() + stepToStepHipAngleDelta.getDoubleValue());

      */

      double angle = -Math.tan((nextFootStepX-robot.getBodyPositionX())/robot.getBodyHeight());
      return angle;
   }

   private double getDesiredHipRollAngle()
   {
      double legLength = robot.upperLinkLength + desiredKneeStance.getDoubleValue();
      rollAngleForCapture.set(HipAngleCapturePointCalculator3D.getHipRollAngle(robot.getBodyVelocityY(), legLength));
      rollAngleForCapture.set(rollAngleForCapture.getDoubleValue() * Math.signum(robot.getBodyVelocityY()));

      //only use a fraction of it
      //angleForCapture.set(0.8 *angleForCapture.getDoubleValue());

      //limit this angle
      rollAngleForCapture.set(MathTools.clamp(rollAngleForCapture.getDoubleValue(), MAX_HIP_ANGLE));

      //angle is opposite sign of desired velocity
      double velocityError = (filteredDesiredVelocityY.getDoubleValue() - robot.getBodyVelocityY());
      velocityErrorRollAngle.set(-velocityError * scaleForVelToAngle.getDoubleValue());
      velocityErrorRollAngle.set(MathTools.clamp(velocityErrorRollAngle.getDoubleValue(), maxVelocityErrorAngle.getDoubleValue()));

      feedForwardAngle.set(-filteredDesiredVelocityY.getDoubleValue() * feedForwardGain.getDoubleValue());
      double angle = rollAngleForCapture.getDoubleValue() + feedForwardAngle.getDoubleValue() + velocityErrorRollAngle.getDoubleValue();

      //angle = MathTools.clamp(angle, rollAngleForCapture.getDoubleValue() - stepToStepHipAngleDelta.getDoubleValue(),
      //                        rollAngleForCapture.getDoubleValue() + stepToStepHipAngleDelta.getDoubleValue());
      return angle;

   }

   private void controlHipToMaintainPitch(RobotSide robotSide)
   {
      double currentPitch = robot.getBodyPitch();
      double currentPitchRate = robot.getBodyPitchVelocity();

      double controlEffort = -hipPitchControllers.get(robotSide).compute(currentPitch, desiredPitch.getDoubleValue(), currentPitchRate, 0.0, deltaT);
      robot.setHipPitchTorque(robotSide, controlEffort);
   }

   private void controlHipToMaintainRoll(RobotSide robotSide)
   {
      double currentRoll = robot.getBodyRoll();
      double currentRollRate = robot.getBodyRollVelocity();

      double controlEffort = -hipRollControllers.get(robotSide).compute(currentRoll, desiredRoll.getDoubleValue(), currentRollRate, 0.0, deltaT);
      robot.setHipRollTorque(robotSide, controlEffort);
   }

   private void controlHipToMaintainYaw(RobotSide robotSide)
   {
      double currentYaw = robot.getBodyYaw();
      double currentYawRate = robot.getBodyYawVelocity();

      double controlEffort = -hipYawControllers.get(robotSide).compute(currentYaw, 0.0, currentYawRate, 0.0, deltaT);
      robot.setHipYawTorque(robotSide, controlEffort);
   }

   private void addOppositeLegHipPitchTorque(RobotSide legToAddTorque)
   {
      double oppositeLegTorque = robot.getHipPitchTorque(legToAddTorque.getOppositeSide());
      double currentTorque = robot.getHipPitchTorque(legToAddTorque);
      robot.setHipPitchTorque(legToAddTorque, currentTorque - oppositeLegTorque);
   }

   private void addOppositeLegHipRollTorque(RobotSide legToAddTorque)
   {
      double oppositeLegTorque = robot.getHipRollTorque(legToAddTorque.getOppositeSide());
      double currentTorque = robot.getHipRollTorque(legToAddTorque);
      robot.setHipRollTorque(legToAddTorque, currentTorque - oppositeLegTorque);
   }

   private void addInertiaControlPitch(RobotSide robotSide)
   {
      double tauHipInertia = -1 * angularMomentumPitchControllers.get(robotSide).compute(robot.getBodyVelocityX(), filteredDesiredVelocityX.getDoubleValue(),
                                                                                         robot.getBodyAccelerationX(), 0.0, deltaT);
      double currentTorque = robot.getHipPitchTorque(robotSide);
      robot.setHipPitchTorque(robotSide, currentTorque + tauHipInertia);
   }

   private void addInertiaControlRoll(RobotSide robotSide)
   {
      double tauHipInertia = 1 * angularMomentumRollControllers.get(robotSide).compute(robot.getBodyVelocityY(), filteredDesiredVelocityY.getDoubleValue(),
                                                                                       robot.getBodyAccelerationY(), 0.0, deltaT);
      double currentTorque = robot.getHipRollTorque(robotSide);
      robot.setHipRollTorque(robotSide, currentTorque + tauHipInertia);
   }

   private void addCoPPitchControl(RobotSide robotSide)
   {

      double anklePositionX = robot.getBodyPositionX() - Math.tan(robot.getHipPitchPosition(robotSide)) / robot.getZ0();
      double omega0sq = -robot.getGravity() / robot.getZ0();
      double CoPX = (omega0sq * robot.getBodyPositionX() - robot.getBodyAccelerationX()) / omega0sq;

      double dCoPX = -anklePitchControllers.get(robotSide)
                                           .compute(robot.getBodyVelocityX(), filteredDesiredVelocityX.getDoubleValue(), 0.0 * robot.getBodyAccelerationX(), 0.0,
                                                    deltaT);
      coPLocationX.set(CoPX - anklePositionX);
      if (coPLocationX.getDoubleValue() < robot.getFootSizeX())
      {
         double tauAnkle = dCoPX * Math.cos(robot.getHipPitchPosition(robotSide)) * robot.getKneeTorque(robotSide);
         ankleControlTorquePitch.set(MathTools.clamp(tauAnkle, -0.4 * robot.getFootSizeX() * robot.getBodyMass() * robot.getGravity()));
         robot.setAnklePitchTorque(robotSide, ankleControlTorquePitch.getDoubleValue());
      }
   }

   private void addCoPRollControl(RobotSide robotSide)
   {

      double anklePositionY = robot.getBodyPositionY() + Math.tan(robot.getHipRollPosition(robotSide)) / robot.getZ0();
      double omega0sq = -robot.getGravity() / robot.getZ0();
      double CoPY = (omega0sq * robot.getBodyPositionY() - robot.getBodyAccelerationY()) / omega0sq;

      double dCoPY = ankleRollControllers.get(robotSide)
                                         .compute(robot.getBodyVelocityY(), filteredDesiredVelocityY.getDoubleValue(), 0.0 * robot.getBodyAccelerationY(), 0.0,
                                                  deltaT);
      coPLocationY.set(CoPY - anklePositionY);
      if (coPLocationY.getDoubleValue() < robot.getFootSizeY())
      {
         double tauAnkle = dCoPY * Math.cos(robot.getHipRollPosition(robotSide)) * robot.getKneeTorque(robotSide);
         ankleControlTorqueRoll.set(MathTools.clamp(tauAnkle, -0.4 * robot.getFootSizeY() * robot.getBodyMass() * robot.getGravity()));
         robot.setAnkleRollTorque(robotSide, ankleControlTorqueRoll.getDoubleValue());
      }
   }

   private void addAnkleYawTorque(RobotSide robotSide)
   {
      double torque =
            ((robot.getHipRollTorque(robotSide) - robot.getAnkleRollTorque(robotSide)) / robot.getZ0()) * (Math.tan(robot.getHipPitchPosition(robotSide))
                  / robot.getZ0()) + ((robot.getHipPitchTorque(robotSide) - robot.getAnklePitchTorque(robotSide)) / robot.getZ0()) * (
                  Math.tan(robot.getHipRollPosition(robotSide)) / robot.getZ0()) + robot.getHipYawTorque(robotSide);
      robot.setAnkleYawTorque(robotSide, torque);
   }

   private void controlKneeToMaintainBodyHeight(RobotSide robotSide)
   {
      double currentHeight = robot.getBodyHeight();
      double currentHeightRate = robot.getBodyHeightVelocity();

      double controlEffort = kneeControllers.get(robotSide).compute(currentHeight, desiredHeight.getDoubleValue(), currentHeightRate, 0.0, deltaT);
      robot.setKneeTorque(robotSide, controlEffort);
   }

   private void controlKneeToBodyHeight(RobotSide robotSide, double desiredHeight, double desiredHeightRate)
   {
      double currentHeight = robot.getBodyHeight();
      double currentHeightRate = robot.getBodyHeightVelocity();

      double controlEffort = kneeControllers.get(robotSide).compute(currentHeight, desiredHeight, currentHeightRate, desiredHeightRate, deltaT);
      robot.setKneeTorque(robotSide, controlEffort);
   }

   private void controlKneeToPosition(RobotSide robotSide, double desiredPosition, double desiredVelocity)
   {
      double kneePosition = robot.getKneePosition(robotSide);
      double kneePositionRate = robot.getKneeVelocity(robotSide);

      double controlEffort = kneeControllers.get(robotSide).compute(kneePosition, desiredPosition, kneePositionRate, desiredVelocity, deltaT);
      robot.setKneeTorque(robotSide, controlEffort);
   }

   private void controlKneeToPositionSoft(RobotSide robotSide, double desiredPosition, double desiredVelocity)
   {
      double kneePosition = robot.getKneePosition(robotSide);
      double kneePositionRate = robot.getKneeVelocity(robotSide);

      double controlEffort = kneeControllers.get(robotSide).compute(kneePosition, desiredPosition, kneePositionRate, desiredVelocity, deltaT);
      robot.setKneeTorque(robotSide, controlEffort);
   }

   private void controlKneeToPositionV2(RobotSide robotSide, double desiredRobotHeight, double desiredVelocity)
   {
      double legLenght = robot.upperLinkLength + robot.getKneePosition(robotSide);
      double kneePositionRate = robot.getKneeVelocity(robotSide);
      double hipAngle = robot.getHipPitchPosition(robotSide);


      double controlEffort = kneeControllers.get(robotSide).compute(legLenght*Math.cos(hipAngle), desiredRobotHeight, kneePositionRate, desiredVelocity, deltaT);
      robot.setKneeTorque(robotSide, controlEffort);
   }

   private void createEvenStepPlan(double nStepsNoCurrent, double stepLength)
   {
      footStepPlan.add(0.0);
      double xPos = stepLength;
      for(int i = 0; i<nStepsNoCurrent; i++)
      {
         footStepPlan.add(xPos);
         xPos=xPos+stepLength;
      }
   }


   public void setDesiredBodyVelocityX(double velocityX)
   {
      desiredBodyVelocityX.set(velocityX);
   }

   public void setDesiredBodyVelocityY(double velocityY)
   {
      desiredBodyVelocityY.set(velocityY);
   }


   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
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
      //      for (RobotSide robotSide : RobotSide.values)
      //      {
      //         double currentPosition = robot.getKneePosition(robotSide);
      //         double currentVelocity = robot.getKneeVelocity(robotSide);
      //
      //         double effort = kneeControllers.get(robotSide).compute(currentPosition, desiredKneeExtension.getDoubleValue(), currentVelocity, 0.0, deltaT);
      //         robot.setKneeTorque(robotSide, effort);
      //      }

      filteredDesiredVelocityX.update();
      filteredDesiredVelocityY.update();

      centerOfMassPosition.set(robot.getBodyPositionX(), robot.getBodyPositionY(), robot.getBodyHeight());
      centerOfMassVelocity.set(robot.getBodyVelocityX(), robot.getBodyVelocityY(), robot.getBodyHeightVelocity());
      centerOfMassAcceleration.set(robot.getBodyAccelerationX(),robot.getBodyAccelerationY(),0.0);
      centerOfMassPosition2D.set(robot.getBodyPositionX(),robot.getBodyPositionY());

      omega.set(sqrt(-robot.getGravityZ() / robot.getBodyHeight()));
      updateCapturePointEstimate();
      updateCenterOfPressureEstimate();

      stateMachine.doActionAndTransition();





   }

   private void updateCapturePointEstimate()
   {

      currentICP.set(centerOfMassVelocity);
      currentICP.scale(1.0 / omega.getDoubleValue());
      currentICP.add(centerOfMassPosition);
      currentICP2D.set(currentICP.getX(),currentICP.getY());

   }

   private void updateCenterOfPressureEstimate()
   {
      currentCOP.set(centerOfMassAcceleration);
      currentCOP.scale(-1.0 / (Math.pow(omega.getDoubleValue(), 2.0)));
      currentCOP.add(centerOfMassPosition);
      currentCOP.scale(1);
      currentCOP2D.set(currentCOP.getX(),currentCOP.getY());
   }



   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public Artifact getDesiredCoPGraphicArtifact()
   {
      return desiredCoPGraphicArtifact;
   }

   public Artifact getCurrentCoPGraphicArtifact()
   {
      return currentCoPGraphicArtifact;
   }

   public Artifact getCurrentCoMGraphicArtifact()
   {
      return currentCoMGraphicArtifact;
   }

   public Artifact getDesiredICPGraphicArtifact()
   {
      return desiredICPGraphicArtifact;
   }

   public Artifact getCurrentICPGraphicArtifact()
   {
      return currentICPGraphicArtifact;
   }

   public Artifact getCurrentLFootGraphicArtifact() { return currentLFootGraphicArtifact;}
   public Artifact getCurrentRFootGraphicArtifact() { return currentRFootGraphicArtifact;}

   public Artifact getFootArtifact()
   {
      return footArtifact;
   }

   public enum ControllerState
   {
      START, LEFT_SUPPORT, RIGHT_SUPPORT;
   }

}
