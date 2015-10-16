package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.LinkNames;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;

public class Step7WalkingController implements RobotController
{

   /**
    * Initialization
    */

   //Variables
   private Step7IDandSCSRobot_pinKnee robot;
   private String name = getClass().getSimpleName();
   private YoVariableRegistry registry = new YoVariableRegistry("controllerRegistry");
   private double deltaT;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoGraphicsList yoGraphicsList = new YoGraphicsList(name);
   private final ArtifactList artifactList = new ArtifactList(name);

   private PIDController controllerBodyZ;
   private PIDController controllerBodyPitchDoubleSupport, controllerHipPitch, controllerBodyPitchSingleSupport;
   private PIDController controllerKneePitchSwing, controllerKneePitchStraighten;
   private PIDController controllerAnkleStraighten, controllerAnkleToeOff, controllerAnkleSwing;

   private DoubleYoVariable desiredBodyZ, desiredKneePitchSwing, desiredKneePitchStraighten;
   private DoubleYoVariable desiredHipPitch, desiredBodyPitch;
   private DoubleYoVariable desiredAnklePitchSwing, desiredAnklePitchToeOff, desiredAnklePitchStraighten;

   private DoubleYoVariable ankleTauToeOff;
   private DoubleYoVariable kneeTau;
   private DoubleYoVariable hipTau;
   private DoubleYoVariable ankleTau;

   private Quat4d rotationToPack = new Quat4d();
   private Vector3d velocityToPack = new Vector3d();

   private boolean heelOnTheFloor, toeOnTheFloor;
   private final DoubleYoVariable minSupportTime = new DoubleYoVariable("minSupportTime", registry);
   private final DoubleYoVariable swingTime = new DoubleYoVariable("swingTime", registry);

   // State Machine
   private enum States
   {
      SUPPORT, TOE_OFF, SWING //, STRAIGHTEN
   }

   private final StateMachine<States> leftStateMachine, rightStateMachine;
   private final SideDependentList<StateMachine<States>> stateMachines;

   //ICP calculation and graphics
   private double comPosX, comPosZ, comVelX, icpPosX;
   private YoGraphicPosition icpGraphics;
   private DoubleYoVariable icpYoPosX, icpYoPosY, icpYoPosZ;
   private FramePoint2d capturePoint, centerOfMassInWorld, desiredICP;
   private FrameVector2d centerOfMassVelocityInWorld;

   // new stuff
   private final YoFramePoint2d yoDesiredCoP = new YoFramePoint2d("desiredCenterOfPressure", worldFrame, registry);
   private final YoFramePoint2d yoDesiredICP = new YoFramePoint2d("yoDesiredCapturePoint", worldFrame, registry);
   private final YoFramePoint yodesiredPositionSwingFoot = new YoFramePoint("yoDesiredPositionSwingFoot", worldFrame, registry);

   private final SideDependentList<YoFramePoint> yoFootPositions;

   private ParabolicCartesianTrajectoryGenerator swingTrajectory;
   private static final double groundClearance = 0.15;
   private BagOfBalls swingTrajectoryViz;

   private final SideDependentList<DoubleYoVariable> distanceFootToCoPbasedWeights = new SideDependentList<>();

   private final int numOfBalls = 75;

   private final DoubleYoVariable stepLengthFactor = new DoubleYoVariable("stepLengthFactor", registry);
   private final DoubleYoVariable stepLengthCorrectionFactor = new DoubleYoVariable("stepLengthCorrectionFactor", registry);
   private final DoubleYoVariable defaultStepLength = new DoubleYoVariable("defaultStepLength", registry);
   private final DoubleYoVariable maxStepLength = new DoubleYoVariable("maxStepLength", registry);

   private final DoubleYoVariable desiredWalkingVelocity = new DoubleYoVariable("desiredWalkingVelocity", registry);
   private final DoubleYoVariable alphaDesiredVelocity = new DoubleYoVariable("alphaDesiredVelocity", registry);
   private final AlphaFilteredYoVariable desiredWalkingVelocityFiltered = new AlphaFilteredYoVariable("desiredWalkingVelocitySmoothed", registry,
         alphaDesiredVelocity, desiredWalkingVelocity); // filter the velocity so that the transition from one to another is smoother
   private final DoubleYoVariable desiredVelocityToICPFactorDistanceThreshold = new DoubleYoVariable("desiredVelocityToICPFactorDistanceThreshold", registry);

   private final DoubleYoVariable kpSupportKnee = new DoubleYoVariable("kpSupportKnee", registry);
   private final DoubleYoVariable kdSupportKnee = new DoubleYoVariable("kdSupportKnee", registry);
   private final DoubleYoVariable kpSwingFoot = new DoubleYoVariable("kpSwingFoot", registry);
   private final DoubleYoVariable kdSwingFoot = new DoubleYoVariable("kdSwingFoot", registry);
   private final DoubleYoVariable kpCapturePoint = new DoubleYoVariable("kpCapturePoint", registry);
   private final DoubleYoVariable qDesSupportKnee = new DoubleYoVariable("qDesSupportKnee", registry);

   /**
    * Constructor
    */

   public Step7WalkingController(Step7IDandSCSRobot_pinKnee rob, String name, double deltaT)
   {
      this.robot = rob;
      this.name = name;
      this.deltaT = deltaT;
      yoFootPositions = rob.getYoFootPositions();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         distanceFootToCoPbasedWeights.put(robotSide, new DoubleYoVariable(sidePrefix + "distanceFootToCoPbasedWeights", registry));
      }

      // Create the state machines:
      leftStateMachine = new StateMachine<States>("leftState", "leftSwitchTime", States.class, rob.getYoTime(), registry);
      rightStateMachine = new StateMachine<States>("rightState", "rightSwitchTime", States.class, rob.getYoTime(), registry);
      stateMachines = new SideDependentList<StateMachine<States>>(leftStateMachine, rightStateMachine);

      //      // ICP calculation and graphics
      //      icpYoPosX = new DoubleYoVariable("icpYoPosX", registry);
      //      icpYoPosX.set(icpPosX);
      //      icpYoPosY = new DoubleYoVariable("icpYoPosY", registry);
      //      icpYoPosY.set(0.0); 
      //      icpYoPosZ = new DoubleYoVariable("icpYoPosZ", registry);
      //      icpYoPosZ.set(0.0);
      //      icpGraphics = new YoGraphicPosition("icpGraph", icpYoPosX, icpYoPosY, icpYoPosZ, 0.1, YoAppearance.Tomato());
      //      yoGraphicsListRegistry.registerYoGraphic("icp", icpGraphics);

      initialConditions();
      initializeControls();
      initializeVizualizers();
      setupStateMachines();
   }

   /**
    *  Initializations
    */

   public void initializeVizualizers()
   {
      // 2D CoP and ICP visualizer
      YoArtifactPosition desiredCoPArtifact = new YoArtifactPosition("DesiredCoP", yoDesiredCoP.getYoX(), yoDesiredCoP.getYoY(),
            GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.Red().getColor().get(), 0.01);
      YoArtifactPosition desiredICPArtifact = new YoArtifactPosition("DesiredICP", yoDesiredICP.getYoX(), yoDesiredICP.getYoY(),
            GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.Blue().getColor().get(), 0.01);
      artifactList.add(desiredCoPArtifact);
      artifactList.add(desiredICPArtifact);

      // 3D trajectory visualizer
      YoGraphicPosition desiredPositionSwingFootViz = new YoGraphicPosition("desiredPositionSwingFoot", yodesiredPositionSwingFoot, 0.05, YoAppearance.Green());
      yoGraphicsList.add(desiredPositionSwingFootViz);

      DoubleProvider swingTimeProvider = new YoVariableDoubleProvider(swingTime);
      swingTrajectory = new ParabolicCartesianTrajectoryGenerator("swingTrajectory", worldFrame, swingTimeProvider, groundClearance, registry);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      swingTrajectoryViz = new BagOfBalls(numOfBalls, 0.015, registry, yoGraphicsListRegistry);
      yoGraphicsList.addAll(yoGraphicsListRegistry.getYoGraphicsLists().get(0).getYoGraphics());
   }

   public void initializeControls()
   {
      controllerBodyZ = new PIDController("bodyZ", registry);
      controllerBodyZ.setProportionalGain(1000.0);
      controllerBodyZ.setDerivativeGain(500.0);
      desiredBodyZ = new DoubleYoVariable("desiredBodyZ", registry);
      desiredBodyZ.set(1.5);

      controllerBodyPitchDoubleSupport = new PIDController("bodyPitch", registry);
      controllerBodyPitchDoubleSupport.setProportionalGain(500.0);
      controllerBodyPitchDoubleSupport.setDerivativeGain(50.0);
      desiredBodyPitch = new DoubleYoVariable("desiredBodyPitch", registry);
      desiredBodyPitch.set(0.0);

      controllerBodyPitchSingleSupport = new PIDController("bodyPitchSingleSupport", registry);
      controllerBodyPitchSingleSupport.setProportionalGain(5000.0);
      controllerBodyPitchSingleSupport.setDerivativeGain(500.0);

      controllerHipPitch = new PIDController("hipPitch", registry);
      controllerHipPitch.setProportionalGain(700.0);
      controllerHipPitch.setDerivativeGain(70.0);
      desiredHipPitch = new DoubleYoVariable("desiredHipPitch", registry);
      //      desiredHipPitch.set(-0.42);
      desiredHipPitch.set(-0.6);

      controllerKneePitchSwing = new PIDController("kneePitchSwing", registry);
      controllerKneePitchSwing.setProportionalGain(25000.0);
      controllerKneePitchSwing.setDerivativeGain(2000.0);
      desiredKneePitchSwing = new DoubleYoVariable("desiredKneePitchSwing", registry);
      desiredKneePitchSwing.set(1.3);

      controllerKneePitchStraighten = new PIDController("kneePitchStraighten", registry);
      controllerKneePitchStraighten.setProportionalGain(10000.0);
      controllerKneePitchStraighten.setDerivativeGain(1000.0);
      desiredKneePitchStraighten = new DoubleYoVariable("desiredKneePitchStraighten", registry);
      desiredKneePitchStraighten.set(0.0);

      controllerAnkleStraighten = new PIDController("ankleStraighten", registry);
      controllerAnkleStraighten.setProportionalGain(50.0);
      controllerAnkleStraighten.setDerivativeGain(5.0);
      desiredAnklePitchStraighten = new DoubleYoVariable("desiredAnklePitchStraighten", registry);
      desiredAnklePitchStraighten.set(0.2);

      controllerAnkleToeOff = new PIDController("ankleToeOff", registry);
      controllerAnkleToeOff.setProportionalGain(500.0);
      controllerAnkleToeOff.setDerivativeGain(50.0);
      desiredAnklePitchToeOff = new DoubleYoVariable("desiredAnklePitchToeOff", registry);
      desiredAnklePitchToeOff.set(0.08);

      controllerAnkleSwing = new PIDController("ankleSwing", registry);
      controllerAnkleSwing.setProportionalGain(100.0);
      controllerAnkleSwing.setDerivativeGain(10.0);
      desiredAnklePitchSwing = new DoubleYoVariable("desiredAnklePitchSwing", registry);
      desiredAnklePitchSwing.set(-0.65);

      hipTau = new DoubleYoVariable("hipTau", registry);
      kneeTau = new DoubleYoVariable("kneeTau", registry);
      ankleTau = new DoubleYoVariable("ankleTau", registry);

      ankleTauToeOff = new DoubleYoVariable("ankleTauToeOff", registry);
      ankleTauToeOff.set(200.0);

      //new
      kpSupportKnee.set(200.0);
      kdSupportKnee.set(100.0);
      qDesSupportKnee.set(0.0);

      kpSwingFoot.set(2000.0);
      kdSwingFoot.set(200.0);

      kpCapturePoint.set(2.0);
   }

   /**
    * State Machine Related Methods
    */

   public void initialConditions()
   {
      swingTime.set(0.35); //swingTime.set(0.4);   //swingTime.set(0.22); 
      minSupportTime.set(0.3);

      alphaDesiredVelocity.set(0.9999);
      desiredWalkingVelocity.set(0.3);
      defaultStepLength.set(0.30);
      maxStepLength.set(0.70);

      desiredVelocityToICPFactorDistanceThreshold.set(0.7);
      stepLengthFactor.set(0.2);
      stepLengthCorrectionFactor.set(0.5);
   }

   private void setupStateMachines()
   {
      // States and Actions:
      State<States> leftSupportState = new SupportState(RobotSide.LEFT, States.SUPPORT);
      State<States> rightSupportState = new SupportState(RobotSide.RIGHT, States.SUPPORT);
      State<States> leftToeOffState = new ToeOffState(RobotSide.LEFT, States.TOE_OFF);
      State<States> rightToeOffState = new ToeOffState(RobotSide.RIGHT, States.TOE_OFF);
      State<States> leftSwingState = new SwingState(RobotSide.LEFT, States.SWING);
      State<States> rightSwingState = new SwingState(RobotSide.RIGHT, States.SWING);
//      State<States> leftStraightenState = new StraightenState(RobotSide.LEFT, States.STRAIGHTEN);
//      State<States> rightStraightenState = new StraightenState(RobotSide.RIGHT, States.STRAIGHTEN);

      // Transition Conditions:
      StateTransitionCondition leftHeelUnloaded = new HeelOffGroundCondition(RobotSide.LEFT);
      StateTransitionCondition leftToeUnloaded = new ToeOffGroundCondition(RobotSide.LEFT);
      StateTransitionCondition leftHeelTouchedDown = new HeelOnGroundCondition(RobotSide.LEFT);

      StateTransitionCondition rightHeelUnloaded = new HeelOffGroundCondition(RobotSide.RIGHT);
      StateTransitionCondition rightToeUnloaded = new ToeOffGroundCondition(RobotSide.RIGHT);
      StateTransitionCondition rightHeelTouchedDown = new HeelOnGroundCondition(RobotSide.RIGHT);

      // Left State Transitions:
      StateTransition<States> leftSupportToToeOff = new StateTransition<States>(States.TOE_OFF, leftHeelUnloaded);
      leftSupportToToeOff.addTimePassedCondition(minSupportTime);
      leftSupportState.addStateTransition(leftSupportToToeOff);

      StateTransition<States> leftToeOffToSwing = new StateTransition<States>(States.SWING, leftToeUnloaded);
      leftToeOffState.addStateTransition(leftToeOffToSwing);

//      StateTransition<States> leftSwingToStraighten = new StateTransition<States>(States.STRAIGHTEN, swingTime);
//      leftSwingState.addStateTransition(leftSwingToStraighten);
//
//      StateTransition<States> leftStraightenToSupport = new StateTransition<States>(States.SUPPORT, leftHeelTouchedDown);
//      leftStraightenState.addStateTransition(leftStraightenToSupport);
      
      StateTransition<States> leftSwingToSupport = new StateTransition<States>(States.SUPPORT, leftHeelTouchedDown);
      leftSwingState.addStateTransition(leftSwingToSupport);

      // Right State Transitions:
      StateTransition<States> rightSupportToToeOff = new StateTransition<States>(States.TOE_OFF, rightHeelUnloaded);
      rightSupportToToeOff.addTimePassedCondition(minSupportTime);
      rightSupportState.addStateTransition(rightSupportToToeOff);

      StateTransition<States> rightToeOffToSwing = new StateTransition<States>(States.SWING, rightToeUnloaded);
      rightToeOffState.addStateTransition(rightToeOffToSwing);

//      StateTransition<States> rightSwingToStraighten = new StateTransition<States>(States.STRAIGHTEN, swingTime);
//      rightSwingState.addStateTransition(rightSwingToStraighten);
//
//      StateTransition<States> rightStraightenToSupport = new StateTransition<States>(States.SUPPORT, rightHealTouchedDown);
//      rightStraightenState.addStateTransition(rightStraightenToSupport);

      StateTransition<States> rightSwingToSupport = new StateTransition<States>(States.SUPPORT, rightHeelTouchedDown);
      leftSwingState.addStateTransition(rightSwingToSupport);
      
      // Assemble the Left State Machine:
      leftStateMachine.addState(leftSupportState);
      leftStateMachine.addState(leftToeOffState);
      leftStateMachine.addState(leftSwingState);
//      leftStateMachine.addState(leftStraightenState);

      // Assemble the Right State Machine:
      rightStateMachine.addState(rightSupportState);
      rightStateMachine.addState(rightToeOffState);
      rightStateMachine.addState(rightSwingState);
//      rightStateMachine.addState(rightStraightenState);

      // Set the Initial States:
//      leftStateMachine.setCurrentState(States.STRAIGHTEN);
      leftStateMachine.setCurrentState(States.SWING);
      rightStateMachine.setCurrentState(States.SUPPORT);
   }

   private void walkingStateMachine()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         // Do action and check transition conditions
         stateMachines.get(robotSide).doAction();
         StateMachine<States> stateMachine = stateMachines.get(robotSide);
         stateMachine.checkTransitionConditions();

         // ID and SCS cross talk
         robot.updatePositionsIDrobot();
         robot.updateTorquesSCSrobot(); //Note: this is getting called twice, once per side
      }
   }

   /**
    * Body pitch controllers
    */
   private DoubleYoVariable controlBodyPitchDoubleSupport()
   {
      robot.getBodyPitch(rotationToPack);
      double pitchFromQuaternion = RotationTools.getPitchFromQuaternion(rotationToPack);

      robot.getBodyAngularVel(velocityToPack);
      double bodyAngularVel = velocityToPack.getY();

      hipTau.set(controllerBodyPitchDoubleSupport.compute(pitchFromQuaternion, desiredBodyPitch.getDoubleValue(), bodyAngularVel, 0.0, deltaT));
      return hipTau;
   }

   private DoubleYoVariable controlBodyPitchSingleSupport()
   {
      robot.getBodyPitch(rotationToPack);
      double pitchFromQuaternion = RotationTools.getPitchFromQuaternion(rotationToPack);

      robot.getBodyAngularVel(velocityToPack);
      double bodyAngularVel = velocityToPack.getY();

      hipTau.set(controllerBodyPitchSingleSupport.compute(pitchFromQuaternion, desiredBodyPitch.getDoubleValue(), bodyAngularVel, 0.0, deltaT));
      return hipTau;
   }

   /**
    * State Classes
    */

   /////////////////////////////////// (1) Support
   private class SupportState extends State<States>
   {
      private final RobotSide robotSide;

      public SupportState(RobotSide robotSide, States stateEnum)
      {
         super(States.SUPPORT);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         // Hip --> Body pitch (single support)
         hipTau = controlBodyPitchSingleSupport();
         robot.setHipTau(robotSide, -hipTau.getDoubleValue());

         // Knee --> Body height
         kneeTau.set(controllerBodyZ.compute(robot.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -robot.getKneeVelocity(robotSide), 0.0, deltaT));
         robot.setKneeTau(robotSide, -kneeTau.getDoubleValue());

         // Ankle --> No action
      }

      public void doTransitionIntoAction()
      {
      }

      public void doTransitionOutOfAction()
      {
      }
   }

   ///////////////////////////////////  (2) Toe Off
   private class ToeOffState extends State<States>
   {
      private final RobotSide robotSide;

      public ToeOffState(RobotSide robotSide, States stateEnum)
      {
         super(States.TOE_OFF);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         // Hip --> Body pitch (double support)
         hipTau = controlBodyPitchDoubleSupport();
         robot.setHipTau(robotSide, -hipTau.getDoubleValue());

         // Knee --> Body height
         kneeTau.set(controllerBodyZ.compute(robot.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -robot.getKneeVelocity(robotSide), 0.0, deltaT));
         robot.setKneeTau(robotSide, -kneeTau.getDoubleValue());

         // Ankle --> Ankle pitch
         if (robot.getBodyPositionX() > robot.getToeX(robotSide))
         {
            ankleTau.set(controllerAnkleToeOff.compute(robot.getAnklePitch(robotSide), desiredAnklePitchToeOff.getDoubleValue(),
                  robot.getBodyVelX(velocityToPack), 0.7, deltaT));
            robot.setAnkleTau(robotSide, ankleTau.getDoubleValue());
         }

      }

      public void doTransitionIntoAction()
      {
      }

      public void doTransitionOutOfAction()
      {
      }
   }

   ///////////////////////////////////  (3) Swing
   private class SwingState extends State<States>
   {

      private final RobotSide robotSide;

      // swing trajectory variables 
      private final FramePoint initialPosition = new FramePoint();
      private final FrameVector initialVelocity = new FrameVector();
      private final FrameVector initialAcceleration = new FrameVector();
      private final FramePoint finalDesiredPosition = new FramePoint();
      private final FrameVector finalDesiredVelocity = new FrameVector();
      private int nTicksSinceTrajectoryIsDone = 0;

      private final FramePoint desiredPosition = new FramePoint();
      private final FramePoint desiredPositionTrajectoryViz = new FramePoint();
      private final FrameVector pControl = new FrameVector();

      private final FrameVector currentVelocity = new FrameVector();
      private final FrameVector desiredVelocity = new FrameVector();
      private final FrameVector dControl = new FrameVector();

      private final ReferenceFrame swingSoleFrame;
      private final GeometricJacobian swingLegJacobian;
   
      private final Wrench desiredSwingFootWrench = new Wrench();

      public SwingState(RobotSide robotSide, States stateEnum)
      {
         super(States.SWING);
         this.robotSide = robotSide;
         swingSoleFrame = robot.getSoleFrame(robotSide);
         swingLegJacobian = robot.getLegJacobian(robotSide);
      }

      public void doAction()
      {
         // ************** Hip and knee --> Foot follows parabolic trajectory
         updateSwingTrajectory();
         swingTrajectory.compute(getTimeInCurrentState());

         if (swingTrajectory.isDone())
         {
            nTicksSinceTrajectoryIsDone++;
            desiredVelocity.setIncludingFrame(finalDesiredVelocity);
         }

         else
         {
            swingTrajectory.packVelocity(desiredVelocity);
         }

         // 1) proportional control
         swingTrajectory.packPosition(desiredPosition);
         desiredPosition.add(0.0, 0.0, nTicksSinceTrajectoryIsDone * finalDesiredVelocity.getZ() * deltaT);
         yodesiredPositionSwingFoot.setAndMatchFrame(desiredPosition);
         desiredPosition.changeFrame(swingSoleFrame);
         pControl.setIncludingFrame(desiredPosition);
         pControl.scale(kpSwingFoot.getDoubleValue());

         // 2) derivative control   
         robot.getFootLinearVelocity(robotSide, currentVelocity); //from heel GC point
         dControl.setIncludingFrame(desiredVelocity);
         dControl.sub(currentVelocity);
         dControl.changeFrame(swingSoleFrame);
         dControl.scale(kdSwingFoot.getDoubleValue());

         // 3) computing torques using Wrench and Jacobian
         desiredSwingFootWrench.setToZero(robot.getLegRigidBody(LinkNames.LOWER_LINK, robotSide).getBodyFixedFrame(), swingSoleFrame);
         desiredSwingFootWrench.setLinearPart(pControl);
         desiredSwingFootWrench.addLinearPart(dControl);

         DenseMatrix64F desiredSwingTaus = swingLegJacobian.computeJointTorques(desiredSwingFootWrench);
//         System.out.println(desiredSwingTaus);
         robot.getHipJointID(robotSide).setTau(desiredSwingTaus.get(0, 0));
         robot.getKneeJointID(robotSide).setTau(desiredSwingTaus.get(1, 0));

         // ****************  Ankle --> Keep foot parallel to ground
         ankleTau.set(controllerAnkleSwing.compute(robot.getAnklePitch(robotSide), desiredAnklePitchSwing.getDoubleValue(), robot.getAnkleVelocity(robotSide), 0.0, deltaT));
         robot.setAnkleTau(robotSide, ankleTau.getDoubleValue());
      }
      
      // Update swing trajectory
      private void updateSwingTrajectory()
      {
         yoFootPositions.get(robotSide.getOppositeSide()).getFrameTuple(finalDesiredPosition);
         finalDesiredPosition.changeFrame(worldFrame);

         double dx = Math.signum(robot.getBodyVelX(velocityToPack)) * defaultStepLength.getDoubleValue(); // Use the sign to make it walk in both directions depending on if v>0 or v<0
         dx += stepLengthFactor.getDoubleValue() * desiredWalkingVelocityFiltered.getDoubleValue();
         dx -= stepLengthCorrectionFactor.getDoubleValue() * (desiredWalkingVelocityFiltered.getDoubleValue() - robot.getBodyVelX(velocityToPack));

         if (Math.abs(dx) > maxStepLength.getDoubleValue())
            dx = Math.signum(dx) * maxStepLength.getDoubleValue(); //just take the biggest step possible

         finalDesiredPosition.add(dx, 0.0, 0.0);
         finalDesiredVelocity.setZ(-0.30);
         swingTrajectory.initialize(initialPosition, initialVelocity, initialAcceleration, finalDesiredPosition, finalDesiredVelocity);

         double deltaT = swingTime.getDoubleValue() / numOfBalls;
         for (int i = 0; i < numOfBalls; i++)
         {
            swingTrajectory.computeNextTick(desiredPositionTrajectoryViz, deltaT);
            swingTrajectoryViz.setBall(desiredPositionTrajectoryViz, i);
         }
      }

      public void doTransitionIntoAction()
      {
         nTicksSinceTrajectoryIsDone = 0;
         yoDesiredCoP.setByProjectionOntoXYPlane(yoFootPositions.get(robotSide.getOppositeSide()));
         initialPosition.setToZero(swingSoleFrame);
         initialPosition.changeFrame(worldFrame);
         updateSwingTrajectory();
      }
      
      public void doTransitionOutOfAction()
      {
      }
   }

   ///////////////////////////////////  (4)  Straighten
//   private class StraightenState extends State<States>
//   {
//      private final RobotSide robotSide;
//
//      public StraightenState(RobotSide robotSide, States stateEnum)
//      {
//         super(States.STRAIGHTEN);
//         this.robotSide = robotSide;
//      }
//
//      public void doAction()
//      {
//         //Hip --> Hip pitch      
//         hipTau.set(controllerHipPitch.compute(robot.getHipPitch(robotSide), desiredHipPitch.getDoubleValue(), robot.getHipVelocity(robotSide), 0.0, deltaT));
//         robot.setHipTau(robotSide, hipTau.getDoubleValue());
//
//         //Knee --> Leg straight
//         kneeTau.set(controllerKneePitchStraighten.compute(robot.getKneePitch(robotSide), desiredKneePitchStraighten.getDoubleValue(),
//               robot.getKneeVelocity(robotSide), 0.0, deltaT));
//         robot.setKneeTau(robotSide, kneeTau.getDoubleValue());
//
//         //Ankle --> Ankle pitch
//         ankleTau.set(controllerAnkleStraighten.compute(robot.getAnklePitch(robotSide), desiredAnklePitchStraighten.getDoubleValue(),
//               robot.getAnkleVelocity(robotSide), 0.0, deltaT));
//         robot.setAnkleTau(robotSide, ankleTau.getDoubleValue());
//      }
//
//      public void doTransitionIntoAction()
//      {
//      }
//
//      public void doTransitionOutOfAction()
//      {
//      }
//   }

   /**
    * State Transition Conditions
    */

   // Support To ToeOff
   public class HeelOffGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOffGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         heelOnTheFloor = robot.heelOnTheFloor(robotSide); //onTheFloor will be true if the heel is on the floor, but we want the opposite
         boolean oppositeHeelOnTheFloor = robot.heelOnTheFloor(robotSide.getOppositeSide());
         return (!heelOnTheFloor) && oppositeHeelOnTheFloor;
      }
   }

   // ToeOff To Swing
   public class ToeOffGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public ToeOffGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         toeOnTheFloor = robot.toeOnTheFloor(robotSide);
         boolean oppositeHeelOnTheFloor = robot.heelOnTheFloor(robotSide.getOppositeSide());
         boolean heelZ = robot.getHeelZ(robotSide) > 0.31;
         return !toeOnTheFloor && oppositeHeelOnTheFloor && heelZ;
      }
   }

   // Swing To Support
   public class HeelOnGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOnGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         heelOnTheFloor = robot.heelOnTheFloor(robotSide);
         return heelOnTheFloor;
      }
   }

   /**
    *  Getters and setters
    */
   public ArtifactList getArtifactList()
   {
      return artifactList;
   }

   public YoGraphicsList getYoGraphicsList()
   {
      return yoGraphicsList;
   }

   private void updateDesiredICP()
   {

      if (Math.abs(desiredWalkingVelocityFiltered.getDoubleValue()) < 1.0e-3)
      {
         desiredICP.setToZero(robot.getMidSoleZUpFrame());
         desiredICP.changeFrame(worldFrame);
      }

      else
      {
         robot.getCoM(desiredICP);
         desiredICP
               .add(desiredVelocityToICPFactorDistanceThreshold.getDoubleValue() / robot.getOmega0() * desiredWalkingVelocityFiltered.getDoubleValue(), 0.0);
      }

      yoDesiredICP.set(desiredICP);
      robot.getCapturePoint(capturePoint);
   }

   /////////////////////////////////////////////////////////////////////////////////////////////
   public void doControl()
   {
      if (desiredWalkingVelocity.getDoubleValue() == 0.0)  //not working well
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            // Hip --> Body pitch (double support)
            hipTau = controlBodyPitchDoubleSupport();
            robot.setHipTau(robotSide, -hipTau.getDoubleValue());
            
            // Knee --> Body height
            kneeTau.set(controllerBodyZ.compute(robot.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -robot.getKneeVelocity(robotSide), 0.0, deltaT));
            robot.setKneeTau(robotSide, -kneeTau.getDoubleValue());           
         }
      }

      else
         walkingStateMachine();
      //      updateDesiredICP();
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

}
