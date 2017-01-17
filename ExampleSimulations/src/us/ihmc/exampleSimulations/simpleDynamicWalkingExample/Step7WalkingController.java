package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters2.LinkNames;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

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
   private PIDController controllerBodyPitchDoubleSupport, controllerBodyPitchSingleSupport;
   private PIDController controllerAnkleToeOff, controllerAnkleSwing;

   private DoubleYoVariable desiredBodyZ, desiredBodyPitch;
   private DoubleYoVariable desiredAnklePitchSwing, desiredAnklePitchToeOff;

   private DoubleYoVariable kneeTau, hipTau, ankleTau;

   private Quat4d rotationToPack = new Quat4d();
   private Vector3d velocityToPack = new Vector3d();

   private boolean heelOnTheFloor, toeOnTheFloor;
   private final DoubleYoVariable minSupportTime = new DoubleYoVariable("minSupportTime", registry);
   private final DoubleYoVariable swingTime = new DoubleYoVariable("swingTime", registry);   

   // new stuff
   private FramePoint2d capturePoint, desiredICP;
   private final YoFramePoint2d yoDesiredCoP = new YoFramePoint2d("desiredCenterOfPressure", worldFrame, registry);
   private final YoFramePoint2d yoDesiredICP = new YoFramePoint2d("yoDesiredCapturePoint", worldFrame, registry);
   private final YoFramePoint yodesiredPositionSwingFoot = new YoFramePoint("yoDesiredPositionSwingFoot", worldFrame, registry);
   private final SideDependentList<YoFramePoint> yoFootPositions;
   private final SideDependentList<DoubleYoVariable> distanceFootToCoPbasedWeights = new SideDependentList<>();

   private ParabolicCartesianTrajectoryGenerator swingTrajectory;
   private static final double groundClearance = 0.20;
   private BagOfBalls swingTrajectoryViz;
   private final int numOfBalls = 50;

   private final DoubleYoVariable stepLengthFactor = new DoubleYoVariable("stepLengthFactor", registry);
   private final DoubleYoVariable stepLengthCorrectionFactor = new DoubleYoVariable("stepLengthCorrectionFactor", registry);
   private final DoubleYoVariable defaultStepLength = new DoubleYoVariable("defaultStepLength", registry);
   private final DoubleYoVariable maxStepLength = new DoubleYoVariable("maxStepLength", registry);

   private final DoubleYoVariable desiredWalkingVelocity = new DoubleYoVariable("desiredWalkingVelocity", registry);
   private final DoubleYoVariable alphaDesiredVelocity = new DoubleYoVariable("alphaDesiredVelocity", registry);
   private final AlphaFilteredYoVariable desiredWalkingVelocityFiltered = new AlphaFilteredYoVariable("desiredWalkingVelocitySmoothed", registry, alphaDesiredVelocity, desiredWalkingVelocity); // filter the velocity so that the transition from one to another is smoother
   private final DoubleYoVariable desiredVelocityToICPFactorDistanceThreshold = new DoubleYoVariable("desiredVelocityToICPFactorDistanceThreshold", registry);

   private final DoubleYoVariable kpSwingFoot = new DoubleYoVariable("kpSwingFoot", registry);
   private final DoubleYoVariable kdSwingFoot = new DoubleYoVariable("kdSwingFoot", registry);
   private final DoubleYoVariable kpCapturePoint = new DoubleYoVariable("kpCapturePoint", registry);
   
   // State Machine
   private enum States
   {
	   SUPPORT, TOE_OFF, SWING 
   }
   private final StateMachine<States> leftStateMachine, rightStateMachine;
   private final SideDependentList<StateMachine<States>> stateMachines;

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

      // Create the state machines
      leftStateMachine = new StateMachine<States>("leftState", "leftSwitchTime", States.class, rob.getYoTime(), registry);
      rightStateMachine = new StateMachine<States>("rightState", "rightSwitchTime", States.class, rob.getYoTime(), registry);
      stateMachines = new SideDependentList<StateMachine<States>>(leftStateMachine, rightStateMachine);

      // Initialize cositas variadas
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
      YoArtifactPosition desiredCoPArtifact = new YoArtifactPosition("DesiredCoP", yoDesiredCoP.getYoX(), yoDesiredCoP.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.Red().getColor().get(), 0.01);
      YoArtifactPosition desiredICPArtifact = new YoArtifactPosition("DesiredICP", yoDesiredICP.getYoX(), yoDesiredICP.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.Blue().getColor().get(), 0.01);
      artifactList.add(desiredCoPArtifact);
      artifactList.add(desiredICPArtifact);

      // 3D trajectory visualizer
      YoGraphicPosition desiredPositionSwingFootViz = new YoGraphicPosition("desiredPositionSwingFoot", yodesiredPositionSwingFoot, 0.02, YoAppearance.Green());
      System.out.println("initial desired position foot " + yodesiredPositionSwingFoot);
      yoGraphicsList.add(desiredPositionSwingFootViz);

      DoubleProvider swingTimeProvider = new YoVariableDoubleProvider(swingTime);
      swingTrajectory = new ParabolicCartesianTrajectoryGenerator("swingTrajectory", worldFrame, swingTimeProvider, groundClearance, registry);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      swingTrajectoryViz = new BagOfBalls(numOfBalls, 0.009, registry, yoGraphicsListRegistry);
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

      controllerAnkleToeOff = new PIDController("ankleToeOff", registry);
      controllerAnkleToeOff.setProportionalGain(200.0);
      controllerAnkleToeOff.setDerivativeGain(20.0);
      desiredAnklePitchToeOff = new DoubleYoVariable("desiredAnklePitchToeOff", registry);
      desiredAnklePitchToeOff.set(0.06);

      controllerAnkleSwing = new PIDController("ankleSwing", registry);
      controllerAnkleSwing.setProportionalGain(100.0);
      controllerAnkleSwing.setDerivativeGain(10.0);
      desiredAnklePitchSwing = new DoubleYoVariable("desiredAnklePitchSwing", registry);
      desiredAnklePitchSwing.set(-0.3);

      kpSwingFoot.set(20000.0);
      kdSwingFoot.set(200.0);
      kpCapturePoint.set(2.0);
      
      hipTau = new DoubleYoVariable("hipTau", registry);
      kneeTau = new DoubleYoVariable("kneeTau", registry);
      ankleTau = new DoubleYoVariable("ankleTau", registry);
   }

   /**
    * State Machine Related Methods
    */

   public void initialConditions()
   {
	  robot.setInitialVelocity(0.75);
	  
	  swingTime.set(0.5);  
      minSupportTime.set(0.3);

      alphaDesiredVelocity.set(0.9999);
      desiredWalkingVelocity.set(0.5);
      defaultStepLength.set(0.30);
      maxStepLength.set(0.60);

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

      // Transition Conditions:
      StateTransitionCondition leftHeelUnloaded = new HeelOffGroundCondition(RobotSide.LEFT);
      StateTransitionCondition leftToeUnloaded = new ToeOffGroundCondition(RobotSide.LEFT);
      StateTransitionCondition leftHeelTouchedDown = new HeelOnGroundCondition(RobotSide.LEFT);

      StateTransitionCondition rightHeelUnloaded = new HeelOffGroundCondition(RobotSide.RIGHT);
      StateTransitionCondition rightToeUnloaded = new ToeOffGroundCondition(RobotSide.RIGHT);
      StateTransitionCondition rightHeelTouchedDown = new HeelOnGroundCondition(RobotSide.RIGHT);

      // Left State Transitions:
      StateTransition<States> leftSupportToToeOff = new StateTransition<States>(States.TOE_OFF, leftHeelUnloaded);
//      leftSupportToToeOff.addTimePassedCondition(minSupportTime);
      leftSupportState.addStateTransition(leftSupportToToeOff);

      StateTransition<States> leftToeOffToSwing = new StateTransition<States>(States.SWING, leftToeUnloaded);
      leftToeOffState.addStateTransition(leftToeOffToSwing);
      
      StateTransition<States> leftSwingToSupport = new StateTransition<States>(States.SUPPORT, leftHeelTouchedDown);
      leftSwingState.addStateTransition(leftSwingToSupport);

      // Right State Transitions:
      StateTransition<States> rightSupportToToeOff = new StateTransition<States>(States.TOE_OFF, rightHeelUnloaded);
      rightSupportToToeOff.addTimePassedCondition(minSupportTime);
      rightSupportState.addStateTransition(rightSupportToToeOff);

      StateTransition<States> rightToeOffToSwing = new StateTransition<States>(States.SWING, rightToeUnloaded);
      rightToeOffState.addStateTransition(rightToeOffToSwing);

      StateTransition<States> rightSwingToSupport = new StateTransition<States>(States.SUPPORT, rightHeelTouchedDown);
      rightSwingState.addStateTransition(rightSwingToSupport);
      
      // Assemble the Left State Machine:
      leftStateMachine.addState(leftSupportState);
      leftStateMachine.addState(leftToeOffState);
      leftStateMachine.addState(leftSwingState);

      // Assemble the Right State Machine:
      rightStateMachine.addState(rightSupportState);
      rightStateMachine.addState(rightToeOffState);
      rightStateMachine.addState(rightSwingState);

      // Set the Initial States:
      leftStateMachine.setCurrentState(States.SUPPORT);
      rightStateMachine.setCurrentState(States.SWING);
   }

   private void walkingStateMachine()
   {
      for (RobotSide robotSide : RobotSide.values())
      {  
         // Do action and check transition conditions
         stateMachines.get(robotSide).doAction();
         StateMachine<States> stateMachine = stateMachines.get(robotSide);
         stateMachine.checkTransitionConditions();
      }     
   }

   /**
    * Body pitch controllers
    */
   private DoubleYoVariable controlBodyPitchDoubleSupport()
   {
      robot.getBodyPitch(rotationToPack);
      double pitchFromQuaternion = RotationTools.computePitch(rotationToPack);

      robot.getBodyAngularVel(velocityToPack);
      double bodyAngularVel = velocityToPack.getY();

      hipTau.set(controllerBodyPitchDoubleSupport.compute(pitchFromQuaternion, desiredBodyPitch.getDoubleValue(), bodyAngularVel, 0.0, deltaT));
      return hipTau;
   }

   private DoubleYoVariable controlBodyPitchSingleSupport()
   {
      robot.getBodyPitch(rotationToPack);
      double pitchFromQuaternion = RotationTools.computePitch(rotationToPack);

      robot.getBodyAngularVel(velocityToPack);
      double bodyAngularVel = velocityToPack.getY();

      hipTau.set(controllerBodyPitchSingleSupport.compute(pitchFromQuaternion, desiredBodyPitch.getDoubleValue(), bodyAngularVel, 0.0, deltaT));
      return hipTau;
   }

   /**
    * State Classes
    */

   //TODO take into account CoP location while in double support
   
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
         
//         updateDesiredICP(); //TODO here?
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

//         updateDesiredICP();	//TODO really?
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
      private final YoGraphicPosition debugIniPosViz;
      private final YoFramePoint iniPosDebug;
      private final FrameVector initialVelocity = new FrameVector();
      private final FrameVector initialAcceleration = new FrameVector();
      private final FramePoint finalDesiredPositionToPack = new FramePoint();
      private final FrameVector finalDesiredVelocity = new FrameVector();
      private int nTicksSinceTrajectoryIsDone = 0;

      private final FramePoint desiredPosition = new FramePoint();
      private final FramePoint desiredPositionTrajectoryViz = new FramePoint();
      private final FrameVector pControl = new FrameVector();

      private final FrameVector currentVelocity = new FrameVector();
      private final FrameVector desiredVelocity = new FrameVector();
      private final FrameVector dControl = new FrameVector();

      private final ReferenceFrame soleFrame;
      private final GeometricJacobian swingLegJacobian;
   
      private final Wrench desiredSwingFootWrench = new Wrench();

      public SwingState(RobotSide robotSide, States stateEnum)
      {
         super(States.SWING);
         this.robotSide = robotSide;
         soleFrame = robot.getSoleFrame(robotSide);
         swingLegJacobian = robot.getLegJacobian(robotSide);
         
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         iniPosDebug = new YoFramePoint(sidePrefix + "iniPosDebug", worldFrame, registry);
         debugIniPosViz = new YoGraphicPosition("debugViz", iniPosDebug, 0.02, YoAppearance.BlackMetalMaterial());
         yoGraphicsList.add(debugIniPosViz);
      }

      public void doAction()
      {
         // ************** Hip and knee --> Foot follows parabolic trajectory
         updateSwingTrajectory(robotSide);
         swingTrajectory.compute(getTimeInCurrentState());

         if (swingTrajectory.isDone())
         {
            nTicksSinceTrajectoryIsDone++;
            desiredVelocity.setIncludingFrame(finalDesiredVelocity);
         }

         else
         {
            swingTrajectory.getVelocity(desiredVelocity);
         }

         // 1) proportional control
         swingTrajectory.getPosition(desiredPosition);
         desiredPosition.add(0.0, 0.0, nTicksSinceTrajectoryIsDone * finalDesiredVelocity.getZ() * deltaT);
//         System.out.println(desiredPosition);
         yodesiredPositionSwingFoot.setAndMatchFrame(desiredPosition);
         desiredPosition.changeFrame(soleFrame);
//         System.out.println(desiredPosition);
         pControl.setIncludingFrame(desiredPosition);
         pControl.scale(kpSwingFoot.getDoubleValue());

         // 2) derivative control   
         robot.getFootLinearVelocity(robotSide, currentVelocity); //from heel GC point
         dControl.setIncludingFrame(desiredVelocity);
         dControl.sub(currentVelocity);
         dControl.changeFrame(soleFrame);
         dControl.scale(kdSwingFoot.getDoubleValue());

         // 3) computing torques using Wrench and Jacobian
         desiredSwingFootWrench.setToZero(robot.getLegRigidBody(LinkNames.LOWER_LINK, robotSide).getBodyFixedFrame(), soleFrame);
         desiredSwingFootWrench.setLinearPart(pControl);
         desiredSwingFootWrench.addLinearPart(dControl);

         DenseMatrix64F desiredSwingTaus = swingLegJacobian.computeJointTorques(desiredSwingFootWrench);
         robot.getHipJointID(robotSide).setTau(desiredSwingTaus.get(0, 0));
         robot.getKneeJointID(robotSide).setTau(desiredSwingTaus.get(1, 0));

         // ****************  Ankle --> Keep foot parallel to ground
         ankleTau.set(controllerAnkleSwing.compute(robot.getAnklePitch(robotSide), desiredAnklePitchSwing.getDoubleValue(), robot.getAnkleVelocity(robotSide), 0.0, deltaT));
         robot.setAnkleTau(robotSide, ankleTau.getDoubleValue());
      }
      
      // Update swing trajectory
      private void updateSwingTrajectory(RobotSide robotSide)
      {
         yoFootPositions.get(robotSide.getOppositeSide()).getFrameTuple(finalDesiredPositionToPack);  //TODO oppositeSide?
         finalDesiredPositionToPack.changeFrame(worldFrame);

         double dx = defaultStepLength.getDoubleValue(); 
         dx += stepLengthFactor.getDoubleValue() * desiredWalkingVelocityFiltered.getDoubleValue() - stepLengthCorrectionFactor.getDoubleValue() * (desiredWalkingVelocityFiltered.getDoubleValue() - robot.getBodyVelX(velocityToPack));  //TODO correction with minus sign?

         if (Math.abs(dx) > maxStepLength.getDoubleValue())
            dx = Math.signum(dx) * maxStepLength.getDoubleValue(); //take the biggest step possible

         double dy = 2.0 * robot.getHipOffsetY();
         if (robotSide == RobotSide.RIGHT)
        	 dy = -dy;
         
         finalDesiredPositionToPack.add(dx, dy, 0.0);
         
         // System.out.println("initialPosition " + initialPosition +  "\ninitialVelocity" + initialVelocity +  "\ninitialAcceleration" + initialAcceleration + "\nfinalDesiredPosition" + finalDesiredPositionToPack + "\nfinalDesiredVelocity" + finalDesiredVelocity + "\n");
         swingTrajectory.initialize(initialPosition, initialVelocity, initialAcceleration, finalDesiredPositionToPack, finalDesiredVelocity);  //initialize trajectory every tick in case there is a change in behavior that requires an immediate change in the planned trajectory (eg: change direction or speed)

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
         initialPosition.setToZero(soleFrame);
         initialPosition.changeFrame(worldFrame);
         
         // Used to debug
         iniPosDebug.set(initialPosition);
         
         updateSwingTrajectory(robotSide);
      }
      
      public void doTransitionOutOfAction()
      {
      }
   }


   /**
    * State Transition Conditions
    */

   // Support To ToeOff
   public class HeelOffGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;
      private boolean swingOppositeLegFinished;

      public HeelOffGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {      
    	 
    	 if (robotSide == RobotSide.RIGHT)
    	 { 
    		swingOppositeLegFinished = leftStateMachine.isCurrentState(States.SUPPORT);
    	 }
    	 
    	 else
    	 {		 
    		 swingOppositeLegFinished = rightStateMachine.isCurrentState(States.SUPPORT);
    	 }
    	  
    	 heelOnTheFloor = robot.heelOnTheFloor(robotSide); //onTheFloor will be true if the heel is on the floor, but we want the opposite
//         boolean oppositeHeelOnTheFloor = robot.heelOnTheFloor(robotSide.getOppositeSide());
         return (!heelOnTheFloor) &&  swingOppositeLegFinished; //oppositeHeelOnTheFloor 
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
         boolean heelZ = robot.getHeelZ(robotSide) > 0.03;
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

   
   /**
    * Update ICP
    */
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
         desiredICP.add(desiredVelocityToICPFactorDistanceThreshold.getDoubleValue() / robot.getOmega0() * desiredWalkingVelocityFiltered.getDoubleValue(), 0.0);
      }

      yoDesiredICP.set(desiredICP);
      robot.getCapturePoint(capturePoint);
   }

   /////////////////////////////////////////////////////////////////////////////////////////////
   public void doControl()
   {
	// 1. Update ID robot
		  robot.updatePositionsIDrobot();
		  
	// 2. Update velocity and state machine
		  desiredWalkingVelocityFiltered.update();
		  walkingStateMachine();
	   
	// 3. Update SCS robot
	      robot.updateTorquesSCSrobot();  
	      
//     updateDesiredICP();
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
