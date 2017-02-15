package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

public class Step6WalkingController implements RobotController
{

   /**
    * Initialization
    */

   //Variables
   private Step6IDandSCSRobot_pinKnee rob;
   private String name;
   private YoVariableRegistry controllerRegistry = new YoVariableRegistry("controllerRegistry");
   private double deltaT;

   private PIDController controllerBodyZ;
   private PIDController controllerBodyPitch, controllerHipPitch, controllerBodyPitchSingleSupport;
   private PIDController controllerKneePitchSwing, controllerKneePitchStraighten;
   private PIDController controllerAnkleStraighten, controllerAnkleToeOff, controllerAnkleSwing;

   private DoubleYoVariable desiredBodyZ, desiredKneePitchSwing, desiredKneePitchStraighten;
   private DoubleYoVariable desiredHipPitch, desiredBodyPitch;
   private DoubleYoVariable desiredAnklePitchSwing , desiredAnklePitchToeOff, desiredAnklePitchStraighten;

   private DoubleYoVariable ankleTauToeOff;
   private DoubleYoVariable kneeTau;
   private DoubleYoVariable hipTau;
   private DoubleYoVariable ankleTau;

   private Quat4d rotationToPack = new Quat4d();
   private Vector3d velocityToPack = new Vector3d();

   private boolean heelOnTheFloor, toeOnTheFloor;
   private final DoubleYoVariable minSupportTime = new DoubleYoVariable("minSupportTime", controllerRegistry);
   private final DoubleYoVariable swingTime = new DoubleYoVariable("swingTime", controllerRegistry);
    
   // State Machine
   private enum States
   {
      SUPPORT, TOE_OFF, SWING, STRAIGHTEN
   }

   private final StateMachine<States> leftStateMachine, rightStateMachine;
   private final SideDependentList<StateMachine<States>> stateMachines;

   //ICP calculation and graphics
   private double comPosX, comPosZ, comVelX, icpPosX;
   private YoGraphicPosition icpGraphics;
   private DoubleYoVariable icpYoPosX, icpYoPosY, icpYoPosZ;
   private FramePoint2d capturePoint, centerOfMassInWorld; 
   private FrameVector2d centerOfMassVelocityInWorld;
   
   
   /**
    * Constructor
    */

   public Step6WalkingController(Step6IDandSCSRobot_pinKnee rob, String name, double deltaT, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.rob = rob;
      this.name = name;
      this.deltaT = deltaT;

      
      // Controllers and Gains
      controllerBodyZ = new PIDController("bodyZ", controllerRegistry);
      controllerBodyZ.setProportionalGain(1000.0);
      controllerBodyZ.setDerivativeGain(500.0); 
      desiredBodyZ = new DoubleYoVariable("desiredBodyZ", controllerRegistry);
      desiredBodyZ.set(1.5); 

      controllerBodyPitch = new PIDController("bodyPitch", controllerRegistry);
      controllerBodyPitch.setProportionalGain(500.0);
      controllerBodyPitch.setDerivativeGain(50.0);
      desiredBodyPitch = new DoubleYoVariable("desiredBodyPitch", controllerRegistry);
      desiredBodyPitch.set(0.0);
      
      controllerBodyPitchSingleSupport = new PIDController("bodyPitchSingleSupport", controllerRegistry);
      controllerBodyPitchSingleSupport.setProportionalGain(5000.0);
      controllerBodyPitchSingleSupport.setDerivativeGain(500.0);
      
      controllerHipPitch = new PIDController("hipPitch", controllerRegistry);
      controllerHipPitch.setProportionalGain(700.0);
      controllerHipPitch.setDerivativeGain(70.0);
      desiredHipPitch = new DoubleYoVariable("desiredHipPitch", controllerRegistry);
      desiredHipPitch.set(-0.6);
      
      controllerKneePitchSwing = new PIDController("kneePitchSwing", controllerRegistry); 
      controllerKneePitchSwing.setProportionalGain(25000.0);
      controllerKneePitchSwing.setDerivativeGain(2000.0);
      desiredKneePitchSwing = new DoubleYoVariable("desiredKneePitchSwing", controllerRegistry);
      desiredKneePitchSwing.set(1.3); 
      
      controllerKneePitchStraighten = new PIDController("kneePitchStraighten", controllerRegistry);
      controllerKneePitchStraighten.setProportionalGain(10000.0);
      controllerKneePitchStraighten.setDerivativeGain(1000.0);
      desiredKneePitchStraighten = new DoubleYoVariable("desiredKneePitchStraighten", controllerRegistry);
      desiredKneePitchStraighten.set(0.0);  
      
      controllerAnkleStraighten = new PIDController("ankleStraighten", controllerRegistry);
      controllerAnkleStraighten.setProportionalGain(50.0);
      controllerAnkleStraighten.setDerivativeGain(5.0); 
      desiredAnklePitchStraighten = new DoubleYoVariable("desiredAnklePitchStraighten", controllerRegistry);
      desiredAnklePitchStraighten.set(0.2);
         
      controllerAnkleToeOff = new PIDController("ankleToeOff", controllerRegistry); 
      controllerAnkleToeOff.setProportionalGain(500.0);
      controllerAnkleToeOff.setDerivativeGain(50.0);
      desiredAnklePitchToeOff = new DoubleYoVariable("desiredAnklePitchToeOff", controllerRegistry);
      desiredAnklePitchToeOff.set(0.08);  
      
      controllerAnkleSwing = new PIDController("ankleSwing", controllerRegistry);
      controllerAnkleSwing.setProportionalGain(100.0);
      controllerAnkleSwing.setDerivativeGain(10.0);
      desiredAnklePitchSwing = new DoubleYoVariable("desiredAnklePitchSwing", controllerRegistry);
      desiredAnklePitchSwing.set(-0.65);
      
      hipTau = new DoubleYoVariable("hipTau", controllerRegistry);
      kneeTau = new DoubleYoVariable("kneeTau", controllerRegistry);
      ankleTau = new DoubleYoVariable("ankleTau", controllerRegistry);
      
      ankleTauToeOff = new DoubleYoVariable("ankleTauToeOff", controllerRegistry);
      ankleTauToeOff.set(200.0);
      
      // Create the state machines:
      leftStateMachine = new StateMachine<States>("leftState", "leftSwitchTime", States.class, rob.getYoTime(), controllerRegistry);
      rightStateMachine = new StateMachine<States>("rightState", "rightSwitchTime", States.class, rob.getYoTime(), controllerRegistry);
      stateMachines = new SideDependentList<StateMachine<States>>(leftStateMachine, rightStateMachine);
     
      initConditions();
      setupStateMachines();
      
      //ICP calculation and graphics
      icpYoPosX = new DoubleYoVariable("icpYoPosX", controllerRegistry);
      icpYoPosX.set(icpPosX);
      icpYoPosY = new DoubleYoVariable("icpYoPosY", controllerRegistry);
      icpYoPosY.set(0.0); 
      icpYoPosZ = new DoubleYoVariable("icpYoPosZ", controllerRegistry);
      icpYoPosZ.set(0.0);
      icpGraphics = new YoGraphicPosition("icpGraph", icpYoPosX, icpYoPosY, icpYoPosZ, 0.1, YoAppearance.Tomato());
      yoGraphicsListRegistry.registerYoGraphic("icp", icpGraphics);
      
   }

   /**
    * State Machine Related Methods
    */

   public void initConditions()
   {
      minSupportTime.set(0.3); 
      swingTime.set(0.4); 
      rob.setInitialVelocity();
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
      State<States> leftStraightenState = new StraightenState(RobotSide.LEFT, States.STRAIGHTEN);
      State<States> rightStraightenState = new StraightenState(RobotSide.RIGHT, States.STRAIGHTEN);

      // Transition Conditions:
      StateTransitionCondition leftHeelUnloaded = new HeelOffGroundCondition(RobotSide.LEFT);
      StateTransitionCondition leftToeUnloaded = new ToeOffGroundCondition(RobotSide.LEFT);
      StateTransitionCondition leftToeTouchedDown = new HeelOnGroundCondition(RobotSide.LEFT);

      StateTransitionCondition rightHeelUnloaded = new HeelOffGroundCondition(RobotSide.RIGHT);
      StateTransitionCondition rightToeUnloaded = new ToeOffGroundCondition(RobotSide.RIGHT);
      StateTransitionCondition rightHealTouchedDown = new HeelOnGroundCondition(RobotSide.RIGHT);

      // Left State Transitions:
      StateTransition<States> leftSupportToToeOff = new StateTransition<States>(States.TOE_OFF, leftHeelUnloaded);
      leftSupportToToeOff.addTimePassedCondition(minSupportTime);
      leftSupportState.addStateTransition(leftSupportToToeOff);

      StateTransition<States> leftToeOffToSwing = new StateTransition<States>(States.SWING, leftToeUnloaded);
      leftToeOffState.addStateTransition(leftToeOffToSwing);

      StateTransition<States> leftSwingToStraighten = new StateTransition<States>(States.STRAIGHTEN, swingTime);
      leftSwingState.addStateTransition(leftSwingToStraighten);

      StateTransition<States> leftStraightenToSupport = new StateTransition<States>(States.SUPPORT, leftToeTouchedDown);
      leftStraightenState.addStateTransition(leftStraightenToSupport);

      // Right State Transitions:
      StateTransition<States> rightSupportToToeOff = new StateTransition<States>(States.TOE_OFF, rightHeelUnloaded);
      rightSupportToToeOff.addTimePassedCondition(minSupportTime);
      rightSupportState.addStateTransition(rightSupportToToeOff);

      StateTransition<States> rightToeOffToSwing = new StateTransition<States>(States.SWING, rightToeUnloaded);
      rightToeOffState.addStateTransition(rightToeOffToSwing);

      StateTransition<States> rightSwingToStraighten = new StateTransition<States>(States.STRAIGHTEN, swingTime);
      rightSwingState.addStateTransition(rightSwingToStraighten);

      StateTransition<States> rightStraightenToSupport = new StateTransition<States>(States.SUPPORT, rightHealTouchedDown);
      rightStraightenState.addStateTransition(rightStraightenToSupport);

      // Assemble the Left State Machine:
      leftStateMachine.addState(leftSupportState);
      leftStateMachine.addState(leftToeOffState);
      leftStateMachine.addState(leftSwingState);
      leftStateMachine.addState(leftStraightenState);

      // Assemble the Right State Machine:
      rightStateMachine.addState(rightSupportState);
      rightStateMachine.addState(rightToeOffState);
      rightStateMachine.addState(rightSwingState);
      rightStateMachine.addState(rightStraightenState);

      // Set the Initial States:
      leftStateMachine.setCurrentState(States.STRAIGHTEN);
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
         rob.updatePositionsIDrobot();
         rob.updateTorquesSCSrobot(); //Note: this is getting called twice, one per side
      }
   }

   private DoubleYoVariable controlBodyPitch()
   {
      rob.getBodyPitch(rotationToPack);
      double pitchFromQuaternion = RotationTools.computePitch(rotationToPack);

      rob.getBodyAngularVel(velocityToPack);
      double bodyAngularVel = velocityToPack.getY();

      hipTau.set(controllerBodyPitch.compute(pitchFromQuaternion, desiredBodyPitch.getDoubleValue(), bodyAngularVel, 0.0, deltaT));
      return hipTau;
   }
   
   private DoubleYoVariable controlBodyPitchSingleSupport()
   {
      rob.getBodyPitch(rotationToPack);
      double pitchFromQuaternion = RotationTools.computePitch(rotationToPack);
      
      rob.getBodyAngularVel(velocityToPack);
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
         rob.setHipTau(robotSide, -hipTau.getDoubleValue());

         // Knee --> Body height
         kneeTau.set(controllerBodyZ.compute(rob.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -rob.getKneeVelocity(robotSide), 0.0, deltaT));
         rob.setKneeTau(robotSide, -kneeTau.getDoubleValue()); 

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
         hipTau = controlBodyPitch();
         rob.setHipTau(robotSide, -hipTau.getDoubleValue());

         // Knee --> Body height
         kneeTau.set(controllerBodyZ.compute(rob.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -rob.getKneeVelocity(robotSide), 0.0, deltaT));
         rob.setKneeTau(robotSide, -kneeTau.getDoubleValue() ); //+ ffComponent); //TODO feed forward?
         
         // Ankle --> Ankle pitch
         if (rob.getBodyPositionX() > rob.getToeX(robotSide))
         {
         ankleTau.set(controllerAnkleToeOff.compute(rob.getAnklePitch(robotSide), desiredAnklePitchToeOff.getDoubleValue(), rob.getBodyVelX(velocityToPack), 0.7, deltaT));
         rob.setAnkleTau(robotSide, ankleTau.getDoubleValue());
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
      //      private final DoubleYoVariable startTime;
      
      public SwingState(RobotSide robotSide, States stateEnum)
      {
         super(States.SWING);
         this.robotSide = robotSide;    
         //         startTime = new DoubleYoVariable("startTime", controllerRegistry); //TODO timer =)
      }

      public void doAction()
      {       
         // Hip --> Hip pitch
         hipTau.set(controllerHipPitch.compute(rob.getHipPitch(robotSide), desiredHipPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
         rob.setHipTau(robotSide, hipTau.getDoubleValue()); 

         // Knee --> Leg contracted
         kneeTau.set(controllerKneePitchSwing.compute(rob.getKneePosition(robotSide), desiredKneePitchSwing.getDoubleValue(), rob.getKneeVelocity(robotSide), 0.0, deltaT));
         rob.setKneeTau(robotSide, kneeTau.getDoubleValue());
         
         // Ankle --> Keep foot parallel to ground
         ankleTau.set(controllerAnkleSwing.compute(rob.getAnklePitch(robotSide), desiredAnklePitchSwing.getDoubleValue(), rob.getAnkleVelocity(robotSide), 0.0, deltaT));
         rob.setAnkleTau(robotSide, ankleTau.getDoubleValue());
      }

      public void doTransitionIntoAction()
      {
         //         startTime.set(rob.getYoTime().getDoubleValue());
      }

      public void doTransitionOutOfAction()
      {
      }
   }

   ///////////////////////////////////  (4)  Straighten
   private class StraightenState extends State<States>
   {
      private final RobotSide robotSide;

      public StraightenState(RobotSide robotSide, States stateEnum)
      {
         super(States.STRAIGHTEN);
         this.robotSide = robotSide;
      }

      public void doAction()
      {
         //Hip --> Hip pitch      
         hipTau.set(controllerHipPitch.compute(rob.getHipPitch(robotSide), desiredHipPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
         rob.setHipTau(robotSide, hipTau.getDoubleValue()); 

         //Knee --> Leg straight
         kneeTau.set(controllerKneePitchStraighten.compute(rob.getKneePosition(robotSide), desiredKneePitchStraighten.getDoubleValue(), rob.getKneeVelocity(robotSide), 0.0, deltaT));
         rob.setKneeTau(robotSide, kneeTau.getDoubleValue());

         //Ankle --> Ankle pitch
         ankleTau.set(controllerAnkleStraighten.compute(rob.getAnklePitch(robotSide), desiredAnklePitchStraighten.getDoubleValue(), rob.getAnkleVelocity(robotSide), 0.0, deltaT));
         rob.setAnkleTau(robotSide, ankleTau.getDoubleValue());
      }

      public void doTransitionIntoAction()
      {
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

      public HeelOffGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         heelOnTheFloor = rob.heelOnTheFloor(robotSide); //onTheFloor will be true if the heel is on the floor, but we want the opposite
         boolean oppositeHeelOnTheFloor = rob.heelOnTheFloor(robotSide.getOppositeSide());
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
         toeOnTheFloor = rob.toeOnTheFloor(robotSide);
         boolean oppositeHeelOnTheFloor = rob.heelOnTheFloor(robotSide.getOppositeSide());
         boolean heelZ = rob.getHeelZ(robotSide) > 0.31;  
         return !toeOnTheFloor && oppositeHeelOnTheFloor && heelZ;
      }
   }
   
   // Swing To Straighten (TIMER)  -- not actually needed, but timers seem like a useful thing
   //   public class SwingToStraightenCondition implements StateTransitionCondition
   //   {
   //      public SwingToStraightenCondition(DoubleYoVariable startTime)
   //      {
   //      }
   //      
   //      public boolean checkCondition()
   //      {
   //         stopSwing = (rob.getYoTime().getDoubleValue() - startTime.getDoubleValue()) > 0.15;
   //         return stopSwing;
   //      }
   //   }

   // Straighten To Support
   public class HeelOnGroundCondition implements StateTransitionCondition
   {
      private final RobotSide robotSide;

      public HeelOnGroundCondition(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      public boolean checkCondition()
      {
         heelOnTheFloor = rob.heelOnTheFloor(robotSide);
         return heelOnTheFloor;
      }
   }

   /**
    * Foot placement --> ICP
    */
 private void calculateAndPlotICP()
 {
    // Calculation
    comVelX = rob.getBodyVelX(velocityToPack);
    comPosX = rob.getBodyPositionX();
    comPosZ = rob.getBodyPositionZ();
    double omega0 = Math.sqrt(9.81 / comPosZ);
    
    capturePoint = new FramePoint2d();
    centerOfMassInWorld = new FramePoint2d(ReferenceFrame.getWorldFrame(), comPosX, 0.0);
    centerOfMassVelocityInWorld = new FrameVector2d(ReferenceFrame.getWorldFrame(), comVelX, 0.0);
    CapturePointCalculator.computeCapturePoint(capturePoint, centerOfMassInWorld, centerOfMassVelocityInWorld, omega0);
    icpPosX = capturePoint.getX();
    //    System.out.println("comPos" + centerOfMassInWorld + "\ncomVel" + centerOfMassVelocityInWorld + "\nicp" + capturePoint + "\nicpPosX" + icpPosX ); 

    // Visualization
    icpGraphics.setPosition(icpPosX, 0.0, 0.0);
   
 }
   
   /////////////////////////////////////////////////////////////////////////////////////////////
   public void doControl()
   {
      walkingStateMachine();
      calculateAndPlotICP();
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return controllerRegistry;
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
