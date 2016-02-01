package us.ihmc.moonwalking.models.OneLeggedHopper;

import java.awt.Container;

import javax.swing.BoxLayout;
import javax.swing.JFrame;

import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariableType;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class OneLeggedHopperController implements RobotController
{
  private static final double ON_GROUND_COMP_LENGTH = 0.2;
  private static final double ON_GROUND_EXT_LENGTH = ON_GROUND_COMP_LENGTH;
  private static final double IN_AIR_LENGTH = -ON_GROUND_COMP_LENGTH;

  private static final double KP_LEG_SOFT = 1000.0;
  private static final double KD_LEG_SOFT = 300.0;

  private static final double KP_LEG_HARD = 1000.0;
  private static final double KD_LEG_HARD = 300.0;

  private final YoVariableRegistry yoVariableRegistry;
  private final YoVariable tauHip;
  private final YoVariable tauLeg;

  private final YoVariable qHip_d;
  private final YoVariable qdHip_d;

  private final YoVariable qLeg_d;
  private final YoVariable qdLeg_d;

  private final YoVariable errorLeg;
  private final YoVariable errorLegD;

  private final YoVariable kpLegHard;
  private final YoVariable kdLegHard;

  private final YoVariable kpLegSoft;
  private final YoVariable kdLegSoft;

  private final YoVariable hopperAngle;
  private final YoVariable doneWithState;

  private final YoVariable qLeg_d_inAir;
  private final YoVariable qLeg_d_onGroundComp;
  private final YoVariable qLeg_d_onGroundExt;

  private final OneLeggedHopperRobot oneLeggedHopperRobot;

  private final double gravity;

  private StateMachine stateMachine;

  public OneLeggedHopperController(OneLeggedHopperRobot oneLeggedHopperRobot)
  {
    this.oneLeggedHopperRobot = oneLeggedHopperRobot;

    this.gravity = oneLeggedHopperRobot.getGravity();
//    this.pendulumLength = invertedPendulumWithFoot.getPendulumLength();
//    this.pendulumMass = invertedPendulumWithFoot.getPendulumMass();

    yoVariableRegistry = new YoVariableRegistry("controller");
    tauHip = new YoVariable("tauHip", yoVariableRegistry);
    qHip_d = new YoVariable("qHip_d", yoVariableRegistry);
    qdHip_d = new YoVariable("qdHip_d", yoVariableRegistry);

    tauLeg = new YoVariable("tauLeg", yoVariableRegistry);
    qLeg_d = new YoVariable("qLeg_d", yoVariableRegistry);
    qdLeg_d = new YoVariable("qdLeg_d", yoVariableRegistry);

    qLeg_d_inAir = new YoVariable("qLeg_d_inAir", yoVariableRegistry);
    qLeg_d_onGroundComp = new YoVariable("qLeg_d_onGroundComp", yoVariableRegistry);
    qLeg_d_onGroundExt = new YoVariable("qLeg_d_onGroundExt", yoVariableRegistry);

    errorLeg = new YoVariable("errorLeg", yoVariableRegistry);
    errorLegD = new YoVariable("errorLegD", yoVariableRegistry);


    qLeg_d_inAir.val = IN_AIR_LENGTH;
    qLeg_d_onGroundComp.val = ON_GROUND_COMP_LENGTH;
    qLeg_d_onGroundExt.val = ON_GROUND_EXT_LENGTH;

    kpLegHard = new YoVariable("kpLegHard", yoVariableRegistry);
    kdLegHard = new YoVariable("kdLegHard", yoVariableRegistry);

    kpLegSoft = new YoVariable("kpLegSoft", yoVariableRegistry);
    kdLegSoft = new YoVariable("kdLegSoft", yoVariableRegistry);

    kpLegHard.val = KP_LEG_HARD;
    kdLegHard.val = KD_LEG_HARD;

    kpLegSoft.val = KP_LEG_SOFT;
    kdLegSoft.val = KD_LEG_SOFT;

    hopperAngle = new YoVariable("hopperAngle", yoVariableRegistry);

    doneWithState = new YoVariable("doneWithState", YoVariableType.BOOLEAN, yoVariableRegistry);

    setupStateMachine(yoVariableRegistry);

    stateMachine.setCurrentState(HoppingState.INITIAL);

    createStateMachineWindow();
  }

  public void doControl()
  {


    hopperAngle.val = oneLeggedHopperRobot.getBodyPitch() + oneLeggedHopperRobot.getHipAngle();

    stateMachine.doAction();

//
//    double gravityCompensationTorque;
//    if (oneLeggedHopperRobot.isFootOnGround())
//      gravityCompensationTorque = getGravityCompensationTorque(hopperAngle.val);
//    else
//      gravityCompensationTorque = 0.0;

//    tauLeg.val = servoTorque;

    oneLeggedHopperRobot.setLegTorque(tauLeg.val);

    stateMachine.checkTransitionConditions();
  }

  private void setupStateMachine(YoVariableRegistry yoVariableRegistry)
  {
    InitialState initialState = new InitialState(HoppingState.INITIAL);
    OnGroundCompressionState onGroundCompressionState = new OnGroundCompressionState(HoppingState.ON_GROUND_COMP);
    OnGroundExtensionState onGroundExtensionState = new OnGroundExtensionState(HoppingState.ON_GROUND_EXTEND);
    InAirState inAirState = new InAirState(HoppingState.IN_AIR);

    StateTransitionCondition doneWithStateTransitionCondition = new StateTransitionCondition()
    {
      public boolean checkCondition()
      {
        return doneWithState.getBooleanValue();
      }
    };


    StateTransition toInAir = new StateTransition(inAirState.getStateEnum(), doneWithStateTransitionCondition);
    StateTransition toOnGroundCompression = new StateTransition(onGroundCompressionState.getStateEnum(), doneWithStateTransitionCondition);
    StateTransition toOnGroundExtension = new StateTransition(onGroundExtensionState.getStateEnum(), doneWithStateTransitionCondition);

    initialState.addStateTransition(toInAir);
    onGroundCompressionState.addStateTransition(toOnGroundExtension);
    onGroundExtensionState.addStateTransition(toInAir);
    inAirState.addStateTransition(toOnGroundCompression);

    stateMachine = new StateMachine("state", "switch_time",
                                      HoppingState.values(), oneLeggedHopperRobot.getTimeVariable(), yoVariableRegistry);

     stateMachine.addState(initialState);
     stateMachine.addState(inAirState);
     stateMachine.addState(onGroundCompressionState);
     stateMachine.addState(onGroundExtensionState);

  }

  public YoVariableRegistry getYoVariableRegistry()
  {
    return yoVariableRegistry;
  }

  private double getGravityCompensationTorque(double hopperAngle)
  {
    return -(OneLeggedHopperRobot.hopperMass + OneLeggedHopperRobot.legMass/2.0)* gravity * Math.cos(hopperAngle);
  }

  private double servoLegLength(double desiredLegLength, double desiredLegnLengthDot, double kp, double kd)
  {
    errorLeg.val = desiredLegLength - oneLeggedHopperRobot.getLegLength();

    errorLegD.val = desiredLegnLengthDot - oneLeggedHopperRobot.getLegLengthDot();

    double tau =  kp * (errorLeg.val) + kd * (errorLegD.val);
    return tau;
  }

  public void createStateMachineWindow()
  {
    JFrame jFrame = new JFrame("Hopper State Machines");

    Container contentPane = jFrame.getContentPane();

    contentPane.setLayout(new BoxLayout(contentPane, BoxLayout.X_AXIS));
    StateMachineJPanel stateMachinePanel = new StateMachineJPanel(stateMachine);


    jFrame.getContentPane().add(stateMachinePanel);

    jFrame.pack();
    jFrame.setSize(450, 300);
    jFrame.setAlwaysOnTop(true);
    jFrame.setVisible(true);

    // Doing the following will cause redraw when the state changes, but not during replay or rewind:
    stateMachine.attachStateChangedListener(stateMachinePanel);

    // Doing this will cause redraw every specified miliseconds:
    stateMachinePanel.createUpdaterThread(250);
  }



  //Begin: Set up states
 private class InitialState extends State
 {
    public InitialState(Enum stateName)
    {
       super(stateName);
    }

    public void doAction()
    {
      doneWithState.set(true);
    }

    public void doTransitionIntoAction()
    {

    }

    public void doTransitionOutOfAction()
    {
    }
 }

 private class OnGroundCompressionState extends State
 {
   public OnGroundCompressionState(Enum stateName)
   {
     super(stateName);
   }

   public void doAction()
   {
     qLeg_d.val = qLeg_d_onGroundComp.val;
     qdLeg_d.val = 0.0;

     tauLeg.val = servoLegLength(qLeg_d.val, qdLeg_d.val, kpLegSoft.val, kdLegSoft.val);

     if (oneLeggedHopperRobot.getLegLengthDot() > 0.0)
       doneWithState.set(true);
   }

   public void doTransitionIntoAction()
   {
     doneWithState.set(false);
   }

   public void doTransitionOutOfAction()
   {
   }
 }

 private class OnGroundExtensionState extends State
 {
   public OnGroundExtensionState(Enum stateName)
   {
     super(stateName);
   }

   public void doAction()
   {
     qLeg_d.val = qLeg_d_onGroundExt.val;
     qdLeg_d.val = 0.0;

     tauLeg.val = servoLegLength(qLeg_d.val, qdLeg_d.val, kpLegHard.val, kdLegHard.val);

     if (!oneLeggedHopperRobot.isFootOnGround())
       doneWithState.set(true);
   }

   public void doTransitionIntoAction()
   {
     doneWithState.set(false);
   }

   public void doTransitionOutOfAction()
   {
   }
 }



 private class InAirState extends State
 {
   public InAirState(Enum stateName)
   {
     super(stateName);
   }

   public void doAction()
   {
     qLeg_d.val = qLeg_d_inAir.val;
     qdLeg_d.val = 0.0;

     if (oneLeggedHopperRobot.isFootOnGround())
       doneWithState.set(true);
   }

   public void doTransitionIntoAction()
   {
     doneWithState.set(false);
   }

   public void doTransitionOutOfAction()
   {
   }
 }



  private enum HoppingState
  {
    INITIAL, IN_AIR, ON_GROUND_COMP, ON_GROUND_EXTEND
  }
}
