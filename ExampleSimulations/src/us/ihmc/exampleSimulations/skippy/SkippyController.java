
package us.ihmc.exampleSimulations.skippy;

import java.awt.Container;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.exampleSimulations.skippy.SkippyRobot.RobotType;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepOverheadPath;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateMachinesJPanel;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;

public class SkippyController implements RobotController
{

   /**
    *
    * Outline of SkippyToDo: JUMP_FORWARD: If Skippy model is selected, robot
    * JUMP_SIDEWAYS: If Skippy model is selected, robot will jump/balance in x
    * direction (torso's rotation axis) BALANCE: If Skippy/Tippy model is
    * selected, robot will balance POSITION: If Tippy model is selected, robot
    * will balance with the help of LEG joint (not tested)
    *
    * Note: First three SkippyStatuses will allow model to balance according
    * to: q_d_hip: desired angle of TORSO q_d_shoulder: desired angle of
    * SHOULDER
    *
    */

   private enum SkippyToDo
   {
      JUMP_FORWARD, // change initialBodySidewaysLean in SkippyRobot.java to
      // 0.0
      BALANCE,
      POSITION
   }

   private enum States
   {
      BALANCE, PREPARE, LEAN, LIFTOFF, REPOSITION, RECOVER
   }

   private enum SkippyPlaneControlMode
   {
      BALANCE, POSITION
   }

   private StateMachine<States> stateMachine;

   private final YoVariableRegistry registry = new YoVariableRegistry("SkippyController");

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
   // private DoubleYoVariable q_foot_X, q_hip, qHipIncludingOffset, qd_foot_X,
   // qd_hip, qd_shoulder;
   private final DoubleYoVariable k1, k2, k3, k4, k5, k6, k7, k8, angleToCoMInYZPlane, angleToCoMInXZPlane, angularVelocityToCoMYZPlane,
         angularVelocityToCoMXZPlane; // controller
   // gain
   // parameters
   private final DoubleYoVariable planarDistanceYZPlane, planarDistanceXZPlane;

   private YoVariableRegistry controllerRegistry = new YoVariableRegistry("controllerRegistry");
   private PIDController hipAngleController, comToFoorErrorController, comVelocityController, icpToFootErrorController;

   private final DoubleYoVariable alphaAngularVelocity;
   private final FilteredVelocityYoVariable angularVelocityToCoMYZPlane2, angularVelocityToCoMXZPlane2;

   private final YoFramePoint bodyLocation = new YoFramePoint("body", ReferenceFrame.getWorldFrame(), registry);

   private final ExternalForcePoint forceToCOM;
   private final YoFramePoint com = new YoFramePoint("com", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector comVelocity = new YoFrameVector("comVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector comAcceleration = new YoFrameVector("comAcceleration", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredReactionForce = new YoFrameVector("desiredReactionForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector actualReactionForce = new YoFrameVector("actualReactionForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector angularMomentum = new YoFrameVector("angularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector lastAngularMomentum = new YoFrameVector("lastAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector linearMomentum = new YoFrameVector("linearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector lastLinearMomentum = new YoFrameVector("lastLinearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector rateOfChangeOfLinearMomentum = new YoFrameVector("rateOfChangeOfLinearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector rateOfChangeOfAngularMomentum = new YoFrameVector("rateOfChangeOfAngularMomentum", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector icpToFootError = new YoFrameVector("actualIcpToFootError", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector comToFootError = new YoFrameVector("comToFootError", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector actualIcpToFootErrorVelocity = new YoFrameVector("actualIcpToFootErrorVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable fixedW0 = new DoubleYoVariable("fixedW0", registry);
   private final DoubleYoVariable averagedW0 = new DoubleYoVariable("averagedW0", registry);

   private final YoFrameVector tauShoulderJoint = new YoFrameVector("tauShoulderJoint", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable tauOnShoulderJointAxis = new DoubleYoVariable("tauOnShoulderJointAxis", registry);

   private final YoFrameVector tauHipJoint = new YoFrameVector("tauHipJoint", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable tauOnHipJointAxis = new DoubleYoVariable("tauOnHipJointAxis", registry);

   private final YoFramePoint hipJointPosition = new YoFramePoint("hipJointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint previousHipJointPosition = new YoFramePoint("previousHipJointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipJointUnitVector = new YoFrameVector("hipJointUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipToFootPositionVector = new YoFrameVector("hipToFootPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipToFootUnitVector = new YoFrameVector("hipToFootUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint shoulderJointPosition = new YoFramePoint("shoulderJointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint previousShoulderJointPosition = new YoFramePoint("previousShoulderJointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector shoulderJointUnitVector = new YoFrameVector("shoulderJointUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector shoulderToFootPositionVector = new YoFrameVector("shoulderToFootPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector shoulderToFootUnitVector = new YoFrameVector("shoulderToFootUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector cmpToComPositionVector = new YoFrameVector("cmpToComPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector cmpToComUnitVector = new YoFrameVector("cmpToComUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector footToComPositionVector = new YoFrameVector("footToComPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector footToCoMInBodyFrame;
   private final YoFramePoint icp = new YoFramePoint("actualICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint icpVelocity = new YoFramePoint("icpVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint actualCMPFromDefinition = new YoFramePoint("actualCMPFromDefinition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredCmpFromIcp = new YoFramePoint("desiredCMPFromICP", ReferenceFrame.getWorldFrame(), registry);
   //   private PIDController controllerCmpX;
   //   private PIDController controllerCmpY;
   private final YoFramePoint footLocation = new YoFramePoint("footLocation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint previousFootLocation = new YoFramePoint("previousFootLocation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint achievedCMP = new YoFramePoint("achievedCMP", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable robotMass = new DoubleYoVariable("robotMass", registry);
   private final DoubleYoVariable robotWeight = new DoubleYoVariable("robotWeight", registry);
   private final DoubleYoVariable reactionModulus = new DoubleYoVariable("reactionModulus", registry);
   private final DoubleYoVariable qHipIncludingOffset = new DoubleYoVariable("qHipIncludingOffset", registry);
   private final DoubleYoVariable qDHipIncludingOffset = new DoubleYoVariable("qDHipIncludingOffset", registry);
   private final DoubleYoVariable qDShoulderIncludingOffset = new DoubleYoVariable("qDShoulderIncludingOffset", registry);
   private final DoubleYoVariable qd_hip = new DoubleYoVariable("qd_hip", registry);
   private final DoubleYoVariable qShoulderIncludingOffset = new DoubleYoVariable("qShoulderIncludingOffset", registry);
   private final DoubleYoVariable qd_shoulder = new DoubleYoVariable("qd_shoulder", registry);
   private final DoubleYoVariable tauHipForAngleControl = new DoubleYoVariable("tauHipForAngleControl", registry);
   private final DoubleYoVariable tauHipForComVelocityControl = new DoubleYoVariable("tauHipForComVelocityControl", registry);
   private final DoubleYoVariable tauHipForIcpToFootErrorControl = new DoubleYoVariable("tauHipForIcpToFootErrorControl", registry);
   private final DoubleYoVariable tauHipForComToFootErrorControl = new DoubleYoVariable("tauHipForComToFootErrorControl", registry);
   private final DoubleYoVariable legToTorsoAngle = new DoubleYoVariable("legToTorsoAngle", registry);

   private final FramePoint tempFootLocation = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePoint tempCoMLocation = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FrameVector tempFootToCoM = new FrameVector(ReferenceFrame.getWorldFrame());

   private final DoubleYoVariable z0 = new DoubleYoVariable("z0", registry);
   private final DoubleYoVariable averageZ0 = new DoubleYoVariable("averageZ0", registry);
   private final DoubleYoVariable kCapture = new DoubleYoVariable("kCapture", registry);
   private final DoubleYoVariable desiredLegToTorsoAngle = new DoubleYoVariable("desiredLegToTorsoAngle", registry);

   private final EnumYoVariable<SkippyToDo> skippyToDo = new EnumYoVariable<SkippyToDo>("SkippyToDo", registry, SkippyToDo.class);
   private final EnumYoVariable<SkippyPlaneControlMode> hipPlaneControlMode = new EnumYoVariable<SkippyPlaneControlMode>("hipPlaneControlMode", registry,
                                                                                                                         SkippyPlaneControlMode.class);
   private final EnumYoVariable<SkippyPlaneControlMode> shoulderPlaneControlMode = new EnumYoVariable<SkippyPlaneControlMode>("shoulderPlaneControlMode",
                                                                                                                              registry,
                                                                                                                              SkippyPlaneControlMode.class);
   private final double deltaT = (double) SkippySimulation.DT;
   /*
    * For debug purposes
    */
   private final YoFrameVector crossHipPositionVector = new YoFrameVector("dotHipPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector crossShoulderPositionVector = new YoFrameVector("dotShoulderPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector previousShoulderToFootUnitVector = new YoFrameVector("previousShoulderToFootUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector previousHipToFootUnitVector = new YoFrameVector("previousHipToFootUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable crossShoulderLength = new DoubleYoVariable("crossShoulderLength", registry);
   private final DoubleYoVariable crossHipLength = new DoubleYoVariable("crossHipLength", registry);
   
   boolean firstStick = true;

   private String name;
   private SkippyRobot robot;
   private RobotType robotType;

   private double legIntegralTermX = 0.0;
   private double legIntegralTermY = 0.0;
   private double hipIntegralTerm = 0.0;
   private double shoulderIntegralTerm = 0.0;

   double angularMomentumIntegralError = 0.0;
   double lastReactionForce = 0.0;
   int counterForZ0Average = 1;
//   boolean printOnce = true;
   PrintWriter writer = null;
   PrintWriter writer1 = null;
   boolean firstEnterBalanceState = true;

   /*
    * Debug stuff
    */
   boolean setUpFiles = true;//false; //
   boolean trace = false;//true;// 
   boolean traceCrossProduct = true;   //false;// 
   double tol = 0.1; //threshold for debug distances

   public SkippyController(SkippyRobot robot, RobotType robotType, String name, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistries)
   {
      this.name = name;
      this.robot = robot;
      this.robotType = robotType;
      listeners(false);//true);

      /*
       * Hip angle torque controller
       */
      hipAngleController = new PIDController("hipAngle", controllerRegistry);
      comVelocityController = new PIDController("comVelocity", controllerRegistry);
      comToFoorErrorController = new PIDController("comToFoot", controllerRegistry);
      icpToFootErrorController = new PIDController("icpToFoot", controllerRegistry);
      
//      hipAngleController.setProportionalGain(5.0);   
//      hipAngleController.setDerivativeGain(0.0);
//      hipAngleController.setIntegralGain(0.1);
      /*
       * z0 and KCapture
       */
      z0.set(1.216); // got from averaged CoM_Z during simulation
      averageZ0.set(0.0);
      kCapture.set(1.5);// 2.0);//0.9);
      robotMass.set(robot.getMass());
      robotWeight.set(robotMass.getDoubleValue() * Math.abs(robot.getGravityZ()));
      setUpOutputFiles(setUpFiles);

      footToCoMInBodyFrame = new YoFrameVector("footToCoMInBody", robot.updateAndGetBodyFrame(), registry);
      forceToCOM = new ExternalForcePoint("FORCETOCOM", robot);

      k1 = new DoubleYoVariable("k1", registry);
      k2 = new DoubleYoVariable("k2", registry);
      k3 = new DoubleYoVariable("k3", registry);
      k4 = new DoubleYoVariable("k4", registry);
      k5 = new DoubleYoVariable("k5", registry);
      k6 = new DoubleYoVariable("k6", registry);
      k7 = new DoubleYoVariable("k7", registry);
      k8 = new DoubleYoVariable("k8", registry);

      skippyToDo.set(SkippyToDo.JUMP_FORWARD);
      hipPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);
      shoulderPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);

      if (skippyToDo.getEnumValue() == SkippyToDo.BALANCE)
      {
         //         qd_hip.set(0.6);
         //         qd_shoulder.set(0.0);
      }
      else if (skippyToDo.getEnumValue() == SkippyToDo.JUMP_FORWARD)
      {
         //         qd_hip.set(0.0);
         //         qd_shoulder.set(0.0);
      }

      planarDistanceYZPlane = new DoubleYoVariable("planarDistanceYZPlane", registry);
      planarDistanceXZPlane = new DoubleYoVariable("planarDistanceXZPlane", registry);
      angleToCoMInYZPlane = new DoubleYoVariable("angleToCoMYZPlane", registry);
      angleToCoMInXZPlane = new DoubleYoVariable("angleToCoMXZPlane", registry);
      angularVelocityToCoMYZPlane = new DoubleYoVariable("angularVelocityToCoMYZPlane", registry);
      angularVelocityToCoMXZPlane = new DoubleYoVariable("angularVelocityToCoMXZPlane", registry);

      alphaAngularVelocity = new DoubleYoVariable("alphaAngularVelocity", registry);
      alphaAngularVelocity.set(0.8);
      angularVelocityToCoMYZPlane2 = new FilteredVelocityYoVariable("angularVelocityToCoMYZPlane2", "", alphaAngularVelocity, angleToCoMInYZPlane, controlDT,
                                                                    registry);
      angularVelocityToCoMXZPlane2 = new FilteredVelocityYoVariable("angularVelocityToCoMXZPlane2", "", alphaAngularVelocity, angleToCoMInXZPlane, controlDT,
                                                                    registry);

      if (skippyToDo.getEnumValue() != SkippyToDo.BALANCE && skippyToDo.getEnumValue() != SkippyToDo.POSITION || true)
      {
         stateMachine = new StateMachine<States>("stateMachine", "stateMachineTime", States.class, robot.t, registry);
         setUpStateMachines();
         createStateMachineWindow();
      }
      /*
       * Boolean variables for artifacts drawing
       */
      boolean drawCenterOfMass = true;
      boolean drawActualICP = true;
      boolean drawFootLocation = true;
      boolean drawActualCMPFromDefinition = false;// true;
      boolean drawAchievedCMP = false;// true;
      boolean drawDesiredCMPFromICP = true;
      boolean drawDesiredReactionForce = true;// false;//
      boolean drawActualReactionForce = true;// false;//
      boolean drawCmpToComPositionVector = true;// false;//
      boolean drawHipToFootPositionVector = true;//false;// 
      boolean drawHipJointUnitVector = true;// false;// 
      boolean drawTauHipJoint = true;// false;//
      boolean drawShoulderToFootPositionVector = true;//false;// 
      boolean drawShoulderJointUnitVector = true;//false;// 
      boolean drawTauShoulderJoint = false;// true;//
      boolean drawRateOfChangeOfAngularMomentum = true;//false;// 
      /*
       * CoM
       */
      if (drawCenterOfMass)
      {
         YoGraphicPosition comPositionYoGraphic = new YoGraphicPosition("CoM", com, 0.01, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistries.registerYoGraphic("allGraphics", comPositionYoGraphic);
         yoGraphicsListRegistries.registerArtifact("allGraphics", comPositionYoGraphic.createArtifact());
      }
      /*
       * ICP
       */
      if (drawActualICP)
      {
         YoGraphicPosition icpPositionYoGraphic = new YoGraphicPosition("ICP", icp, 0.01, YoAppearance.Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsListRegistries.registerYoGraphic("allGraphics", icpPositionYoGraphic);
         yoGraphicsListRegistries.registerArtifact("allGraphics", icpPositionYoGraphic.createArtifact());
      }
      /*
       * Foot
       */
      if (drawFootLocation)
      {
         YoGraphicPosition footPositionYoGraphic = new YoGraphicPosition("Foot", footLocation, 0.01, YoAppearance.DarkBlue(), GraphicType.BALL);
         yoGraphicsListRegistries.registerYoGraphic("allGraphics", footPositionYoGraphic);
         yoGraphicsListRegistries.registerArtifact("allGraphics", footPositionYoGraphic.createArtifact());
      }
      /*
       * CMP from definition
       */
      if (drawActualCMPFromDefinition)
      {
         YoGraphicPosition cmpPositionYoGraphic = new YoGraphicPosition("CMP from definition", actualCMPFromDefinition, 0.01, YoAppearance.Red(),
                                                                        GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsListRegistries.registerYoGraphic("allGraphics", cmpPositionYoGraphic);
         yoGraphicsListRegistries.registerArtifact("allGraphics", cmpPositionYoGraphic.createArtifact());
      }
      /*
       * CMP from CoM and reaction geometric relationship fig 16 of book chapter
       */
      if (drawAchievedCMP)
      {
         YoGraphicPosition achievedCMPYoGraphic = new YoGraphicPosition("Achieved CMP", achievedCMP, 0.01, YoAppearance.DarkGreen(),
                                                                        GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsListRegistries.registerYoGraphic("allGraphics", achievedCMPYoGraphic);
         yoGraphicsListRegistries.registerArtifact("allGraphics", achievedCMPYoGraphic.createArtifact());
      }
      /*
       * CMP from ICP
       */
      if (drawDesiredCMPFromICP)
      {
         YoGraphicPosition cmpFromIcpPositionYoGraphic = new YoGraphicPosition("CMP from ICP", desiredCmpFromIcp, 0.0125, YoAppearance.DarkMagenta(),
                                                                               GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsListRegistries.registerYoGraphic("actuallGraphicsalICP", cmpFromIcpPositionYoGraphic);
         yoGraphicsListRegistries.registerArtifact("allGraphics", cmpFromIcpPositionYoGraphic.createArtifact());
      }
      /*
       * Desired reaction force
       */
      if (drawDesiredReactionForce)
      {
         YoGraphicVector desiredGRFYoGraphic = new YoGraphicVector("desiredGRFYoGraphic", footLocation, desiredReactionForce, 0.05, YoAppearance.Orange(),
                                                                   true);
         yoGraphicsListRegistries.registerYoGraphic("desiredReactionForce", desiredGRFYoGraphic);
      }
      /*
       * Acquired reaction force
       */
      if (drawActualReactionForce)
      {
         YoGraphicVector actualGRFYoGraphic = new YoGraphicVector("actualGRFYoGraphic", footLocation, actualReactionForce, 0.05, YoAppearance.DarkGreen(),
                                                                  true);
         yoGraphicsListRegistries.registerYoGraphic("actualReactionForce", actualGRFYoGraphic);
      }
      /*
       * CMP to CoM position vector
       */
      if (drawCmpToComPositionVector)
      {
         YoGraphicVector cmpToComPositionVectorYoGraphic = new YoGraphicVector("cmpToComPositionVectorYoGraphic", desiredCmpFromIcp, cmpToComPositionVector,
                                                                               1.0, YoAppearance.LightBlue(), true);
         yoGraphicsListRegistries.registerYoGraphic("cmpToComPositionVector", cmpToComPositionVectorYoGraphic);
      }
      /*
       * Hip to foot position vector
       */
      if (drawHipToFootPositionVector)
      {
         YoGraphicVector hipToFootPositionVectorYoGraphic = new YoGraphicVector("hipToFootPositionVector", hipJointPosition, hipToFootPositionVector, 1.0,
                                                                                YoAppearance.Red(), true);
         yoGraphicsListRegistries.registerYoGraphic("hipToFootPositionVector", hipToFootPositionVectorYoGraphic);
      }
      /*
       * Hip joint axis
       */
      if (drawHipJointUnitVector)
      {
         YoGraphicVector hipJointAxisYoGraphic = new YoGraphicVector("hipJointAxisYoGraphic", hipJointPosition, hipJointUnitVector, 0.5,
                                                                     YoAppearance.AliceBlue(), true);
         yoGraphicsListRegistries.registerYoGraphic("hipJointUnitVector", hipJointAxisYoGraphic);
      }
      /*
       * Hip joint torque
       */
      if (drawTauHipJoint)
      {
         YoGraphicVector tauHipJointAxisYoGraphic = new YoGraphicVector("tauHipJointAxisYoGraphic", hipJointPosition, tauHipJoint, 0.05,
                                                                        YoAppearance.MediumSlateBlue(), true);
         yoGraphicsListRegistries.registerYoGraphic("tauHipJoint", tauHipJointAxisYoGraphic);
      }
      /*
       * Shoulder to foot joint position vector
       */
      if (drawShoulderToFootPositionVector)
      {
         YoGraphicVector shoulderToFootPositionVectorYoGraphic = new YoGraphicVector("footToShoulderPositionVectorYoGraphic", shoulderJointPosition,
                                                                                     shoulderToFootPositionVector, 1.0, YoAppearance.White(), true);
         yoGraphicsListRegistries.registerYoGraphic("shoulderToFootPositionVector", shoulderToFootPositionVectorYoGraphic);
      }
      /*
       * Shoulder joint axis
       */
      if (drawShoulderJointUnitVector)
      {
         YoGraphicVector shoulderJointAxisYoGraphic = new YoGraphicVector("shoulderJointAxisYoGraphic", shoulderJointPosition, shoulderJointUnitVector, 0.5,
                                                                          YoAppearance.AliceBlue(), true);
         yoGraphicsListRegistries.registerYoGraphic("shoulderJointUnitVector", shoulderJointAxisYoGraphic);
      }
      /*
       * Shoulder joint torque
       */
      if (drawTauShoulderJoint)
      {
         YoGraphicVector tauShoulderJointYoGraphic = new YoGraphicVector("tauShoulderJointYoGraphic", shoulderJointPosition, tauShoulderJoint, 0.05,
                                                                         YoAppearance.Yellow(), true);
         yoGraphicsListRegistries.registerYoGraphic("tauShoulderJoint", tauShoulderJointYoGraphic);
      }
      /*
       * Rate of change of Angular Momentum
       */
      if (drawRateOfChangeOfAngularMomentum)
      {
         YoGraphicVector rateOfChangeOfAngularMomentumYoGraphic = new YoGraphicVector("angulerMomentum", com, rateOfChangeOfAngularMomentum, 0.05,
                                                                                      YoAppearance.Yellow(), true);
         yoGraphicsListRegistries.registerYoGraphic("rateOfChangeOfAngularMomentum", rateOfChangeOfAngularMomentumYoGraphic);
      }

      initialize();

   }

   /**
    * 
    */
   public void setUpOutputFiles(boolean setFiles)
   {
      if (setFiles)
      {
         System.out.println("Set up files for debug.");
         /*
          * Set up files for debug
          */
         try
         {
            writer = new PrintWriter("Output.txt", "UTF-8");
         }
         catch (FileNotFoundException | UnsupportedEncodingException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
         /*
          * Set up a file for output
          */
         try
         {
            writer1 = new PrintWriter("Output1.txt", "UTF-8");
         }
         catch (FileNotFoundException | UnsupportedEncodingException e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
      }
   }

   public void doControl()
   {
      actualCoMAndCoMVelocity();
      computeICP();
      actualGroundReactionForce();
      linearAndAngularMomentumRateOfChange();
      cmpFromDefinition();
      computeAchievedCMP();
      //      cmpFromIcpDynamics();
      setParametersForControlModes();
      computeFootToCenterOfMassLocation();
      if (skippyToDo.getEnumValue() == SkippyToDo.BALANCE)
      {
         if (trace && setUpFiles)
            writer.println(stateMachine.getCurrentStateEnum() + " " + skippyToDo + " " + robot.getTime());
         balanceControl();
      }
      else if (skippyToDo.getEnumValue() == SkippyToDo.POSITION)
      {
         if (trace && setUpFiles)
            writer.println(stateMachine.getStateYoVariableName() + " " + skippyToDo + " " + robot.getTime());
         positionControl();
      }
      else
      {
         if (trace && setUpFiles)
            writer.println(stateMachine.getStateYoVariableName() + " " + skippyToDo + " " + robot.getTime());
         jumpControl();
      }
   }

   private void setParametersForControlModes()
   {
      switch (shoulderPlaneControlMode.getEnumValue())
      {
      case BALANCE:
      {
         setShoulderPlaneParametersForBalancing();
         break;
      }

      case POSITION:
      {
         setShoulderPlaneParametersForPositionControl();
         break;
      }
      }

      switch (hipPlaneControlMode.getEnumValue())
      {
      case BALANCE:
      {
         setHipPlaneParametersForBalancing();
         break;
      }

      case POSITION:
      {
         setHipPlaneParametersForPositionControl();
         break;
      }
      }
   }

   /**
    * Torque on shoulder joint from reaction on foot
    * 
    * @param footReaction
    *            TODO
    * @param tauOnShoulderToPack
    *            TODO
    * @param tauOnShoulderAxisToPack
    *            TODO
    */
   public void tauOnShoulderJoint(YoFrameVector footReaction, YoFrameVector tauOnShoulderToPack, DoubleYoVariable tauOnShoulderAxisToPack)
   {
      tauOnShoulderToPack.cross(shoulderToFootPositionVector, footReaction);
      /*
       * Shoulder joint torque projection modulus
       */
      double tempTauShoulderJointProjectionModulus = tauOnShoulderToPack.dot(shoulderJointUnitVector);
      tauOnShoulderAxisToPack.set(tempTauShoulderJointProjectionModulus);
   }

   /**
    * Torque on hip joint from reaction on foot
    * 
    * @param footReaction
    *            TODO
    * @param tauOnHipToPack
    *            TODO
    * @param tauOnHipAxisToPack
    *            TODO
    */
   public void tauOnHipJoint(YoFrameVector footReaction, YoFrameVector tauOnHipToPack, DoubleYoVariable tauOnHipAxisToPack)
   {
      tauOnHipToPack.cross(hipToFootPositionVector, footReaction);
      /*
       * Hip joint torque projection modulus
       */
      double tempTauHipJointProjectionModulus = tauOnHipToPack.dot(hipJointUnitVector);
      tauOnHipAxisToPack.set(tempTauHipJointProjectionModulus);
   }

	/**
	 * Hip and Shoulder to Foot joints position vectors
	 */
	public void jointsToFootPositionVectors() {
		FrameVector hipToFootInWorld = new FrameVector(ReferenceFrame.getWorldFrame());
		FrameVector shoulderToFootInWorld = new FrameVector(ReferenceFrame.getWorldFrame());
		Point3d footLocationInWorld = new Point3d();
		/*
		 * Foot location in world
		 */
		footLocationInWorld.set(robot.computeFootLocation());
		/*
		 * Foot to hip position vector
		 */
		robot.getHipJoint().getTranslationToWorld(hipToFootInWorld.getVector());
		hipJointPosition.set(hipToFootInWorld);
		hipToFootPositionVector.sub(footLocationInWorld, hipToFootInWorld.getVector());
		hipToFootUnitVector.set(hipToFootPositionVector);
		hipToFootUnitVector.normalize();
		/*
		 * Shoulder to Foot position vector
		 */
		robot.getShoulderJoint().getTranslationToWorld(shoulderToFootInWorld.getVector());
		shoulderJointPosition.set(shoulderToFootInWorld);
		shoulderToFootPositionVector.sub(footLocationInWorld, shoulderToFootInWorld.getVector());
		shoulderToFootUnitVector.set(shoulderToFootPositionVector);
		shoulderToFootUnitVector.normalize();
		/*
		 * ONLY FOR DEBUG
		 * Cross product between actual and previous position vectors
		 */
		crossShoulderPositionVector.cross(shoulderToFootUnitVector, previousShoulderToFootUnitVector);
		crossShoulderLength.set(crossShoulderPositionVector.length());
		crossHipPositionVector.cross(hipToFootUnitVector, previousHipToFootUnitVector);
		crossHipLength.set(crossHipPositionVector.length());
//		if(crossShoulderLength.getDoubleValue()>1.0E-3 || crossHipLength.getDoubleValue()>1.0E-3)
//			System.out.println(crossShoulderLength.toString()+"\t"+crossHipLength.toString());
		if (traceCrossProduct && setUpFiles) 
			writer.println(crossHipLength.toString()+crossShoulderLength.toString());
		/*
		 * Distance between consecutive points
		 */
		double hipJointDistance = hipJointPosition.distance(previousHipJointPosition);
		double shoulderJointDistance = shoulderJointPosition.distance(previousShoulderJointPosition);
		double footDistance = footLocation.distance(previousFootLocation);
//		if(hipJointDistance>1.0E-3 || shoulderJointDistance>1.0E-3)
//			System.out.println(robot.getTime()+"\t"+hipJointDistance+"\t"+shoulderJointDistance);
		if (traceCrossProduct && setUpFiles) 
			writer1.println(robot.getTime()+"\t"+footDistance+"\t"+hipJointDistance+"\t"+shoulderJointDistance);
//		if(footDistance>1.0E-3 || hipJointDistance>1.0E-3  || shoulderJointDistance>1.0E-3)
//			System.out.println(robot.getTime()+"\t"+footDistance+"\t"+hipJointDistance+"\t"+shoulderJointDistance);
		/*
		 * Actualize previous variables
		 */
		previousFootLocation.set(footLocationInWorld);
		previousHipJointPosition.set(hipJointPosition);
		previousShoulderJointPosition.set(shoulderJointPosition);
		previousHipToFootUnitVector.set(hipToFootUnitVector);
		previousShoulderToFootUnitVector.set(shoulderToFootUnitVector);
		/*
		 * leg to torso angle
		 */
		legToTorsoAngle.set(Math.acos(hipToFootUnitVector.dot(shoulderToFootUnitVector)));
	}

   /**
    * Hip and Shoulder joint unit vectors
    */
   private void hipAndShoulderUnitVectors()
   {
      /*
       * Hip
       */
      Vector3d tempHipJointAxis = new Vector3d();
      RigidBodyTransform transformHipToWorld = new RigidBodyTransform();
      robot.getHipJoint().getJointAxis(tempHipJointAxis);
      robot.getHipJoint().getTransformToWorld(transformHipToWorld);
      transformHipToWorld.transform(tempHipJointAxis);
      hipJointUnitVector.set(tempHipJointAxis);
      hipJointUnitVector.normalize();
      /*
       * Shoulder
       */
      Vector3d tempShoulderJointAxis = new Vector3d();
      RigidBodyTransform transformShoulderToWorld = new RigidBodyTransform();
      robot.getShoulderJoint().getJointAxis(tempShoulderJointAxis);
      robot.getShoulderJoint().getTransformToWorld(transformShoulderToWorld);
      transformShoulderToWorld.transform(tempShoulderJointAxis);
      shoulderJointUnitVector.set(tempShoulderJointAxis);
      shoulderJointUnitVector.normalize();
   }

   /**
    * Ground reaction force from foot ground contact point
    */
   public void actualGroundReactionForce()
   {
      /*
       * Ground reaction force
       */
      Vector3d tempReactionForce = new Vector3d();
      /*
       * tempReactionForce is packed from a YoFrameVector in WorldFrame
       */
      robot.computeFootContactForce(tempReactionForce);
      actualReactionForce.set(tempReactionForce);
      //      reactionUnitVector.set(actualReactionForce);
      //      reactionUnitVector.normalize();
      //      Vector3d tempNormal = new Vector3d();
      //      robot.footGroundContactPoint.getSurfaceNormal(tempNormal);
   }

   /**
    * CoM and CoM velocity
    */
   public void actualCoMAndCoMVelocity()
   {

      Point3d tempCOMPosition = new Point3d();
      Vector3d tempComVelocity = new Vector3d();
      Vector3d tempAngularMomentum = new Vector3d();
      /*
       * CoM and CoM velocity in WorldFrame
       */
      double totalMass = robot.computeCOMMomentum(tempCOMPosition, tempComVelocity, tempAngularMomentum);
      com.set(tempCOMPosition);
      if (robot.getFootFS())
         averageZ0.set(((counterForZ0Average - 1) * averageZ0.getDoubleValue() + com.getZ()) / counterForZ0Average);
      linearMomentum.set(tempComVelocity);
      angularMomentum.set(tempAngularMomentum);
      tempComVelocity.scale(1.0 / totalMass);
      comVelocity.set(tempComVelocity);
      /*
       * CoM to Foot error
       */
      comToFootError.sub(com.getFrameTuple(), footLocation.getFrameTuple());
   }

   /**
    * CMP to CoM position vector
    */
   public void positionVectorFomCmpToCom()
   {
      cmpToComPositionVector.set(com);
      cmpToComPositionVector.sub(desiredCmpFromIcp);
      cmpToComPositionVector.sub(com, desiredCmpFromIcp);
   }

   /**
    * Foot to CoM position vector
    */
   public void positionVectorFomFootToCom(YoFramePoint actualFootPosition)
   {
      Vector3d tempFootToComPositionVector = new Vector3d();
      Point3d footLocationInWorld = new Point3d();
      footLocationInWorld.set(robot.computeFootLocation());
      com.get(tempFootToComPositionVector);
      footToComPositionVector.setVector(tempFootToComPositionVector);
      footToComPositionVector.sub(footLocationInWorld);
   }

   /**
    * CMP computed from ICP and CMP coupled dynamics from [2] (Eq. 4)
    */
   public void cmpFromIcpDynamics()
   {
      /*
       * ICP to Foot error
       */
      icpToFootError.sub(icp.getFrameTuple(), footLocation.getFrameTuple());
      /*
       * Compute CMP from ICP-CMP dynamics
       */
      desiredCmpFromIcp.set(icpToFootError);
      desiredCmpFromIcp.scale(kCapture.getDoubleValue());
      desiredCmpFromIcp.add(icp);
   }

   /**
    * CMP computed from its definition from [1] (Eq. 2 and 3) when foot is on
    * the ground
    */
   public void cmpFromDefinition()
   {
      if (robot.getFootFS())
      {
         actualCMPFromDefinition.setX((-rateOfChangeOfAngularMomentum.getY() + com.getX() * actualReactionForce.getZ()
               - com.getZ() * actualReactionForce.getX()) / actualReactionForce.getZ());
         actualCMPFromDefinition.setY((-rateOfChangeOfAngularMomentum.getX() + com.getY() * actualReactionForce.getZ()
               - com.getZ() * actualReactionForce.getY()) / actualReactionForce.getZ());
         actualCMPFromDefinition.setZ(0.0);
      }

   }

   /**
    * Compute rate of change (ROC) of CoM linear and angular momentum
    */
   public void linearAndAngularMomentumRateOfChange()
   {
      double deltaT = (double) SkippySimulation.DT;
      rateOfChangeOfLinearMomentum.set(linearMomentum);
      rateOfChangeOfLinearMomentum.sub(lastLinearMomentum);
      rateOfChangeOfLinearMomentum.scale(1 / deltaT);
      /*
       * Compute CoM acceleration
       */
      comAcceleration.set(rateOfChangeOfLinearMomentum);
      comAcceleration.scale(robotMass.getDoubleValue());
      /*
       * Compute rate of change of CoM angular momentum
       */
      rateOfChangeOfAngularMomentum.set(angularMomentum);
      rateOfChangeOfAngularMomentum.sub(lastAngularMomentum);
      rateOfChangeOfAngularMomentum.scale(1 / deltaT);
      /*
       * Atualize last angular and linear momentum
       */
      lastLinearMomentum.set(linearMomentum);
      lastAngularMomentum.set(angularMomentum);
   }

   public void computeAchievedCMP()
   {
      FramePoint2d achievedCMP = new FramePoint2d(ReferenceFrame.getWorldFrame());

      achievedCMP.set(actualReactionForce.getFrameVector2dCopy());
      if (actualReactionForce.getZ() != 0.0)
         achievedCMP.scale(-com.getZ() / actualReactionForce.getZ());
      else
         achievedCMP.set(0.0, 0.0);
      achievedCMP.add(com.getFramePoint2dCopy());
      this.achievedCMP.setXY(achievedCMP);
   }

   /*
    * Compute ICP
    */
   private void computeICP()
   {
      /*
       * Compute ICP
       */
      averagedW0.set(Math.sqrt(averageZ0.getDoubleValue() / Math.abs(robot.getGravityt())));
      fixedW0.set(Math.sqrt(z0.getDoubleValue() / Math.abs(robot.getGravityt())));
      icp.scaleAdd(fixedW0/* averagedW0 */.getDoubleValue(), comVelocity, com);
      icp.setZ(0.0);
      /*
       * Compute ICP velocity
       */
      icpVelocity.scaleAdd(fixedW0/* averagedW0 */.getDoubleValue(), comAcceleration, comVelocity);
      
   }

   private void computeFootToCenterOfMassLocation()
   {
      ReferenceFrame bodyFrame = robot.updateAndGetBodyFrame();

      FramePoint bodyPoint = new FramePoint(bodyFrame);
      bodyPoint.changeFrame(ReferenceFrame.getWorldFrame());

      bodyLocation.set(bodyPoint);

      footLocation.set(robot.computeFootLocation());

      footLocation.getFrameTupleIncludingFrame(tempFootLocation);
      com.getFrameTupleIncludingFrame(tempCoMLocation);

      // footToLastCoMLocation.set(tempFootToCoM.getVectorCopy());
      // lastCoMLocation.set(tempCoMLocation);

      tempFootLocation.changeFrame(bodyFrame);
      tempCoMLocation.changeFrame(bodyFrame);

      tempFootToCoM.setIncludingFrame(tempCoMLocation);
      tempFootToCoM.sub(tempFootLocation);

      footToCoMInBodyFrame.set(tempFootToCoM);
   }

   /**
    * jumpControl: Allows Skippy model to jump sideways or forward
    */
   private void jumpControl()
   {

      if (trace)
         System.out.println("jumpControl");
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      balanceControl();
   }

   /**
    * balanceControl: Balances Tippy/Skippy based on q_d_hip and q_d_shoulder
    */
   private void balanceControl()
   {
      
//      /*
//       * ICP to Foot error
//       */
//      icpToFootError.sub(icp.getFrameTuple(), footLocation.getFrameTuple());
//      /*
//       * Compute CMP from ICP-CMP dynamics
//       */
//      desiredCmpFromIcp.set(icpToFootError);
//      desiredCmpFromIcp.scale(kCapture.getDoubleValue());
//      desiredCmpFromIcp.add(icp);
      /*
       * Compute position and unit vectors from CMP to COM
       */
      cmpToComPositionVector.sub(com, desiredCmpFromIcp);
      /*
       * Unit vector from CMP to COM
       */
      cmpToComUnitVector.set(cmpToComPositionVector);
      cmpToComUnitVector.normalize();
      /*
       * Compute desired reaction force.
       */
      reactionModulus.set(robotWeight.getDoubleValue() / cmpToComUnitVector.getZ());
      desiredReactionForce.set(cmpToComUnitVector);
      desiredReactionForce.scale(reactionModulus.getDoubleValue());
      /*
       * Compute foot to joints position vectors
       */
      jointsToFootPositionVectors();
      hipAndShoulderUnitVectors();
      /*
       * Compute tau on hip joint axis from desired reaction
       */
      tauOnHipJoint(desiredReactionForce, tauHipJoint, tauOnHipJointAxis);
      tauOnShoulderJoint(desiredReactionForce, tauShoulderJoint, tauOnShoulderJointAxis);
   }

   private void applyTorqueToHip(double hipDesired)
   {
      /*
       * angular pos : angle created w/ com to groundpoint against vertical
       */

      // double footToComZ = centerOfMass.getZ()-footLocation.getZ();
      // double footToComY = centerOfMass.getY()-footLocation.getY();

      double footToComZ = footToCoMInBodyFrame.getZ();
      double footToComY = footToCoMInBodyFrame.getY();

      planarDistanceYZPlane.set(Math.sqrt(Math.pow(com.getY() - footLocation.getY(), 2) + Math.pow(footToComZ, 2)));
      double angle = (Math.atan2(footToComY, footToComZ));
      angleToCoMInYZPlane.set(angle);

      /*
       * angular vel : angle created w/ com to groundpoint against vertical
       */
      Vector3d linearMomentum = new Vector3d();
      robot.computeLinearMomentum(linearMomentum);

      // 1: projection vector
      Vector3d componentPerpendicular = new Vector3d(0, 1, -com.getY() / com.getZ());
      componentPerpendicular.normalize();
      double angleVel = componentPerpendicular.dot(linearMomentum) / componentPerpendicular.length();
      angleVel = angleVel / robotMass.getDoubleValue();
      angularVelocityToCoMYZPlane.set(angleVel);
      angularVelocityToCoMYZPlane2.update();
      double angularVelocityForControl = angularVelocityToCoMYZPlane2.getDoubleValue();

      /*
       * angular pos/vel of hipjoint
       */
      double hipAngle = 0;
      double hipAngleVel = 0;
      double[] hipAngleValues = new double[2];

      hipAngleValues = calculateAnglePosAndDerOfHipJointTippy(robot.getHipJointTippy());

      hipAngle = hipAngleValues[0];
      hipAngleVel = hipAngleValues[1];
      qHipIncludingOffset.set((hipAngle));
      qDHipIncludingOffset.set(hipAngleVel);
      robot.getHipJointTippy().setTau(k1.getDoubleValue() * (0.0 - angle) + k2.getDoubleValue() * (0.0 - angularVelocityForControl)
            + k3.getDoubleValue() * (hipDesired - hipAngle) + k4.getDoubleValue() * (0.0 - hipAngleVel));
   }

   private Vector3d createVectorInDirectionOfHipJointAlongHip()
   {
      Vector3d rootJointCoordinates = new Vector3d();
      robot.getHipJointSkippy().getTranslationToWorld(rootJointCoordinates);
      Vector3d hipEndPointCoordinates = new Vector3d();
      robot.getGroundContactPoints().get(1).getPosition(hipEndPointCoordinates);
      rootJointCoordinates.sub(hipEndPointCoordinates);
      return rootJointCoordinates;
   }

   private void applyTorqueToShoulder(double shoulderDesired)
   {
      /*
       * angular pos : angle created w/ com to groundpoint against vertical
       */

      double footToComZ = footToCoMInBodyFrame.getZ();
      double footToComX = footToCoMInBodyFrame.getX();

      planarDistanceXZPlane.set(Math.sqrt(Math.pow(footToComX, 2) + Math.pow(footToComZ, 2)));
      double angle = (Math.atan2(footToComX, footToComZ));
      angleToCoMInXZPlane.set(angle);

      /*
       * angular vel : angle created w/ com to groundpoint against vertical
       */
      Vector3d linearMomentum = new Vector3d();
      robot.computeLinearMomentum(linearMomentum);

      // 1: projection vector
      Vector3d componentPerpendicular = new Vector3d(1, 0, -com.getX() / com.getZ());
      componentPerpendicular.normalize();
      double angleVel = componentPerpendicular.dot(linearMomentum) / componentPerpendicular.length();
      angleVel = angleVel / robotMass.getDoubleValue();

      // 2: not used
      // double angleVel = Math.pow(Math.pow(linearMomentum.getY(), 2) +
      // Math.pow(linearMomentum.getZ(), 2), 0.5)/robotMass;
      // angleVel = angleVel / planarDistanceYZPlane;

      // 3: average rate of change (buggy)
      // double angleVel = (angle - prevAngleHip) / SkippySimulation.DT;

      angularVelocityToCoMXZPlane.set(angleVel);
      angularVelocityToCoMXZPlane2.update();
      double angularVelocityForControl = angularVelocityToCoMXZPlane2.getDoubleValue();

      /*
       * angular pos/vel of hipjoint
       */
      double shoulderAngle = 0;
      double shoulderAngleVel = 0;
      double[] shoulderAngleValues = new double[2];

      shoulderAngleValues = calculateAnglePosAndDerOfShoulderJointTippy(robot.getShoulderJoint());

      shoulderAngle = shoulderAngleValues[0];
      shoulderAngleVel = shoulderAngleValues[1];
      qShoulderIncludingOffset.set((shoulderAngle));
      qDShoulderIncludingOffset.set(shoulderAngleVel);

      double shoulderAngleError = AngleTools.computeAngleDifferenceMinusPiToPi(shoulderDesired, shoulderAngle);
      robot.getShoulderJoint().setTau(k5.getDoubleValue() * Math.sin(0.0 - angle) + k6.getDoubleValue() * (0.0 - angularVelocityForControl)
            + k7.getDoubleValue() * (shoulderAngleError) + k8.getDoubleValue() * (0.0 - shoulderAngleVel));

   }

   private Vector3d createVectorInDirectionOfShoulderJointAlongShoulder()
   {
      Vector3d shoulderJointCoordinates = new Vector3d();
      robot.getShoulderJoint().getTranslationToWorld(shoulderJointCoordinates);
      Vector3d shoulderEndPointCoordinates = new Vector3d();
      robot.getGroundContactPoints().get(2).getPosition(shoulderEndPointCoordinates);
      shoulderEndPointCoordinates.sub(shoulderJointCoordinates);
      return shoulderEndPointCoordinates;
   }

   private double[] calculateAnglePosAndDerOfHipJointTippy(PinJoint joint)
   {
      double[] finale = new double[2];

      // for different definition of hipJointAngle (angle b/w hipJoint and
      // vertical (z axis) )
      // double firstAngle =
      // robot.getLegJoint().getQ().getDoubleValue()%(Math.PI*2);
      // if(firstAngle>Math.PI)
      // firstAngle = (Math.PI*2-firstAngle)*-1;
      // double angle =
      // (joint.getQ().getDoubleValue())%(Math.PI*2)+firstAngle;
      // if(angle > Math.PI)
      // angle = angle - Math.PI*2;

      double angle = joint.getQYoVariable().getDoubleValue();
      double angleVel = joint.getQDYoVariable().getDoubleValue();
      finale[0] = angle;
      finale[1] = (angleVel);
      return finale;
   }

   private double[] calculateAnglePosAndDerOfHipJointSkippy(FloatingJoint joint)
   {
      /*
       * Using groundcontact points to create vectors
       */
      double[] finale = new double[2];

      Vector3d verticalVector = new Vector3d(0.0, 0.0, 1.0);
      Vector3d floatVector = createVectorInDirectionOfHipJointAlongHip();
      verticalVector.setX(0.0);
      floatVector.setX(0.0); // angle wrt yz plane only

      double cosineTheta = (floatVector.dot(verticalVector) / (floatVector.length() * verticalVector.length()));
      double angle = Math.acos(cosineTheta);
      if (floatVector.getY() < 0)
         angle = angle * -1;
      /*
       * // increases same speed wrt angle diff. between root and leg
       */
      double angleVel = robot.getLegJoint().getQDYoVariable().getDoubleValue();
      finale[0] = angle;
      finale[1] = (angleVel);
      return finale;
   }

   private double[] calculateAnglePosAndDerOfShoulderJointTippy(PinJoint joint)
   {
      double[] finale = new double[2];

      // for different definition of shoulderJointAngle (angle b/w
      // shoulderJoint and vertical (z-axis) )
      // double firstAngle = 0;
      //
      // firstAngle =
      // (robot.getLegJoint().getSecondJoint().getQ().getDoubleValue())%(Math.PI*2);
      // if(firstAngle>Math.PI)
      // firstAngle = (Math.PI*2-firstAngle)*-1;
      // double angle =
      // (joint.getQ().getDoubleValue())%(Math.PI*2)+firstAngle;
      // if(angle > Math.PI)
      // angle = angle - Math.PI*2;

      double angle = joint.getQYoVariable().getDoubleValue();
      double angleVel = joint.getQDYoVariable().getDoubleValue();

      finale[0] = angle;
      finale[1] = angleVel;
      return finale;
   }

   private double[] calculateAnglePosAndDerOfShoulderJointSkippy(PinJoint joint)
   {
      double[] finale = new double[2];

      Vector3d horizontalVector = new Vector3d(1.0, 0.0, 0.0);
      Vector3d shoulderVector = createVectorInDirectionOfShoulderJointAlongShoulder();
      horizontalVector.setY(0);
      shoulderVector.setY(0);

      double cosineTheta = (horizontalVector.dot(shoulderVector) / (horizontalVector.length() * shoulderVector.length()));
      double angle = Math.abs(Math.acos(cosineTheta));

      Vector3d shoulderJointPosition = new Vector3d();
      joint.getTranslationToWorld(shoulderJointPosition);

      if (robot.getGroundContactPoints().get(2).getZ() < shoulderJointPosition.getZ())
         angle = angle * -1;

      double angleVel = robot.getShoulderJoint().getQDYoVariable().getDoubleValue();
      finale[0] = angle;
      finale[1] = angleVel;
      return finale;
   }

   private double fromRadiansToDegrees(double radians)
   {
      return radians * 180 / Math.PI;
   }

   /**
    * positionControl: positions Tippy model in whatever position desired
    * (specified within method)
    */
   private void positionControl()
   {
      double desiredX = 0.0;
      double desiredY = Math.PI / 6;
      double desiredHip = -2 * Math.PI / 6;
      double desiredShoulder = 0.0;

      positionJointsBasedOnError(robot.getLegJoint(), desiredX, legIntegralTermX, 20000, 150, 2000, true);
      positionJointsBasedOnError(robot.getLegJoint().getSecondJoint(), desiredY, legIntegralTermY, 20000, 150, 2000, false);
      positionJointsBasedOnError(robot.getHipJointTippy(), desiredHip, hipIntegralTerm, 20000, 150, 2000, false);
      positionJointsBasedOnError(robot.getShoulderJoint(), desiredShoulder, shoulderIntegralTerm, 20000, 150, 2000, false);
   }

   public void positionJointsBasedOnError(PinJoint joint, double desiredValue, double integralTerm, double positionErrorGain, double integralErrorGain,
                                          double derivativeErrorGain, boolean isBasedOnWorldCoordinates)
   {
      // try to change position based on angular position wrt xyz coordinate
      // system
      Matrix3d rotationMatrixForWorld = new Matrix3d();
      joint.getRotationToWorld(rotationMatrixForWorld);
      double rotationToWorld = Math.asin((rotationMatrixForWorld.getM21()));
      // if(rotationMatrixForWorld.getM11()<0)
      // rotationToWorld = rotationToWorld * -1;
      if (isBasedOnWorldCoordinates)
      {
         // System.out.println(joint.getName() + " " +
         // (joint.getQ().getDoubleValue()) + " " + rotationToWorld);
      }
      else
         rotationToWorld = joint.getQYoVariable().getDoubleValue();

      double positionError = (positionErrorGain) * ((desiredValue - rotationToWorld));
      integralTerm += (integralErrorGain) * positionError * SkippySimulation.DT;
      double derivativeError = (derivativeErrorGain) * (0 - joint.getQDYoVariable().getDoubleValue());
      joint.setTau(positionError + integralTerm + derivativeError);
      // System.out.print(joint.getName() + ": " +
      // (joint.getQ().getDoubleValue() - desiredValue));
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public void initialize()
   {
   }

   public String getDescription()
   {
      return getName();
   }

   /*
    * STATE MACHINES
    */

   private void setUpStateMachines()
   {
      // states
      State<States> balanceState = new BalanceState(skippyToDo.getEnumValue());
      State<States> prepareState = new PrepareState(skippyToDo.getEnumValue());
      State<States> leanState = new LeanState(skippyToDo.getEnumValue());
      State<States> liftoffState = new LiftoffState(skippyToDo.getEnumValue());
      State<States> repositionState = new RepositionState(skippyToDo.getEnumValue());
      State<States> recoverState = new RecoverState(skippyToDo.getEnumValue());

      // transitions
      StateTransitionCondition balanceToPrepareTransitionCondition = new BalanceToPrepareTransitionCondition();
      StateTransitionCondition prepareToLeanTransitionCondition = new PrepareToLeanTransitionCondition(skippyToDo.getEnumValue());
      StateTransitionCondition leanToLiftoffTransitionCondition = new LeanToLiftoffTransitionCondition(skippyToDo.getEnumValue());
      StateTransitionCondition liftoffToRepositionTransitionCondition = new LiftoffToRepositionTransitionCondition(skippyToDo.getEnumValue());
      StateTransitionCondition repositionToRecoverTransitionCondition = new RepositionToRecoverTransitionCondition(skippyToDo.getEnumValue());
      StateTransitionCondition recoverToBalanceTransitionCondition = new RecoverToBalanceTransitionCondition(skippyToDo.getEnumValue());

      StateTransition<States> balanceToPrepare = new StateTransition<States>(States.PREPARE, balanceToPrepareTransitionCondition);
      balanceState.addStateTransition(balanceToPrepare);

      StateTransition<States> prepareToLean = new StateTransition<States>(States.LEAN, prepareToLeanTransitionCondition);
      prepareState.addStateTransition(prepareToLean);

      StateTransition<States> leanToLiftoff = new StateTransition<States>(States.LIFTOFF, leanToLiftoffTransitionCondition);
      leanState.addStateTransition(leanToLiftoff);

      StateTransition<States> liftoffToReposition = new StateTransition<States>(States.REPOSITION, liftoffToRepositionTransitionCondition);
      liftoffState.addStateTransition(liftoffToReposition);

      StateTransition<States> repositionToRecover = new StateTransition<States>(States.RECOVER, repositionToRecoverTransitionCondition);
      repositionState.addStateTransition(repositionToRecover);

      StateTransition<States> recoverToBalance = new StateTransition<States>(States.BALANCE, recoverToBalanceTransitionCondition);
      recoverState.addStateTransition(recoverToBalance);

      stateMachine.addState(balanceState);
      stateMachine.addState(prepareState);
      stateMachine.addState(leanState);
      stateMachine.addState(liftoffState);
      stateMachine.addState(repositionState);
      stateMachine.addState(recoverState);

      stateMachine.setCurrentState(States.BALANCE);
   }

   public void createStateMachineWindow()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         public void run()
         {
            createStateMachineWindowLocal();
         }
      });
   }

   public void createStateMachineWindowLocal()
   {
      JFrame frame = new JFrame("Skippy Jump State Machine");
      Container contentPane = frame.getContentPane();
      contentPane.setLayout(new BoxLayout(contentPane, BoxLayout.X_AXIS));

      StateMachinesJPanel<States> stateMachinePanel = new StateMachinesJPanel<States>(stateMachine, true);

      frame.getContentPane().add(stateMachinePanel);

      frame.pack();
      frame.setSize(450, 300);
      frame.setAlwaysOnTop(false);
      frame.setVisible(true);

      stateMachine.attachStateChangedListener(stateMachinePanel);
   }

   private void setHipPlaneParametersForPositionControl()
   {
      k1.set(0.0);
      k2.set(0.0);
      k3.set(300.0);
      k4.set(30.0);
   }

   private void setShoulderPlaneParametersForPositionControl()
   {
      k5.set(0.0);
      k6.set(0.0);
      k7.set(300.0);
      k8.set(30.0);
   }

   private void setShoulderPlaneParametersForBalancing()
   {
      k5.set(-1900);
      k6.set(-490.0);
      k7.set(-60.0);
      k8.set(-45.0);
   }

   private void setHipPlaneParametersForBalancing()
   {
      k1.set(-3600.0);
      k2.set(-1500.0);
      k3.set(-170.0);
      k4.set(-130.0);
   }

   public class BalanceToPrepareTransitionCondition implements StateTransitionCondition
   {
      public BalanceToPrepareTransitionCondition()
      {
      }

      public boolean checkCondition()
      {
         return false;
         //         if(trace)
         //            System.out.println("BalanceToPrepareTransitionCondition");
         //         if (skippyToDo.getEnumValue() == SkippyToDo.JUMP_FORWARD)
         //         {
         //            double time = stateMachine.timeInCurrentState();
         //            return time >= 1.0;//4.0;  //
         //         }
         //         else
         //            return false;
      }
   }

   public class PrepareToLeanTransitionCondition implements StateTransitionCondition
   {

      private final SkippyToDo direction;

      public PrepareToLeanTransitionCondition(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public boolean checkCondition()
      {
         if (trace)
            System.out.println("PrepareToLeanTransitionCondition");
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            double time = stateMachine.timeInCurrentState();
            return time < 1.3 && time > 1.29;// < 2.3 && time > 2.29;//< 7.01 && time > 6.99; //
         }
         else
            return false;
      }
   }

   public class LeanToLiftoffTransitionCondition implements StateTransitionCondition
   {

      private final SkippyToDo direction;

      public LeanToLiftoffTransitionCondition(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public boolean checkCondition()
      {
         if (trace)
            System.out.println("LeanToLiftoffTransitionCondition");
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            double time = stateMachine.timeInCurrentState();
            return time > 0.5; //> 0.2;   //true; // 
         }
         else
            return false;
      }
   }

   public class LiftoffToRepositionTransitionCondition implements StateTransitionCondition
   {

      private final SkippyToDo direction;

      public LiftoffToRepositionTransitionCondition(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public boolean checkCondition()
      {
         if (trace)
            System.out.println("checkCondition");
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            double time = stateMachine.timeInCurrentState();
            return time > 0.3 && robot.getFootFS();//  time < 0.46 && time > 0.45;//time < 0.20 && time > 0.19;//
         }
         else
            return false;

         // Vector3d angMom = new Vector3d();
         // robot.computeAngularMomentum(angMom);
         // return angMom.length() < 0.01;

      }
   }

   public class RepositionToRecoverTransitionCondition implements StateTransitionCondition
   {

      private final SkippyToDo direction;

      public RepositionToRecoverTransitionCondition(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public boolean checkCondition()
      {
         if (trace)
            System.out.println("RepositionToRecoverTransitionCondition");
         double time = stateMachine.timeInCurrentState();
         return time < 0.80 && time > 0.79; //< 0.60 && time > 0.59; //< 0.70 && time > 0.69;// 
      }
   }

   public class RecoverToBalanceTransitionCondition implements StateTransitionCondition
   {

      private final SkippyToDo direction;

      public RecoverToBalanceTransitionCondition(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public boolean checkCondition()
      {
         if (trace)
            System.out.println("RecoverToBalanceTransitionCondition");
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            double time = stateMachine.timeInCurrentState();
            return time < 4.01 && time > 3.99;
         }
         return false;
      }
   }

   private class BalanceState extends State<States>
   {

      private final SkippyToDo direction;

      public BalanceState(SkippyToDo direction)
      {
         super(States.BALANCE);
         this.direction = direction;
      }

      public void doAction()
      {
         //         qd_hip.set(0.6);
         if (robot.getFootFS())
         {
            /*
             * Torque on hip for hip angle control
             */
            hipAngleController.setProportionalGain(0.0); //(179.53125);//(2500.0);//
            hipAngleController.setDerivativeGain(0.0);// 0.00602454);//1000.0);//
            hipAngleController.setIntegralGain(0.0);//0.116299896953656563); //100.00);//

            tauHipForAngleControl.set(hipAngleController.compute(/*legToTorsoAngle*/robot.getQ_hip().getDoubleValue(), desiredLegToTorsoAngle.getDoubleValue(),
                                                                  +robot.getQd_hip().getDoubleValue(), 0.0, deltaT));
            /*
             * Torque on hip for CoM velocity control
             */
            comVelocityController.setProportionalGain(0.0);
            comVelocityController.setDerivativeGain(0.0);
            comVelocityController.setIntegralGain(0.0);
            tauHipForComVelocityControl.set(comVelocityController.compute(comVelocity.getX(), 0.0, comAcceleration.getX(), 0.0, deltaT));
            tauHipForComVelocityControl.add(comVelocityController.compute(comVelocity.getY(), 0.0, comAcceleration.getY(), 0.0, deltaT));
            /*
             * Tau on hip for CoM to foot error
             */
            comToFoorErrorController.setProportionalGain(100.0);
            comToFoorErrorController.setDerivativeGain(0.0);
            comToFoorErrorController.setIntegralGain(0.0);
            tauHipForComToFootErrorControl.set(comToFoorErrorController.compute(com.getX(), footLocation.getX(), comVelocity.getX(), 0.0, deltaT));
            tauHipForComToFootErrorControl.add(comToFoorErrorController.compute(com.getY(), footLocation.getY(), comVelocity.getY(), 0.0, deltaT));
            /*
             * Torque on hip for ICP to foot error control
             */
            icpToFootErrorController.setProportionalGain(0.0); 
            icpToFootErrorController.setDerivativeGain(0.0); 
            icpToFootErrorController.setIntegralGain(0.0);
            tauHipForIcpToFootErrorControl.set(icpToFootErrorController.compute(icpToFootError.getX(), 0.0, icpVelocity.getVector3dCopy().getX(), 0.0, deltaT));
            tauHipForIcpToFootErrorControl.add(icpToFootErrorController.compute(icpToFootError.getY(), 0.0, icpVelocity.getVector3dCopy().getY(), 0.0, deltaT));
            
            double totalTauHip = tauHipForAngleControl.getDoubleValue()+tauHipForComVelocityControl.getDoubleValue()+tauHipForIcpToFootErrorControl.getDoubleValue();
            /*
             * Apply torque to the joints
             */
            robot.getHipJointTippy()
                 .setTau(totalTauHip);
            robot.getShoulderJoint().setTau(tauOnShoulderJointAxis.getDoubleValue());
            if (trace && setUpFiles)
               writer.println(stateMachine.getCurrentState() + " CMP controller------------------------" + stateMachine.timeInCurrentState());
         }

      }

      public void doTransitionIntoAction()
      {
         //         qd_hip.set(0.6);
         if (firstEnterBalanceState)
         {
            firstEnterBalanceState = false;
            desiredLegToTorsoAngle.set(7*Math.PI/16);//desiredLegToTorsoAngle.getDoubleValue());//robot.getQ_hip().getDoubleValue());
         }
      }

      public void doTransitionOutOfAction()
      {

         //         desiredLegToTorsoAngle.set(robot.getQ_hip().getDoubleValue()); //

      }
   }

   private class PrepareState extends State<States>
   {

      private final SkippyToDo direction;

      public PrepareState(SkippyToDo direction)
      {
         super(States.PREPARE);
         this.direction = direction;
      }

      public void doAction()
      {
         if (robot.getFootFS())
         {
            /*
             * Torque on hip for keeping track the angle between torso and leg
             */
            //            desiredLegToTorsoAngle.set(-0.75075); //
            tauHipForAngleControl.set(hipAngleController.compute(robot.getQ_hip().getDoubleValue(), desiredLegToTorsoAngle.getDoubleValue(),
                                                                  -robot.getQd_hip().getDoubleValue(), 0.0, deltaT));
            /*
             * Apply torque to the joints
             */
            robot.getHipJointTippy().setTau(tauOnHipJointAxis.getDoubleValue() + tauHipForAngleControl.getDoubleValue()); // );//
            robot.getShoulderJoint().setTau(tauOnShoulderJointAxis.getDoubleValue());
            if (trace && setUpFiles)
               writer.println(stateMachine.getCurrentState() + " CMP controller------------------------" + stateMachine.timeInCurrentState());
         }
      }

      public void doTransitionIntoAction()
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            //            qd_hip.set(1.6);
         }
      }

      public void doTransitionOutOfAction()
      {

      }
   }

   private class LeanState extends State<States>
   {
      private final SkippyToDo direction;

      public LeanState(SkippyToDo direction)
      {
         super(States.LEAN);
         this.direction = direction;
      }

      public void doAction()
      {
      }

      public void doTransitionIntoAction()
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            hipPlaneControlMode.set(SkippyPlaneControlMode.POSITION);
            //            qd_hip.set(1.4);
            balanceControl();
         }
      }

      public void doTransitionOutOfAction()
      {

      }
   }

   private class LiftoffState extends State<States>
   {

      private final SkippyToDo direction;

      public LiftoffState(SkippyToDo direction)
      {
         super(States.LIFTOFF);
         this.direction = direction;
      }

      public void doAction()
      {
         if (robot.getFootFS())
         {
            /*
             * To jump, applying torques from previous controller
             */
            applyTorqueToHip(qd_hip.getDoubleValue());
            applyTorqueToShoulder(qd_shoulder.getDoubleValue());
            if (trace && setUpFiles)
               writer.println(stateMachine.getCurrentState() + " Non CMP controller " + stateMachine.timeInCurrentState());
         }
         else
         {
            /*
             * Torque on hip for keeping track the angle between torso and leg
             */
            desiredLegToTorsoAngle.set(1.5075 / 2.0); //
            tauHipForAngleControl.set(hipAngleController.compute(robot.getQ_hip().getDoubleValue(), desiredLegToTorsoAngle.getDoubleValue(),
                                                                  -robot.getQd_hip().getDoubleValue(), 0.0, deltaT));
            applyTorqueToHip(/* qd_hip.getDoubleValue()+ */tauHipForAngleControl.getDoubleValue());
            applyTorqueToShoulder(qd_shoulder.getDoubleValue());
         }
      }

      public void doTransitionIntoAction()
      {
         //         qd_hip.set(0.45);
      }

      public void doTransitionOutOfAction()
      {
      }
   }

   private class RepositionState extends State<States>
   {

      private final SkippyToDo direction;

      public RepositionState(SkippyToDo direction)
      {
         super(States.REPOSITION);
         this.direction = direction;
      }

      public void doAction()
      {
         if (robot.getFootFS())
         {
            /*
             * Torque on hip for keeping track the angle between torso and leg
             */
            desiredLegToTorsoAngle.set(-0.745);//-0.5075);   
            tauHipForAngleControl.set(hipAngleController.compute(robot.getQ_hip().getDoubleValue(), desiredLegToTorsoAngle.getDoubleValue(),
                                                                  -robot.getQd_hip().getDoubleValue(), 0.0, deltaT));
            /*
             * Apply torque to the joints
             */
            robot.getHipJointTippy().setTau(tauOnHipJointAxis.getDoubleValue() + tauHipForAngleControl.getDoubleValue()); //);//
            robot.getShoulderJoint().setTau(tauOnShoulderJointAxis.getDoubleValue());
         }
      }

      public void doTransitionIntoAction()
      {
         //         qd_hip.set(-1.3);
      }

      public void doTransitionOutOfAction()
      {

      }
   }

   private class RecoverState extends State<States>
   {

      private final SkippyToDo direction;

      public RecoverState(SkippyToDo direction)
      {
         super(States.RECOVER);
         this.direction = direction;
      }

      public void doAction()
      {
         if (robot.getFootFS())
         {
            /*
             * Torque on hip for keeping track the angle between torso and leg
             */
            //            desiredLegToTorsoAngle.set(0.5);//7854); //
            hipAngleController.setProportionalGain(50.0);
            tauHipForAngleControl.set(hipAngleController.compute(robot.getQ_hip().getDoubleValue(), desiredLegToTorsoAngle.getDoubleValue(),
                                                                  -robot.getQd_hip().getDoubleValue(), 0.0, deltaT));
            hipAngleController.setProportionalGain(5.0);
            /*
             * Apply torques to balance robot on the ground after reposition
             */
            /*
             * Torque on hip for keeping track the angle between torso and leg
             */
            robot.getHipJointTippy().setTau(tauOnHipJointAxis.getDoubleValue());// + tauHipForAngleTracking.getDoubleValue());
            robot.getShoulderJoint().setTau(tauOnShoulderJointAxis.getDoubleValue());

            if (trace && setUpFiles)
               writer.println(stateMachine.getCurrentState() + " CMP controller------------------------" + stateMachine.timeInCurrentState());
         }

      }

      public void doTransitionIntoAction()
      {
         // hipPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);
         // shoulderPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);

         //         qd_hip.set(-0.9);
         //         qd_shoulder.set(0.0);
         robot.glueDownToGroundPoint.setForce(0.0, 0.0, -1450.0);
      }

      public void doTransitionOutOfAction()
      {

      }
   }

   /**
    * @param setListeners
    */

   public void listeners(boolean setListeners)
   {
      /*
       * Begin YoVariable Listener
       */
      if (setListeners)
      {
         DoubleYoVariable footX = (DoubleYoVariable) robot.getVariable("gc_foot_x");
         footX.addVariableChangedListener(new VariableChangedListener()
         {
            double previousFootX = 0.0;

            @Override
            public void variableChanged(YoVariable<?> v)
            {
               if (Math.abs(v.getValueAsDouble() - previousFootX) > tol)
               //            if ((previousValue > 0.0) && (v.getValueAsDouble() < 0.0))
               {
                  System.out.print(robot.getTime()+" Xdist= " + Math.abs(v.getValueAsDouble() - previousFootX));//v);
               }

               previousFootX = v.getValueAsDouble();
            }
         });

         DoubleYoVariable footY = (DoubleYoVariable) robot.getVariable("gc_foot_y");

         footY.addVariableChangedListener(new VariableChangedListener()
         {
            double previousFootY = 0.0;

            @Override
            public void variableChanged(YoVariable<?> v)
            {
               if (Math.abs(v.getValueAsDouble() - previousFootY) > tol)
               //               if ((v.getValueAsDouble() > -0.001) && (v.getValueAsDouble() < 0.001))
               {
                  System.out.print(robot.getTime()+" Ydist= " + Math.abs(v.getValueAsDouble() - previousFootY));//v);
               }

               previousFootY = v.getValueAsDouble();
            }
         });

         DoubleYoVariable footZ = (DoubleYoVariable) robot.getVariable("gc_foot_z");

         footZ.addVariableChangedListener(new VariableChangedListener()
         {
            double previousFootZ = 0.0;

            @Override
            public void variableChanged(YoVariable<?> v)
            {
               if (Math.abs(v.getValueAsDouble() - previousFootZ) > tol)
               //               if ((v.getValueAsDouble() > -0.001) && (v.getValueAsDouble() < 0.001))
               {
                  System.out.println(robot.getTime()+" Zdist= " + Math.abs(v.getValueAsDouble() - previousFootZ));//v);
               }

               previousFootZ = v.getValueAsDouble();
            }
         });

      }

      /*
       * End YoVariable Listener
       */
   }

   public void closeFile()
   {
      System.out.println("Closed the file");
      if(setUpFiles)
         writer.close();
   }
}
