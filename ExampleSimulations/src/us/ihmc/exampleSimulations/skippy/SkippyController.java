
package us.ihmc.exampleSimulations.skippy;

import java.awt.Container;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.exampleSimulations.skippy.SkippyRobot.RobotType;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
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
   private PIDController hipAngleController, comToFootErrorController, comVelocityController, icpToFootErrorController;

   private final DoubleYoVariable alphaAngularVelocity;
   private final FilteredVelocityYoVariable angularVelocityToCoMYZPlane2, angularVelocityToCoMXZPlane2;

   private final YoFramePoint bodyLocation = new YoFramePoint("body", ReferenceFrame.getWorldFrame(), registry);

   private final ExternalForcePoint forceToCOM;
   private final YoFramePoint com = new YoFramePoint("com", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector comVelocity = new YoFrameVector("comVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector comAcceleration = new YoFrameVector("comAcceleration", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredReactionForce = new YoFrameVector("desiredReactionForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector reactionForce = new YoFrameVector("reactionForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector reactionUnitVector = new YoFrameVector("reactionUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector surfaceNormal = new YoFrameVector("surfaceNormal", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector angularMomentum = new YoFrameVector("angularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector lastAngularMomentum = new YoFrameVector("lastAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector linearMomentum = new YoFrameVector("linearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector lastLinearMomentum = new YoFrameVector("lastLinearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector rateOfChangeOfLinearMomentum = new YoFrameVector("rateOfChangeOfLinearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector rateOfChangeOfAngularMomentum = new YoFrameVector("rateOfChangeOfAngularMomentum", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector icpToFootError = new YoFrameVector("icpToFootError", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector comToFootError = new YoFrameVector("comToFootError", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable w0 = new DoubleYoVariable("fixedW0", registry);

   private final YoFrameVector tauShoulderFromReaction = new YoFrameVector("tauShoulderJoint", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable tauOnShoulderAxis = new DoubleYoVariable("tauOnShoulderJointAxis", registry);

   private final YoFrameVector tauHipFromReaction = new YoFrameVector("tauHipJoint", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable tauOnHipAxis = new DoubleYoVariable("tauOnHipJointAxis", registry);

   private final DoubleYoVariable q_d_hip = new DoubleYoVariable("q_d_hip", registry);
   private final DoubleYoVariable q_d_shoulder = new DoubleYoVariable("q_d_shoulder", registry);

   private final YoFramePoint hipJointPosition = new YoFramePoint("hipJointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipJointUnitVector = new YoFrameVector("hipJointUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipToFootPositionVector = new YoFrameVector("hipToFootPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector hipToFootUnitVector = new YoFrameVector("hipToFootUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint shoulderJointPosition = new YoFramePoint("shoulderJointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector shoulderJointUnitVector = new YoFrameVector("shoulderJointUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector shoulderToFootPositionVector = new YoFrameVector("shoulderToFootPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector shoulderToFootUnitVector = new YoFrameVector("shoulderToFootUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector cmpToComPositionVector = new YoFrameVector("cmpToComPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector cmpToComUnitVector = new YoFrameVector("cmpToComUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector footToComPositionVector = new YoFrameVector("footToComPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector footToCoMInBodyFrame;
   private final YoFramePoint icp = new YoFramePoint("icp", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint icpVelocity = new YoFramePoint("icpVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint cmpFromDefinition = new YoFramePoint("cmpFromDefinition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint cmpFromIcp = new YoFramePoint("cmpFromIcp", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint cmpFromParameterizedReaction = new YoFramePoint("cmpFromParametrizedReaction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint footLocation = new YoFramePoint("footLocation", ReferenceFrame.getWorldFrame(), registry);

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
   private final DoubleYoVariable legToTorsoAngle = new DoubleYoVariable("legToTorsoAngle", registry);

   private final FramePoint tempFootLocation = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePoint tempCoMLocation = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FrameVector tempFootToCoM = new FrameVector(ReferenceFrame.getWorldFrame());

   private final DoubleYoVariable z0 = new DoubleYoVariable("z0", registry);
   private final DoubleYoVariable kCapture = new DoubleYoVariable("kCapture", registry);
   private final DoubleYoVariable desiredLegToTorsoAngle = new DoubleYoVariable("desiredLegToTorsoAngle", registry);

   private final EnumYoVariable<SkippyToDo> skippyToDo = new EnumYoVariable<SkippyToDo>("SkippyToDo", registry, SkippyToDo.class);
   private final EnumYoVariable<SkippyPlaneControlMode> hipPlaneControlMode = new EnumYoVariable<SkippyPlaneControlMode>("hipPlaneControlMode", registry,
                                                                                                                         SkippyPlaneControlMode.class);
   private final EnumYoVariable<SkippyPlaneControlMode> shoulderPlaneControlMode = new EnumYoVariable<SkippyPlaneControlMode>("shoulderPlaneControlMode",
                                                                                                                              registry,
                                                                                                                              SkippyPlaneControlMode.class);
   private final double deltaT = (double) SkippySimulation.DT;

   private String name;
   private SkippyRobot robot;
   private RobotType robotType;

   private double legIntegralTermX = 0.0;
   private double legIntegralTermY = 0.0;
   private double hipIntegralTerm = 0.0;
   private double shoulderIntegralTerm = 0.0;

   double angularMomentumIntegralError = 0.0;
   double lastReactionForce = 0.0;
   int counterForAveragedZ0 = 1;
   //   boolean printOnce = true;
   PrintWriter writer = null;
   PrintWriter writer1 = null;
   boolean firstEnterBalanceState = true;
   /*
    * Much debug stuff
    */
   boolean setUpFiles = false; //true;//
   boolean trace = false;//true;//
   boolean crossProductAndPointsDistance = false;// true; //
   boolean traceCom = true; //false;//
   boolean traceCmpToCom = false; //true; //
   double tol = 0.1; //threshold for debug distances

   boolean icpBasedController = false;

   public SkippyController(SkippyRobot robot, RobotType robotType, String name, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistries)
   {
      this.name = name;
      this.robot = robot;
      this.robotType = robotType;

      /*
       * Hip angle torque controller
       */
      hipAngleController = new PIDController("hipAngle", controllerRegistry);
      comVelocityController = new PIDController("comVelocity", controllerRegistry);
      comToFootErrorController = new PIDController("comToFoot", controllerRegistry);
      icpToFootErrorController = new PIDController("icpToFoot", controllerRegistry);

      //      hipAngleController.setProportionalGain(5.0);
      //      hipAngleController.setDerivativeGain(0.0);
      //      hipAngleController.setIntegralGain(0.1);
      /*
       * z0 and KCapture
       */
      z0.set(1.216); // got from averaged CoM_Z during simulation
      kCapture.set(1.5);//4.0);// 2.0);//0.9);
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

      skippyToDo.set(SkippyToDo.JUMP_FORWARD);//skippyToDo.set(SkippyToDo.BALANCE);
      hipPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);
      shoulderPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);

      if (skippyToDo.getEnumValue() == SkippyToDo.BALANCE)
      {
         qd_hip.set(0.6);
         qd_shoulder.set(0.0);
      }
      else if (skippyToDo.getEnumValue() == SkippyToDo.JUMP_FORWARD)
      {
         q_d_hip.set(0.6);//qd_hip.set(0.0);
         qd_shoulder.set(0.0);
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
      boolean drawICP = true;
      boolean drawFootLocation = true;
      boolean drawCMPFromDefinition = true; //false;//
      boolean drawCmpFromReaction = true; //false;//
      boolean drawCMPFromIcp = false;//true; //
      boolean drawDesiredReactionForce = false;//true;//
      boolean drawActualReactionForce = false;//true;//
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
      if (drawICP)
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
      if (drawCMPFromDefinition)
      {
         YoGraphicPosition cmpPositionYoGraphic = new YoGraphicPosition("CMP from definition", cmpFromDefinition, 0.01, YoAppearance.Magenta(),
                                                                        GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsListRegistries.registerYoGraphic("allGraphics", cmpPositionYoGraphic);
         yoGraphicsListRegistries.registerArtifact("allGraphics", cmpPositionYoGraphic.createArtifact());
      }
      /*
       * CMP from CoM and reaction geometric relationship fig 16 of book chapter
       */
      if (drawCmpFromReaction)
      {
         YoGraphicPosition achievedCMPYoGraphic = new YoGraphicPosition("CMP from parametrized reaction", cmpFromParameterizedReaction, 0.01,
                                                                        YoAppearance.Crimson(), GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsListRegistries.registerYoGraphic("allGraphics", achievedCMPYoGraphic);
         yoGraphicsListRegistries.registerArtifact("allGraphics", achievedCMPYoGraphic.createArtifact());
      }
      /*
       * CMP from ICP
       */
      if (drawCMPFromIcp)
      {
         YoGraphicPosition cmpFromIcpPositionYoGraphic = new YoGraphicPosition("CMP from ICP", cmpFromIcp, 0.0125, YoAppearance.DarkMagenta(),
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
         YoGraphicVector actualGRFYoGraphic = new YoGraphicVector("actualGRFYoGraphic", footLocation, reactionForce, 0.05, YoAppearance.DarkGreen(), true);
         yoGraphicsListRegistries.registerYoGraphic("actualReactionForce", actualGRFYoGraphic);
      }
      /*
       * CMP to CoM position vector
       */
      if (drawCmpToComPositionVector)
      {
         YoGraphicVector cmpToComPositionVectorYoGraphic = new YoGraphicVector("cmpToComPositionVectorYoGraphic",
                                                                               /* cmpFromIcp */ cmpFromDefinition /* cmpFromParametrizedReaction */,
                                                                               cmpToComPositionVector, 1.0, YoAppearance.LightBlue(), true);
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
         YoGraphicVector tauHipJointAxisYoGraphic = new YoGraphicVector("tauHipJointAxisYoGraphic", hipJointPosition, tauHipFromReaction, 0.05,
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
         YoGraphicVector tauShoulderJointYoGraphic = new YoGraphicVector("tauShoulderJointYoGraphic", shoulderJointPosition, tauShoulderFromReaction, 0.05,
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
      if (traceCom && setUpFiles)
         writer.println("Trace CoM; DoControl stick starts.");

      comAndComVelocity();
      if (traceCom && setUpFiles)
         writer.println("1.  " + com.toString());

      linearAndAngularMomentumRateOfChange();
      if (traceCom && setUpFiles)
         writer.println("2.  " + com.toString());

      computeICP();
      if (traceCom && setUpFiles)
         writer.println("3.  " + com.toString());

      groundReactionForce();
      if (traceCom && setUpFiles)
         writer.println("4.  " + com.toString());

      cmpFromDefinition(); //cmpFromDefinition
      if (traceCom && setUpFiles)
         writer.println("5.  " + com.toString());

      cmpFromParameterizedReaction(); //cmpFromParametrizedReaction
      if (traceCom && setUpFiles)
         writer.println("6.  " + com.toString());

      cmpFromIcpDynamics(); //cmpFromIcp
      if (traceCom && setUpFiles)
         writer.println("7.  " + com.toString());

      setParametersForControlModes();
      computeFootToCenterOfMassLocation();
      if (traceCom && setUpFiles)
         writer.println("8.  " + com.toString());

      if (skippyToDo.getEnumValue() == SkippyToDo.BALANCE)
      {
         if (trace && setUpFiles)
            writer.println(stateMachine.getCurrentStateEnum() + " " + skippyToDo + " " + robot.getTime());

         balanceControl();
         if (traceCom && setUpFiles)
            writer.println("9.  " + com.toString());
      }
      else if (skippyToDo.getEnumValue() == SkippyToDo.POSITION)
      {
         if (trace && setUpFiles)
            writer.println(stateMachine.getStateYoVariableName() + " " + skippyToDo + " " + robot.getTime());

         positionControl();
         if (traceCom && setUpFiles)
            System.out.println("10. " + com.toString());
      }
      else
      {
         if (trace && setUpFiles)
            writer.println(stateMachine.getStateYoVariableName() + " " + skippyToDo + " " + robot.getTime());

         jumpControl();
         if (traceCom && setUpFiles)
            writer.println("12. " + com.toString());
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
    * Torque on joint from reaction on foot
    */
   public void tauOnJointFromReactionOnCmp(YoFrameVector jointUnitVector, YoFrameVector jointToFootPositionVector, YoFrameVector footReaction,
                                           YoFrameVector tauOnJointToPack, DoubleYoVariable tauOnJointAxisToPack)
   {
      tauOnJointToPack.cross(jointToFootPositionVector, footReaction);
      /*
       * Joint torque projection modulus
       */
      double tempTauJointProjectionModulus = tauOnJointToPack.dot(jointUnitVector);
      tauOnJointAxisToPack.set(tempTauJointProjectionModulus);
   }

   /**
    * Hip and Shoulder to Foot joints position vectors
    */
   public void jointsToFootPositionVectors()
   {
      FrameVector hipToFootInWorld = new FrameVector(ReferenceFrame.getWorldFrame());
      FrameVector shoulderToFootInWorld = new FrameVector(ReferenceFrame.getWorldFrame());
      /*
       * Foot location in world
       */
      footLocation.set(robot.computeFootLocation());
      /*
       * Foot to hip position vector
       */
      robot.getHipJoint().getTranslationToWorld(hipToFootInWorld.getVector());
      hipJointPosition.set(hipToFootInWorld);
      hipToFootPositionVector.sub(footLocation.getVector3dCopy(), hipToFootInWorld.getVector());
      hipToFootUnitVector.set(hipToFootPositionVector);
      hipToFootUnitVector.normalize();
      /*
       * Shoulder to Foot position vector
       */
      robot.getShoulderJoint().getTranslationToWorld(shoulderToFootInWorld.getVector());
      shoulderJointPosition.set(shoulderToFootInWorld);
      shoulderToFootPositionVector.sub(footLocation.getVector3dCopy(), shoulderToFootInWorld.getVector());
      shoulderToFootUnitVector.set(shoulderToFootPositionVector);
      shoulderToFootUnitVector.normalize();
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
    * Ground reaction force, reaction unit vector and surface normal
    */
   public void groundReactionForce()
   {
      /*
       * Ground reaction force and unit vector
       */
      Vector3d tempReactionForce = new Vector3d();
      robot.computeFootContactForce(tempReactionForce);
      reactionForce.set(tempReactionForce);
      reactionUnitVector.set(reactionForce);
      double norm = reactionUnitVector.getX() * reactionUnitVector.getX() + reactionUnitVector.getY() * reactionUnitVector.getY()
            + reactionUnitVector.getZ() * reactionUnitVector.getZ();
      if (norm > 0.0)
         reactionUnitVector.scale(1 / norm);
      /*
       * Surface normal
       */
      Vector3d tempSurfaceNormal = new Vector3d();
      robot.footGroundContactPoint.getSurfaceNormal(tempSurfaceNormal);
      surfaceNormal.set(tempSurfaceNormal);
      surfaceNormal.normalize();
   }

   /**
    * CoM and CoM velocity
    */
   public void comAndComVelocity()
   {

      Point3d tempCOMPosition = new Point3d();
      Vector3d tempLinearMomentum = new Vector3d();
      Vector3d tempAngularMomentum = new Vector3d();
      /*
       * CoM and CoM velocity in WorldFrame
       */
      double totalMass = robot.computeCOMMomentum(tempCOMPosition, tempLinearMomentum, tempAngularMomentum);
      com.set(tempCOMPosition);
      linearMomentum.set(tempLinearMomentum);
      angularMomentum.set(tempAngularMomentum);
      tempLinearMomentum.scale(1.0 / totalMass);
      comVelocity.set(tempLinearMomentum);
      /*
       * CoM to Foot error
       */
      comToFootError.sub(com.getFrameTuple(), footLocation.getFrameTuple());
      comToFootError.setZ(0.0);
   }

   /**
    * CMP to CoM position vector
    * @param cmpFrom TODO
    */
   public void positionVectorFomCmpToCom(YoFramePoint cmpFrom)
   {
      cmpToComPositionVector.set(com);
      cmpToComPositionVector.sub(cmpFrom);
      cmpToComPositionVector.sub(com, cmpFrom);
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
      cmpFromIcp.scaleAdd(kCapture.getDoubleValue(), icpToFootError, icp);
      cmpFromIcp.setZ(0.0);
   }

   /**
    * CMP computed from definition [1] (Eq. 2 and 3)
    */
   public void cmpFromDefinition()
   {
      if (reactionForce.getZ() != 0.0) //robot.getFootFS())
      {
         cmpFromDefinition.setX((/*-rateOfChangeOfAngularMomentum.getY()*/ +com.getX() * reactionForce.getZ() - com.getZ() * reactionForce.getX())
               / reactionForce.getZ());
         cmpFromDefinition.setY((/*-rateOfChangeOfAngularMomentum.getX()*/ +com.getY() * reactionForce.getZ() - com.getZ() * reactionForce.getY())
               / reactionForce.getZ());
         cmpFromDefinition.setZ(0.0);
      }
   }

   /**
    * Compute rate of change (ROC) of CoM linear and angular momentum
    */
   public void linearAndAngularMomentumRateOfChange()
   {
      double deltaT = (double) SkippySimulation.DT;
      rateOfChangeOfLinearMomentum.set(linearMomentum);
      if (robot.getTime() == deltaT)
         lastLinearMomentum.set(linearMomentum);
      rateOfChangeOfLinearMomentum.sub(lastLinearMomentum);
      rateOfChangeOfLinearMomentum.scale(1 / deltaT);
      /*
       * Compute CoM acceleration
       */
      comAcceleration.set(rateOfChangeOfLinearMomentum);
      comAcceleration.scale(1 / robotMass.getDoubleValue());
      /*
       * Compute rate of change of CoM angular momentum
       */
      rateOfChangeOfAngularMomentum.set(angularMomentum);
      rateOfChangeOfAngularMomentum.sub(lastAngularMomentum);
      rateOfChangeOfAngularMomentum.scale(1 / deltaT);
      /*
       * Actualize last angular and linear momentum
       */
      lastLinearMomentum.set(linearMomentum);
      lastAngularMomentum.set(angularMomentum);
   }

   /*
    * Compute CPM according to book chapter page 26 (notebook pags 58 e 65)
    */
   public void cmpFromParameterizedReaction()
   {
      FramePoint2d tempCMP = new FramePoint2d(ReferenceFrame.getWorldFrame());

      tempCMP.set(reactionForce.getFrameVector2dCopy());
      if (reactionForce.getZ() != 0.0)
         tempCMP.scale(-com.getZ() / reactionForce.getZ());
      else
         tempCMP.set(0.0, 0.0);
      tempCMP.add(com.getFramePoint2dCopy());
      this.cmpFromParameterizedReaction.setXY(tempCMP);
   }

   /*
    * Compute ICP and ICP velocity
    */
   private void computeICP()
   {
      /*
       * Compute ICP
       */
      w0.set(Math.sqrt(z0.getDoubleValue() / Math.abs(robot.getGravity())));
      icp.scaleAdd(w0.getDoubleValue(), comVelocity, com);
      icp.setZ(0.0);
      /*
       * Compute ICP velocity
       */
      icpVelocity.scaleAdd(w0/* averagedW0 */.getDoubleValue(), comAcceleration, comVelocity);
      icpVelocity.setZ(0.0);
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
      if (traceCom && setUpFiles)
         writer.println("11. " + com.toString());
      balanceControl();
   }

   /**
    * balanceControl: Balances Tippy/Skippy based on q_d_hip and q_d_shoulder
    */
   private void balanceControl()
   {
      if (icpBasedController)
      {

         /*
          * Compute position and unit vectors from CMP to COM
          */
         cmpToComPositionVector.sub(com, /* cmpFromIcp */ cmpFromDefinition /* cmpFromParametrizedReaction */ );
         if (traceCmpToCom && setUpFiles)
         {
            writer.println("Time: " + robot.getTime() + " CoM:" + com.toString());
            writer.println("Time: " + robot.getTime() + " CMP:" + cmpFromDefinition.toString());
            writer.println("Time: " + robot.getTime() + " PV  " + cmpToComPositionVector.toString());
         }
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
         if (robot.getFootFS())
         {
            tauOnJointFromReactionOnCmp(hipJointUnitVector, hipToFootPositionVector, desiredReactionForce, tauHipFromReaction, tauOnHipAxis);
            tauOnJointFromReactionOnCmp(shoulderJointUnitVector, shoulderToFootPositionVector, desiredReactionForce, tauShoulderFromReaction,
                                        tauOnShoulderAxis);
         }
         /*
          * Torque on hip for hip angle control
          */
         hipAngleController.setProportionalGain(400.0);//800.0);//0.0);//200.0);//50.0);//31.25); //(179.53125);//(2500.0);//
         hipAngleController.setDerivativeGain(100.0);//200.0);//0.0);//50.0);//40.0);//20.0);//10.0);//0.00602454);//1000.0);//
         hipAngleController.setIntegralGain(0.0);//10.0);//0.116299896953656563); //100.00);//
         tauOnHipAxis.add(hipAngleController.compute(/* legToTorsoAngle */robot.getQ_hip().getDoubleValue(), desiredLegToTorsoAngle.getDoubleValue(),
                                                     +robot.getQd_hip().getDoubleValue(), 0.0, deltaT));
         /*
          * Torque on hip for CoM velocity control
          */
         comVelocityController.setProportionalGain(0.0); //40.0);  //
         comVelocityController.setDerivativeGain(0.0); //1.0);  //
         comVelocityController.setIntegralGain(0.0);
         tauOnHipAxis.add(comVelocityController.compute(comVelocity.getX(), 0.0, comAcceleration.getX(), 0.0, deltaT));
         tauOnHipAxis.add(comVelocityController.compute(comVelocity.getY(), 0.0, comAcceleration.getY(), 0.0, deltaT));
         /*
          * Tau on hip for CoM to foot error
          */
         comToFootErrorController.setProportionalGain(0.0); //1000.0); //10.0); //
         comToFootErrorController.setDerivativeGain(0.0); //100.0);   //10.0);  //
         comToFootErrorController.setIntegralGain(0.0);
         tauOnHipAxis.add(comToFootErrorController.compute(com.getX(), footLocation.getX(), comVelocity.getX(), 0.0, deltaT));
         tauOnHipAxis.add(comToFootErrorController.compute(com.getY(), footLocation.getY(), comVelocity.getY(), 0.0, deltaT));
         /*
          * Torque on hip for ICP to foot error control
          */
         icpToFootErrorController.setProportionalGain(0.0); //100.0);   //
         icpToFootErrorController.setDerivativeGain(0.0); //10.0);  //
         icpToFootErrorController.setIntegralGain(0.0);
         tauOnHipAxis.add(icpToFootErrorController.compute(icpToFootError.getX(), 0.0, icpVelocity.getVector3dCopy().getX(), 0.0, deltaT));
         tauOnHipAxis.add(icpToFootErrorController.compute(icpToFootError.getY(), 0.0, icpVelocity.getVector3dCopy().getY(), 0.0, deltaT));
      }
      else
      {
         applyTorqueToHip(q_d_hip.getDoubleValue());
         applyTorqueToShoulder(q_d_shoulder.getDoubleValue());
      }
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
      StateTransitionCondition balanceToPrepareTransitionCondition = new BalanceToPrepareTransitionCondition(skippyToDo.getEnumValue());
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
      private final SkippyToDo direction;

      public BalanceToPrepareTransitionCondition(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public boolean checkCondition()
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            double time = stateMachine.timeInCurrentState();
            return time < 4.01 && time > 3.99;
         }
         else
            return false;
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
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            double time = stateMachine.timeInCurrentState();
            return time < 7.01 && time > 6.99; //
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
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            double time = stateMachine.timeInCurrentState();
//            return time < 0.0 && time > 0.09;
            return true;
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
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            double time = stateMachine.timeInCurrentState();
            return time < 0.36 && time > 0.35;//
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
         return time < 0.60 && time > 0.59;
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
         qd_hip.set(0.6);
         if (icpBasedController) //Else do nothing.
         {
            if (robot.getFootFS())
            {
               /*
                * Apply torque to the joints
                */
               robot.getHipJointTippy().setTau(tauOnHipAxis.getDoubleValue());
               robot.getShoulderJoint().setTau(tauOnShoulderAxis.getDoubleValue());
               if (trace && setUpFiles)
                  writer.println(stateMachine.getCurrentState() + " CMP controller------------------------" + stateMachine.timeInCurrentState());
            }
         }
      }

      public void doTransitionIntoAction()
      {
         qd_hip.set(0.6);
         if (icpBasedController) //Else do nothing.
         {
            if (firstEnterBalanceState)
            {
               firstEnterBalanceState = false;
               desiredLegToTorsoAngle.set(robot.getQ_hip().getDoubleValue()); //7 * Math.PI / 16);//desiredLegToTorsoAngle.getDoubleValue());//
            }
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
         if (icpBasedController) //Else do nothing.
         {
            if (robot.getFootFS())
            {
               /*
                * Torque on hip for keeping track the angle between torso and
                * leg
                */
               //            desiredLegToTorsoAngle.set(-0.75075); //
               tauHipForAngleControl.set(hipAngleController.compute(robot.getQ_hip().getDoubleValue(), desiredLegToTorsoAngle.getDoubleValue(),
                                                                    -robot.getQd_hip().getDoubleValue(), 0.0, deltaT));
               /*
                * Apply torque to the joints
                */
               robot.getHipJointTippy().setTau(tauOnHipAxis.getDoubleValue() + tauHipForAngleControl.getDoubleValue()); // );//
               robot.getShoulderJoint().setTau(tauOnShoulderAxis.getDoubleValue());
               if (trace && setUpFiles)
                  writer.println(stateMachine.getCurrentState() + " CMP controller------------------------" + stateMachine.timeInCurrentState());
            }
         }
      }

      public void doTransitionIntoAction()
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            qd_hip.set(1.6);
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
            qd_hip.set(1.4);
            //            balanceControl();
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
         if (icpBasedController) //Else do nothing.
         {
            if (robot.getFootFS())
            {
               /*
                * To jump, applying torques from previous controller
                */
               applyTorqueToHip(qd_hip.getDoubleValue());
               applyTorqueToShoulder(qd_shoulder.getDoubleValue());
            }
            else
            {
               /*
                * Torque on hip for keeping track the angle between torso and
                * leg
                */
               desiredLegToTorsoAngle.set(1.5075 / 2.0); //
               tauHipForAngleControl.set(hipAngleController.compute(robot.getQ_hip().getDoubleValue(), desiredLegToTorsoAngle.getDoubleValue(),
                                                                    -robot.getQd_hip().getDoubleValue(), 0.0, deltaT));
               applyTorqueToHip(/* qd_hip.getDoubleValue()+ */tauHipForAngleControl.getDoubleValue());
               applyTorqueToShoulder(qd_shoulder.getDoubleValue());
            }
         }
      }

      public void doTransitionIntoAction()
      {
         qd_hip.set(0.45);
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
         if (icpBasedController) //Else do nothing.
         {
            if (robot.getFootFS())
            {
               /*
                * Torque on hip for keeping track the angle between torso and
                * leg
                */
               desiredLegToTorsoAngle.set(-0.745);//-0.5075);
               tauHipForAngleControl.set(hipAngleController.compute(robot.getQ_hip().getDoubleValue(), desiredLegToTorsoAngle.getDoubleValue(),
                                                                    -robot.getQd_hip().getDoubleValue(), 0.0, deltaT));
               /*
                * Apply torque to the joints
                */
               robot.getHipJointTippy().setTau(tauOnHipAxis.getDoubleValue() + tauHipForAngleControl.getDoubleValue()); //);//
               robot.getShoulderJoint().setTau(tauOnShoulderAxis.getDoubleValue());
            }
         }
      }

      public void doTransitionIntoAction()
      {
         qd_hip.set(-1.3);
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
         if (icpBasedController) //Else do nothing.
         {
            if (robot.getFootFS())
            {
               /*
                * Torque on hip for keeping track the angle between torso and
                * leg
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
                * Torque on hip for keeping track the angle between torso and
                * leg
                */
               robot.getHipJointTippy().setTau(tauOnHipAxis.getDoubleValue());// + tauHipForAngleTracking.getDoubleValue());
               robot.getShoulderJoint().setTau(tauOnShoulderAxis.getDoubleValue());

               if (trace && setUpFiles)
                  writer.println(stateMachine.getCurrentState() + " CMP controller------------------------" + stateMachine.timeInCurrentState());
            }
         }
      }

      public void doTransitionIntoAction()
      {
         hipPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);
         shoulderPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);

         qd_hip.set(-0.9);
         qd_shoulder.set(0.0);
         robot.glueDownToGroundPoint.setForce(0.0, 0.0, -1450.0);
      }

      public void doTransitionOutOfAction()
      {

      }

   }


   public void closeFile()
   {
      System.out.println("Closed the file");
      if (setUpFiles)
         writer.close();
   }
}
