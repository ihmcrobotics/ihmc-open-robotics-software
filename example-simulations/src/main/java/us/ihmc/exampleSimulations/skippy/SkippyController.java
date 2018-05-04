
package us.ihmc.exampleSimulations.skippy;

import java.awt.Container;
import java.io.PrintWriter;

import javax.swing.BoxLayout;
import javax.swing.JFrame;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.skippy.SkippyRobot.RobotType;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.extra.StateMachinesJPanel;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class SkippyController implements RobotController
{

   /**
    *
    * Outline of SkippyToDo: JUMP_FORWARD: If Skippy model is selected, robot JUMP_SIDEWAYS: If Skippy
    * model is selected, robot will jump/balance in x direction (torso's rotation axis) BALANCE: If
    * Skippy/Tippy model is selected, robot will balance POSITION: If Tippy model is selected, robot
    * will balance with the help of LEG joint (not tested)
    *
    * Note: First three SkippyStatuses will allow model to balance according to: q_d_hip: desired angle
    * of TORSO q_d_shoulder: desired angle of SHOULDER
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

   private StateMachine<States, State> stateMachine;

   private final YoVariableRegistry registry = new YoVariableRegistry("SkippyController");

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
   // private YoDouble q_foot_X, q_hip, qHipIncludingOffset, qd_foot_X,
   // qd_hip, qd_shoulder;
   private final YoDouble k1, k2, k3, k4, k5, k6, k7, k8, angleToCoMInYZPlane, angleToCoMInXZPlane, angularVelocityToCoMYZPlane, angularVelocityToCoMXZPlane; // controller
   // gain
   // parameters
   private final YoDouble planarDistanceYZPlane, planarDistanceXZPlane;

   private final YoDouble alphaAngularVelocity;
   private final FilteredVelocityYoVariable angularVelocityToCoMYZPlane2, angularVelocityToCoMXZPlane2;

   private final YoFramePoint3D bodyLocation = new YoFramePoint3D("body", ReferenceFrame.getWorldFrame(), registry);

   private final ExternalForcePoint forceToCOM;
   private final YoFramePoint3D com = new YoFramePoint3D("com", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D comVelocity = new YoFrameVector3D("comVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D comAcceleration = new YoFrameVector3D("comAcceleration", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D desiredReactionForce = new YoFrameVector3D("desiredReactionForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D reactionForce = new YoFrameVector3D("reactionForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D reactionUnitVector = new YoFrameVector3D("reactionUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D surfaceNormal = new YoFrameVector3D("surfaceNormal", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D angularMomentum = new YoFrameVector3D("angularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D lastAngularMomentum = new YoFrameVector3D("lastAngularMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D linearMomentum = new YoFrameVector3D("linearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D lastLinearMomentum = new YoFrameVector3D("lastLinearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D rateOfChangeOfLinearMomentum = new YoFrameVector3D("rateOfChangeOfLinearMomentum", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D rateOfChangeOfAngularMomentum = new YoFrameVector3D("rateOfChangeOfAngularMomentum", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D icpToFootError = new YoFrameVector3D("icpToFootError", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D comToFootError = new YoFrameVector3D("comToFootError", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble w0 = new YoDouble("fixedW0", registry);

   private final YoFrameVector3D tauShoulderFromReaction = new YoFrameVector3D("tauShoulderJoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D tauHipFromReaction = new YoFrameVector3D("tauHipJoint", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble q_d_hip = new YoDouble("q_d_hip", registry);
   private final YoDouble q_d_shoulder = new YoDouble("q_d_shoulder", registry);

   private final YoFramePoint3D hipJointPosition = new YoFramePoint3D("hipJointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D hipJointUnitVector = new YoFrameVector3D("hipJointUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D hipToFootPositionVector = new YoFrameVector3D("hipToFootPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D hipToFootUnitVector = new YoFrameVector3D("hipToFootUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D shoulderJointPosition = new YoFramePoint3D("shoulderJointPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D shoulderJointUnitVector = new YoFrameVector3D("shoulderJointUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D shoulderToFootPositionVector = new YoFrameVector3D("shoulderToFootPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D shoulderToFootUnitVector = new YoFrameVector3D("shoulderToFootUnitVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D cmpToComPositionVector = new YoFrameVector3D("cmpToComPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D footToComPositionVector = new YoFrameVector3D("footToComPositionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D footToCoMInBodyFrame;
   private final YoFramePoint3D icp = new YoFramePoint3D("icp", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D icpVelocity = new YoFramePoint3D("icpVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D cmpFromDefinition = new YoFramePoint3D("cmpFromDefinition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D cmpFromIcp = new YoFramePoint3D("cmpFromIcp", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D cmpFromParameterizedReaction = new YoFramePoint3D("cmpFromParametrizedReaction", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D footLocation = new YoFramePoint3D("footLocation", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble robotMass = new YoDouble("robotMass", registry);
   private final YoDouble robotWeight = new YoDouble("robotWeight", registry);
   private final YoDouble qHipIncludingOffset = new YoDouble("qHipIncludingOffset", registry);
   private final YoDouble qDHipIncludingOffset = new YoDouble("qDHipIncludingOffset", registry);
   private final YoDouble qDShoulderIncludingOffset = new YoDouble("qDShoulderIncludingOffset", registry);
   private final YoDouble qd_hip = new YoDouble("qd_hip", registry);
   private final YoDouble qShoulderIncludingOffset = new YoDouble("qShoulderIncludingOffset", registry);
   private final YoDouble qd_shoulder = new YoDouble("qd_shoulder", registry);

   private final FramePoint3D tempFootLocation = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FramePoint3D tempCoMLocation = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempFootToCoM = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private final YoDouble z0 = new YoDouble("z0", registry);
   private final YoDouble kCapture = new YoDouble("kCapture", registry);

   private final YoEnum<SkippyToDo> skippyToDo = new YoEnum<SkippyToDo>("SkippyToDo", registry, SkippyToDo.class);
   private final YoEnum<SkippyPlaneControlMode> hipPlaneControlMode = new YoEnum<SkippyPlaneControlMode>("hipPlaneControlMode", registry,
                                                                                                         SkippyPlaneControlMode.class);
   private final YoEnum<SkippyPlaneControlMode> shoulderPlaneControlMode = new YoEnum<SkippyPlaneControlMode>("shoulderPlaneControlMode", registry,
                                                                                                              SkippyPlaneControlMode.class);
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
   boolean trace = false;//true;//
   boolean crossProductAndPointsDistance = false;// true; //
   boolean traceCom = true; //false;//
   boolean traceCmpToCom = false; //true; //

   public SkippyController(SkippyRobot robot, RobotType robotType, String name, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistries)
   {
      this.name = name;
      this.robot = robot;
      this.robotType = robotType;

      /*
       * z0 and KCapture
       */
      z0.set(1.216); // got from averaged CoM_Z during simulation
      kCapture.set(1.5);
      robotMass.set(robot.getMass());
      robotWeight.set(robotMass.getDoubleValue() * Math.abs(robot.getGravityZ()));

      footToCoMInBodyFrame = new YoFrameVector3D("footToCoMInBody", robot.updateAndGetBodyFrame(), registry);
      forceToCOM = new ExternalForcePoint("FORCETOCOM", robot);

      k1 = new YoDouble("k1", registry);
      k2 = new YoDouble("k2", registry);
      k3 = new YoDouble("k3", registry);
      k4 = new YoDouble("k4", registry);
      k5 = new YoDouble("k5", registry);
      k6 = new YoDouble("k6", registry);
      k7 = new YoDouble("k7", registry);
      k8 = new YoDouble("k8", registry);

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

      planarDistanceYZPlane = new YoDouble("planarDistanceYZPlane", registry);
      planarDistanceXZPlane = new YoDouble("planarDistanceXZPlane", registry);
      angleToCoMInYZPlane = new YoDouble("angleToCoMYZPlane", registry);
      angleToCoMInXZPlane = new YoDouble("angleToCoMXZPlane", registry);
      angularVelocityToCoMYZPlane = new YoDouble("angularVelocityToCoMYZPlane", registry);
      angularVelocityToCoMXZPlane = new YoDouble("angularVelocityToCoMXZPlane", registry);

      alphaAngularVelocity = new YoDouble("alphaAngularVelocity", registry);
      alphaAngularVelocity.set(0.8);
      angularVelocityToCoMYZPlane2 = new FilteredVelocityYoVariable("angularVelocityToCoMYZPlane2", "", alphaAngularVelocity, angleToCoMInYZPlane, controlDT,
                                                                    registry);
      angularVelocityToCoMXZPlane2 = new FilteredVelocityYoVariable("angularVelocityToCoMXZPlane2", "", alphaAngularVelocity, angleToCoMInXZPlane, controlDT,
                                                                    registry);

      if (skippyToDo.getEnumValue() != SkippyToDo.BALANCE && skippyToDo.getEnumValue() != SkippyToDo.POSITION || true)
      {
         stateMachine = setUpStateMachines();
         createStateMachineWindow();
      }
      /*
       * Boolean variables for artifacts drawing
       */
      boolean drawCenterOfMass = true;
      boolean drawICP = true;
      boolean drawFootLocation = true;
      boolean drawCMPFromDefinition = false;//true; //
      boolean drawCmpFromReaction = false;//true; //
      boolean drawCMPFromIcp = true; //false;//
      boolean drawDesiredReactionForce = false;//true;//
      boolean drawActualReactionForce = false;//true;//
      boolean drawCmpToComPositionVector = false;//true;//
      boolean drawHipToFootPositionVector = false;//true;//
      boolean drawHipJointUnitVector = false;//true;//
      boolean drawTauHipJoint = false;//true;//
      boolean drawShoulderToFootPositionVector = false;//true;//
      boolean drawShoulderJointUnitVector = false;//true;//
      boolean drawTauShoulderJoint = false;//true;//
      boolean drawRateOfChangeOfAngularMomentum = false;//true;//
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

   public void doControl()
   {
      comAndComVelocity();
      linearAndAngularMomentumRateOfChange();
      computeICP();
      groundReactionForce();
      cmpFromDefinition(); //cmpFromDefinition
      cmpFromParameterizedReaction(); //cmpFromParametrizedReaction
      cmpFromIcpDynamics(); //cmpFromIcp
      setParametersForControlModes();
      computeFootToCenterOfMassLocation();
      if (skippyToDo.getEnumValue() == SkippyToDo.BALANCE)
      {
         balanceControl();
      }
      else if (skippyToDo.getEnumValue() == SkippyToDo.POSITION)
      {
         positionControl();
      }
      else
      {
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
    * Torque on joint from reaction on foot
    */
   public void tauOnJointFromReactionOnCmp(YoFrameVector3D jointUnitVector, YoFrameVector3D jointToFootPositionVector, YoFrameVector3D footReaction,
                                           YoFrameVector3D tauOnJointToPack, YoDouble tauOnJointAxisToPack)
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
      FrameVector3D hipToFootInWorld = new FrameVector3D(ReferenceFrame.getWorldFrame());
      FrameVector3D shoulderToFootInWorld = new FrameVector3D(ReferenceFrame.getWorldFrame());
      /*
       * Foot location in world
       */
      footLocation.set(robot.computeFootLocation());
      /*
       * Foot to hip position vector
       */
      robot.getHipJoint().getTranslationToWorld(hipToFootInWorld);
      hipJointPosition.set(hipToFootInWorld);
      hipToFootPositionVector.sub(footLocation, hipToFootInWorld);
      hipToFootUnitVector.set(hipToFootPositionVector);
      hipToFootUnitVector.normalize();
      /*
       * Shoulder to Foot position vector
       */
      robot.getShoulderJoint().getTranslationToWorld(shoulderToFootInWorld);
      shoulderJointPosition.set(shoulderToFootInWorld);
      shoulderToFootPositionVector.sub(footLocation, shoulderToFootInWorld);
      shoulderToFootUnitVector.set(shoulderToFootPositionVector);
      shoulderToFootUnitVector.normalize();
   }

   /**
    * Ground reaction force, reaction unit vector and surface normal
    */
   public void groundReactionForce()
   {
      /*
       * Ground reaction force and unit vector
       */
      Vector3D tempReactionForce = new Vector3D();
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
      Vector3D tempSurfaceNormal = new Vector3D();
      robot.footGroundContactPoint.getSurfaceNormal(tempSurfaceNormal);
      surfaceNormal.set(tempSurfaceNormal);
      surfaceNormal.normalize();
   }

   /**
    * CoM and CoM velocity
    */
   public void comAndComVelocity()
   {

      Point3D tempCOMPosition = new Point3D();
      Vector3D tempLinearMomentum = new Vector3D();
      Vector3D tempAngularMomentum = new Vector3D();
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
      comToFootError.sub(com, footLocation);
      comToFootError.setZ(0.0);
   }

   /**
    * CMP to CoM position vector
    * 
    * @param cmpFrom TODO
    */
   public void positionVectorFomCmpToCom(YoFramePoint3D cmpFrom)
   {
      cmpToComPositionVector.set(com);
      cmpToComPositionVector.sub(cmpFrom);
      cmpToComPositionVector.sub(com, cmpFrom);
   }

   /**
    * Foot to CoM position vector
    */
   public void positionVectorFomFootToCom(YoFramePoint3D actualFootPosition)
   {
      Vector3D tempFootToComPositionVector = new Vector3D();
      Point3D footLocationInWorld = new Point3D();
      footLocationInWorld.set(robot.computeFootLocation());
      tempFootToComPositionVector.set(com);
      footToComPositionVector.set(tempFootToComPositionVector);
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
      icpToFootError.sub(icp, footLocation);
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
      FramePoint2D tempCMP = new FramePoint2D(ReferenceFrame.getWorldFrame());

      tempCMP.set(reactionForce);
      if (reactionForce.getZ() != 0.0)
         tempCMP.scale(-com.getZ() / reactionForce.getZ());
      else
         tempCMP.set(0.0, 0.0);
      tempCMP.add(new FramePoint2D(com));
      this.cmpFromParameterizedReaction.set(tempCMP, 0.0);
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

      FramePoint3D bodyPoint = new FramePoint3D(bodyFrame);
      bodyPoint.changeFrame(ReferenceFrame.getWorldFrame());

      bodyLocation.set(bodyPoint);

      footLocation.set(robot.computeFootLocation());

      tempFootLocation.setIncludingFrame(footLocation);
      tempCoMLocation.setIncludingFrame(com);
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
      stateMachine.doActionAndTransition();
      balanceControl();
   }

   /**
    * balanceControl: Balances Tippy/Skippy based on q_d_hip and q_d_shoulder
    */
   private void balanceControl()
   {
      applyTorqueToHip(q_d_hip.getDoubleValue());
      applyTorqueToShoulder(q_d_shoulder.getDoubleValue());
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
      Vector3D linearMomentum = new Vector3D();
      robot.computeLinearMomentum(linearMomentum);

      // 1: projection vector
      Vector3D componentPerpendicular = new Vector3D(0, 1, -com.getY() / com.getZ());
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

   private Vector3D createVectorInDirectionOfHipJointAlongHip()
   {
      Vector3D rootJointCoordinates = new Vector3D();
      robot.getHipJointSkippy().getTranslationToWorld(rootJointCoordinates);
      Vector3D hipEndPointCoordinates = new Vector3D();
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
      Vector3D linearMomentum = new Vector3D();
      robot.computeLinearMomentum(linearMomentum);

      // 1: projection vector
      Vector3D componentPerpendicular = new Vector3D(1, 0, -com.getX() / com.getZ());
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

   private Vector3D createVectorInDirectionOfShoulderJointAlongShoulder()
   {
      Vector3D shoulderJointCoordinates = new Vector3D();
      robot.getShoulderJoint().getTranslationToWorld(shoulderJointCoordinates);
      Vector3D shoulderEndPointCoordinates = new Vector3D();
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

      Vector3D verticalVector = new Vector3D(0.0, 0.0, 1.0);
      Vector3D floatVector = createVectorInDirectionOfHipJointAlongHip();
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

      Vector3D horizontalVector = new Vector3D(1.0, 0.0, 0.0);
      Vector3D shoulderVector = createVectorInDirectionOfShoulderJointAlongShoulder();
      horizontalVector.setY(0);
      shoulderVector.setY(0);

      double cosineTheta = (horizontalVector.dot(shoulderVector) / (horizontalVector.length() * shoulderVector.length()));
      double angle = Math.abs(Math.acos(cosineTheta));

      Vector3D shoulderJointPosition = new Vector3D();
      joint.getTranslationToWorld(shoulderJointPosition);

      if (robot.getGroundContactPoints().get(2).getZ() < shoulderJointPosition.getZ())
         angle = angle * -1;

      double angleVel = robot.getShoulderJoint().getQDYoVariable().getDoubleValue();
      finale[0] = angle;
      finale[1] = angleVel;
      return finale;
   }

   /**
    * positionControl: positions Tippy model in whatever position desired (specified within method)
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
      RotationMatrix rotationMatrixForWorld = new RotationMatrix();
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

   private StateMachine<States, State> setUpStateMachines()
   {
      StateMachineFactory<States, State> factory = new StateMachineFactory<>(States.class);
      factory.setNamePrefix("stateMachine").setRegistry(registry).buildYoClock(robot.t);

      // states
      factory.addState(States.BALANCE, new BalanceState(skippyToDo.getEnumValue()));
      factory.addState(States.PREPARE, new PrepareState(skippyToDo.getEnumValue()));
      factory.addState(States.LEAN, new LeanState(skippyToDo.getEnumValue()));
      factory.addState(States.LIFTOFF, new LiftoffState(skippyToDo.getEnumValue()));
      factory.addState(States.REPOSITION, new RepositionState(skippyToDo.getEnumValue()));
      factory.addState(States.RECOVER, new RecoverState(skippyToDo.getEnumValue()));

      // transitions
      StateTransitionCondition balanceToPrepareTransitionCondition = new BalanceToPrepareTransitionCondition(skippyToDo.getEnumValue());
      StateTransitionCondition prepareToLeanTransitionCondition = new PrepareToLeanTransitionCondition(skippyToDo.getEnumValue());
      StateTransitionCondition leanToLiftoffTransitionCondition = new LeanToLiftoffTransitionCondition(skippyToDo.getEnumValue());
      StateTransitionCondition liftoffToRepositionTransitionCondition = new LiftoffToRepositionTransitionCondition(skippyToDo.getEnumValue());
      StateTransitionCondition repositionToRecoverTransitionCondition = new RepositionToRecoverTransitionCondition(skippyToDo.getEnumValue());
      StateTransitionCondition recoverToBalanceTransitionCondition = new RecoverToBalanceTransitionCondition(skippyToDo.getEnumValue());

      factory.addTransition(States.BALANCE, States.PREPARE, balanceToPrepareTransitionCondition);
      factory.addTransition(States.PREPARE, States.LEAN, prepareToLeanTransitionCondition);
      factory.addTransition(States.LEAN, States.LIFTOFF, leanToLiftoffTransitionCondition);
      factory.addTransition(States.LIFTOFF, States.REPOSITION, liftoffToRepositionTransitionCondition);
      factory.addTransition(States.REPOSITION, States.RECOVER, repositionToRecoverTransitionCondition);
      factory.addTransition(States.RECOVER, States.BALANCE, recoverToBalanceTransitionCondition);

      return factory.build(States.BALANCE);
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

      stateMachine.addStateChangedListener(stateMachinePanel);
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

      public boolean testCondition(double timeInState)
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            return timeInState < 4.01 && timeInState > 3.99;
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

      public boolean testCondition(double timeInState)
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            return timeInState < 7.01 && timeInState > 6.99;
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

      public boolean testCondition(double timeInState)
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            //            return timeInState > 0.0 && timeInState < 0.09;
            //            return true;
            return timeInState > 0.2;
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

      public boolean testCondition(double timeInState)
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            return timeInState < 0.36 && timeInState > 0.35;//
         }
         else
            return false;
      }
   }

   public class RepositionToRecoverTransitionCondition implements StateTransitionCondition
   {

      private final SkippyToDo direction;

      public RepositionToRecoverTransitionCondition(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public boolean testCondition(double timeInState)
      {
         return timeInState < 0.60 && timeInState > 0.59;
      }
   }

   public class RecoverToBalanceTransitionCondition implements StateTransitionCondition
   {

      private final SkippyToDo direction;

      public RecoverToBalanceTransitionCondition(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public boolean testCondition(double timeInState)
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            return timeInState < 4.01 && timeInState > 3.99;
         }
         return false;
      }
   }

   private class BalanceState implements State
   {

      private final SkippyToDo direction;

      public BalanceState(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public void doAction(double timeInState)
      {
         qd_hip.set(0.6);
      }

      public void onEntry()
      {
         qd_hip.set(0.6);
      }

      public void onExit()
      {

      }
   }

   private class PrepareState implements State
   {

      private final SkippyToDo direction;

      public PrepareState(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public void doAction(double timeInState)
      {

      }

      public void onEntry()
      {
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            qd_hip.set(1.6);
         }
      }

      public void onExit()
      {

      }
   }

   private class LeanState implements State
   {
      private final SkippyToDo direction;

      public LeanState(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public void doAction(double timeInState)
      {
      }

      public void onEntry()
      {
         System.out.println("LeanState");
         if (direction == SkippyToDo.JUMP_FORWARD)
         {
            System.out.println("LeanState: direction == SkippyToDo.JUMP_FORWARD");
            hipPlaneControlMode.set(SkippyPlaneControlMode.POSITION);
            qd_hip.set(1.4);
            balanceControl();
         }
      }

      public void onExit()
      {

      }
   }

   private class LiftoffState implements State
   {

      private final SkippyToDo direction;

      public LiftoffState(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public void doAction(double timeInState)
      {

      }

      public void onEntry()
      {
         q_d_hip.set(0.45);
      }

      public void onExit()
      {
      }
   }

   private class RepositionState implements State
   {

      private final SkippyToDo direction;

      public RepositionState(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public void doAction(double timeInState)
      {
      }

      public void onEntry()
      {
         qd_hip.set(-1.3);
      }

      public void onExit()
      {

      }
   }

   private class RecoverState implements State
   {

      private final SkippyToDo direction;

      public RecoverState(SkippyToDo direction)
      {
         this.direction = direction;
      }

      public void doAction(double timeInState)
      {

      }

      public void onEntry()
      {
         hipPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);
         shoulderPlaneControlMode.set(SkippyPlaneControlMode.BALANCE);

         qd_hip.set(-0.9);
         qd_shoulder.set(0.0);
         //robot.glueDownToGroundPoint.setForce(0.0, 0.0, -1450.0);
      }

      public void onExit()
      {

      }

   }

}
