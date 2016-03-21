package us.ihmc.quadrupedRobotics.controller;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.quadrupedRobotics.gait.TrotPair;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.trajectory.QuadrupedSwingTrajectoryGenerator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoTwist;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLine;

public class QuadrupedTrotWalkController extends QuadrupedController
{
   private static final double GRAVITY = 9.81;
   private static final double ESTIMATED_MASS = 63.9; // TODO PDControl this when z-vel=0
   private static final double ESTIMATED_ROTATIONAL_INERTIA = 5.0; // TODO PDControl this when z-vel=0
   private static final double COEFFICIENT_OF_FRICTION = 0.7;
   private final double dt;
   private final YoVariableRegistry registry = new YoVariableRegistry("TrotWalkController");
   private final QuadrupedReferenceFrames referenceFrames;
   private final SDFFullRobotModel fullRobotModel;
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;
   private final QuadrantDependentList<YoFramePoint> feetLocations = new QuadrantDependentList<YoFramePoint>();
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final YoFramePoint icp = new YoFramePoint("icp", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition icpViz = new YoGraphicPosition("icpViz", icp, 0.01, YoAppearance.DarkSlateBlue(), GraphicType.SQUARE);
   private final YoArtifactLine hindRightFrontLeftTrotLine;
   private final YoArtifactLine hindLeftFrontRightTrotLine;

   private final YoFramePoint centerOfPressure = new YoFramePoint("centerOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredCenterOfPressure = new YoFramePoint("desiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredICP = new YoFramePoint("desiredICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredICPFromCentroid = new YoFramePoint("desiredICPFromCentroid", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint frontMidPoint = new FramePoint();
   private final FramePoint hindMidPoint = new FramePoint();
   private final YoGraphicPosition centerOfPressureViz = new YoGraphicPosition("centerOfPressureViz", centerOfPressure, 0.01, YoAppearance.Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
   private final DoubleYoVariable desiredCoPRatioFrontToBack = new DoubleYoVariable("desiredCoPRatioFrontToBack", registry);
   private final DoubleYoVariable distanceDesiredCoPFromMidline = new DoubleYoVariable("distanceDesiredCoPFromMidline", registry);
   private final DoubleYoVariable halfStanceWidth = new DoubleYoVariable("halfStanceWidth", registry);
   private final DoubleYoVariable desiredCoPRatioCenterToSide = new DoubleYoVariable("desiredCoPRatioCenterToSide", registry);
   
   private final BooleanYoVariable enableTrot = new BooleanYoVariable("enableTrot", registry);
   private final DoubleYoVariable timeInTrot = new DoubleYoVariable("timeInTrot", registry);

   private final DoubleYoVariable kp_x = new DoubleYoVariable("k_x", registry);
   private final DoubleYoVariable kp_y = new DoubleYoVariable("k_y", registry);
   private final DoubleYoVariable kp_z = new DoubleYoVariable("k_z", registry);
   private final DoubleYoVariable kp_roll = new DoubleYoVariable("k_roll", registry);
   private final DoubleYoVariable kp_pitch = new DoubleYoVariable("k_pitch", registry);
   private final DoubleYoVariable kp_yaw = new DoubleYoVariable("k_yaw", registry);

   private final DoubleYoVariable kd_x = new DoubleYoVariable("b_x", registry);
   private final DoubleYoVariable kd_y = new DoubleYoVariable("b_y", registry);
   private final DoubleYoVariable kd_z = new DoubleYoVariable("b_z", registry);
   private final DoubleYoVariable kd_roll = new DoubleYoVariable("b_roll", registry);
   private final DoubleYoVariable kd_pitch = new DoubleYoVariable("b_pitch", registry);
   private final DoubleYoVariable kd_yaw = new DoubleYoVariable("b_yaw", registry);

   private final QuadrantDependentList<ArrayList<OneDoFJoint>> oneDofJoints = new QuadrantDependentList<>();
   private final HashMap<String, DoubleYoVariable> desiredTorques = new HashMap<>();

   private final IntegerYoVariable numberOfFeetInContact = new IntegerYoVariable("numberOfFeetInContact", registry);
   
   private final YoFramePoint centerOfMass = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector centerOfMassVelocity = new YoFrameVector("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfMassXYProjection = new YoFramePoint("centerOfMassXYProjection", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition centerOfMassViz = new YoGraphicPosition("centerOfMassViz", centerOfMassXYProjection, 0.02, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);
   
   private final QuadrupedSupportPolygon supportPolygon = new QuadrupedSupportPolygon();
   private final YoFramePoint centroid = new YoFramePoint("centroid", ReferenceFrame.getWorldFrame(), registry);
   private final QuadrantDependentList<YoFrameVector[]> basisForceVectors = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector[]> basisTorqueVectors = new QuadrantDependentList<>();
   private final QuadrantDependentList<double[]> rhoScalars = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> footToCoMVectors = new QuadrantDependentList<>();
   private final DenseMatrix64F bodyWrenchMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F basisMatrix = new DenseMatrix64F(6, 16);
   private final DenseMatrix64F rhoMatrix = new DenseMatrix64F(16, 1);
   private final SolvePseudoInverseSvd solver = new SolvePseudoInverseSvd();
   private final QuadrantDependentList<YoFrameVector> vmcFootForcesWorld = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> vmcFootForces = new QuadrantDependentList<>();
   private YoFramePose bodyPoseWorld;
   private final YoTwist bodyTwist;
   private final YoFramePose stancePose;
   private final YoFramePose desiredStancePose;
   private final YoFrameVector bodyLinearAcceleration;
   private final YoFrameVector bodyAngularAcceleration;
   private final YoFrameVector desiredBodyForce;
   private final YoFrameVector desiredBodyTorque;
   private final Wrench desiredBodyWrench;
   private final YoTwist desiredBodyTwist;
   private final SideDependentList<List<YoGraphicVector>> forceDistributionYoGraphicVectors = new SideDependentList<>();
   private final QuadrantDependentList<YoGraphicVector[]> basisForceYoGraphicVectors = new QuadrantDependentList<>();
   private final FramePoint footInBodyZUp = new FramePoint();
   private final FramePoint jointInBodyZUp = new FramePoint();
   private final FrameVector jointToFootVector = new FrameVector();
   private final FrameVector vmcRequestedTorqueFromJointXYZ = new FrameVector();
   private final FrameVector jointAxis = new FrameVector();
   
   private final QuadrantDependentList<QuadrupedSwingTrajectoryGenerator> swingTrajectoryGenerators;
   
   private final StateMachine<QuadrupedWalkingState> stateMachine;
   private final EnumYoVariable<QuadrupedWalkingState> nextState = new EnumYoVariable<QuadrupedWalkingState>("nextState", "", registry, QuadrupedWalkingState.class, false);
   private enum QuadrupedWalkingState
   {
      QuadSupport, RightTrotLine, LeftTrotLine;
   }

   public QuadrupedTrotWalkController(QuadrupedRobotParameters robotParameters, SDFFullRobotModel fullRobotModel, QuadrantDependentList<FootSwitchInterface> footSwitches, double DT,
         DoubleYoVariable yoTime, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(QuadrupedControllerState.TROT_WALK);
      this.fullRobotModel = fullRobotModel;
      this.footSwitches = footSwitches;
      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
      this.dt = DT;
      
      centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      QuadrupedJointNameMap quadrupedJointMap = robotParameters.getJointMap();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ArrayList<OneDoFJoint> jointsToControl = new ArrayList<OneDoFJoint>();
         String jointBeforeFootName = quadrupedJointMap.getJointBeforeFootName(robotQuadrant);
         OneDoFJoint oneDoFJointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         fullRobotModel.getOneDoFJointsFromRootToHere(oneDoFJointBeforeFoot, jointsToControl);
         oneDofJoints.set(robotQuadrant, jointsToControl);
         for (OneDoFJoint joint : jointsToControl)
         {
            desiredTorques.put(joint.getName(), new DoubleYoVariable(joint.getName() + "_tau_d", registry));
         }
      }
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         vmcFootForcesWorld.set(robotQuadrant, new YoFrameVector("vmcFootForcesWorld" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         vmcFootForces.set(robotQuadrant, new YoFrameVector("vmcFootForces" + robotQuadrant.getPascalCaseName(), referenceFrames.getCenterOfMassZUpFrame(), registry));
      }
      
      stateMachine = new StateMachine<QuadrupedWalkingState>("walkingState", "switchTime", QuadrupedWalkingState.class, yoTime, registry);
      setupStateMachine();

      yoGraphicsListRegistry.registerArtifact("icpViz", icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfMassViz", centerOfMassViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfPressureViz", centerOfPressureViz.createArtifact());
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
       
         YoFramePoint footPosition = new YoFramePoint(prefix, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition footPositionViz = new YoGraphicPosition(prefix + "FootPositionViz", footPosition, 0.02, YoAppearance.Color(robotQuadrant.getColor()), GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerArtifact("feet", footPositionViz.createArtifact());
         feetLocations.set(robotQuadrant, footPosition);
      }
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         basisForceVectors.set(robotQuadrant, new YoFrameVector[4]);
         basisTorqueVectors.set(robotQuadrant, new YoFrameVector[4]);
         rhoScalars.set(robotQuadrant, new double[4]);
         footToCoMVectors.set(robotQuadrant, new YoFrameVector("footToCoMVector" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         
         for (int i = 0; i < 4; i++)
         {
            basisForceVectors.get(robotQuadrant)[i] = new YoFrameVector("basisForceVector" + robotQuadrant.getPascalCaseName() + i, ReferenceFrame.getWorldFrame(), registry);
            basisTorqueVectors.get(robotQuadrant)[i] = new YoFrameVector("basisTorqueVector" + robotQuadrant.getPascalCaseName() + i, ReferenceFrame.getWorldFrame(), registry);
         }
      }
      
      for (TrotPair trotPair : TrotPair.values)
      {
         RobotSide side = trotPair.getFrontQuadrant().getSide();
         
         forceDistributionYoGraphicVectors.set(side, new ArrayList<YoGraphicVector>());
         forceDistributionYoGraphicVectors.get(side).add(new YoGraphicVector("frontFootForces" + side.getCamelCaseNameForMiddleOfExpression(),
                                                                              feetLocations.get(RobotQuadrant.getQuadrant(RobotEnd.FRONT, side)),
                                                                              vmcFootForcesWorld.get(RobotQuadrant.getQuadrant(RobotEnd.FRONT, side)), 0.007, YoAppearance.Yellow(), true, 0.01));
         forceDistributionYoGraphicVectors.get(side).add(new YoGraphicVector("hindFootForces" + side.getCamelCaseNameForMiddleOfExpression(),
                                                                              feetLocations.get(RobotQuadrant.getQuadrant(RobotEnd.HIND, side.getOppositeSide())),
                                                                              vmcFootForcesWorld.get(RobotQuadrant.getQuadrant(RobotEnd.HIND, side.getOppositeSide())), 0.007, YoAppearance.Yellow(), true, 0.01));
      }
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         basisForceYoGraphicVectors.set(robotQuadrant, new YoGraphicVector[4]);
         
         for (int i = 0; i < 4; i++)
         {
            basisForceYoGraphicVectors.get(robotQuadrant)[i] = new YoGraphicVector("basisForceYoGraphicVectors" + robotQuadrant.getPascalCaseName() + i,
                                                                                    feetLocations.get(robotQuadrant),
                                                                                    basisForceVectors.get(robotQuadrant)[i], 0.007, YoAppearance.Red(), true, 0.01);
            yoGraphicsListRegistry.registerYoGraphic("trotWalk", basisForceYoGraphicVectors.get(robotQuadrant)[i]);
         }
      }
      
      bodyPoseWorld = new YoFramePose("body", ReferenceFrame.getWorldFrame(), registry);
      desiredStancePose = new YoFramePose("desiredStance", ReferenceFrame.getWorldFrame(), registry);
      stancePose = new YoFramePose("stance", ReferenceFrame.getWorldFrame(), registry);
      bodyLinearAcceleration = new YoFrameVector("bodyLinearAcceleration", ReferenceFrame.getWorldFrame(), registry);
      bodyAngularAcceleration = new YoFrameVector("bodyAngularAcceleration", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyForce = new YoFrameVector("desiredBodyForce", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyTorque = new YoFrameVector("desiredBodyTorque", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyWrench = new Wrench(referenceFrames.getBodyZUpFrame(), ReferenceFrame.getWorldFrame());
      desiredBodyTwist = new YoTwist("desiredBodyTwist", referenceFrames.getBodyFrame(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), registry);
      bodyTwist = new YoTwist("bodyTwist", referenceFrames.getBodyFrame(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), registry);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         for (YoGraphicVector yoGraphicVector : forceDistributionYoGraphicVectors.get(robotSide))
         {
            yoGraphicsListRegistry.registerYoGraphic("trotWalk", yoGraphicVector);
         }
      }
      
      YoFramePoint hindRightFoot = feetLocations.get(RobotQuadrant.HIND_RIGHT);
      YoFramePoint hindLeftFoot = feetLocations.get(RobotQuadrant.HIND_LEFT);
      YoFramePoint frontLeftFoot = feetLocations.get(RobotQuadrant.FRONT_LEFT);
      YoFramePoint frontRightFoot = feetLocations.get(RobotQuadrant.FRONT_RIGHT);
      Color hindRightYoAppearance = RobotQuadrant.HIND_RIGHT.getColor();
      Color hindLeftYoAppearance = RobotQuadrant.HIND_LEFT.getColor();
      hindRightFrontLeftTrotLine = new YoArtifactLine("hindRightFrontLeftTrotLine", hindRightFoot, frontLeftFoot, hindRightYoAppearance);
      hindLeftFrontRightTrotLine = new YoArtifactLine("hindLeftFrontRightTrotLine", hindLeftFoot, frontRightFoot, hindLeftYoAppearance);
      
      swingTrajectoryGenerators = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
//         swingTrajectoryGenerators.set(robotQuadrant, new QuadrupedSwingTrajectoryGenerator(robotQuadrant, registry, yoGraphicsListRegistry, dt));
      }
      
      yoGraphicsListRegistry.registerArtifact("trotLines", hindRightFrontLeftTrotLine);
      yoGraphicsListRegistry.registerArtifact("trotLines", hindLeftFrontRightTrotLine);
      
      parentRegistry.addChild(registry);
   }

   private void setupStateMachine()
   {
      QuadSupportState quadSupportState = new QuadSupportState();
      RightTrotState rightTrotState = new RightTrotState();
      LeftTrotState leftTrotState = new LeftTrotState();

      StateTransitionCondition quadToRightTrotStateTransitionCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return enableTrot.getBooleanValue();
         }
      };
      StateTransition<QuadrupedWalkingState> quadToRightTrotStateTransition = new StateTransition<>(QuadrupedWalkingState.RightTrotLine, quadToRightTrotStateTransitionCondition);
      quadSupportState.addStateTransition(quadToRightTrotStateTransition);
      
      StateTransition<QuadrupedWalkingState> rightTrotToLeftTrotStateTransition = new StateTransition<QuadrupedWalkingState>(QuadrupedWalkingState.LeftTrotLine, timeInTrot);
      rightTrotState.addStateTransition(rightTrotToLeftTrotStateTransition);
      
      StateTransition<QuadrupedWalkingState> leftTrotToRightTrotStateTransition = new StateTransition<QuadrupedWalkingState>(QuadrupedWalkingState.RightTrotLine, timeInTrot);
      leftTrotState.addStateTransition(leftTrotToRightTrotStateTransition);
      
      stateMachine.addState(quadSupportState);
      stateMachine.addState(rightTrotState);
      stateMachine.addState(leftTrotState);
   }

   @Override
   public void doAction()
   {
      updateEstimates();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
   }

   private void updateEstimates()
   {
      //update frames
      referenceFrames.updateFrames();

      //update feet locations
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         feetLocations.get(robotQuadrant).setFromReferenceFrame(referenceFrames.getFootFrame(robotQuadrant));
      }
      
      for (RobotQuadrant robotQuadrant : supportPolygon.getSupportingQuadrantsInOrder())
      {
         supportPolygon.setFootstep(robotQuadrant, feetLocations.get(robotQuadrant).getFrameTuple());
      }
      
      bodyPoseWorld.setFromReferenceFrame(referenceFrames.getBodyFrame());

      // update centroid with hacky GC-free set
      FramePoint frameTuple = centroid.getFrameTuple();
      supportPolygon.getCentroid2d(frameTuple);
      centroid.set(frameTuple.getX(), frameTuple.getY(), frameTuple.getZ());
      
      bodyTwist.setLinearPart(fullRobotModel.getRootJoint().getLinearVelocityForReading());
      bodyTwist.setAngularPart(fullRobotModel.getRootJoint().getAngularVelocityForReading());

      // Stance is just the bodyPose in World minus the foot centroid
      stancePose.set(bodyPoseWorld);
      stancePose.getPosition().sub(centroid);

      // compute center of mass position and velocity
      centerOfMass.setFromReferenceFrame(referenceFrames.getCenterOfMassZUpFrame());
      centerOfMassXYProjection.set(centerOfMass.getReferenceFrame(), centerOfMass.getX(), centerOfMass.getY(), 0.0);
      
      centerOfMassJacobian.compute();
      
      FrameVector tempVector = centerOfMassVelocity.getFrameTuple();
      centerOfMassJacobian.getCenterOfMassVelocity(tempVector);
      tempVector.changeFrame(centerOfMassVelocity.getReferenceFrame());
      centerOfMassVelocity.set(tempVector.getX(), tempVector.getY(), tempVector.getZ());

      // compute instantaneous capture point
      double zDelta = centerOfMass.getZ() - centroid.getZ();
      double omega = Math.sqrt(GRAVITY / zDelta);
      icp.setX(centerOfMass.getX() + centerOfMassVelocity.getX() / omega);
      icp.setY(centerOfMass.getY() + centerOfMassVelocity.getY() / omega);
      icp.setZ(centroid.getZ());
      
      // update CoP
      double fzTotal = 0.0;
      FramePoint tempCenterOfPressure = centerOfPressure.getFrameTuple();
      tempCenterOfPressure.setToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         tempCenterOfPressure.add(feetLocations.get(robotQuadrant).getX() * vmcFootForces.get(robotQuadrant).getZ(), 0.0, 0.0);
         tempCenterOfPressure.add(0.0, feetLocations.get(robotQuadrant).getY() * vmcFootForces.get(robotQuadrant).getZ(), 0.0);
         tempCenterOfPressure.add(0.0, 0.0, feetLocations.get(robotQuadrant).getZ() * vmcFootForces.get(robotQuadrant).getZ());
         fzTotal += vmcFootForces.get(robotQuadrant).getZ();
      }
      if (fzTotal < 1e-14)
      {
         tempCenterOfPressure.set(Double.NaN, Double.NaN, Double.NaN);
      }
      else
      {
         tempCenterOfPressure.scale(1.0 / fzTotal);
      }
      centerOfPressure.set(tempCenterOfPressure.getX(), tempCenterOfPressure.getY(), tempCenterOfPressure.getZ());
      
      for (RobotQuadrant robotQuadrant : supportPolygon.getSupportingQuadrantsInOrder())
      {
         footToCoMVectors.get(robotQuadrant).set(centerOfMass);
         footToCoMVectors.get(robotQuadrant).sub(feetLocations.get(robotQuadrant));
         calculateBasisVectors(robotQuadrant, footToCoMVectors.get(robotQuadrant));
      }
   }

   private void computeFeetContactState()
   {
      if (footSwitches != null)
      {
         numberOfFeetInContact.set(0);
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (footSwitches.get(robotQuadrant).hasFootHitGround())
            {
               numberOfFeetInContact.increment();
            }
         }
      }
   }

   //Control X and Y using Center of Pressure on each trot line, SR and SL.
   private void doTrotControl()
   {
      double distanceFH = feetLocations.get(RobotQuadrant.HIND_LEFT).distance(feetLocations.get(RobotQuadrant.FRONT_LEFT));
      
      GeometryTools.averagePoints(feetLocations.get(RobotQuadrant.FRONT_LEFT).getFrameTuple(), feetLocations.get(RobotQuadrant.FRONT_RIGHT).getFrameTuple(), frontMidPoint);
      GeometryTools.averagePoints(feetLocations.get(RobotQuadrant.HIND_LEFT).getFrameTuple(), feetLocations.get(RobotQuadrant.HIND_RIGHT).getFrameTuple(), hindMidPoint);
      
      desiredICP.set(centroid);
      desiredICP.add(desiredICPFromCentroid.getX(), desiredICPFromCentroid.getY(), 0.0);
      
      desiredCenterOfPressure.set(icp);
      desiredCenterOfPressure.sub(desiredICP);
      desiredCenterOfPressure.scale(1.0); // K
      desiredCenterOfPressure.add(icp);
      
      desiredBodyTwist.setToZero();
      
      double distanceFrontToDesiredCoP = desiredCenterOfPressure.distance(frontMidPoint);
      distanceDesiredCoPFromMidline.set(GeometryTools.distanceFromPointToLine2d(desiredCenterOfPressure.getFrameTuple(), frontMidPoint, hindMidPoint));
      double distanceDesiredCoPToLeftSide = GeometryTools.distanceFromPointToLine2d(desiredCenterOfPressure.getFrameTuple(), feetLocations.get(RobotQuadrant.HIND_LEFT).getFrameTuple(), feetLocations.get(RobotQuadrant.FRONT_LEFT).getFrameTuple());
      double distanceDesiredCoPToRightSide = GeometryTools.distanceFromPointToLine2d(desiredCenterOfPressure.getFrameTuple(), feetLocations.get(RobotQuadrant.HIND_RIGHT).getFrameTuple(), feetLocations.get(RobotQuadrant.FRONT_RIGHT).getFrameTuple());
      
      halfStanceWidth.set(feetLocations.get(RobotQuadrant.FRONT_LEFT).distance(feetLocations.get(RobotQuadrant.FRONT_RIGHT)) / 2.0);
      
      if (halfStanceWidth.getDoubleValue() > 1e-7)
      {
         desiredCoPRatioCenterToSide.set(distanceDesiredCoPFromMidline.getDoubleValue() / halfStanceWidth.getDoubleValue());
      }
      
      if (distanceDesiredCoPToLeftSide >= distanceDesiredCoPToRightSide)
      {
         desiredCoPRatioCenterToSide.set(-desiredCoPRatioCenterToSide.getDoubleValue());
      }
      
      desiredCoPRatioFrontToBack.set(distanceFrontToDesiredCoP / distanceFH);
      
      bodyAngularAcceleration.setToZero();
      bodyAngularAcceleration.getYoX().add(kp_roll.getDoubleValue() * (desiredStancePose.getRoll() - stancePose.getRoll()));
      bodyAngularAcceleration.getYoX().add(kd_roll.getDoubleValue() * (desiredBodyTwist.getAngularPartX() - bodyTwist.getAngularPartX()));
      bodyAngularAcceleration.getYoY().add(kp_pitch.getDoubleValue() * (desiredStancePose.getPitch() - stancePose.getPitch()));
      bodyAngularAcceleration.getYoY().add(kd_pitch.getDoubleValue() * (desiredBodyTwist.getAngularPartY() - bodyTwist.getAngularPartY()));
      bodyAngularAcceleration.getYoZ().add(kp_yaw.getDoubleValue() * (desiredStancePose.getYaw() - stancePose.getYaw()));
      bodyAngularAcceleration.getYoZ().add(kd_yaw.getDoubleValue() * (desiredBodyTwist.getAngularPartZ() - bodyTwist.getAngularPartZ()));
      
      desiredBodyTorque.setToZero();
      desiredBodyTorque.add(bodyAngularAcceleration);
      desiredBodyTorque.scale(ESTIMATED_ROTATIONAL_INERTIA);
      
      bodyLinearAcceleration.setToZero();
      bodyLinearAcceleration.getYoX().add(kp_x.getDoubleValue() * (desiredStancePose.getX() - stancePose.getX()));
      bodyLinearAcceleration.getYoX().add(kd_x.getDoubleValue() * (desiredBodyTwist.getLinearPartX() - bodyTwist.getLinearPartX()));
      bodyLinearAcceleration.getYoY().add(kp_y.getDoubleValue() * (desiredStancePose.getY() - stancePose.getY()));
      bodyLinearAcceleration.getYoY().add(kd_y.getDoubleValue() * (desiredBodyTwist.getLinearPartY() - bodyTwist.getLinearPartY()));
      bodyLinearAcceleration.getYoZ().add(kp_z.getDoubleValue() * (desiredStancePose.getZ() - stancePose.getZ()));
      bodyLinearAcceleration.getYoZ().add(kd_z.getDoubleValue() * (desiredBodyTwist.getLinearPartZ() - bodyTwist.getLinearPartZ()));
      
      desiredBodyForce.setToZero();
      desiredBodyForce.add(0.0, 0.0, GRAVITY);
      desiredBodyForce.add(bodyLinearAcceleration);
      desiredBodyForce.scale(ESTIMATED_MASS);
      
      desiredBodyWrench.set(desiredBodyForce.getFrameTuple(), desiredBodyTorque.getFrameTuple());
   }

   private void calculateBasisVectors(RobotQuadrant robotQuadrant, YoFrameVector footToCoMVector)
   {
      basisForceVectors.get(robotQuadrant)[0].set(COEFFICIENT_OF_FRICTION, 0.0, 1.0);
      basisForceVectors.get(robotQuadrant)[0].normalize();
      basisForceVectors.get(robotQuadrant)[1].set(0.0, COEFFICIENT_OF_FRICTION, 1.0);
      basisForceVectors.get(robotQuadrant)[1].normalize();
      basisForceVectors.get(robotQuadrant)[2].set(-COEFFICIENT_OF_FRICTION, 0.0, 1.0);
      basisForceVectors.get(robotQuadrant)[2].normalize();
      basisForceVectors.get(robotQuadrant)[3].set(0.0, -COEFFICIENT_OF_FRICTION, 1.0);
      basisForceVectors.get(robotQuadrant)[3].normalize();
      
      basisTorqueVectors.get(robotQuadrant)[0].cross(footToCoMVector, basisForceVectors.get(robotQuadrant)[0]);
      basisTorqueVectors.get(robotQuadrant)[0].normalize();
      basisTorqueVectors.get(robotQuadrant)[1].cross(footToCoMVector, basisForceVectors.get(robotQuadrant)[1]);
      basisTorqueVectors.get(robotQuadrant)[1].normalize();
      basisTorqueVectors.get(robotQuadrant)[2].cross(footToCoMVector, basisForceVectors.get(robotQuadrant)[2]);
      basisTorqueVectors.get(robotQuadrant)[2].normalize();
      basisTorqueVectors.get(robotQuadrant)[3].cross(footToCoMVector, basisForceVectors.get(robotQuadrant)[3]);
      basisTorqueVectors.get(robotQuadrant)[3].normalize();
   }
   
   private void distributeForcesToFeet()
   {
      // New QP stuff here
      bodyWrenchMatrix.set(0, 0, desiredBodyWrench.getLinearPartX());
      bodyWrenchMatrix.set(1, 0, desiredBodyWrench.getLinearPartY());
      bodyWrenchMatrix.set(2, 0, desiredBodyWrench.getLinearPartZ());
      bodyWrenchMatrix.set(3, 0, desiredBodyWrench.getAngularPartX());
      bodyWrenchMatrix.set(4, 0, desiredBodyWrench.getAngularPartY());
      bodyWrenchMatrix.set(5, 0, desiredBodyWrench.getAngularPartZ());
      
      basisMatrix.reshape(6, supportPolygon.size() * 4);
      rhoMatrix.reshape(supportPolygon.size() * 4, 1);
      
      for (int quadrantIndex = 0; quadrantIndex < supportPolygon.getSupportingQuadrantsInOrder().length; quadrantIndex++)
      {
         RobotQuadrant robotQuadrant = supportPolygon.getSupportingQuadrantsInOrder()[quadrantIndex];
         for (int basisIndex = 0; basisIndex < 4; basisIndex++)
         {
            basisMatrix.set(0, quadrantIndex * 4 + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getX());
            basisMatrix.set(1, quadrantIndex * 4 + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getY());
            basisMatrix.set(2, quadrantIndex * 4 + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getZ());
            basisMatrix.set(3, quadrantIndex * 4 + basisIndex, basisTorqueVectors.get(robotQuadrant)[basisIndex].getX());
            basisMatrix.set(4, quadrantIndex * 4 + basisIndex, basisTorqueVectors.get(robotQuadrant)[basisIndex].getY());
            basisMatrix.set(5, quadrantIndex * 4 + basisIndex, basisTorqueVectors.get(robotQuadrant)[basisIndex].getZ());
         }
      }
      
      solver.setA(basisMatrix);
      solver.solve(bodyWrenchMatrix, rhoMatrix);
      
      for (int quadrantIndex = 0; quadrantIndex < supportPolygon.getSupportingQuadrantsInOrder().length; quadrantIndex++)
      {
         RobotQuadrant robotQuadrant = supportPolygon.getSupportingQuadrantsInOrder()[quadrantIndex];
         for (int basisIndex = 0; basisIndex < 4; basisIndex++)
         {
            rhoScalars.get(robotQuadrant)[basisIndex] = rhoMatrix.get(quadrantIndex * 4 + basisIndex, 0);
         }
      }
      
      clearFootForces();
      for (RobotQuadrant robotQuadrant : supportPolygon.getSupportingQuadrantsInOrder())
      {
         for (int basisIndex = 0; basisIndex < 4; basisIndex++)
         {
            basisForceVectors.get(robotQuadrant)[basisIndex].scale(rhoScalars.get(robotQuadrant)[basisIndex]);
            vmcFootForcesWorld.get(robotQuadrant).add(basisForceVectors.get(robotQuadrant)[basisIndex]);
            FrameVector frameTupleForFrameChange = vmcFootForcesWorld.get(robotQuadrant).getFrameTuple();
            frameTupleForFrameChange.changeFrame(referenceFrames.getCenterOfMassZUpFrame());
            vmcFootForces.get(robotQuadrant).set(frameTupleForFrameChange);
         }
      }
   }

   private void computeStanceJacobians()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         computeStanceJacobianForLeg(robotQuadrant);
      }
   }
   
   private void applyPositionControlledSwingTorques(boolean hindLeft, boolean hindRight, boolean frontLeft, boolean frontRight)
   {
      
   }

   private void computeStanceJacobianForLeg(RobotQuadrant robotQuadrant)
   {
      getFootInBodyZUpFrame(robotQuadrant, footInBodyZUp);

      for (int i = 0; i < oneDofJoints.get(robotQuadrant).size(); i++)
      {
         OneDoFJoint oneDoFJoint = oneDofJoints.get(robotQuadrant).get(i);
         ReferenceFrame jointFrame = oneDoFJoint.getFrameBeforeJoint();
         
         jointInBodyZUp.setToZero(jointFrame);
         jointInBodyZUp.changeFrame(referenceFrames.getCenterOfMassZUpFrame());

         jointToFootVector.setIncludingFrame(footInBodyZUp);
         jointToFootVector.sub(jointInBodyZUp);

         vmcRequestedTorqueFromJointXYZ.setToZero(referenceFrames.getCenterOfMassZUpFrame());
         vmcRequestedTorqueFromJointXYZ.cross(jointToFootVector, vmcFootForces.get(robotQuadrant).getFrameTuple());
         vmcRequestedTorqueFromJointXYZ.changeFrame(jointFrame);

         oneDoFJoint.getJointAxis(jointAxis);
         double torque = jointAxis.dot(vmcRequestedTorqueFromJointXYZ);
         
         desiredTorques.get(oneDoFJoint.getName()).set(-torque);
         oneDoFJoint.setTau(-torque);
      }
   }

   private void getFootInBodyZUpFrame(RobotQuadrant footQuadrant, FramePoint framePointToPack)
   {
      ReferenceFrame footFrame = referenceFrames.getFootFrame(footQuadrant);
      framePointToPack.setToZero(footFrame);
      framePointToPack.changeFrame(referenceFrames.getCenterOfMassZUpFrame());
   }

   private void clearFootForces()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         vmcFootForces.get(robotQuadrant).setToZero();
         vmcFootForcesWorld.get(robotQuadrant).setToZero();
      }
   }

   private class QuadSupportState extends State<QuadrupedWalkingState>
   {
      public QuadSupportState()
      {
         super(QuadrupedWalkingState.QuadSupport);
      }

      @Override
      public void doAction()
      {
         computeFeetContactState();

         doTrotControl();
         distributeForcesToFeet();
         
         computeStanceJacobians();
      }

      @Override
      public void doTransitionIntoAction()
      {
         supportPolygon.clear();
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            supportPolygon.setFootstep(robotQuadrant, feetLocations.get(robotQuadrant).getFrameTuple());
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         
      }
   }
   
   private class RightTrotState extends State<QuadrupedWalkingState>
   {
      public RightTrotState()
      {
         super(QuadrupedWalkingState.RightTrotLine);
      }

      @Override
      public void doAction()
      {
         computeFeetContactState();

         doTrotControl();
         distributeForcesToFeet();

         computeStanceJacobians();
      }

      @Override
      public void doTransitionIntoAction()
      {
         supportPolygon.clear();
         for (RobotQuadrant robotQuadrant : TrotPair.TROT_RIGHT.quadrants())
         {
            supportPolygon.setFootstep(robotQuadrant, feetLocations.get(robotQuadrant).getFrameTuple());
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         
      }
   }
   
   private class LeftTrotState extends State<QuadrupedWalkingState>
   {
      public LeftTrotState()
      {
         super(QuadrupedWalkingState.LeftTrotLine);
      }

      @Override
      public void doAction()
      {
         computeFeetContactState();

         doTrotControl();
         distributeForcesToFeet();

         computeStanceJacobians();
      }

      @Override
      public void doTransitionIntoAction()
      {
         supportPolygon.clear();
         for (RobotQuadrant robotQuadrant : TrotPair.TROT_LEFT.quadrants())
         {
            supportPolygon.setFootstep(robotQuadrant, feetLocations.get(robotQuadrant).getFrameTuple());
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {
         
      }
   }

   public String getName()
   {
      return "TrotWalkController";
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void initialize()
   {
      supportPolygon.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygon.setFootstep(robotQuadrant, feetLocations.get(robotQuadrant).getFrameTuple());
      }
      
      updateEstimates();
      
      desiredStancePose.set(stancePose);
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ArrayList<OneDoFJoint> legJoints = oneDofJoints.get(robotQuadrant);
         for (int i = 0; i < legJoints.size(); i++)
         {
            legJoints.get(i).setUnderPositionControl(false);
         }
      }
      
      enableTrot.set(false);
      timeInTrot.set(0.01);

      kp_x.set(50.0);
      kd_x.set(10.0);

      kp_y.set(50.0);
      kd_y.set(10.0);

      kp_roll.set(-700.0);
      kd_roll.set(-20.0);

      kp_pitch.set(-700.0);
      kd_pitch.set(-30.0);

      kp_yaw.set(-700.0);
      kd_yaw.set(-100.0);

      kp_z.set(300.0);
      kd_z.set(25.0);
   }

   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doTransitionIntoAction()
   {
      initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {

   }

   @Override
   public RobotMotionStatus getMotionStatus()
   {
      return RobotMotionStatus.IN_MOTION;
   }
}
