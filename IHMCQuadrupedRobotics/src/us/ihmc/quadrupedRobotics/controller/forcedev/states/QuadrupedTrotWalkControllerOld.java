package us.ihmc.quadrupedRobotics.controller.forcedev.states;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.planning.gait.TrotPair;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
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

/**
 * This controller is to allow the other trot walk controller to only include latest active developments
 * while preserving the compilation of these past features.
 * 
 * @author Duncan
 */
public class QuadrupedTrotWalkControllerOld
{
   private static final double GRAVITY = 9.81;
   private static final double ESTIMATED_MASS = 63.9; // TODO PDControl this when z-vel=0
   private static final double ESTIMATED_ROTATIONAL_INERTIA = 5.0; // TODO PDControl this when z-vel=0
   private static final double COEFFICIENT_OF_FRICTION = 0.7;
   private final double dt;
   private final YoVariableRegistry registry = new YoVariableRegistry("TrotWalkController");
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final QuadrupedReferenceFrames referenceFrames;
   private final SDFFullRobotModel fullRobotModel;
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;
   private final QuadrantDependentList<YoFramePoint> feetLocations = new QuadrantDependentList<YoFramePoint>();
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final YoFrameVector centerOfMassVelocity = new YoFrameVector("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint coMPosition = new FramePoint();
   private final YoFramePoint centerOfMassPosition = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfMassPositionXYProjection = new YoFramePoint("centerOfMassXYProjection", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition centerOfMassViz = new YoGraphicPosition("centerOfMassViz", centerOfMassPosition, 0.02, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);

   private final YoFramePoint icp = new YoFramePoint("icp", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition icpViz = new YoGraphicPosition("icpViz", icp, 0.01, YoAppearance.DarkSlateBlue(), GraphicType.SQUARE);
   private final YoGraphicPosition copHindRightFrontLeftTrotLineViz = new YoGraphicPosition("copHindRightFrontLeftTrotLineViz", icp, 0.01, YoAppearance.Purple());
   private final YoGraphicPosition cophindLeftFrontRightTrotLineViz = new YoGraphicPosition("cophindLeftFrontRightTrotLineViz", icp, 0.01, YoAppearance.Purple());
   private final YoArtifactLine hindRightFrontLeftTrotLine;
   private final YoArtifactLine hindLeftFrontRightTrotLine;

   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final FramePose bodyPoseWorld = new FramePose(worldFrame);

   /** Trot alpha is the percentage of support given to the right trot pair vs. the left trot pair */
   private final DoubleYoVariable trotAlpha = new DoubleYoVariable("quadAlpha", registry);

   private final FramePoint copFramePoint = new FramePoint();
   private final YoFramePoint centerOfPressure = new YoFramePoint("centerOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredCenterOfPressure = new YoFramePoint("desiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredICP = new YoFramePoint("desiredICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredICPFromCentroid = new YoFramePoint("desiredICPFromCentroid", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfPressureSRLocation = new YoFramePoint("centerOfPressureSRLocation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfPressureSLLocation = new YoFramePoint("centerOfPressureSLLocation", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint frontMidPoint = new FramePoint();
   private final FramePoint hindMidPoint = new FramePoint();
   private final YoGraphicPosition centerOfPressureViz = new YoGraphicPosition("centerOfPressureViz", centerOfPressure, 0.01, YoAppearance.Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
   private final YoGraphicPosition trotBetaRightViz = new YoGraphicPosition("trotBetaRightViz", centerOfPressureSRLocation, 0.01, YoAppearance.Purple(), GraphicType.BALL_WITH_ROTATED_CROSS);
   private final YoGraphicPosition trotBetaLeftViz = new YoGraphicPosition("trotBetaLeftViz", centerOfPressureSLLocation, 0.01, YoAppearance.Crimson(), GraphicType.BALL_WITH_ROTATED_CROSS);
   /** Trot beta is the percentage of force on the front foot vs the hind foot on a trot line */
   private final SideDependentList<DoubleYoVariable> trotBetas = new SideDependentList<>();
   private final DoubleYoVariable desiredCoPRatioFrontToBack = new DoubleYoVariable("desiredCoPRatioFrontToBack", registry);
   private final DoubleYoVariable distanceDesiredCoPFromMidline = new DoubleYoVariable("distanceDesiredCoPFromMidline", registry);
   private final DoubleYoVariable halfStanceWidth = new DoubleYoVariable("halfStanceWidth", registry);
   private final DoubleYoVariable desiredCoPRatioCenterToSide = new DoubleYoVariable("desiredCoPRatioCenterToSide", registry);
   private final DoubleYoVariable trotBetaRightBeforeAdjustments = new DoubleYoVariable("trotBetaRightBeforeAdjustments", registry);
   private final DoubleYoVariable trotBetaLeftBeforeAdjustments = new DoubleYoVariable("trotBetaLeftBeforeAdjustments", registry);
   
   private final BooleanYoVariable enableTrot = new BooleanYoVariable("enableTrot", registry);
   private final DoubleYoVariable timeInTrot = new DoubleYoVariable("timeInTrot", registry);

   private final DoubleYoVariable k_x = new DoubleYoVariable("k_x", registry);
   private final DoubleYoVariable k_y = new DoubleYoVariable("k_y", registry);
   private final DoubleYoVariable k_z = new DoubleYoVariable("k_z", registry);
   private final DoubleYoVariable k_roll = new DoubleYoVariable("k_roll", registry);
   private final DoubleYoVariable k_pitch = new DoubleYoVariable("k_pitch", registry);
   private final DoubleYoVariable k_yaw = new DoubleYoVariable("k_yaw", registry);

   private final DoubleYoVariable b_x = new DoubleYoVariable("b_x", registry);
   private final DoubleYoVariable b_y = new DoubleYoVariable("b_y", registry);
   private final DoubleYoVariable b_z = new DoubleYoVariable("b_z", registry);
   private final DoubleYoVariable b_roll = new DoubleYoVariable("b_roll", registry);
   private final DoubleYoVariable b_pitch = new DoubleYoVariable("b_pitch", registry);
   private final DoubleYoVariable b_yaw = new DoubleYoVariable("b_yaw", registry);

   private final DoubleYoVariable fz_limit = new DoubleYoVariable("fz_limit", registry);

   private final DoubleYoVariable q_roll = new DoubleYoVariable("q_roll", registry);
   private final DoubleYoVariable q_pitch = new DoubleYoVariable("q_pitch", registry);
   private final DoubleYoVariable q_yaw = new DoubleYoVariable("q_yaw", registry);

//   private final DoubleYoVariable q_d_roll = new DoubleYoVariable("q_d_roll", registry);
//   private final DoubleYoVariable q_d_pitch = new DoubleYoVariable("q_d_pitch", registry);
//   private final DoubleYoVariable q_d_yaw = new DoubleYoVariable("q_d_yaw", registry);


   private final QuadrantDependentList<ArrayList<OneDoFJoint>> oneDofJoints = new QuadrantDependentList<>();
   private final HashMap<String, DoubleYoVariable> desiredTorques = new HashMap<>();

   private final IntegerYoVariable numberOfFeetInContact = new IntegerYoVariable("numberOfFeetInContact", registry);
   
   private final QuadrupedSupportPolygon supportPolygon = new QuadrupedSupportPolygon();
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
   private YoFrameVector bodyAngularVelocity;
   private YoFrameVector bodyLinearVelocity;
   private final YoFramePoint stancePosition;
   private final YoFramePoint bodyPosition;
   private final FramePoint hindFootPosition = new FramePoint();
   private final FramePoint frontFootPosition = new FramePoint();
   private final YoFramePoint desiredStancePosition;
   private final YoFrameOrientation desiredBodyOrientation;
   private final YoFrameVector bodyLinearAcceleration;
   private final YoFrameVector bodyAngularAcceleration;
   private final YoFrameVector desiredBodyForce;
   private final YoFrameVector desiredBodyTorque;
   private final Wrench desiredBodyWrench;
   private final YoFrameVector desiredBodyLinearVelocity = new YoFrameVector("desiredBodyLinearVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredBodyAngularVelocity = new YoFrameVector("desiredBodyAngularVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final Twist desiredBodyTwist;
   private final SideDependentList<Wrench> desiredBodyWrenches = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> hindToBodyVectors = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> frontToBodyVectors = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> frontFootBodyMinusHindFootBody = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> unknownTorqueQuantitys = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> unknownTorqueQuantity2s = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> desiredBodyForceVectors = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> desiredBodyTorqueVectors = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> frontFootForceVectors = new SideDependentList<>();
   private final SideDependentList<YoFrameVector> hindFootForceVectors = new SideDependentList<>();
   private final SideDependentList<List<YoGraphicVector>> forceDistributionYoGraphicVectors = new SideDependentList<>();
   private final QuadrantDependentList<YoGraphicVector[]> basisForceYoGraphicVectors = new QuadrantDependentList<>();
   private final FramePoint footInBodyZUp = new FramePoint();
   private final FramePoint jointInBodyZUp = new FramePoint();
   private final FrameVector jointToFootVector = new FrameVector();
   private final FrameVector vmcRequestedTorqueFromJointXYZ = new FrameVector();
   private final FrameVector jointAxis = new FrameVector();
   
   private boolean hasInitializedInheritedYoVariables = false;
   
   private final StateMachine<QuadrupedWalkingState> stateMachine;
   private final EnumYoVariable<QuadrupedWalkingState> nextState = new EnumYoVariable<QuadrupedWalkingState>("nextState", "", registry, QuadrupedWalkingState.class, false);
   private enum QuadrupedWalkingState
   {
      QuadSupport, RightTrotLine, LeftTrotLine;
   }

   public QuadrupedTrotWalkControllerOld(QuadrupedPhysicalProperties physicalProperties, SDFFullQuadrupedRobotModel fullRobotModel, QuadrantDependentList<FootSwitchInterface> footSwitches, double DT,
         DoubleYoVariable yoTime, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.footSwitches = footSwitches;
      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      this.centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      this.dt = DT;
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ArrayList<OneDoFJoint> jointsToControl = new ArrayList<OneDoFJoint>();
         OneDoFJoint oneDoFJointBeforeFoot = fullRobotModel.getOneDoFJointBeforeFoot(robotQuadrant);
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
      yoGraphicsListRegistry.registerArtifact("centerOfPressureSRViz", trotBetaRightViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfPressureSLViz", trotBetaLeftViz.createArtifact());
      
      bodyPosition = new YoFramePoint("bodyPosition", ReferenceFrame.getWorldFrame(), registry);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
       
         YoFramePoint footPosition = new YoFramePoint(prefix, worldFrame, registry);
         YoGraphicPosition footPositionViz = new YoGraphicPosition(prefix + "FootPositionViz", footPosition, 0.02, YoAppearance.Color(robotQuadrant.getColor()), GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerArtifact("feet", footPositionViz.createArtifact());
         feetLocations.set(robotQuadrant, footPosition);
      }
      
      for (TrotPair trotPair : TrotPair.values)
      {
         trotBetas.set(trotPair.getSide(), new DoubleYoVariable("trotBeta" + trotPair.getCamelCaseName(), parentRegistry));
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
         hindToBodyVectors.set(side, new YoFrameVector("hindToBodyVector" + side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
         frontToBodyVectors.set(side, new YoFrameVector("frontToBodyVector" + side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
         frontFootBodyMinusHindFootBody.set(side, new YoFrameVector("frontFootBodyMinusHindFootBody" + side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
         unknownTorqueQuantitys.set(side, new YoFrameVector("unknownTorqueQuantity" + side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
         unknownTorqueQuantity2s.set(side, new YoFrameVector("unknownTorqueQuantity2" + side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
         desiredBodyForceVectors.set(side, new YoFrameVector("desiredBodyForceVector" + side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
         desiredBodyTorqueVectors.set(side, new YoFrameVector("desiredBodyTorqueVector" + side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
         frontFootForceVectors.set(side, new YoFrameVector("frontFootForceVector" + side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
         hindFootForceVectors.set(side, new YoFrameVector("hindFootForceVector" + side.getCamelCaseNameForMiddleOfExpression(), ReferenceFrame.getWorldFrame(), registry));
         
         forceDistributionYoGraphicVectors.set(side, new ArrayList<YoGraphicVector>());
         forceDistributionYoGraphicVectors.get(side).add(new YoGraphicVector("hindToBodyVectors" + side.getCamelCaseNameForMiddleOfExpression(),
                                                                              feetLocations.get(RobotQuadrant.getQuadrant(RobotEnd.HIND, side.getOppositeSide())),
                                                                              hindToBodyVectors.get(side), 1.0, YoAppearance.Magenta(), true, 0.01));
         forceDistributionYoGraphicVectors.get(side).add(new YoGraphicVector("frontToBodyVectors" + side.getCamelCaseNameForMiddleOfExpression(),
                                                                              feetLocations.get(RobotQuadrant.getQuadrant(RobotEnd.FRONT, side)),
                                                                              frontToBodyVectors.get(side), 1.0, YoAppearance.Magenta(), true, 0.01));
         forceDistributionYoGraphicVectors.get(side).add(new YoGraphicVector("frontFootBodyMinusHindFootBody" + side.getCamelCaseNameForMiddleOfExpression(),
                                                                              feetLocations.get(RobotQuadrant.getQuadrant(RobotEnd.FRONT, side)),
                                                                              frontFootBodyMinusHindFootBody.get(side), 1.0, YoAppearance.Orange(), true, 0.01));
         forceDistributionYoGraphicVectors.get(side).add(new YoGraphicVector("desiredBodyForceVectors" + side.getCamelCaseNameForMiddleOfExpression(),
                                                                              centerOfMassPosition,
                                                                              desiredBodyForceVectors.get(side), 0.01, YoAppearance.Blue(), true, 0.01));
         forceDistributionYoGraphicVectors.get(side).add(new YoGraphicVector("desiredBodyTorqueVectors" + side.getCamelCaseNameForMiddleOfExpression(),
                                                                              centerOfMassPosition,
                                                                              desiredBodyTorqueVectors.get(side), 0.01, YoAppearance.DarkRed(), true, 0.01));
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
      
      desiredStancePosition = new YoFramePoint("desiredStancePosition", ReferenceFrame.getWorldFrame(), registry);
      stancePosition = new YoFramePoint("stancePosition", ReferenceFrame.getWorldFrame(), registry);
      bodyLinearAcceleration = new YoFrameVector("bodyLinearAcceleration", ReferenceFrame.getWorldFrame(), registry);
      bodyAngularAcceleration = new YoFrameVector("bodyAngularAcceleration", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyOrientation = new YoFrameOrientation("desiredBodyOrientation", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyForce = new YoFrameVector("desiredBodyForce", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyTorque = new YoFrameVector("desiredBodyTorque", ReferenceFrame.getWorldFrame(), registry);
      desiredBodyWrench = new Wrench(referenceFrames.getBodyZUpFrame(), ReferenceFrame.getWorldFrame());
      desiredBodyTwist = new Twist(referenceFrames.getBodyFrame(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      for (TrotPair trotPair : TrotPair.values)
      {
         desiredBodyWrenches.set(trotPair.getSide(), new Wrench(referenceFrames.getBodyZUpFrame(), ReferenceFrame.getWorldFrame()));
      }
      
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
      
      yoGraphicsListRegistry.registerArtifact("trotLines", hindRightFrontLeftTrotLine);
      yoGraphicsListRegistry.registerArtifact("trotLines", hindLeftFrontRightTrotLine);
      
      initialize();
      parentRegistry.addChild(registry);
   }
   
   private void initializeInheritedVariables()
   {
      if (!hasInitializedInheritedYoVariables)
      {
         hasInitializedInheritedYoVariables = true;
         
         YoVariableRegistry rootRegistry = registry;
         while (rootRegistry.getParent() != null)
         {
            rootRegistry = rootRegistry.getParent();
         }
         
         DoubleYoVariable bodyVelocityX = (DoubleYoVariable) rootRegistry.getVariable("root.babyBeastSimple", "qd_x");
         DoubleYoVariable bodyVelocityY = (DoubleYoVariable) rootRegistry.getVariable("root.babyBeastSimple", "qd_y");
         DoubleYoVariable bodyVelocityZ = (DoubleYoVariable) rootRegistry.getVariable("root.babyBeastSimple", "qd_z");
         bodyLinearVelocity = new YoFrameVector(bodyVelocityX, bodyVelocityY, bodyVelocityZ, ReferenceFrame.getWorldFrame());
         DoubleYoVariable bodyVelocityWX = (DoubleYoVariable) rootRegistry.getVariable("root.babyBeastSimple", "qd_wx");
         DoubleYoVariable bodyVelocityWY = (DoubleYoVariable) rootRegistry.getVariable("root.babyBeastSimple", "qd_wy");
         DoubleYoVariable bodyVelocityWZ = (DoubleYoVariable) rootRegistry.getVariable("root.babyBeastSimple", "qd_wz");
         bodyAngularVelocity = new YoFrameVector(bodyVelocityWX, bodyVelocityWY, bodyVelocityWZ, ReferenceFrame.getWorldFrame());
      }
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

   public void doAction()
   {
      initializeInheritedVariables();
      updateEstimates();

      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
   }

   private final FramePoint footLocation = new FramePoint();
   private final FramePoint centroid = new FramePoint();
   private final FrameVector comVelocity = new FrameVector();

   private void updateEstimates()
   {
      //update frames
      referenceFrames.updateFrames();
      bodyPoseWorld.setToZero(referenceFrames.getBodyFrame());
      bodyPoseWorld.changeFrame(worldFrame);
      bodyPosition.set(bodyPoseWorld.getX(), bodyPoseWorld.getY(), bodyPoseWorld.getZ());

      //update orientation qs
      q_roll.set(bodyPoseWorld.getRoll());
      q_pitch.set(bodyPoseWorld.getPitch());
      q_yaw.set(bodyPoseWorld.getYaw());

      //update feet locations
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);
         footLocation.setToZero(footFrame);
         footLocation.changeFrame(ReferenceFrame.getWorldFrame());
         fourFootSupportPolygon.setFootstep(robotQuadrant, footLocation);
         feetLocations.get(robotQuadrant).set(footLocation);
      }

      //update centroid
      fourFootSupportPolygon.getCentroid(centroid);

      //update relative offset from center of feet to center of body
      double body_x = bodyPoseWorld.getX() - centroid.getX();
      double body_y = bodyPoseWorld.getY() - centroid.getY();
      double yaw = bodyPoseWorld.getYaw();

      stancePosition.setX(Math.cos(yaw) * body_x + Math.sin(yaw) * body_y);
      stancePosition.setY(-Math.sin(yaw) * body_x + Math.cos(yaw) * body_y);

      // compute center of mass position and velocity
      coMPosition.setToZero(referenceFrames.getCenterOfMassZUpFrame());
      coMPosition.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassPosition.set(coMPosition);
      centerOfMassPositionXYProjection.set(centerOfMassPosition.getReferenceFrame(), centerOfMassPosition.getX(), centerOfMassPosition.getY(), 0.0);
      centerOfMassJacobian.compute();
      centerOfMassJacobian.getCenterOfMassVelocity(comVelocity);
      comVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassVelocity.set(comVelocity);

      // compute instantaneous capture point
      double lowestFootZ = fourFootSupportPolygon.getLowestFootstepZHeight();
      double zDelta = coMPosition.getZ() - lowestFootZ;
      double omega = Math.sqrt(GRAVITY / zDelta);
      icp.setX(coMPosition.getX() + centerOfMassVelocity.getX() / omega);
      icp.setY(coMPosition.getY() + centerOfMassVelocity.getY() / omega);
      icp.setZ(lowestFootZ);
      
      //update CoP
      double fzTotal = 0.0;
      copFramePoint.setToZero(worldFrame);
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoFramePoint foot = feetLocations.get(robotQuadrant);
         YoFrameVector legForce = vmcFootForces.get(robotQuadrant);

         double fz = legForce.getZ();
         fzTotal += fz;

         double x = foot.getX() * fz;
         double y = foot.getY() * fz;
         double z = foot.getZ() * fz;
         
         copFramePoint.add(x, y, z);
      }

      if (fzTotal < 1e-14)
      {
         copFramePoint.set(Double.NaN, Double.NaN, Double.NaN);
      }
      else
      {
         copFramePoint.scale(1.0 / fzTotal);
      }
      centerOfPressure.set(copFramePoint);
      
      for (RobotQuadrant robotQuadrant : supportPolygon.getSupportingQuadrantsInOrder())
      {
         supportPolygon.setFootstep(robotQuadrant, feetLocations.get(robotQuadrant).getFrameTuple());
         footToCoMVectors.get(robotQuadrant).set(coMPosition);
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
      
      desiredBodyLinearVelocity.setToZero();
      desiredBodyAngularVelocity.setToZero();
      desiredBodyTwist.setLinearPart(desiredBodyLinearVelocity.getFrameTuple());
      desiredBodyTwist.setAngularPart(desiredBodyAngularVelocity.getFrameTuple());
      
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
      
      trotBetaRightBeforeAdjustments.set(desiredCoPRatioFrontToBack.getDoubleValue() + desiredCoPRatioCenterToSide.getDoubleValue());
      trotBetaLeftBeforeAdjustments.set(desiredCoPRatioFrontToBack.getDoubleValue() - desiredCoPRatioCenterToSide.getDoubleValue());
      
      trotBetas.get(TrotPair.TROT_RIGHT.getSide()).set(trotBetaRightBeforeAdjustments.getDoubleValue());
      trotBetas.get(TrotPair.TROT_LEFT.getSide()).set(trotBetaLeftBeforeAdjustments.getDoubleValue());
      
      centerOfPressureSRLocation.set(feetLocations.get(RobotQuadrant.HIND_LEFT));
      centerOfPressureSRLocation.sub(feetLocations.get(RobotQuadrant.FRONT_RIGHT));
      centerOfPressureSRLocation.scale(trotBetas.get(TrotPair.TROT_RIGHT.getSide()).getDoubleValue());
      centerOfPressureSRLocation.add(feetLocations.get(RobotQuadrant.FRONT_RIGHT));
      
      centerOfPressureSLLocation.set(feetLocations.get(RobotQuadrant.HIND_RIGHT));
      centerOfPressureSLLocation.sub(feetLocations.get(RobotQuadrant.FRONT_LEFT));
      centerOfPressureSLLocation.scale(trotBetas.get(TrotPair.TROT_LEFT.getSide()).getDoubleValue());
      centerOfPressureSLLocation.add(feetLocations.get(RobotQuadrant.FRONT_LEFT));
      
      bodyAngularAcceleration.setToZero();
      bodyAngularAcceleration.getYoX().add(k_roll.getDoubleValue() * (desiredBodyOrientation.getRoll().getDoubleValue() - q_roll.getDoubleValue()));
      bodyAngularAcceleration.getYoX().add(b_roll.getDoubleValue() * (0.0 - bodyAngularVelocity.getX()));
      bodyAngularAcceleration.getYoY().add(k_pitch.getDoubleValue() * (desiredBodyOrientation.getPitch().getDoubleValue() - q_pitch.getDoubleValue()));
      bodyAngularAcceleration.getYoY().add(b_pitch.getDoubleValue() * (0.0 - bodyAngularVelocity.getY()));
      bodyAngularAcceleration.getYoZ().add(k_yaw.getDoubleValue() * (desiredBodyOrientation.getYaw().getDoubleValue() - q_yaw.getDoubleValue()));
      bodyAngularAcceleration.getYoZ().add(b_yaw.getDoubleValue() * (0.0 - bodyAngularVelocity.getZ()));
      
      desiredBodyTorque.setToZero();
      desiredBodyTorque.add(bodyAngularAcceleration);
      desiredBodyTorque.scale(ESTIMATED_ROTATIONAL_INERTIA);
      
      bodyLinearAcceleration.setToZero();
      bodyLinearAcceleration.getYoX().add(k_x.getDoubleValue() * (desiredStancePosition.getX() - stancePosition.getX()));
      bodyLinearAcceleration.getYoX().add(b_x.getDoubleValue() * (0.0 - bodyLinearVelocity.getX()));
      bodyLinearAcceleration.getYoY().add(k_y.getDoubleValue() * (desiredStancePosition.getY() - stancePosition.getY()));
      bodyLinearAcceleration.getYoY().add(b_y.getDoubleValue() * (0.0 - bodyLinearVelocity.getY()));
      bodyLinearAcceleration.getYoZ().add(k_z.getDoubleValue() * (desiredStancePosition.getZ() - stancePosition.getZ()));
      bodyLinearAcceleration.getYoZ().add(b_z.getDoubleValue() * (0.0 - bodyLinearVelocity.getZ()));
      
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

   private void computeBodyRelativePositionsVelocities()
   {
      double footZ = fourFootSupportPolygon.getLowestFootstepZHeight();
      stancePosition.setZ(bodyPoseWorld.getZ() - footZ);
      
      // Calcuate stance X and Y
   }

   private void distributeForcesFourLegs()
   {
//      desiredBodyWrenches.get(TrotPair.TROT_RIGHT.getSide()).set(desiredBodyWrench);
//      desiredBodyWrenches.get(TrotPair.TROT_RIGHT.getSide()).scale(trotAlpha.getDoubleValue());
//      
//      desiredBodyWrenches.get(TrotPair.TROT_LEFT.getSide()).set(desiredBodyWrench);
//      desiredBodyWrenches.get(TrotPair.TROT_LEFT.getSide()).scale((1.0 - trotAlpha.getDoubleValue()));
//
//      clearLegForces();
//      distributeForcesTwoDiagonalLegsWithBeta(TrotPair.TROT_RIGHT, trotBetas.get(TrotPair.TROT_RIGHT.getSide()).getDoubleValue(), desiredBodyWrenches.get(TrotPair.TROT_RIGHT.getSide()));
//      distributeForcesTwoDiagonalLegsWithBeta(TrotPair.TROT_LEFT, trotBetas.get(TrotPair.TROT_LEFT.getSide()).getDoubleValue(), desiredBodyWrenches.get(TrotPair.TROT_LEFT.getSide()));
      
//      distributeForcesTwoDiagonalLegs(TrotPair.TROT_RIGHT, desiredBodyWrenches.get(TrotPair.TROT_RIGHT.getSide()));
//      distributeForcesTwoDiagonalLegs(TrotPair.TROT_LEFT, desiredBodyWrenches.get(TrotPair.TROT_LEFT.getSide()));
      
      distributeForcesToFeet();
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
      
//      System.out.println("Basis Matrix:\n" + basisMatrix.toString());
//      System.out.println("Body Wrench Matrix:\n" + bodyWrenchMatrix.toString());
//      System.out.println("Rho Matrix:\n" + rhoMatrix.toString());
//      
//      CommonOps.mult(basisMatrix, rhoMatrix, bodyWrenchMatrix);
//      System.out.println("Result Body Wrench Matrix:\n" + bodyWrenchMatrix.toString());
      
      for (int quadrantIndex = 0; quadrantIndex < supportPolygon.getSupportingQuadrantsInOrder().length; quadrantIndex++)
      {
         RobotQuadrant robotQuadrant = supportPolygon.getSupportingQuadrantsInOrder()[quadrantIndex];
         for (int basisIndex = 0; basisIndex < 4; basisIndex++)
         {
            rhoScalars.get(robotQuadrant)[basisIndex] = rhoMatrix.get(quadrantIndex * 4 + basisIndex, 0);
         }
      }
      
      clearLegForces();
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
   
   private void distributeForcesTwoDiagonalLegs(TrotPair trotPair, Wrench desiredBodyWrench)
   {
      getFootInBodyZUpFrame(trotPair.getFrontQuadrant(), frontFootPosition);
      getFootInBodyZUpFrame(trotPair.getHindQuadrant(), hindFootPosition);
      
      RobotSide side = trotPair.getSide();
      hindFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
      frontFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
      hindToBodyVectors.get(side).sub(centerOfMassPosition.getFrameTuple(), hindFootPosition);
      frontToBodyVectors.get(side).sub(centerOfMassPosition.getFrameTuple(), frontFootPosition);
      frontFootBodyMinusHindFootBody.get(side).sub(frontToBodyVectors.get(side), hindToBodyVectors.get(side));
      
      desiredBodyForceVectors.get(side).set(desiredBodyWrench.getLinearPart());
      desiredBodyTorqueVectors.get(side).set(desiredBodyWrench.getAngularPart());
      unknownTorqueQuantitys.get(side).cross(desiredBodyForceVectors.get(side), hindToBodyVectors.get(side));
      unknownTorqueQuantity2s.get(side).sub(desiredBodyTorqueVectors.get(side), unknownTorqueQuantitys.get(side));
      frontFootForceVectors.get(side).cross(frontFootBodyMinusHindFootBody.get(side), unknownTorqueQuantity2s.get(side));
      frontFootForceVectors.get(side).scale(0.5);
      hindFootForceVectors.get(side).sub(desiredBodyForceVectors.get(side), frontFootForceVectors.get(side));
      
      vmcFootForces.get(trotPair.getFrontQuadrant()).setX(frontFootForceVectors.get(side).getX());
      vmcFootForces.get(trotPair.getFrontQuadrant()).setY(frontFootForceVectors.get(side).getY());
      vmcFootForces.get(trotPair.getFrontQuadrant()).setZ(frontFootForceVectors.get(side).getZ());
      vmcFootForces.get(trotPair.getHindQuadrant()).setX(hindFootForceVectors.get(side).getX());
      vmcFootForces.get(trotPair.getHindQuadrant()).setY(hindFootForceVectors.get(side).getY());
      vmcFootForces.get(trotPair.getHindQuadrant()).setZ(hindFootForceVectors.get(side).getZ());
   }

   private void distributeForcesTwoDiagonalLegsWithBeta(TrotPair trotPair, double trotBeta, Wrench desiredBodyWrench)
   {
      getFootInBodyZUpFrame(trotPair.getFrontQuadrant(), frontFootPosition);
      getFootInBodyZUpFrame(trotPair.getHindQuadrant(), hindFootPosition);
      
      DenseMatrix64F forceConstraintsMatrix = new DenseMatrix64F(5, 6);
      forceConstraintsMatrix.zero();

      forceConstraintsMatrix.set(0, 5, 1.0 / desiredBodyWrench.getLinearPartZ());
      forceConstraintsMatrix.set(1, 2, 1.0);
      forceConstraintsMatrix.set(1, 5, 1.0);

      forceConstraintsMatrix.set(2, 1, -frontFootPosition.getZ());
      forceConstraintsMatrix.set(2, 2, frontFootPosition.getY());
      forceConstraintsMatrix.set(2, 4, -hindFootPosition.getZ());
      forceConstraintsMatrix.set(2, 5, hindFootPosition.getY());

      forceConstraintsMatrix.set(3, 0, frontFootPosition.getZ());
      forceConstraintsMatrix.set(3, 2, -frontFootPosition.getX());
      forceConstraintsMatrix.set(3, 3, hindFootPosition.getZ());
      forceConstraintsMatrix.set(3, 5, -hindFootPosition.getX());

      forceConstraintsMatrix.set(4, 0, -frontFootPosition.getY());
      forceConstraintsMatrix.set(4, 1, frontFootPosition.getX());
      forceConstraintsMatrix.set(4, 3, -hindFootPosition.getY());
      forceConstraintsMatrix.set(4, 4, hindFootPosition.getX());

      DenseMatrix64F totalForcesOnTheBody = new DenseMatrix64F(5, 1);

      totalForcesOnTheBody.set(0, 0, trotBeta);
      totalForcesOnTheBody.set(1, 0, desiredBodyWrench.getLinearPartZ());
      totalForcesOnTheBody.set(2, 0, desiredBodyWrench.getAngularPartX());
      totalForcesOnTheBody.set(3, 0, desiredBodyWrench.getAngularPartY());
      totalForcesOnTheBody.set(4, 0, desiredBodyWrench.getAngularPartZ());

      DenseMatrix64F forceConstraintsMatrixInverse = new DenseMatrix64F(6, 5);
      try
      {
         CommonOps.pinv(forceConstraintsMatrix, forceConstraintsMatrixInverse);
      }
      catch (Exception e)
      {
         System.err.println("forceConstraintsMatrix = " + forceConstraintsMatrix);
         return;
      }

      DenseMatrix64F distributedForcesOnTheLegs = new DenseMatrix64F(6, 1);
      CommonOps.mult(forceConstraintsMatrixInverse, totalForcesOnTheBody, distributedForcesOnTheLegs);

      vmcFootForces.get(trotPair.getFrontQuadrant()).setX(distributedForcesOnTheLegs.get(0, 0));
      vmcFootForces.get(trotPair.getFrontQuadrant()).setY(distributedForcesOnTheLegs.get(1, 0));
//      vmcFootForces.get(trotPair.getFrontQuadrant()).setZ(distributedForcesOnTheLegs.get(2, 0));
      vmcFootForces.get(trotPair.getHindQuadrant()).setX(distributedForcesOnTheLegs.get(3, 0));
      vmcFootForces.get(trotPair.getHindQuadrant()).setY(distributedForcesOnTheLegs.get(4, 0));
//      vmcFootForces.get(trotPair.getHindQuadrant()).setZ(distributedForcesOnTheLegs.get(5, 0));
      vmcFootForces.get(trotPair.getFrontQuadrant()).getYoZ().set((1.0 - trotBeta) * desiredBodyWrench.getLinearPartZ());
      vmcFootForces.get(trotPair.getHindQuadrant()).getYoZ().set(trotBeta * desiredBodyWrench.getLinearPartZ());
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
//         jointInBodyZUp.changeFrame(referenceFrames.getBodyZUpFrame());
         jointInBodyZUp.changeFrame(referenceFrames.getCenterOfMassZUpFrame());

         jointToFootVector.setIncludingFrame(footInBodyZUp);
         jointToFootVector.sub(jointInBodyZUp);

//         vmcRequestedTorqueFromJointXYZ.setToZero(referenceFrames.getBodyZUpFrame());
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
//      framePointToPack.changeFrame(referenceFrames.getBodyZUpFrame());
      framePointToPack.changeFrame(referenceFrames.getCenterOfMassZUpFrame());
   }

   private void clearLegForces()
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
         computeBodyRelativePositionsVelocities();

         if (nextState.getEnumValue() == QuadrupedWalkingState.RightTrotLine)
         {
            // computeCOMPQ(leftHindLimb, rightForeLimb);
         }
         else
         {
            // computeCOMPQ(rightHindLimb, leftForeLimb);
         }

         doTrotControl();
         distributeForcesFourLegs();

//         preventSlippingForces();

         computeStanceJacobians();
      }

      @Override
      public void doTransitionIntoAction()
      {
         trotAlpha.set(0.5);
         
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
         computeBodyRelativePositionsVelocities();

         doTrotControl();
         distributeForcesFourLegs();

//         preventSlippingForces();

         computeStanceJacobians();
      }

      @Override
      public void doTransitionIntoAction()
      {
         trotAlpha.set(1.01);
         
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
         computeBodyRelativePositionsVelocities();

         doTrotControl();
         distributeForcesFourLegs();

//         preventSlippingForces();

         computeStanceJacobians();
      }

      @Override
      public void doTransitionIntoAction()
      {
         trotAlpha.set(-0.01);
         
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

   private void preventSlippingForces()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (vmcFootForces.get(robotQuadrant).getYoZ().getDoubleValue() < 2.0)
         {
            vmcFootForces.get(robotQuadrant).getYoZ().set(2.0);
         }
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
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ArrayList<OneDoFJoint> legJoints = oneDofJoints.get(robotQuadrant);
         for (int i = 0; i < legJoints.size(); i++)
         {
            legJoints.get(i).setUnderPositionControl(false);
         }
      }

      updateEstimates();

      trotAlpha.set(0.5);
      
      enableTrot.set(false);
      timeInTrot.set(0.2);
      
      trotBetas.get(TrotPair.TROT_RIGHT.getSide()).set(0.5);
      trotBetas.get(TrotPair.TROT_LEFT.getSide()).set(0.5);

      k_x.set(10.0); // 2000.0);
      b_x.set(5.0);

      k_y.set(10.0); // 2000.0);
      b_y.set(5.0); // 0.0);    // 50 for pace, 0 for trot.

      k_roll.set(-700.0); // 4000.0
      b_roll.set(-7.0); // 50.0

      k_pitch.set(-700.0); // 4000.0 // 80.0);
      b_pitch.set(-7.0); // 50.0 // 20.0);

      k_yaw.set(-700.0); // 3000.0 // 80.0);    // 250.0);
      b_yaw.set(-7.0); // 40.0 // 20.0);    // 100.0);
      
//      k_x.set(2000.0); // 2000.0);
//      b_x.set(25.0);
//
//      k_y.set(2000.0); // 2000.0);
//      b_y.set(25.0); // 0.0);    // 50 for pace, 0 for trot.
//
//      k_roll.set(4000.0); // 4000.0
//      b_roll.set(50.0); // 50.0
//
//      k_pitch.set(4000.0); // 4000.0 // 80.0);
//      b_pitch.set(50.0); // 50.0 // 20.0);
//
//      k_yaw.set(3000.0); // 3000.0 // 80.0);    // 250.0);
//      b_yaw.set(50.0);

      k_z.set(10.0);
      b_z.set(5.0);
      fz_limit.set(1000.0);
      desiredStancePosition.setZ(bodyPoseWorld.getZ());
      
      supportPolygon.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygon.setFootstep(robotQuadrant, feetLocations.get(robotQuadrant).getFrameTuple());
      }
   }

   public String getDescription()
   {
      return getName();
   }

   public void doTransitionIntoAction()
   {
      initialize();
   }

   public void doTransitionOutOfAction()
   {

   }

   public RobotMotionStatus getMotionStatus()
   {
      return RobotMotionStatus.IN_MOTION;
   }
}
