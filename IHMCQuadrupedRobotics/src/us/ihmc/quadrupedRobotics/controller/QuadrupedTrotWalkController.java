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
import us.ihmc.quadrupedRobotics.gait.QuadrupedGaitCycle;
import us.ihmc.quadrupedRobotics.gait.QuadrupedSupportConfiguration;
import us.ihmc.quadrupedRobotics.gait.TrotPair;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.YoQuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoTwist;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLine;

public class QuadrupedTrotWalkController extends QuadrupedController
{
   private static final double GRAVITY = 9.81;
   private static final double INITIAL_STANCE_HEIGHT = 0.625;
   private static final double ESTIMATED_MASS = 63.9; // TODO PDControl this when z-vel=0
   private static final double ESTIMATED_ROTATIONAL_INERTIA = 5.0; // TODO PDControl this when z-vel=0
   private static final double COEFFICIENT_OF_FRICTION = 0.7;
   private final double dt;
   private final DoubleYoVariable yoTime;
   private final YoVariableRegistry registry = new YoVariableRegistry("TrotWalkController");
   private final QuadrupedReferenceFrames referenceFrames;
   private final SDFFullRobotModel fullRobotModel;
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;
   private final QuadrantDependentList<YoFramePoint> footPositions = new QuadrantDependentList<YoFramePoint>();
   private final QuadrantDependentList<YoFrameVector> footVelocities = new QuadrantDependentList<YoFrameVector>();
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final YoFramePoint icp = new YoFramePoint("icp", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition icpViz = new YoGraphicPosition("icpViz", icp, 0.01, YoAppearance.DarkSlateBlue(), GraphicType.SQUARE);
   private final YoArtifactLine hindRightFrontLeftTrotLine;
   private final YoArtifactLine hindLeftFrontRightTrotLine;

   private final YoFramePoint centerOfPressure = new YoFramePoint("centerOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredICP = new YoFramePoint("desiredICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition desiredICPViz = new YoGraphicPosition("desiredICPViz", desiredICP, 0.01, YoAppearance.Green(), GraphicType.SQUARE);
   private final YoGraphicPosition centerOfPressureViz = new YoGraphicPosition("centerOfPressureViz", centerOfPressure, 0.01, YoAppearance.Black(), GraphicType.BALL_WITH_ROTATED_CROSS);
   
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
   
   // Balancing
   private final QuadrantDependentList<YoFrameVector[]> basisForceVectors = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector[]> basisTorqueVectors = new QuadrantDependentList<>();
   private final QuadrantDependentList<double[]> rhoScalars = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> footToCoMVectors = new QuadrantDependentList<>();
   private final DenseMatrix64F bodyWrenchMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F basisMatrix = new DenseMatrix64F(6, 16);
   private final DenseMatrix64F rhoMatrix = new DenseMatrix64F(16, 1);
   private final SolvePseudoInverseSvd solver = new SolvePseudoInverseSvd();
   private final QuadrantDependentList<YoFrameVector> vmcFootForces = new QuadrantDependentList<>();
   private final YoFramePose bodyPoseWorld;
   private final YoTwist bodyTwist;
   private final YoFramePose stancePose;
   private final YoFramePose desiredStancePose;
   private final YoFramePose desiredStancePoseOffset;
   private final YoFrameVector bodyLinearAcceleration;
   private final YoFrameVector bodyAngularAcceleration;
   private final YoFrameVector desiredBodyForce;
   private final YoFrameVector desiredBodyTorque;
   private final Wrench desiredBodyWrench;
   private final YoTwist desiredBodyTwist;
   private final SideDependentList<List<YoGraphicVector>> forceDistributionYoGraphicVectors = new SideDependentList<>();
   private final QuadrantDependentList<YoGraphicVector[]> basisForceYoGraphicVectors = new QuadrantDependentList<>();
   
   // Jacobian
   private final FramePoint jointPosition = new FramePoint();
   private final FrameVector jointToFootVector = new FrameVector();
   private final FrameVector vmcRequestedTorqueFromJoint = new FrameVector();
   private final FrameVector jointAxis = new FrameVector();
   
   // Walking
   private final EnumYoVariable<QuadrupedGaitCycle> desiredGait = new EnumYoVariable<>("desiredGait", registry, QuadrupedGaitCycle.class);
   private final EnumYoVariable<QuadrupedGaitCycle> nextGait = new EnumYoVariable<>("nextGait", registry, QuadrupedGaitCycle.class);
   private final EnumYoVariable<QuadrupedGaitCycle> currentGait = new EnumYoVariable<>("gait", registry, QuadrupedGaitCycle.class);
   private final QuadrantDependentList<YoPolynomial> swingZTrajectories = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> swingInitialZHeights = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> swingStartTimes = new QuadrantDependentList<>();
   private final DoubleYoVariable phaseStartTime = new DoubleYoVariable("phaseStartTime", registry);
   private final DoubleYoVariable gaitStartTime = new DoubleYoVariable("gaitStartTime", registry);
   private final QuadrantDependentList<DoubleYoVariable> swingDurations = new QuadrantDependentList<>();
   private final EnumYoVariable<QuadrupedSupportConfiguration> previousGaitPhase = new EnumYoVariable<>("previousGaitPhase", registry, QuadrupedSupportConfiguration.class, false);
   private final EnumYoVariable<QuadrupedSupportConfiguration> currentGaitPhase = new EnumYoVariable<>("currentGaitPhase", registry, QuadrupedSupportConfiguration.class, false);
   private final EnumYoVariable<QuadrupedSupportConfiguration> nextGaitPhase = new EnumYoVariable<>("nextGaitPhase", registry, QuadrupedSupportConfiguration.class, false);
   private final DoubleYoVariable swingZHeight = new DoubleYoVariable("swingZHeight", registry);
   private final DoubleYoVariable desiredGaitPeriod = new DoubleYoVariable("desiredGaitPeriod", registry);
   private final DoubleYoVariable currentGaitCompletion = new DoubleYoVariable("currentGaitCompletion", registry);
   private final BooleanYoVariable gaitCompleted = new BooleanYoVariable("gaitCompleted", registry);
   private final DoubleYoVariable impactVelocityZ = new DoubleYoVariable("impactVelocityZ", registry);
   private final YoFramePoint centroid = new YoFramePoint("centroid", ReferenceFrame.getWorldFrame(), registry);
   private final YoQuadrupedSupportPolygon previousSupportPolygon = new YoQuadrupedSupportPolygon("previousSupportPolygon", registry);
   private final YoQuadrupedSupportPolygon currentSupportPolygon = new YoQuadrupedSupportPolygon("currentSupportPolygon", registry);
   private final YoQuadrupedSupportPolygon nextPhaseSupportPolygon = new YoQuadrupedSupportPolygon("nextPhaseSupportPolygon", registry);
   private final VelocityConstrainedPositionTrajectoryGenerator icpTrajectory = new VelocityConstrainedPositionTrajectoryGenerator("icpTrajectory", ReferenceFrame.getWorldFrame(), registry);
   
   // Swing PD Controllers
   private final YoFrameVector kp_swing = new YoFrameVector("kp_swing_", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector kd_swing = new YoFrameVector("kd_swing_", ReferenceFrame.getWorldFrame(), registry);
   private final QuadrantDependentList<YoFramePoint> desiredFootPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> desiredFootVelocities = new QuadrantDependentList<>();

   public QuadrupedTrotWalkController(QuadrupedRobotParameters robotParameters, SDFFullRobotModel fullRobotModel, QuadrantDependentList<FootSwitchInterface> footSwitches, double DT,
         DoubleYoVariable yoTime, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(QuadrupedControllerState.TROT_WALK);
      this.fullRobotModel = fullRobotModel;
      this.footSwitches = footSwitches;
      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
      this.dt = DT;
      this.yoTime = yoTime;
      
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
         vmcFootForces.set(robotQuadrant, new YoFrameVector("vmcFootForces" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
      }

      yoGraphicsListRegistry.registerArtifact("icpViz", icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("desiredICPViz", desiredICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfMassViz", centerOfMassViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfPressureViz", centerOfPressureViz.createArtifact());
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoFramePoint footPosition = new YoFramePoint("footPosition" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition footPositionViz = new YoGraphicPosition("footPosition" + robotQuadrant.getPascalCaseName() + "Viz", footPosition, 0.02, YoAppearance.Color(robotQuadrant.getColor()), GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerArtifact("footPositions", footPositionViz.createArtifact());
         footPositions.set(robotQuadrant, footPosition);
         footVelocities.set(robotQuadrant, new YoFrameVector("footVelocity" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
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
                                                                              footPositions.get(RobotQuadrant.getQuadrant(RobotEnd.FRONT, side)),
                                                                              vmcFootForces.get(RobotQuadrant.getQuadrant(RobotEnd.FRONT, side)), 0.007, YoAppearance.Yellow(), true, 0.01));
         forceDistributionYoGraphicVectors.get(side).add(new YoGraphicVector("hindFootForces" + side.getCamelCaseNameForMiddleOfExpression(),
                                                                              footPositions.get(RobotQuadrant.getQuadrant(RobotEnd.HIND, side.getOppositeSide())),
                                                                              vmcFootForces.get(RobotQuadrant.getQuadrant(RobotEnd.HIND, side.getOppositeSide())), 0.007, YoAppearance.Yellow(), true, 0.01));
      }
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         basisForceYoGraphicVectors.set(robotQuadrant, new YoGraphicVector[4]);
         
         for (int i = 0; i < 4; i++)
         {
            basisForceYoGraphicVectors.get(robotQuadrant)[i] = new YoGraphicVector("basisForceYoGraphicVectors" + robotQuadrant.getPascalCaseName() + i,
                                                                                    footPositions.get(robotQuadrant),
                                                                                    basisForceVectors.get(robotQuadrant)[i], 0.007, YoAppearance.Red(), true, 0.01);
            yoGraphicsListRegistry.registerYoGraphic("trotWalk", basisForceYoGraphicVectors.get(robotQuadrant)[i]);
         }
      }
      
      bodyPoseWorld = new YoFramePose("body", ReferenceFrame.getWorldFrame(), registry);
      desiredStancePose = new YoFramePose("desiredStance", ReferenceFrame.getWorldFrame(), registry);
      desiredStancePoseOffset = new YoFramePose("desiredStanceOffset", ReferenceFrame.getWorldFrame(), registry);
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
      
      YoFramePoint hindRightFoot = footPositions.get(RobotQuadrant.HIND_RIGHT);
      YoFramePoint hindLeftFoot = footPositions.get(RobotQuadrant.HIND_LEFT);
      YoFramePoint frontLeftFoot = footPositions.get(RobotQuadrant.FRONT_LEFT);
      YoFramePoint frontRightFoot = footPositions.get(RobotQuadrant.FRONT_RIGHT);
      Color hindRightYoAppearance = RobotQuadrant.HIND_RIGHT.getColor();
      Color hindLeftYoAppearance = RobotQuadrant.HIND_LEFT.getColor();
      hindRightFrontLeftTrotLine = new YoArtifactLine("hindRightFrontLeftTrotLine", hindRightFoot, frontLeftFoot, hindRightYoAppearance);
      hindLeftFrontRightTrotLine = new YoArtifactLine("hindLeftFrontRightTrotLine", hindLeftFoot, frontRightFoot, hindLeftYoAppearance);
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingZTrajectories.set(robotQuadrant, new YoPolynomial("swingZTrajectory" + robotQuadrant.getPascalCaseName(), 4, registry));
         swingInitialZHeights.set(robotQuadrant, new DoubleYoVariable("swingInitialZHeights" + robotQuadrant.getPascalCaseName(), registry));
         swingStartTimes.set(robotQuadrant, new DoubleYoVariable("swingStartTime" + robotQuadrant.getPascalCaseName(), registry));
         swingDurations.set(robotQuadrant, new DoubleYoVariable("swingDurations" + robotQuadrant.getPascalCaseName(), registry));
         desiredFootPositions.set(robotQuadrant, new YoFramePoint("desiredFootPosition" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         desiredFootVelocities.set(robotQuadrant, new YoFrameVector("desiredFootVelocity" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
      }
      
      yoGraphicsListRegistry.registerArtifact("trotLines", hindRightFrontLeftTrotLine);
      yoGraphicsListRegistry.registerArtifact("trotLines", hindLeftFrontRightTrotLine);
      
      parentRegistry.addChild(registry);
   }

   @Override
   public void doTransitionIntoAction()
   {
      initialize();
   }

   public void initialize()
   {
      currentSupportPolygon.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         currentSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      
      updateEstimates();
      
      desiredStancePose.set(stancePose);
      desiredStancePoseOffset.setToZero();
      icpTrajectory.getFinalPosition().set(centroid);
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ArrayList<OneDoFJoint> legJoints = oneDofJoints.get(robotQuadrant);
         for (int i = 0; i < legJoints.size(); i++)
         {
            legJoints.get(i).setUnderPositionControl(false);
         }
      }
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         desiredFootPositions.get(robotQuadrant).set(footPositions.get(robotQuadrant));
      }
      
      desiredGaitPeriod.set(20.0);
      desiredGait.set(QuadrupedGaitCycle.SAFE_WALK);
      currentGait.set(QuadrupedGaitCycle.SAFE_WALK);
      nextGait.set(QuadrupedGaitCycle.SAFE_WALK);
      swingZHeight.set(0.1);
      impactVelocityZ.set(0.0);
      previousSupportPolygon.setWithoutChecks(currentSupportPolygon);
      previousGaitPhase.set(QuadrupedSupportConfiguration.ALL_FOURS);
      currentGaitPhase.set(QuadrupedSupportConfiguration.ALL_FOURS);
      currentGaitCompletion.set(0.0);
      
      handlePhaseChange();
   
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
      
      kp_swing.setX(-600.0);
      kd_swing.setX(-5.0);
      kp_swing.setY(-600.0);
      kd_swing.setY(-5.0);
      kp_swing.setZ(-600.0);
      kd_swing.setZ(-5.0);
   }

   @Override
   public void doAction()
   {
      updateEstimates();

      computeFeetContactState();
      
      checkGaitTransitionConditions();
      
      doSupportAndSwing();
      
      doControl();
      distributeForcesToFeet();
      computeStanceJacobians();
   }

   private void updateEstimates()
   {
      //update frames
      referenceFrames.updateFrames();

      //update feet locations
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footVelocities.get(robotQuadrant).set(footPositions.get(robotQuadrant));
         footPositions.get(robotQuadrant).setFromReferenceFrame(referenceFrames.getFootFrame(robotQuadrant));
         footVelocities.get(robotQuadrant).sub(footPositions.get(robotQuadrant));
         footVelocities.get(robotQuadrant).scale(-1.0 / dt);
      }
      
      for (RobotQuadrant robotQuadrant : currentSupportPolygon.getSupportingQuadrantsInOrder())
      {
         currentSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      
      bodyPoseWorld.setFromReferenceFrame(referenceFrames.getBodyFrame());
      
      bodyTwist.setLinearPart(fullRobotModel.getRootJoint().getLinearVelocityForReading());
      bodyTwist.setAngularPart(fullRobotModel.getRootJoint().getAngularVelocityForReading());
      
      stancePose.set(bodyPoseWorld);

      // compute center of mass position and velocity
      centerOfMass.setFromReferenceFrame(referenceFrames.getCenterOfMassZUpFrame());
      centerOfMassXYProjection.set(centerOfMass.getReferenceFrame(), centerOfMass.getX(), centerOfMass.getY(), 0.0);
      
      centerOfMassJacobian.compute();
      
      FrameVector tempVector = centerOfMassVelocity.getFrameTuple();
      centerOfMassJacobian.getCenterOfMassVelocity(tempVector);
      tempVector.changeFrame(centerOfMassVelocity.getReferenceFrame());
      centerOfMassVelocity.set(tempVector.getX(), tempVector.getY(), tempVector.getZ());
      
      FramePoint frameTuple = centroid.getFrameTuple();
      currentSupportPolygon.getCentroid(frameTuple);
      centroid.set(frameTuple.getX(), frameTuple.getY(), frameTuple.getZ());

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
         tempCenterOfPressure.add(footPositions.get(robotQuadrant).getX() * vmcFootForces.get(robotQuadrant).getZ(), 0.0, 0.0);
         tempCenterOfPressure.add(0.0, footPositions.get(robotQuadrant).getY() * vmcFootForces.get(robotQuadrant).getZ(), 0.0);
         tempCenterOfPressure.add(0.0, 0.0, footPositions.get(robotQuadrant).getZ() * vmcFootForces.get(robotQuadrant).getZ());
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
      
      for (RobotQuadrant robotQuadrant : currentSupportPolygon.getSupportingQuadrantsInOrder())
      {
         footToCoMVectors.get(robotQuadrant).set(centerOfMass);
         footToCoMVectors.get(robotQuadrant).sub(footPositions.get(robotQuadrant));
         calculateBasisVectors(robotQuadrant, footToCoMVectors.get(robotQuadrant));
      }
      
      currentGaitCompletion.set((yoTime.getDoubleValue() - gaitStartTime.getDoubleValue()) / desiredGaitPeriod.getDoubleValue());
      if (currentGaitCompletion.getDoubleValue() >= 1.0)
      {
         gaitCompleted.set(true);
         currentGaitCompletion.set(currentGaitCompletion.getDoubleValue() % 1.0);
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
   
   private void checkGaitTransitionConditions()
   {
      if (gaitCompleted.getBooleanValue())
      {
         handleGaitChange();
         handlePhaseChange();
      }
      else if (currentGait.getEnumValue().getGaitPhase(currentGaitCompletion.getDoubleValue()) != currentGaitPhase.getEnumValue())
      {
         handlePhaseChange();
      }
   }

   private void handleGaitChange()
   {
      gaitCompleted.set(false);
      gaitStartTime.set(yoTime.getDoubleValue());
      currentGait.set(nextGait.getEnumValue());
   }

   private void handlePhaseChange()
   {
      // Update phases
      phaseStartTime.set(yoTime.getDoubleValue());
      previousGaitPhase.set(currentGaitPhase.getEnumValue());
      currentGaitPhase.set(currentGait.getEnumValue().getGaitPhase(currentGaitCompletion.getDoubleValue()));
      if (currentGait.getEnumValue().isLastPhase(currentGaitCompletion.getDoubleValue()))
      {
         nextGait.set(desiredGait.getEnumValue());
         nextGaitPhase.set(nextGait.getEnumValue().getGaitPhase(0.0));
      }
      else
      {
         nextGaitPhase.set(currentGait.getEnumValue().getNextGaitPhase(currentGaitCompletion.getDoubleValue()));
      }
      
      // Update Z swing parameters
      for (RobotQuadrant robotQuadrant : previousGaitPhase.getEnumValue().supportQuadrants())
      {
         swingInitialZHeights.get(robotQuadrant).set(footPositions.get(robotQuadrant).getZ());
         swingStartTimes.get(robotQuadrant).set(currentGaitCompletion.getDoubleValue());
         
         // Measure durations of legs switching to swing
         if (currentGaitPhase.getEnumValue().isSwingQuadrant(robotQuadrant))
         {
            swingDurations.get(robotQuadrant).set(currentGait.getEnumValue().getRemainingSwingDuration(robotQuadrant, currentGaitCompletion.getDoubleValue()));
         }
      }
      
      // Update support polygons
      previousSupportPolygon.setWithoutChecks(currentSupportPolygon);
      currentSupportPolygon.clear();
      for (RobotQuadrant robotQuadrant : currentGaitPhase.getEnumValue().supportQuadrants())
      {
         currentSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      FramePoint frameTuple = centroid.getFrameTuple();
      currentSupportPolygon.getCentroid(frameTuple);
      centroid.set(frameTuple.getX(), frameTuple.getY(), frameTuple.getZ());
      nextPhaseSupportPolygon.clear();
      for (RobotQuadrant robotQuadrant : nextGaitPhase.getEnumValue().supportQuadrants())
      {
         nextPhaseSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      
      // Update desired ICP trajectory
      icpTrajectory.setTrajectoryTime(currentGait.getEnumValue().getRemainingPhaseDuration(currentGaitCompletion.getDoubleValue()));
      icpTrajectory.getInitialPosition().set(icpTrajectory.getFinalPosition());
      FramePoint nextCentroid = icpTrajectory.getFinalPosition().getFrameTuple();
      if (currentGaitPhase.getEnumValue() == QuadrupedSupportConfiguration.ALL_FOURS)
      {
         nextPhaseSupportPolygon.getCentroid(nextCentroid);
      }
      else
      {
         currentSupportPolygon.getCentroid(nextCentroid);
      }
      icpTrajectory.getFinalPosition().set(nextCentroid.getX(), nextCentroid.getY(), 0.0);
      icpTrajectory.getInitialVelocity().setToZero();
      icpTrajectory.getFinalVelocity().setToZero();
      icpTrajectory.initialize();
   }

   private void doSupportAndSwing()
   {
      for (RobotQuadrant robotQuadrant : currentGaitPhase.getEnumValue().swingQuadrants())
      {
         double timeStart = 0.0;
         double timePeak = swingDurations.get(robotQuadrant).getDoubleValue() / 2.0;
         double timeEnd = swingDurations.get(robotQuadrant).getDoubleValue();
         double zStart = swingInitialZHeights.get(robotQuadrant).getDoubleValue();
         double zMid = swingZHeight.getDoubleValue();
         double zEnd = zStart;
         double zVelocityFinal = impactVelocityZ.getDoubleValue();
         swingZTrajectories.get(robotQuadrant).setCubicWithIntermediatePositionAndFinalVelocityConstraint(timeStart, timePeak, timeEnd, zStart, zMid, zEnd, zVelocityFinal);
         
         if (currentGaitCompletion.getDoubleValue() < swingStartTimes.get(robotQuadrant).getDoubleValue())
         {
            swingZTrajectories.get(robotQuadrant).compute(currentGaitCompletion.getDoubleValue() + (1.0 - swingStartTimes.get(robotQuadrant).getDoubleValue()));
         }
         else
         {
            swingZTrajectories.get(robotQuadrant).compute(currentGaitCompletion.getDoubleValue() - swingStartTimes.get(robotQuadrant).getDoubleValue());
         }
         
         desiredFootPositions.get(robotQuadrant).setZ(swingZTrajectories.get(robotQuadrant).getPosition());
//         desiredFootVelocities.get(robotQuadrant).setZ(swingZTrajectories.get(robotQuadrant).getVelocity());
         desiredFootVelocities.get(robotQuadrant).setToZero();
      }
      
      icpTrajectory.compute((yoTime.getDoubleValue() - phaseStartTime.getDoubleValue()) / desiredGaitPeriod.getDoubleValue());
   }

   private void doControl()
   {
      icpTrajectory.get(desiredICP);
      
      desiredStancePose.setToZero();
      desiredStancePose.setXYZ(desiredICP.getX(), desiredICP.getY(), INITIAL_STANCE_HEIGHT);
      desiredStancePose.add(desiredStancePoseOffset);
      
      desiredBodyTwist.setToZero();
      FrameVector icpVelocity = desiredBodyTwist.getYoLinearPart().getFrameTuple();
      desiredBodyTwist.getYoLinearPart().set(icpVelocity.getX(), icpVelocity.getY(), 0.0);
      
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
      clearFootForces();
      distributeForcesToSupportFeet();
      distributeForcesToSwingFeet();
      
//      clipForces(200.0); // TODO
   }
   
   private void distributeForcesToSwingFeet()
   {
      for (RobotQuadrant robotQuadrant : currentGaitPhase.getEnumValue().swingQuadrants())
      {
         vmcFootForces.get(robotQuadrant).setToZero();
         vmcFootForces.get(robotQuadrant).getYoX().add(kp_swing.getX() * (desiredFootPositions.get(robotQuadrant).getX() - footPositions.get(robotQuadrant).getX()));
         vmcFootForces.get(robotQuadrant).getYoX().add(kd_swing.getX() * (desiredFootVelocities.get(robotQuadrant).getX() - footVelocities.get(robotQuadrant).getX()));
         vmcFootForces.get(robotQuadrant).getYoY().add(kp_swing.getY() * (desiredFootPositions.get(robotQuadrant).getY() - footPositions.get(robotQuadrant).getY()));
         vmcFootForces.get(robotQuadrant).getYoY().add(kd_swing.getY() * (desiredFootVelocities.get(robotQuadrant).getY() - footVelocities.get(robotQuadrant).getY()));
         vmcFootForces.get(robotQuadrant).getYoZ().add(kp_swing.getZ() * (desiredFootPositions.get(robotQuadrant).getZ() - footPositions.get(robotQuadrant).getZ()));
         vmcFootForces.get(robotQuadrant).getYoZ().add(kd_swing.getZ() * (desiredFootVelocities.get(robotQuadrant).getZ() - footVelocities.get(robotQuadrant).getZ()));
      }
   }
   
   private void distributeForcesToSupportFeet()
   {
      // New QP stuff here
      bodyWrenchMatrix.set(0, 0, desiredBodyWrench.getLinearPartX());
      bodyWrenchMatrix.set(1, 0, desiredBodyWrench.getLinearPartY());
      bodyWrenchMatrix.set(2, 0, desiredBodyWrench.getLinearPartZ());
      bodyWrenchMatrix.set(3, 0, desiredBodyWrench.getAngularPartX());
      bodyWrenchMatrix.set(4, 0, desiredBodyWrench.getAngularPartY());
      bodyWrenchMatrix.set(5, 0, desiredBodyWrench.getAngularPartZ());
      
      basisMatrix.reshape(6, currentSupportPolygon.size() * 4);
      rhoMatrix.reshape(currentSupportPolygon.size() * 4, 1);
      
      for (int quadrantIndex = 0; quadrantIndex < currentSupportPolygon.getSupportingQuadrantsInOrder().length; quadrantIndex++)
      {
         RobotQuadrant robotQuadrant = currentSupportPolygon.getSupportingQuadrantsInOrder()[quadrantIndex];
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
      
      for (int quadrantIndex = 0; quadrantIndex < currentSupportPolygon.getSupportingQuadrantsInOrder().length; quadrantIndex++)
      {
         RobotQuadrant robotQuadrant = currentSupportPolygon.getSupportingQuadrantsInOrder()[quadrantIndex];
         for (int basisIndex = 0; basisIndex < 4; basisIndex++)
         {
            rhoScalars.get(robotQuadrant)[basisIndex] = rhoMatrix.get(quadrantIndex * 4 + basisIndex, 0);
         }
      }
      
      for (RobotQuadrant robotQuadrant : currentSupportPolygon.getSupportingQuadrantsInOrder())
      {
         for (int basisIndex = 0; basisIndex < 4; basisIndex++)
         {
            basisForceVectors.get(robotQuadrant)[basisIndex].scale(rhoScalars.get(robotQuadrant)[basisIndex]);
            vmcFootForces.get(robotQuadrant).add(basisForceVectors.get(robotQuadrant)[basisIndex]);
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

   private void computeStanceJacobianForLeg(RobotQuadrant robotQuadrant)
   {
      for (int i = 0; i < oneDofJoints.get(robotQuadrant).size(); i++)
      {
         OneDoFJoint oneDoFJoint = oneDofJoints.get(robotQuadrant).get(i);
         
         jointPosition.setFromReferenceFrame(oneDoFJoint.getFrameBeforeJoint());

         jointToFootVector.set(footPositions.get(robotQuadrant).getFrameTuple());
         jointToFootVector.sub(jointPosition);

         vmcRequestedTorqueFromJoint.setToZero();
         vmcRequestedTorqueFromJoint.cross(jointToFootVector, vmcFootForces.get(robotQuadrant).getFrameTuple());

         oneDoFJoint.getJointAxis(jointAxis);
         jointAxis.changeFrame(ReferenceFrame.getWorldFrame());
         double torque = jointAxis.dot(vmcRequestedTorqueFromJoint);
         
         desiredTorques.get(oneDoFJoint.getName()).set(-torque);
         oneDoFJoint.setTau(-torque);
      }
   }

   private void clipForces(double max)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (vmcFootForces.get(robotQuadrant).getFrameTuple().length() > max)
         {
            vmcFootForces.get(robotQuadrant).scale(max / vmcFootForces.get(robotQuadrant).getFrameTuple().length());
         }
      }
   }

   private void clearFootForces()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         vmcFootForces.get(robotQuadrant).setToZero();
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

   public String getDescription()
   {
      return getName();
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
