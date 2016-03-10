package us.ihmc.quadrupedRobotics.controller;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.trot.TrotPair;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLine;

public class QuadrupedTrotWalkController extends QuadrupedController
{
   private static final double GRAVITY = 9.81;
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

   ///Hacky Sim yoVariables.... Delete this and use the inverse dynamics robot
   private DoubleYoVariable qd_z;
   private DoubleYoVariable qd_wx, qd_wy, qd_wz;

   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final FramePose bodyPose = new FramePose(worldFrame);

   private final DoubleYoVariable percentOfForcesOnRightTrotPair = new DoubleYoVariable("quadAlpha", registry);

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
   private final YoGraphicPosition centerOfPressureSRViz = new YoGraphicPosition("centerOfPressureSRViz", centerOfPressureSRLocation, 0.01, YoAppearance.Purple(), GraphicType.BALL_WITH_ROTATED_CROSS);
   private final YoGraphicPosition centerOfPressureSLViz = new YoGraphicPosition("centerOfPressureSLViz", centerOfPressureSLLocation, 0.01, YoAppearance.Crimson(), GraphicType.BALL_WITH_ROTATED_CROSS);
   private final DoubleYoVariable centerOfPressureSR = new DoubleYoVariable("centerOfPressureSR", registry);
   private final DoubleYoVariable centerOfPressureSL = new DoubleYoVariable("centerOfPressureSL", registry);
   private final DoubleYoVariable desiredCoPRatioFrontToBack = new DoubleYoVariable("desiredCoPRatioFrontToBack", registry);
   private final DoubleYoVariable distanceDesiredCoPFromMidline = new DoubleYoVariable("distanceDesiredCoPFromMidline", registry);
   private final DoubleYoVariable halfStanceWidth = new DoubleYoVariable("halfStanceWidth", registry);
   private final DoubleYoVariable desiredCoPRatioCenterToSide = new DoubleYoVariable("desiredCoPRatioCenterToSide", registry);
   private final DoubleYoVariable hackySR = new DoubleYoVariable("hackySR", registry);
   private final DoubleYoVariable hackySL = new DoubleYoVariable("hackySL", registry);
   
   private final BooleanYoVariable enableTrot = new BooleanYoVariable("enableTrot", registry);
   private final DoubleYoVariable timeInTrot = new DoubleYoVariable("timeInTrot", registry);

   private final DoubleYoVariable forward_vel = new DoubleYoVariable("forward_vel", registry);
   private final DoubleYoVariable sideways_vel = new DoubleYoVariable("sideways_vel", registry);

   private final DoubleYoVariable forward_vel_des = new DoubleYoVariable("forward_vel_des", registry);
   private final DoubleYoVariable sideways_vel_des = new DoubleYoVariable("sideways_vel_des", registry);

   private AlphaFilteredYoVariable forward_vel_des_smoothed = new AlphaFilteredYoVariable("forward_vel_des_smoothed", registry, 0.999);
   private AlphaFilteredYoVariable sideways_vel_des_smoothed = new AlphaFilteredYoVariable("sideways_vel_des_smoothed", registry, 0.999);

   private final DoubleYoVariable body_rel_x = new DoubleYoVariable("body_rel_x", registry);
   private final DoubleYoVariable body_rel_y = new DoubleYoVariable("body_rel_y", registry);
   private final DoubleYoVariable body_rel_z = new DoubleYoVariable("body_rel_z", registry);
   private final DoubleYoVariable body_rel_yaw = new DoubleYoVariable("body_rel_yaw", registry);

   private final DoubleYoVariable Fx = new DoubleYoVariable("Fx", registry);
   private final DoubleYoVariable Fy = new DoubleYoVariable("Fy", registry);
   private final DoubleYoVariable normalForceBodyDesired = new DoubleYoVariable("Fz", registry);
   private final DoubleYoVariable rollTorqueBodyDesired = new DoubleYoVariable("Nx", registry);
   private final DoubleYoVariable pitchTorqueBodyDesired = new DoubleYoVariable("Ny", registry);
   private final DoubleYoVariable yawTorqueBodyDesired = new DoubleYoVariable("Nz", registry);

   private final DoubleYoVariable k_x = new DoubleYoVariable("k_x", registry);
   private final DoubleYoVariable k_y = new DoubleYoVariable("k_y", registry);
   private final DoubleYoVariable k_z = new DoubleYoVariable("k_z", registry);
   private final DoubleYoVariable k_roll = new DoubleYoVariable("k_roll", registry);
   private final DoubleYoVariable k_pitch = new DoubleYoVariable("k_pitch", registry);
   private final DoubleYoVariable k_yaw = new DoubleYoVariable("k_yaw", registry);

   private final DoubleYoVariable b_x = new DoubleYoVariable("b_x", registry);
   private final DoubleYoVariable b_y = new DoubleYoVariable("b_y", registry);
   private final DoubleYoVariable b_z = new DoubleYoVariable("b_z", registry);
   private final DoubleYoVariable ki_z = new DoubleYoVariable("ki_z", registry);
   private final DoubleYoVariable i_z = new DoubleYoVariable("i_z", registry);
   private final DoubleYoVariable b_roll = new DoubleYoVariable("b_roll", registry);
   private final DoubleYoVariable b_pitch = new DoubleYoVariable("b_pitch", registry);
   private final DoubleYoVariable b_yaw = new DoubleYoVariable("b_yaw", registry);

   private final DoubleYoVariable ff_z = new DoubleYoVariable("ff_z", registry);
   private final DoubleYoVariable fz_limit = new DoubleYoVariable("fz_limit", registry);

   private final DoubleYoVariable q_d_x = new DoubleYoVariable("q_d_x", registry);
   private final DoubleYoVariable q_d_y = new DoubleYoVariable("q_d_y", registry);
   private final DoubleYoVariable q_d_z = new DoubleYoVariable("q_d_z", registry);

   private final DoubleYoVariable q_roll = new DoubleYoVariable("q_roll", registry);
   private final DoubleYoVariable q_pitch = new DoubleYoVariable("q_pitch", registry);
   private final DoubleYoVariable q_yaw = new DoubleYoVariable("q_yaw", registry);

   private final DoubleYoVariable q_d_roll = new DoubleYoVariable("q_d_roll", registry);
   private final DoubleYoVariable q_d_pitch = new DoubleYoVariable("q_d_pitch", registry);
   private final DoubleYoVariable q_d_yaw = new DoubleYoVariable("q_d_yaw", registry);

   private final QuadrantDependentList<YoFrameVector> vmcFootForces = new QuadrantDependentList<>();

   private final QuadrantDependentList<ArrayList<OneDoFJoint>> oneDofJoints = new QuadrantDependentList<>();
   private final HashMap<String, DoubleYoVariable> desiredTorques = new HashMap<>();

   private final DoubleYoVariable qd_d_z = new DoubleYoVariable("qd_d_z", registry);
   private final DoubleYoVariable qd_d_yaw = new DoubleYoVariable("qd_d_yaw", registry);

   private final IntegerYoVariable numberOfFeetInContact = new IntegerYoVariable("numberOfFeetInContact", registry);
   
   private final FramePoint hindFootInBodyZUp = new FramePoint();
   private final FramePoint frontFootInBodyZUp = new FramePoint();
   private final FramePoint footInBodyZUp = new FramePoint();
   private final FramePoint jointInBodyZUp = new FramePoint();
   private final FrameVector jointToFootVector = new FrameVector();
   private final FrameVector vmcRequestedTorqueFromJointXYZ = new FrameVector();
   private final FrameVector jointAxis = new FrameVector();
   
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
      this.centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      this.dt = DT;
      
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
         vmcFootForces.set(robotQuadrant, new YoFrameVector("vmcFootForces" + robotQuadrant.getCamelCaseNameForMiddleOfExpression(), referenceFrames.getBodyZUpFrame(), registry));
      }

      stateMachine = new StateMachine<QuadrupedWalkingState>("walkingState", "switchTime", QuadrupedWalkingState.class, yoTime, registry);
      setupStateMachine();

      yoGraphicsListRegistry.registerArtifact("icpViz", icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfMassViz", centerOfMassViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfPressureViz", centerOfPressureViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfPressureSRViz", centerOfPressureSRViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfPressureSLViz", centerOfPressureSLViz.createArtifact());
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
       
         YoFramePoint footPosition = new YoFramePoint(prefix, worldFrame, registry);
         YoGraphicPosition footPositionViz = new YoGraphicPosition(prefix + "FootPositionViz", footPosition, 0.02, YoAppearance.Color(robotQuadrant.getColor()),
               GraphicType.BALL_WITH_CROSS);
         
         yoGraphicsListRegistry.registerArtifact("feet", footPositionViz.createArtifact());
         feetLocations.set(robotQuadrant, footPosition);
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

   /**
    * Delete this and use the inverse dynamics robot
    * @param registry
    */
   private void setupHackyYoVariables(YoVariableRegistry registry)
   {
      if (qd_z == null)
      {
         YoVariableRegistry currentRegistry = registry;
         while (currentRegistry.getParent() != null)
         {
            currentRegistry = currentRegistry.getParent();
         }
         qd_z = (DoubleYoVariable) currentRegistry.getVariable("qd_z");
         qd_wx = (DoubleYoVariable) currentRegistry.getVariable("qd_wx");
         qd_wy = (DoubleYoVariable) currentRegistry.getVariable("qd_wy");
         qd_wz = (DoubleYoVariable) currentRegistry.getVariable("qd_wz");
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

   @Override
   public void doAction()
   {
      setupHackyYoVariables(registry);
      updateEstimates();
      forward_vel_des_smoothed.update(forward_vel_des.getDoubleValue());
      sideways_vel_des_smoothed.update(sideways_vel_des.getDoubleValue());

      q_d_yaw.set(q_d_yaw.getDoubleValue() + qd_d_yaw.getDoubleValue() * dt);

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
      bodyPose.setToZero(referenceFrames.getBodyFrame());
      bodyPose.changeFrame(worldFrame);

      //update orientation qs
      q_roll.set(bodyPose.getRoll());
      q_pitch.set(bodyPose.getPitch());
      q_yaw.set(bodyPose.getYaw());

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
      fourFootSupportPolygon.getCentroid2d(centroid);

      //update relative offset from center of feet to center of body
      double body_x = bodyPose.getX() - centroid.getX();
      double body_y = bodyPose.getY() - centroid.getY();
      double yaw = bodyPose.getYaw();

      body_rel_x.set(Math.cos(yaw) * body_x + Math.sin(yaw) * body_y);
      body_rel_y.set(-Math.sin(yaw) * body_x + Math.cos(yaw) * body_y);

      // compute center of mass position and velocity
      coMPosition.setToZero(referenceFrames.getCenterOfMassFrame());
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
   }

   private void computeFeetContactState()
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

   //Control X and Y using Center of Pressure on each trot line, SR and SL.
   private void doTrotControl()
   {
      double yAdjust = k_y.getDoubleValue() * (q_d_y.getDoubleValue() - body_rel_y.getDoubleValue())
            + b_y.getDoubleValue() * (sideways_vel_des_smoothed.getDoubleValue() - sideways_vel.getDoubleValue());
      yAdjust = yAdjust / 100.0;

      double xAdjust = k_x.getDoubleValue() * (q_d_x.getDoubleValue() - body_rel_x.getDoubleValue())
            + b_x.getDoubleValue() * (forward_vel_des_smoothed.getDoubleValue() - forward_vel.getDoubleValue());
      xAdjust = xAdjust / 100.0;

      if (yAdjust > 0.3)
         yAdjust = 0.3;
      if (yAdjust < -0.3)
         yAdjust = -0.3;

      if (xAdjust > 0.3)
         xAdjust = 0.3;
      if (xAdjust < -0.3)
         xAdjust = -0.3;
      
      double distanceFH = feetLocations.get(RobotQuadrant.HIND_LEFT).distance(feetLocations.get(RobotQuadrant.FRONT_LEFT));
      
      GeometryTools.averagePoints(feetLocations.get(RobotQuadrant.FRONT_LEFT).getFrameTuple(), feetLocations.get(RobotQuadrant.FRONT_RIGHT).getFrameTuple(), frontMidPoint);
      GeometryTools.averagePoints(feetLocations.get(RobotQuadrant.HIND_LEFT).getFrameTuple(), feetLocations.get(RobotQuadrant.HIND_RIGHT).getFrameTuple(), hindMidPoint);
      
      desiredICP.set(centroid);
      desiredICP.add(desiredICPFromCentroid.getX(), desiredICPFromCentroid.getY(), 0.0);
      
      desiredCenterOfPressure.set(icp);
      desiredCenterOfPressure.scale(2.0);
      desiredCenterOfPressure.sub(desiredICP);
      
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
      
      hackySR.set(desiredCoPRatioFrontToBack.getDoubleValue() + desiredCoPRatioCenterToSide.getDoubleValue());
      hackySL.set(desiredCoPRatioFrontToBack.getDoubleValue() - desiredCoPRatioCenterToSide.getDoubleValue());
      
      centerOfPressureSR.set(hackySR.getDoubleValue() - yAdjust + xAdjust);
      centerOfPressureSL.set(hackySL.getDoubleValue() + yAdjust + xAdjust);
      
      centerOfPressureSRLocation.set(feetLocations.get(RobotQuadrant.HIND_LEFT));
      centerOfPressureSRLocation.sub(feetLocations.get(RobotQuadrant.FRONT_RIGHT));
      centerOfPressureSRLocation.scale(centerOfPressureSR.getDoubleValue());
      centerOfPressureSRLocation.add(feetLocations.get(RobotQuadrant.FRONT_RIGHT));
      
      centerOfPressureSLLocation.set(feetLocations.get(RobotQuadrant.HIND_RIGHT));
      centerOfPressureSLLocation.sub(feetLocations.get(RobotQuadrant.FRONT_LEFT));
      centerOfPressureSLLocation.scale(centerOfPressureSL.getDoubleValue());
      centerOfPressureSLLocation.add(feetLocations.get(RobotQuadrant.FRONT_LEFT));

      // Use PD Controller on Fz to control body height
      i_z.set(i_z.getDoubleValue() + (q_d_z.getDoubleValue() - body_rel_z.getDoubleValue()));
      normalForceBodyDesired.set(k_z.getDoubleValue() * (q_d_z.getDoubleValue() - body_rel_z.getDoubleValue())
            + b_z.getDoubleValue() * (qd_d_z.getDoubleValue() - qd_z.getDoubleValue()) + ff_z.getDoubleValue()
            + ki_z.getDoubleValue() * i_z.getDoubleValue());

      // CAP z force.
      if (normalForceBodyDesired.getDoubleValue() > fz_limit.getDoubleValue())
      {
         normalForceBodyDesired.set(fz_limit.getDoubleValue());
      }
      // MIN z force
      if (normalForceBodyDesired.getDoubleValue() < 10.0)
      {
         normalForceBodyDesired.set(10.0);
      }

      // Use PD Controller on Nx, Ny, Nz to control orientation of the body
      rollTorqueBodyDesired.set(k_roll.getDoubleValue() * (q_d_roll.getDoubleValue() - q_roll.getDoubleValue()) - b_roll.getDoubleValue() * qd_wx.getDoubleValue());
      pitchTorqueBodyDesired.set(k_pitch.getDoubleValue() * (q_d_pitch.getDoubleValue() - q_pitch.getDoubleValue()) - b_pitch.getDoubleValue() * qd_wy.getDoubleValue());
      yawTorqueBodyDesired.set(k_yaw.getDoubleValue() * (q_d_yaw.getDoubleValue() - q_yaw.getDoubleValue()) - b_yaw.getDoubleValue() * qd_wz.getDoubleValue());
   }

   private void computeBodyRelativePositionsVelocities()
   {
      double footZ = fourFootSupportPolygon.getLowestFootstepZHeight();
      body_rel_z.set(bodyPose.getZ() - footZ);
   }

   private void distributeForcesFourLegs()
   {
      double normalForceBodyDesiredRight = percentOfForcesOnRightTrotPair.getDoubleValue() * normalForceBodyDesired.getDoubleValue();
      double rollTorqueBodyDesiredRight = percentOfForcesOnRightTrotPair.getDoubleValue() * rollTorqueBodyDesired.getDoubleValue();
      double pitchTorqueBodyDesiredRight = percentOfForcesOnRightTrotPair.getDoubleValue() * pitchTorqueBodyDesired.getDoubleValue();
      double yawTorqueBodyDesiredRight = percentOfForcesOnRightTrotPair.getDoubleValue() * yawTorqueBodyDesired.getDoubleValue();

      double normalForceBodyDesiredLeft = (1.0 - percentOfForcesOnRightTrotPair.getDoubleValue()) * normalForceBodyDesired.getDoubleValue();
      double NrollL = (1.0 - percentOfForcesOnRightTrotPair.getDoubleValue()) * rollTorqueBodyDesired.getDoubleValue();
      double NyL = (1.0 - percentOfForcesOnRightTrotPair.getDoubleValue()) * pitchTorqueBodyDesired.getDoubleValue();
      double NzL = (1.0 - percentOfForcesOnRightTrotPair.getDoubleValue()) * yawTorqueBodyDesired.getDoubleValue();

      clearLegForces();
      distributeForcesTwoDiagonalLegs(TrotPair.TROT_RIGHT, centerOfPressureSR.getDoubleValue(), normalForceBodyDesiredRight, rollTorqueBodyDesiredRight, pitchTorqueBodyDesiredRight, yawTorqueBodyDesiredRight);
      distributeForcesTwoDiagonalLegs(TrotPair.TROT_LEFT, centerOfPressureSL.getDoubleValue(), normalForceBodyDesiredLeft, NrollL, NyL, NzL);
   }

   private void distributeForcesTwoDiagonalLegs(TrotPair trotPair, double centerOfPressureS, double FzAlpha, double NxAlpha, double NyAlpha, double NzAlpha)
   {
      getFootInBodyZUpFrame(trotPair.getFrontQuadrant(), frontFootInBodyZUp);
      getFootInBodyZUpFrame(trotPair.getHindQuadrant(), hindFootInBodyZUp);

      DenseMatrix64F forceConstraintsMatrix = new DenseMatrix64F(5, 6);
      forceConstraintsMatrix.zero();

      forceConstraintsMatrix.set(0, 5, 1.0 / FzAlpha);
      forceConstraintsMatrix.set(1, 2, 1.0);
      forceConstraintsMatrix.set(1, 5, 1.0);

      forceConstraintsMatrix.set(2, 1, -frontFootInBodyZUp.getZ());
      forceConstraintsMatrix.set(2, 2, frontFootInBodyZUp.getY());
      forceConstraintsMatrix.set(2, 4, -hindFootInBodyZUp.getZ());
      forceConstraintsMatrix.set(2, 5, hindFootInBodyZUp.getY());

      forceConstraintsMatrix.set(3, 0, frontFootInBodyZUp.getZ());
      forceConstraintsMatrix.set(3, 2, -frontFootInBodyZUp.getX());
      forceConstraintsMatrix.set(3, 3, hindFootInBodyZUp.getZ());
      forceConstraintsMatrix.set(3, 5, -hindFootInBodyZUp.getX());

      forceConstraintsMatrix.set(4, 0, -frontFootInBodyZUp.getY());
      forceConstraintsMatrix.set(4, 1, frontFootInBodyZUp.getX());
      forceConstraintsMatrix.set(4, 3, -hindFootInBodyZUp.getY());
      forceConstraintsMatrix.set(4, 4, hindFootInBodyZUp.getX());

      DenseMatrix64F totalForcesOnTheBody = new DenseMatrix64F(5, 1);

      totalForcesOnTheBody.set(0, 0, centerOfPressureS);
      totalForcesOnTheBody.set(1, 0, FzAlpha);
      totalForcesOnTheBody.set(2, 0, NxAlpha);
      totalForcesOnTheBody.set(3, 0, NyAlpha);
      totalForcesOnTheBody.set(4, 0, NzAlpha);

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

      vmcFootForces.get(trotPair.getFrontQuadrant()).getYoX().set(distributedForcesOnTheLegs.get(0, 0));
      vmcFootForces.get(trotPair.getFrontQuadrant()).getYoY().set(distributedForcesOnTheLegs.get(1, 0));
//      vmcFootForces.get(trotPair.getFrontQuadrant()).getYoZ().set(distributedForcesOnTheLegs.get(2, 0));
      vmcFootForces.get(trotPair.getHindQuadrant()).getYoX().set(distributedForcesOnTheLegs.get(3, 0));
      vmcFootForces.get(trotPair.getHindQuadrant()).getYoY().set(distributedForcesOnTheLegs.get(4, 0));
//      vmcFootForces.get(trotPair.getHindQuadrant()).getYoZ().set(distributedForcesOnTheLegs.get(5, 0));
      
      vmcFootForces.get(trotPair.getFrontQuadrant()).getYoZ().set((1.0 - centerOfPressureS) * FzAlpha);
      vmcFootForces.get(trotPair.getHindQuadrant()).getYoZ().set(centerOfPressureS * FzAlpha);
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
         jointInBodyZUp.changeFrame(referenceFrames.getBodyZUpFrame());

         jointToFootVector.setIncludingFrame(footInBodyZUp);
         jointToFootVector.sub(jointInBodyZUp);

         vmcRequestedTorqueFromJointXYZ.setToZero(referenceFrames.getBodyZUpFrame());
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
      framePointToPack.changeFrame(referenceFrames.getBodyZUpFrame());
   }

   private void clearLegForces()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         vmcFootForces.get(robotQuadrant).setToZero();
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
         percentOfForcesOnRightTrotPair.set(0.5);
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
         percentOfForcesOnRightTrotPair.set(1.01);
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
         percentOfForcesOnRightTrotPair.set(-0.01);
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

      percentOfForcesOnRightTrotPair.set(0.5);
      
      enableTrot.set(false);
      timeInTrot.set(0.2);
      
      centerOfPressureSR.set(0.5);
      centerOfPressureSL.set(0.5);

      k_x.set(0.0); // 2000.0);
      b_x.set(0.0);

      k_y.set(0.0); // 2000.0);
      b_y.set(50.0); // 0.0);    // 50 for pace, 0 for trot.

      k_roll.set(4000.0);
      b_roll.set(50.0);

      k_pitch.set(4000.0); // 80.0);
      b_pitch.set(50.0); // 20.0);

      k_yaw.set(3000.0); // 80.0);    // 250.0);
      b_yaw.set(40.0); // 20.0);    // 100.0);

      k_z.set(30000.0);
      b_z.set(10000.0);
      ki_z.set(20.0);
      fz_limit.set(1000.0);
      q_d_z.set(bodyPose.getZ());
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
