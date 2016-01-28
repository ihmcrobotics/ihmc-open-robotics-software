package us.ihmc.quadrupedRobotics.controller;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
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
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLine;

public class TrotWalkController extends QuadrupedController
{
   private final double dt;
   private final YoVariableRegistry registry = new YoVariableRegistry("TrotWalkController");
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final QuadrupedReferenceFrames referenceFrames;
   private final SDFFullRobotModel fullRobotModel;
   private final QuadrupedStateEstimator stateEstimator;
   private final QuadrantDependentList<YoFramePoint> feetLocations = new QuadrantDependentList<YoFramePoint>();
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final YoFrameVector centerOfMassVelocity = new YoFrameVector("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint coMPosition = new FramePoint();
   private final YoFramePoint centerOfMassPosition = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition centerOfMassViz = new YoGraphicPosition("centerOfMassViz", centerOfMassPosition, 0.02, YoAppearance.Black(),
         GraphicType.BALL_WITH_CROSS);

   private final YoFramePoint icp = new YoFramePoint("icp", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition icpViz = new YoGraphicPosition("icpViz", icp, 0.01, YoAppearance.DarkSlateBlue());
   private final YoGraphicPosition copHindRightFrontLeftTrotLineViz = new YoGraphicPosition("copHindRightFrontLeftTrotLineViz", icp, 0.01, YoAppearance.Purple());
   private final YoGraphicPosition cophindLeftFrontRightTrotLineViz = new YoGraphicPosition("cophindLeftFrontRightTrotLineViz", icp, 0.01, YoAppearance.Purple());
   private final YoArtifactLine hindRightFrontLeftTrotLine;
   private final YoArtifactLine hindLeftFrontRightTrotLine;

   ///Hacky Sim yoVariables.... Delete this and use the inverse dynamics robot
   private DoubleYoVariable qd_z;
   private DoubleYoVariable qd_wx, qd_wy, qd_wz;
   ///

   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final FramePose bodyPose = new FramePose(worldFrame);

   private final DoubleYoVariable quadAlpha = new DoubleYoVariable("quadAlpha", registry);

   private final FramePoint copFramePoint = new FramePoint();
   private final YoFramePoint centerOfPressure = new YoFramePoint("centerOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition centerOfPressureViz = new YoGraphicPosition("centerOfPressureViz", centerOfPressure, 0.01, YoAppearance.Black());
   private final DoubleYoVariable centerOfPressureS1 = new DoubleYoVariable("centerOfPressureS1", registry);
   private final DoubleYoVariable centerOfPressureS2 = new DoubleYoVariable("centerOfPressureS2", registry);

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
   private final DoubleYoVariable Fz = new DoubleYoVariable("Fz", registry);
   private final DoubleYoVariable Nx = new DoubleYoVariable("Nx", registry);
   private final DoubleYoVariable Ny = new DoubleYoVariable("Ny", registry);
   private final DoubleYoVariable Nz = new DoubleYoVariable("Nz", registry);

   private DoubleYoVariable k_x = new DoubleYoVariable("k_x", registry);
   private DoubleYoVariable k_y = new DoubleYoVariable("k_y", registry);
   private DoubleYoVariable k_z = new DoubleYoVariable("k_z", registry);
   private DoubleYoVariable k_roll = new DoubleYoVariable("k_roll", registry);
   private DoubleYoVariable k_pitch = new DoubleYoVariable("k_pitch", registry);
   private DoubleYoVariable k_yaw = new DoubleYoVariable("k_yaw", registry);

   private DoubleYoVariable b_x = new DoubleYoVariable("b_x", registry);
   private DoubleYoVariable b_y = new DoubleYoVariable("b_y", registry);
   private DoubleYoVariable b_z = new DoubleYoVariable("b_z", registry);
   private DoubleYoVariable b_roll = new DoubleYoVariable("b_roll", registry);
   private DoubleYoVariable b_pitch = new DoubleYoVariable("b_pitch", registry);
   private DoubleYoVariable b_yaw = new DoubleYoVariable("b_yaw", registry);

   private DoubleYoVariable ff_z = new DoubleYoVariable("ff_z", registry);
   private DoubleYoVariable fz_limit = new DoubleYoVariable("fz_limit", registry);

   private DoubleYoVariable q_d_x = new DoubleYoVariable("q_d_x", registry);
   private DoubleYoVariable q_d_y = new DoubleYoVariable("q_d_y", registry);
   private DoubleYoVariable q_d_z = new DoubleYoVariable("q_d_z", registry);

   private DoubleYoVariable q_roll = new DoubleYoVariable("q_roll", registry);
   private DoubleYoVariable q_pitch = new DoubleYoVariable("q_pitch", registry);

   private DoubleYoVariable q_d_roll = new DoubleYoVariable("q_d_roll", registry);
   private DoubleYoVariable q_d_pitch = new DoubleYoVariable("q_d_pitch", registry);
   private DoubleYoVariable q_d_yaw = new DoubleYoVariable("q_d_yaw", registry);

   private final DoubleYoVariable Fx_lfore = new DoubleYoVariable("Fx_lfore", registry);
   private final DoubleYoVariable Fy_lfore = new DoubleYoVariable("Fy_lfore", registry);
   private final DoubleYoVariable Fz_lfore = new DoubleYoVariable("Fz_lfore", registry);
   private final DoubleYoVariable Nx_lfore = new DoubleYoVariable("Nx_lfore", registry);
   private final DoubleYoVariable Ny_lfore = new DoubleYoVariable("Ny_lfore", registry);
   private final DoubleYoVariable Nz_lfore = new DoubleYoVariable("Nz_lfore", registry);
   private final DoubleYoVariable Fx_rfore = new DoubleYoVariable("Fx_rfore", registry);
   private final DoubleYoVariable Fy_rfore = new DoubleYoVariable("Fy_rfore", registry);
   private final DoubleYoVariable Fz_rfore = new DoubleYoVariable("Fz_rfore", registry);
   private final DoubleYoVariable Nx_rfore = new DoubleYoVariable("Nx_rfore", registry);
   private final DoubleYoVariable Ny_rfore = new DoubleYoVariable("Ny_rfore", registry);
   private final DoubleYoVariable Nz_rfore = new DoubleYoVariable("Nz_rfore", registry);
   private final DoubleYoVariable Fx_lhind = new DoubleYoVariable("Fx_lhind", registry);
   private final DoubleYoVariable Fy_lhind = new DoubleYoVariable("Fy_lhind", registry);
   private final DoubleYoVariable Fz_lhind = new DoubleYoVariable("Fz_lhind", registry);
   private final DoubleYoVariable Nx_lhind = new DoubleYoVariable("Nx_lhind", registry);
   private final DoubleYoVariable Ny_lhind = new DoubleYoVariable("Ny_lhind", registry);
   private final DoubleYoVariable Nz_lhind = new DoubleYoVariable("Nz_lhind", registry);
   private final DoubleYoVariable Fx_rhind = new DoubleYoVariable("Fx_rhind", registry);
   private final DoubleYoVariable Fy_rhind = new DoubleYoVariable("Fy_rhind", registry);
   private final DoubleYoVariable Fz_rhind = new DoubleYoVariable("Fz_rhind", registry);
   private final DoubleYoVariable Nx_rhind = new DoubleYoVariable("Nx_rhind", registry);
   private final DoubleYoVariable Ny_rhind = new DoubleYoVariable("Ny_rhind", registry);
   private final DoubleYoVariable Nz_rhind = new DoubleYoVariable("Nz_rhind", registry);

   private final YoFrameVector leftForeLegForce = new YoFrameVector(Fx_lfore, Fy_lfore, Fz_lfore, null);
   private final YoFrameVector leftHindLegForce = new YoFrameVector(Fx_lhind, Fy_lhind, Fz_lhind, null);
   private final YoFrameVector rightForeLegForce = new YoFrameVector(Fx_rfore, Fy_rfore, Fz_rfore, null);
   private final YoFrameVector rightHindLegForce = new YoFrameVector(Fx_rhind, Fy_rhind, Fz_rhind, null);

   private final QuadrantDependentList<YoFrameVector> legForces = new QuadrantDependentList<YoFrameVector>(leftForeLegForce, rightForeLegForce,
         leftHindLegForce, rightHindLegForce);

   private final QuadrantDependentList<ArrayList<OneDoFJoint>> oneDofJoints = new QuadrantDependentList<>();
   private final HashMap<String, DoubleYoVariable> desiredTorques = new HashMap<>();

   private DoubleYoVariable qd_d_z = new DoubleYoVariable("qd_d_z", registry);
   private DoubleYoVariable qd_d_yaw = new DoubleYoVariable("qd_d_yaw", registry);

   private final IntegerYoVariable numberOfFeetInContact = new IntegerYoVariable("numberOfFeetInContact", registry);

   private final EnumYoVariable<QuadrupedWalkingState> nextState = new EnumYoVariable<QuadrupedWalkingState>("nextState", "", registry,
         QuadrupedWalkingState.class, false);

   private enum QuadrupedWalkingState
   {
      QuadSupport, LeftHindRightFrontSupport, RightHindLeftFrontSupport, LeftHindLeftFrontSupport, RightHindRightFrontSupport
   }

   private enum TrotPair
   {
      LeftHindRightFront, RightHindLeftFront;
   }

   private final StateMachine<QuadrupedWalkingState> stateMachine;

   public TrotWalkController(QuadrupedRobotParameters robotParameters, SDFFullRobotModel fullRobotModel, QuadrupedStateEstimator stateEstimator, double DT,
         DoubleYoVariable yoTime, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(QuadrupedControllerState.TROT_WALK);
      this.fullRobotModel = fullRobotModel;
      this.stateEstimator = stateEstimator;
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

      stateMachine = new StateMachine<QuadrupedWalkingState>("walkingState", "switchTime", QuadrupedWalkingState.class, yoTime, registry);
      setupStateMachine();

      yoGraphicsListRegistry.registerArtifact("icpViz", icpViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfMassViz", centerOfMassViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfPressureViz", centerOfPressureViz.createArtifact());
      
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

      stateMachine.addState(quadSupportState);
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
      centerOfMassJacobian.compute();
      centerOfMassJacobian.packCenterOfMassVelocity(comVelocity);
      comVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassVelocity.set(comVelocity);

      // compute instantaneous capture point
      double lowestFootZ = fourFootSupportPolygon.getLowestFootStepZHeight();
      double zDelta = coMPosition.getZ() - lowestFootZ;
      double omega = Math.sqrt(9.81 / zDelta);
      icp.setX(coMPosition.getX() + centerOfMassVelocity.getX() / omega);
      icp.setY(coMPosition.getY() + centerOfMassVelocity.getY() / omega);
      icp.setZ(lowestFootZ);
      
      //update CoP
      double fzTotal = 0.0;
      copFramePoint.setToZero(worldFrame);
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoFramePoint foot = feetLocations.get(robotQuadrant);
         YoFrameVector legForce = legForces.get(robotQuadrant);

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

   public void computeFeetContactState()
   {
      numberOfFeetInContact.set(0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (stateEstimator.isFootInContact(robotQuadrant))
         {
            numberOfFeetInContact.increment();
         }
      }
   }

   public void positionControllersFourLegs()
   {
      //Control X and Y using Center of Pressure on each trot line, S1 and S2.
      quadAlpha.set(0.5);

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
      
      
      
      double icpX = icp.getX();
      double icpY = icp.getY();
      
      double copX = centerOfPressure.getX();
      double copY = centerOfPressure.getY();
      
      
      
      
      centerOfPressureS1.set(0.5 - yAdjust + xAdjust);
      centerOfPressureS2.set(0.5 + yAdjust + xAdjust);

      // Use PD Controller on Fz to control body height
      Fz.set(k_z.getDoubleValue() * (q_d_z.getDoubleValue() - body_rel_z.getDoubleValue())
            + b_z.getDoubleValue() * (qd_d_z.getDoubleValue() - qd_z.getDoubleValue()) + ff_z.getDoubleValue());

      // CAP z force.
      if (Fz.getDoubleValue() > fz_limit.getDoubleValue())
      {
         Fz.set(fz_limit.getDoubleValue());
      }

      if (Fz.getDoubleValue() < 10.0)
      {
         Fz.set(10.0);
      }

      // Use PD Controller on Nx, Ny, Nz to control orientation of the body
      Nx.set(k_roll.getDoubleValue() * (q_d_roll.getDoubleValue() - q_roll.getDoubleValue()) - b_roll.getDoubleValue() * qd_wx.getDoubleValue());
      Ny.set(k_pitch.getDoubleValue() * (q_d_pitch.getDoubleValue() - q_pitch.getDoubleValue()) - b_pitch.getDoubleValue() * qd_wy.getDoubleValue());
      Nz.set(k_yaw.getDoubleValue() * computeYawError(q_d_yaw.getDoubleValue(), body_rel_yaw.getDoubleValue())
            - b_yaw.getDoubleValue() * qd_wz.getDoubleValue());
   }

   public double computeYawError(double q_d_yaw, double q_yaw)
   {
      q_d_yaw = q_d_yaw % (2.0 * Math.PI);
      q_yaw = q_yaw % (2.0 * Math.PI);

      double q_err_yaw = q_d_yaw - q_yaw;

      if (Math.abs(q_err_yaw) > Math.PI)
      {
         if (q_err_yaw > Math.PI)
         {
            q_err_yaw = q_err_yaw - 2.0 * Math.PI;
         }
         else if (q_err_yaw < -Math.PI)
         {
            q_err_yaw = q_err_yaw + 2.0 * Math.PI;
         }
      }

      return q_err_yaw;
   }

   private void computeBodyRelativePositionsVelocities()
   {
      double footZ = fourFootSupportPolygon.getLowestFootStepZHeight();
      body_rel_z.set(bodyPose.getZ() - footZ);
   }

   private void distributeForcesFourLegs(double quadAlpha, double centerOfPressureS1, double centerOfPressureS2, double Fz, double Nx, double Ny, double Nz)
   {
      double Fz1 = quadAlpha * Fz;
      double Nx1 = quadAlpha * Nx;
      double Ny1 = quadAlpha * Ny;
      double Nz1 = quadAlpha * Nz;

      double Fz2 = (1.0 - quadAlpha) * Fz;
      double Nx2 = (1.0 - quadAlpha) * Nx;
      double Ny2 = (1.0 - quadAlpha) * Ny;
      double Nz2 = (1.0 - quadAlpha) * Nz;

      clearLegForces();
      distributeForcesTwoDiagonalLegs(TrotPair.LeftHindRightFront, centerOfPressureS1, Fz1, Nx1, Ny1, Nz1);
      distributeForcesTwoDiagonalLegs(TrotPair.RightHindLeftFront, centerOfPressureS2, Fz2, Nx2, Ny2, Nz2);
   }

   public void clearLegForces()
   {
      Nx_lfore.set(0.0);
      Nx_rfore.set(0.0);
      Nx_lhind.set(0.0);
      Nx_rhind.set(0.0);
      Ny_lfore.set(0.0);
      Ny_rfore.set(0.0);
      Ny_lhind.set(0.0);
      Ny_rhind.set(0.0);
      Nz_lfore.set(0.0);
      Nz_rfore.set(0.0);
      Nz_lhind.set(0.0);
      Nz_rhind.set(0.0);

      Fx_rfore.set(0.0);
      Fy_rfore.set(0.0);
      Fz_rfore.set(0.0);
      Fx_lhind.set(0.0);
      Fy_lhind.set(0.0);
      Fz_lhind.set(0.0);

      Fx_lfore.set(0.0);
      Fy_lfore.set(0.0);
      Fz_lfore.set(0.0);
      Fx_rhind.set(0.0);
      Fy_rhind.set(0.0);
      Fz_rhind.set(0.0);
   }

   private void getFootInBodyZUpFrame(RobotQuadrant footQuadrant, FramePoint framePointToPack)
   {
      ReferenceFrame footFrame = referenceFrames.getFootFrame(footQuadrant);
      framePointToPack.setToZero(footFrame);
      framePointToPack.changeFrame(referenceFrames.getBodyZUpFrame());
   }

   FramePoint hindFootInBodyZUp = new FramePoint();
   FramePoint foreFootInBodyZUp = new FramePoint();

   private void distributeForcesTwoDiagonalLegs(TrotPair trotPair, double centerOfPressureS, double Fz, double Nx, double Ny, double Nz)
   {
      if (Fz <= 0.0)
      {
         System.err.println("Fz <= 0.0");
      }

      if (trotPair == TrotPair.LeftHindRightFront)
      {
         getFootInBodyZUpFrame(RobotQuadrant.FRONT_RIGHT, foreFootInBodyZUp);
         getFootInBodyZUpFrame(RobotQuadrant.HIND_LEFT, hindFootInBodyZUp);
      }
      else
      {
         getFootInBodyZUpFrame(RobotQuadrant.FRONT_LEFT, foreFootInBodyZUp);
         getFootInBodyZUpFrame(RobotQuadrant.HIND_RIGHT, hindFootInBodyZUp);
      }

      DenseMatrix64F forceConstraintsMatrix = new DenseMatrix64F(5, 6);
      forceConstraintsMatrix.zero();

      forceConstraintsMatrix.set(0, 5, 1.0 / Fz);
      forceConstraintsMatrix.set(1, 2, 1.0);
      forceConstraintsMatrix.set(1, 5, 1.0);

      forceConstraintsMatrix.set(2, 1, -foreFootInBodyZUp.getZ());
      forceConstraintsMatrix.set(2, 2, foreFootInBodyZUp.getY());
      forceConstraintsMatrix.set(2, 4, -hindFootInBodyZUp.getZ());
      forceConstraintsMatrix.set(2, 5, hindFootInBodyZUp.getY());

      forceConstraintsMatrix.set(3, 0, foreFootInBodyZUp.getZ());
      forceConstraintsMatrix.set(3, 2, -foreFootInBodyZUp.getX());
      forceConstraintsMatrix.set(3, 3, hindFootInBodyZUp.getZ());
      forceConstraintsMatrix.set(3, 5, -hindFootInBodyZUp.getX());

      forceConstraintsMatrix.set(4, 0, -foreFootInBodyZUp.getY());
      forceConstraintsMatrix.set(4, 1, foreFootInBodyZUp.getX());
      forceConstraintsMatrix.set(4, 3, -hindFootInBodyZUp.getY());
      forceConstraintsMatrix.set(4, 4, hindFootInBodyZUp.getX());

      DenseMatrix64F totalForcesOnTheBody = new DenseMatrix64F(5, 1);

      totalForcesOnTheBody.set(0, 0, centerOfPressureS);
      totalForcesOnTheBody.set(1, 0, Fz);
      totalForcesOnTheBody.set(2, 0, Nx);
      totalForcesOnTheBody.set(3, 0, Ny);
      totalForcesOnTheBody.set(4, 0, Nz);

      DenseMatrix64F forceConstraintsMatrixInverse = new DenseMatrix64F(6, 5);
      try
      {
         CommonOps.pinv(forceConstraintsMatrix, forceConstraintsMatrixInverse);
      }
      catch (Exception e)
      {
         System.out.println("forceConstraintsMatrix = " + forceConstraintsMatrix);

         return;
      }

      DenseMatrix64F distributedForcesOnTheLegs = new DenseMatrix64F(6, 1);
      CommonOps.mult(forceConstraintsMatrixInverse, totalForcesOnTheBody, distributedForcesOnTheLegs);

      //    System.out.println("forceConstraintsMatrix = " + forceConstraintsMatrix);
      //    System.out.println("totalForcesOnTheBody = " + totalForcesOnTheBody);
      //    System.out.println("distributedForcesOnTheLegs = " + distributedForcesOnTheLegs);

      if (trotPair == TrotPair.LeftHindRightFront)
      {
         Fx_rfore.set(distributedForcesOnTheLegs.get(0, 0));
         Fy_rfore.set(distributedForcesOnTheLegs.get(1, 0));
         Fz_rfore.set(distributedForcesOnTheLegs.get(2, 0));
         Fx_lhind.set(distributedForcesOnTheLegs.get(3, 0));
         Fy_lhind.set(distributedForcesOnTheLegs.get(4, 0));
         Fz_lhind.set(distributedForcesOnTheLegs.get(5, 0));
      }
      else
      {
         Fx_lfore.set(distributedForcesOnTheLegs.get(0, 0));
         Fy_lfore.set(distributedForcesOnTheLegs.get(1, 0));
         Fz_lfore.set(distributedForcesOnTheLegs.get(2, 0));
         Fx_rhind.set(distributedForcesOnTheLegs.get(3, 0));
         Fy_rhind.set(distributedForcesOnTheLegs.get(4, 0));
         Fz_rhind.set(distributedForcesOnTheLegs.get(5, 0));
      }
   }

   public void computeStanceJacobiansThreeLegsNew(boolean lHind, boolean rHind, boolean lFore, boolean rFore)
   {
      if (lHind)
         computeStanceJacobianNew(RobotQuadrant.HIND_LEFT);
      if (rHind)
         computeStanceJacobianNew(RobotQuadrant.HIND_RIGHT);
      if (lFore)
         computeStanceJacobianNew(RobotQuadrant.FRONT_LEFT);
      if (rFore)
         computeStanceJacobianNew(RobotQuadrant.FRONT_RIGHT);
   }

   private final FramePoint footInBodyZUp = new FramePoint();

   public void computeStanceJacobianNew(RobotQuadrant robotQuadrant)
   {
      //      ArrayList<ReferenceFrame> legJointFrames = rob.getLegJointFramesForAQuadrant(robotQuadrant);
      ArrayList<OneDoFJoint> legJoints = oneDofJoints.get(robotQuadrant);

      YoFrameVector legForceVector = legForces.get(robotQuadrant);
      getFootInBodyZUpFrame(robotQuadrant, footInBodyZUp);

      Vector3d legForceVector3d = legForceVector.getVector3dCopy();
      FrameVector legForceVectorInBodyZUp = new FrameVector(referenceFrames.getBodyZUpFrame(), legForceVector3d);

      for (int i = 0; i < legJoints.size(); i++)
      {
         OneDoFJoint legJoint = legJoints.get(i);
         ReferenceFrame legJointFrame = legJoint.getFrameBeforeJoint();
         FramePoint jointPositionInBodyZUp = new FramePoint(legJointFrame);
         jointPositionInBodyZUp.changeFrame(referenceFrames.getBodyZUpFrame());

         FrameVector vectorFromJointToFoot = new FrameVector(footInBodyZUp);
         vectorFromJointToFoot.sub(jointPositionInBodyZUp);

         FrameVector torqueVector = new FrameVector(referenceFrames.getBodyZUpFrame());

         torqueVector.cross(vectorFromJointToFoot, legForceVectorInBodyZUp);

         torqueVector.changeFrame(legJointFrame);

         FrameVector jointAxis = new FrameVector();
         legJoint.getJointAxis(jointAxis);

         double torque = jointAxis.dot(torqueVector);
         desiredTorques.get(legJoint.getName()).set(-torque);
         legJoint.setTau(-torque);
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

         if (nextState.getEnumValue() == QuadrupedWalkingState.LeftHindRightFrontSupport)
         {
            // computeCOMPQ(leftHindLimb, rightForeLimb);
         }
         else
         {
            // computeCOMPQ(rightHindLimb, leftForeLimb);
         }

         positionControllersFourLegs();
         distributeForcesFourLegs(quadAlpha.getDoubleValue(), centerOfPressureS1.getDoubleValue(), centerOfPressureS2.getDoubleValue(), Fz.getDoubleValue(),
               Nx.getDoubleValue(), Ny.getDoubleValue(), Nz.getDoubleValue());

         preventSlippingForces();

         computeStanceJacobiansThreeLegsNew(true, true, true, true);
      }

      @Override
      public void doTransitionIntoAction()
      {
         
      }

      @Override
      public void doTransitionOutOfAction()
      {
         
      }
   }

   public void preventSlippingForces()
   {
      if (Fz_lhind.getDoubleValue() < 2.0)
      {
         Fz_lhind.set(2.0);
         Nx_lhind.set(0.0);
         Ny_lhind.set(0.0);
      }

      if (Fz_rhind.getDoubleValue() < 2.0)
      {
         Fz_rhind.set(2.0);
         Nx_rhind.set(0.0);
         Ny_rhind.set(0.0);
      }

      if (Fz_lfore.getDoubleValue() < 2.0)
      {
         Fz_lfore.set(2.0);
         Nx_lfore.set(0.0);
         Ny_lfore.set(0.0);
      }

      if (Fz_rfore.getDoubleValue() < 2.0)
      {
         Fz_rfore.set(2.0);
         Nx_rfore.set(0.0);
         Ny_rfore.set(0.0);
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

      centerOfPressureS1.set(0.5);
      centerOfPressureS2.set(0.5);

      k_x.set(0.0); // 2000.0);
      b_x.set(0.0);

      k_y.set(0.0); // 2000.0);
      b_y.set(50.0); // 0.0);    // 50 for pace, 0 for trot.

      k_roll.set(300.0);
      b_roll.set(50.0);

      k_pitch.set(500.0); // 80.0);
      b_pitch.set(50.0); // 20.0);

      k_yaw.set(300.0); // 80.0);    // 250.0);
      b_yaw.set(50.0); // 20.0);    // 100.0);

      k_z.set(6000.0);
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
