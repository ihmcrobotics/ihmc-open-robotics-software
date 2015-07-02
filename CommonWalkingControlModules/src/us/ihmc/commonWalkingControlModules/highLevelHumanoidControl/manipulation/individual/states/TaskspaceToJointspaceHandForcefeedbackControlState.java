package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.RateLimitedYoVariable;
import us.ihmc.yoUtilities.math.trajectories.PoseTrajectoryGenerator;


public class TaskspaceToJointspaceHandForcefeedbackControlState extends TrajectoryBasedTaskspaceHandControlState
{
   private final static boolean DEBUG = true;


   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private PoseTrajectoryGenerator poseTrajectoryGenerator;

   private final FramePose desiredPose = new FramePose();

   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   private TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator;

   private enum JointControlMode{FORCE_CONTROL, POSITION_CONTROL};
   private JointControlMode jointControlMode;

   private final double dtControl;
   private final OneDoFJoint [] oneDoFJoints;
   private final boolean [] doIntegrateDesiredAccelerations;
   private final LinkedHashMap<OneDoFJoint, PIDController> jointPIDControllers;
   private final LinkedHashMap<OneDoFJoint, RateLimitedYoVariable> jointRateLimitedAccelerations;
   private final DoubleYoVariable jointMaxAcceleration;

   private final DoubleYoVariable currentTimeInState;

   public TaskspaceToJointspaceHandForcefeedbackControlState( String namePrefix, HandControlState stateEnum, MomentumBasedController momentumBasedController, int jacobianId,
         RigidBody base, RigidBody endEffector, boolean doPositionControlOnJoints, YoPIDGains gains, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, stateEnum, momentumBasedController, jacobianId, base, endEffector, parentRegistry);

      currentTimeInState = new DoubleYoVariable(namePrefix + "CurrentTimeInState", registry);
      dtControl = momentumBasedController.getControlDT();



      oneDoFJoints = ScrewTools.createOneDoFJointPath(base, endEffector);
      doIntegrateDesiredAccelerations = new boolean[oneDoFJoints.length];

      for(int i = 0; i < oneDoFJoints.length; i++)
      {
         doIntegrateDesiredAccelerations[i] = oneDoFJoints[i].getIntegrateDesiredAccelerations();
      }


      if(doPositionControlOnJoints)
      {
         jointControlMode = JointControlMode.POSITION_CONTROL;

         // Position Controlled joints, for application on real robot.
         jointPIDControllers = null;
         jointRateLimitedAccelerations = null;
         jointMaxAcceleration = null;
      }
      else
      {
         jointControlMode = JointControlMode.FORCE_CONTROL;
         // Torque Controlled joints, for simulation
         jointPIDControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
         jointRateLimitedAccelerations = new LinkedHashMap<OneDoFJoint, RateLimitedYoVariable>();
         jointMaxAcceleration = gains.getYoMaximumAcceleration();

         for(OneDoFJoint joint : oneDoFJoints)
         {
            String suffix = FormattingTools.lowerCaseFirstLetter(joint.getName());
            PIDController pidController = new PIDController(gains.getYoKp(), gains.getYoKi(), gains.getYoKd(), gains.getYoMaxIntegralError(), suffix, registry);
            jointPIDControllers.put(joint, pidController);

            RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(suffix + "Acceleration", registry, gains.getYoMaximumJerk(), dtControl);
            jointRateLimitedAccelerations.put(joint, rateLimitedAcceleration);
         }
      }
   }

   @Override
   public void setTrajectory(PoseTrajectoryGenerator poseTrajectoryGenerator) {
      this.poseTrajectoryGenerator = poseTrajectoryGenerator;
   }

   @Override
   public void setHoldPositionDuration(double holdPositionDuration) {
      // TODO Auto-generated method stub

   }

   @Override
   public FramePose getDesiredPose() {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public ReferenceFrame getReferenceFrame() {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void setControlModuleForForceControl(RigidBodySpatialAccelerationControlModule handRigidBodySpatialAccelerationControlModule) {
      // TODO Auto-generated method stub

   }

   @Override
   public void setControlModuleForPositionControl(TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator) {
      this.taskspaceToJointspaceCalculator = taskspaceToJointspaceCalculator;

   }

   @Override
   public void doTransitionIntoAction() {
      currentTimeInState.set(0.0);

      poseTrajectoryGenerator.showVisualization();
      poseTrajectoryGenerator.initialize();
      taskspaceToJointspaceCalculator.setSelectionMatrix(selectionMatrix);

      if(jointControlMode == JointControlMode.POSITION_CONTROL)
      {
         enableJointPositionControl();
      }

      PrintTools.debug(this, "Transition into force control state.");
      PrintTools.info(this, "JointControlMode: " + jointControlMode);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if(jointControlMode == JointControlMode.POSITION_CONTROL)
      {
         disableJointPositionControl();
         jointControlMode = null;
      }

   }

   public void enableJointPositionControl()
   {
      for(OneDoFJoint joint : oneDoFJoints)
      {
         joint.setIntegrateDesiredAccelerations(false);
         joint.setUnderPositionControl(true);
      }
   }

   public void disableJointPositionControl()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setIntegrateDesiredAccelerations(doIntegrateDesiredAccelerations[i]);
         joint.setUnderPositionControl(false);
      }
   }

   @Override
   public void doAction() {

      //		System.out.println("Do action in forcecontrolstate");
      currentTimeInState.set(getTimeInCurrentState());

      if(DEBUG)
      {
         // Debug mode: Position controlled Trajectory
         poseTrajectoryGenerator.compute(currentTimeInState.getDoubleValue());
         poseTrajectoryGenerator.get(desiredPose);
         poseTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
         poseTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
         ReferenceFrame controlFrame = taskspaceToJointspaceCalculator.getControlFrame();

         desiredVelocity.changeFrame(controlFrame);
         desiredAngularVelocity.changeFrame(controlFrame);  

         desiredPosition.changeFrame(controlFrame);
         desiredAcceleration.changeFrame(controlFrame);
         desiredOrientation.changeFrame(controlFrame);
         desiredAngularAcceleration.changeFrame(controlFrame);

         taskspaceToJointspaceCalculator.compute(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity);
         taskspaceToJointspaceCalculator.packDesiredJointAnglesIntoOneDoFJoints(oneDoFJoints);
         taskspaceToJointspaceCalculator.packDesiredJointVelocitiesIntoOneDoFJoints(oneDoFJoints);
         taskspaceToJointspaceCalculator.packDesiredJointAccelerationsIntoOneDoFJoints(oneDoFJoints);
      }
      else
      {
         // TODO force feedback contol
      }

      switch(jointControlMode){

      case FORCE_CONTROL :

         for(int i = 0; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];
            double q = joint.getQ();
            double qd = joint.getQd();
            double qDes = joint.getqDesired();
            double qdDes = joint.getQdDesired();

            RateLimitedYoVariable rateLimitedAcceleration = jointRateLimitedAccelerations.get(joint);

            PIDController pidController = jointPIDControllers.get(joint);
            double desiredAcceleration =  pidController.computeForAngles(q, qDes, qd, qdDes, dtControl);

            desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, jointMaxAcceleration.getDoubleValue());
            rateLimitedAcceleration.update(desiredAcceleration);
            desiredAcceleration = rateLimitedAcceleration.getDoubleValue();

            momentumBasedController.setOneDoFJointAcceleration(joint, desiredAcceleration);
         }

         break;

      case POSITION_CONTROL :
         for(OneDoFJoint joint : oneDoFJoints)
         {
            // set desired qdd to zero.
            momentumBasedController.setOneDoFJointAcceleration(joint, 0.0);
         }
         break;

      default :
         PrintTools.error(this, "No joint control mode set.");
      }
   }
}