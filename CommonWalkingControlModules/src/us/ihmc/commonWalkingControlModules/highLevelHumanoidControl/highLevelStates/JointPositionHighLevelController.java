package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import gnu.trove.map.hash.TObjectDoubleHashMap;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.Stoppable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandTrajectoryType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceToJointspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredJointsPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredSteeringWheelProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandComplianceControlParametersProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.MultiJointPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.SingleJointPositionProvider;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.communication.packets.wholebody.JointAnglesPacket;
import us.ihmc.communication.packets.wholebody.MultiJointAnglePacket;
import us.ihmc.communication.packets.wholebody.SingleJointAnglePacket;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FrameMatrix3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePose;
import us.ihmc.yoUtilities.math.trajectories.CirclePoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.SteeringPoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class JointPositionHighLevelController extends HighLevelBehavior implements Stoppable
{   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean VISUALIZE_TASKSPACE_TRAJECTORIES = true;
   private static final boolean DEBUG = false;
   
   private static final double NECK_SPEED = Math.PI / 3.0;
   private static final double MIN_NECK_TRAJECTORY_TIME = 0.5;
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());


   private final OneDoFJoint[] jointsBeingControlled;
   private final FullHumanoidRobotModel fullRobotModel;

   public final static HighLevelState controllerState = HighLevelState.JOINT_POSITION_CONTROL;

   private final HashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> trajectoryGenerator;
   private final HashMap<OneDoFJoint, PIDController >     alternativeController;
   private final HashMap<OneDoFJoint, BooleanYoVariable > useAlternativeController;
   
   private final YoVariableDoubleProvider trajectoryTimeProvider;
   private final DoubleYoVariable timeProvider;

   private double initialTrajectoryTime;
   private final DesiredJointsPositionProvider desiredJointsProvider;
   private final HandPoseProvider handPoseProvider;
   private final SingleJointPositionProvider singleJointPositionProvider;
   private final MultiJointPositionProvider multiJointPositionProvider;
   private boolean firstPacket = true;
   private final HandPoseStatusProducer handPoseStatusProducer;
   private boolean handTrajectoryDoneHasBeenSent;
   private final HeadOrientationProvider headOrientationProvider;
   private final HandComplianceControlParametersProvider handComplianceControlParametersProvider;
   private final DesiredSteeringWheelProvider desiredSteeringWheelProvider;

   private final TObjectDoubleHashMap<OneDoFJoint> previousPosition = new TObjectDoubleHashMap<>();

   private SideDependentList<double[]> armJointAngles, legJointAngles;
   private double[] spineJointAngles;
   private double neckJointAngle;
   
   private final DoubleYoVariable proportionalGain;
   private final DoubleYoVariable derivativeGain;
   private final SideDependentList<DoubleYoVariable> offsetRoll = new SideDependentList<DoubleYoVariable>();
   private final SideDependentList<DoubleYoVariable> offsetPitch = new SideDependentList<DoubleYoVariable>();
   
   private final SideDependentList<BooleanYoVariable> areHandTaskspaceControlled;
   private final SideDependentList<BooleanYoVariable> areHandJointspaceControlled;
   
   // Fields used for taskspace commands
   private final SideDependentList<TaskspaceToJointspaceHandPositionControlState> handTaskspaceControllers;
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
   private final FrameMatrix3D selectionFrameMatrix = new FrameMatrix3D();
   private final FramePose currentDesiredHandPose = new FramePose();
   private final SideDependentList<StraightLinePoseTrajectoryGenerator> handStraightLinePoseTrajectoryGenerators;
   private final FramePoint rotationAxisOrigin = new FramePoint();
   private final FrameVector rotationAxis = new FrameVector();
   private final SideDependentList<CirclePoseTrajectoryGenerator> handCircularPoseTrajectoryGenerators;
   private final SideDependentList<SteeringPoseTrajectoryGenerator> handSteeringPoseTrajectoryGenerators;
   private final SideDependentList<TaskspaceToJointspaceCalculator> handTaskspaceToJointspaceCalculators;
   private final SideDependentList<PoseReferenceFrame> optionalHandControlFrames;
   private final SideDependentList<YoFramePose> optionalHandControlFramePoses;
   private final FramePose tempFramePose = new FramePose();

   private final MomentumBasedController momentumBasedController;
   private final ICPAndMomentumBasedController icpAndMomentumBasedController;
   
   private final static double MAX_DELTA_TO_BELIEVE_DESIRED = 0.05;

   public JointPositionHighLevelController(final MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController,
         VariousWalkingProviders variousWalkingProviders)
   {
      super(controllerState);
      
      timeProvider = momentumBasedController.getYoTime();
      this.desiredJointsProvider = variousWalkingProviders.getDesiredJointsPositionProvider();
      this.handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      this.singleJointPositionProvider = variousWalkingProviders.getSingleJointPositionProvider();
      this.multiJointPositionProvider = variousWalkingProviders.getMultiJointPositionProvider();
      this.momentumBasedController = momentumBasedController;
      this.icpAndMomentumBasedController = icpAndMomentumBasedController;
      this.handPoseStatusProducer = variousWalkingProviders.getHandPoseStatusProducer();
      this.headOrientationProvider = variousWalkingProviders.getDesiredHeadOrientationProvider();
      this.handComplianceControlParametersProvider = variousWalkingProviders.getHandComplianceControlParametersProvider();
      this.desiredSteeringWheelProvider = variousWalkingProviders.getDesiredSteeringWheelProvider();

      trajectoryGenerator = new HashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator>();
      alternativeController = new HashMap<OneDoFJoint, PIDController>();
      useAlternativeController = new HashMap<OneDoFJoint, BooleanYoVariable>();
      
      proportionalGain = new DoubleYoVariable("flatFeetProportionalGain", registry);
      derivativeGain   = new DoubleYoVariable("flatFeetDerivativeGain", registry);
      
      proportionalGain.set(0.01);
      
      offsetRoll.set(RobotSide.LEFT, new DoubleYoVariable("flatFeetLeftOffsetRoll", registry) );
      offsetRoll.set(RobotSide.RIGHT, new DoubleYoVariable("flatFeetRightOffsetRoll", registry) );
      
      offsetPitch.set(RobotSide.LEFT, new DoubleYoVariable("flatFeetLeftOffsetpitch", registry) );
      offsetPitch.set(RobotSide.RIGHT, new DoubleYoVariable("flatFeetRightOffsetpitch", registry) );
      
      fullRobotModel = momentumBasedController.getFullRobotModel();
      trajectoryTimeProvider = new YoVariableDoubleProvider("jointControl_trajectory_time", registry);
      
      HashSet<OneDoFJoint> jointsBeingControlled = new HashSet<>(); 
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         String jointName = joint.getName();

         if (jointName.contains("finger"))
            continue;

         jointsBeingControlled.add(joint);

         OneDoFJointQuinticTrajectoryGenerator generator = new OneDoFJointQuinticTrajectoryGenerator("jointControl_" + joint.getName(), joint,
                                                              trajectoryTimeProvider, registry);
         trajectoryGenerator.put(joint, generator);
         
         PIDController pidController = new PIDController( "jointControl_" + joint.getName() , registry);
         alternativeController.put(joint, pidController);
         
         BooleanYoVariable useAlternative = new BooleanYoVariable( "jointControl_" + joint.getName() + "_enableAlternativePID" , registry);
         useAlternative.set(false);
         useAlternativeController.put(joint, useAlternative ); 
      }
      
      this.jointsBeingControlled = jointsBeingControlled.toArray(new OneDoFJoint[jointsBeingControlled.size()]);
      
      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      handTaskspaceControllers = new SideDependentList<>();
      areHandTaskspaceControlled = new SideDependentList<>();
      areHandJointspaceControlled = new SideDependentList<>();
      handStraightLinePoseTrajectoryGenerators = new SideDependentList<>();
      handCircularPoseTrajectoryGenerators = new SideDependentList<>();
      handSteeringPoseTrajectoryGenerators = new SideDependentList<>();
      handTaskspaceToJointspaceCalculators = new SideDependentList<>();
      optionalHandControlFrames = new SideDependentList<PoseReferenceFrame>();
      optionalHandControlFramePoses = new SideDependentList<>();

      String shortControllerNamePrefix = "JPCtrl";

      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = shortControllerNamePrefix + robotSide.getCamelCaseNameForMiddleOfExpression() + "Hand";
         
         BooleanYoVariable isHandTaskspaceControlled = new BooleanYoVariable(namePrefix + "TaskspaceControl", registry);
         areHandTaskspaceControlled.put(robotSide, isHandTaskspaceControlled);
         BooleanYoVariable isHandJointspaceControlled = new BooleanYoVariable(namePrefix + "JointspaceControl", registry);
         areHandJointspaceControlled.put(robotSide, isHandJointspaceControlled);

         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         double controlDT = momentumBasedController.getControlDT();

         TaskspaceToJointspaceCalculator handTaskspaceToJointspaceCalculator = new TaskspaceToJointspaceCalculator(namePrefix, chest, hand, controlDT, registry);
         ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
         handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(handControlFrame);
         handTaskspaceToJointspaceCalculator.setupWithDefaultParameters();
         handTaskspaceToJointspaceCalculators.put(robotSide, handTaskspaceToJointspaceCalculator);

         TaskspaceToJointspaceHandPositionControlState handTaskspaceController = TaskspaceToJointspaceHandPositionControlState.createControlStateForPositionControlledJoints(namePrefix, robotSide, momentumBasedController, chest, hand, registry);
         handTaskspaceController.setControlModuleForPositionControl(handTaskspaceToJointspaceCalculator);
         handTaskspaceControllers.put(robotSide, handTaskspaceController);

         StraightLinePoseTrajectoryGenerator handStraightLinePoseTrajectoryGenerator = new StraightLinePoseTrajectoryGenerator(namePrefix + "StraightLine", true, worldFrame, registry, VISUALIZE_TASKSPACE_TRAJECTORIES, yoGraphicsListRegistry);
         CirclePoseTrajectoryGenerator handCircularPoseTrajectoryGenerator = new CirclePoseTrajectoryGenerator(namePrefix + "Circular", worldFrame, trajectoryTimeProvider, registry, yoGraphicsListRegistry);
         SteeringPoseTrajectoryGenerator steeringPoseTrajectoryGenerator = new SteeringPoseTrajectoryGenerator(namePrefix, worldFrame, registry, yoGraphicsListRegistry);
         
         handStraightLinePoseTrajectoryGenerators.put(robotSide, handStraightLinePoseTrajectoryGenerator);
         handCircularPoseTrajectoryGenerators.put(robotSide, handCircularPoseTrajectoryGenerator);
         handSteeringPoseTrajectoryGenerators.put(robotSide, steeringPoseTrajectoryGenerator);
         
         PoseReferenceFrame optionalHandControlFrame = new PoseReferenceFrame("optional" + robotSide.getCamelCaseNameForMiddleOfExpression() + "HandControlFrame", handControlFrame);
         optionalHandControlFrames.put(robotSide, optionalHandControlFrame);
         YoFramePose optionalHandControlFramePose = new YoFramePose(optionalHandControlFrame.getName(), worldFrame, registry);
         optionalHandControlFramePoses.put(robotSide, optionalHandControlFramePose);
         
         if (yoGraphicsListRegistry != null)
         {
            YoGraphicCoordinateSystem optionalControlFrameViz = new YoGraphicCoordinateSystem(robotSide.getCamelCaseNameForMiddleOfExpression() + "Steering Control Frame", optionalHandControlFramePose, 0.3);
            yoGraphicsListRegistry.registerYoGraphic("Steering", optionalControlFrameViz);
         }
      }

   }
   
   private FramePoint2d cop = new FramePoint2d();
   private Wrench footWrench = new Wrench();

   
   private void controlOrientationOfFeet()
   {
      if (!flattenFeet){
         return;
      }
      SideDependentList<FootSwitchInterface> feetSensors = momentumBasedController.getFootSwitches();
           
      double minimumZForce = 100; // Nextons ??
      
      boolean feetFlattened = true;
      for (RobotSide side: RobotSide.values)
      {
         feetSensors.get(side).computeAndPackCoP(cop);
         feetSensors.get(side).computeAndPackFootWrench(footWrench);
         
        // OneDoFJoint ankleRoll = fullRobotModel.getLegJoint(side, LegJointName.ANKLE_ROLL);
        // OneDoFJoint anklePitch = fullRobotModel.getLegJoint(side, LegJointName.ANKLE_PITCH);
         
         cop.changeFrame( fullRobotModel.getSoleFrame( side) );
         
 
         if(footWrench.getLinearPartZ() > minimumZForce )
         {
            {
               double errorRoll = 0;
               double Ymax = 0.03;
               double Ymin = -0.03;
               if( cop.getY() > Ymax) errorRoll = cop.getY() - Ymax;
               if( cop.getY() < Ymin) errorRoll = cop.getY() - Ymin;
               
               double deltaQ = errorRoll * proportionalGain.getDoubleValue();
                
               double offset = offsetRoll.get(side).getDoubleValue(); 
               offset += deltaQ;  
               offsetRoll.get(side).set( offset ); 
    
               if (Math.abs(errorRoll) > 0){
                  feetFlattened = false;
               }
            }  
            {
               double errorPitch = 0;
               double Xmax = 0.08;
               double Xmin = -0.08;
               if( cop.getY() > Xmax) errorPitch = cop.getY() - Xmax;
               if( cop.getY() < Xmin) errorPitch = cop.getY() - Xmin;
               
               double deltaQ = errorPitch * proportionalGain.getDoubleValue();
               
               double offset = offsetPitch.get(side).getDoubleValue(); 
               offset += deltaQ;  
               offsetPitch.get(side).set( offset ); 
               
               if (Math.abs(errorPitch) > 0){
                  feetFlattened = false;
               }
            }
         }
      }

      flattenFeet = !feetFlattened;
   }
   
   @Override
   public void doAction()
   {
      initializeFromVariousProviders();
   
      doJointPositionControl(timeProvider.getDoubleValue() - initialTrajectoryTime);
   
      for (RobotSide robotSide : RobotSide.values)
      {
         if (areHandTaskspaceControlled.get(robotSide).getBooleanValue())
         {
            handTaskspaceControllers.get(robotSide).doAction();
            
            if (handTaskspaceControllers.get(robotSide).isDone() && !handTrajectoryDoneHasBeenSent)
            {
               handPoseStatusProducer.sendCompletedStatus(robotSide);
               handTrajectoryDoneHasBeenSent = true;
            }
            else if (!handTaskspaceControllers.get(robotSide).isDone())
            {
               handTrajectoryDoneHasBeenSent = false;
            }
            
            for (ArmJointName jointName : ArmJointName.values)
            {
               OneDoFJoint joint = fullRobotModel.getArmJoint(robotSide, jointName);
               if (joint == null) continue;

               trajectoryGenerator.get(joint).setFinalPosition(joint.getqDesired());
               previousPosition.put(joint, joint.getqDesired());
            }
         }
      }

      momentumBasedController.doPrioritaryControl();
      icpAndMomentumBasedController.update();

      updateVisualization();
   }

   private void updateVisualization()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         tempFramePose.setToZero(optionalHandControlFrames.get(robotSide));
         tempFramePose.changeFrame(worldFrame);
         optionalHandControlFramePoses.get(robotSide).set(tempFramePose);
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (OneDoFJoint joint : jointsBeingControlled)
      {
         joint.setUnderPositionControl(true);
      }
      
      setJointTrajectoriesToCurrent();
      trajectoryTimeProvider.set(0.1);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      for (OneDoFJoint joint : jointsBeingControlled)
      {
         joint.setUnderPositionControl(false);
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void stopExecution()
   {
      setJointTrajectoriesToCurrent();
   }
   
   private boolean flattenFeet = false;

   private void doJointPositionControl(double time)
   {
      boolean trajectoriesAreDone = true;
      
      if( flattenFeet && trajectoriesAreDone)
      {
        //TODO  controlOrientationOfFeet();
      }
      
      for (OneDoFJoint joint : jointsBeingControlled)
      {
         OneDoFJointQuinticTrajectoryGenerator generator = trajectoryGenerator.get(joint);
   
         if (generator.isDone() == false)
         {
            generator.compute(time);
            trajectoriesAreDone = false;
         }
   
         if (useAlternativeController.get(joint).getBooleanValue())
         {
            PIDController controller = alternativeController.get(joint);
            double effort = controller.compute(joint.getQ(), generator.getValue(), joint.getQd(), generator.getVelocity(), 0.003);
   
            joint.setUnderPositionControl(false);
            joint.setTau(effort);
         }
         else
         {
            joint.setUnderPositionControl(true);
            joint.setqDesired(generator.getValue());
            joint.setQdDesired(generator.getVelocity());
         }
         previousPosition.put(joint, generator.getValue());
      }
      
      if (trajectoriesAreDone)
      {
         for (RobotSide side : RobotSide.values)
         {
            if (areHandJointspaceControlled.get(side).getBooleanValue())
            {
               handPoseStatusProducer.sendCompletedStatus(side);
               areHandJointspaceControlled.get(side).set(false);
            }
         }
      }
      
      for (RobotSide side : RobotSide.values)
      {
         OneDoFJoint ankleRoll = fullRobotModel.getLegJoint(side, LegJointName.ANKLE_ROLL);
         OneDoFJoint anklePitch = fullRobotModel.getLegJoint(side, LegJointName.ANKLE_PITCH);

         double Q = ankleRoll.getqDesired() + offsetRoll.get(side).getDoubleValue();
         ankleRoll.setqDesired( Q );
         
         Q = anklePitch.getqDesired() + offsetPitch.get(side).getDoubleValue();
         anklePitch.setqDesired( Q );
      }
      
   }

   private void initializeFromVariousProviders()
   {
      if ((desiredJointsProvider != null) && desiredJointsProvider.checkForNewPacket())
         initializeFromJointAnglesPacket(desiredJointsProvider.getNewPacket());
   
      if (singleJointPositionProvider != null)
      {
         SingleJointAnglePacket newPacket = singleJointPositionProvider.getNewPacket(timeProvider.getDoubleValue());
         if (newPacket != null)
         {
            initializeFromSingleJointAnglePacket(newPacket);
         }
      }
      
      if (multiJointPositionProvider != null && multiJointPositionProvider.checkForNewPacket())
	     initializeFromMultiJointAnglePacket(multiJointPositionProvider.getNewPacket());
      
      for (RobotSide side : RobotSide.values)
      {
         if (handComplianceControlParametersProvider != null && handComplianceControlParametersProvider.checkForNewRequest(side))
         {
            if (handComplianceControlParametersProvider.isResetRequested(side))
            {
               handTaskspaceControllers.get(side).setEnableCompliantControl(false, null, null, null, null, Double.NaN, Double.NaN);
            }
            else
            {
               boolean[] enableLinearCompliance = handComplianceControlParametersProvider.getEnableLinearCompliance(side);
               boolean[] enableAngularCompliance = handComplianceControlParametersProvider.getEnableAngularCompliance(side);
               Vector3d desiredForce = handComplianceControlParametersProvider.getDesiredForce(side);
               Vector3d desiredTorque = handComplianceControlParametersProvider.getDesiredTorque(side);
               double forceDeadzone = handComplianceControlParametersProvider.getForceDeadzone(side);
               double torqueDeadzone = handComplianceControlParametersProvider.getTorqueDeadzone(side);
               handTaskspaceControllers.get(side).setEnableCompliantControl(true, enableLinearCompliance, enableAngularCompliance, desiredForce, desiredTorque, forceDeadzone, torqueDeadzone);
            }
         }

         if (handPoseProvider.checkForNewPose(side))
         {
            DataType dataType = handPoseProvider.checkHandPosePacketDataType(side);
            switch (dataType)
            {
            case JOINT_ANGLES:
               initializeFromJointMap(side, handPoseProvider.getFinalDesiredJointAngleMaps(side), handPoseProvider.getTrajectoryTime());
               break;
            case HAND_POSE:
               initializeFromTaskSpaceHandPosePacket(side, handPoseProvider.getDesiredHandPose(side), handPoseProvider.getDesiredReferenceFrame(side));
               break;
            default:
               PrintTools.error(this, "Specified packet type, " + dataType + ", is not supported!");
               break;
            }
         }
         else if (handPoseProvider.checkForNewRotateAboutAxisPacket(side))
         {
            initializeFromRotateAboutAxisPacket(side, handPoseProvider.getRotationAxisOriginInWorld(side), handPoseProvider.getRotationAxisInWorld(side),
                  handPoseProvider.getRotationAngleRightHandRule(side), handPoseProvider.getGraspOffsetFromControlFrame(side), handPoseProvider.controlHandAngleAboutAxis(side));
         }

         if (desiredSteeringWheelProvider.checkForNewSteeringWheelInformation(side))
         {
            updateSteeringWheelInformation(side);
         }

         if (desiredSteeringWheelProvider.checkForNewDesiredAbsoluteSteeringAngle(side))
         {
            initializeSteeringWheelTrajectory(side);
         }
      }
      
      if (headOrientationProvider != null && headOrientationProvider.isNewHeadOrientationInformationAvailable())
      {
         pitchNeck(headOrientationProvider.getDesiredHeadOrientation().getPitch());
      }
   }

   private void pitchNeck(double pitch)
   {
      OneDoFJoint neckPitchJoint = fullRobotModel.getNeckJoint(NeckJointName.LOWER_NECK_PITCH);
      double currentPitch = neckPitchJoint.getQ();
      double trajctoryTime = Math.max(Math.abs(currentPitch - pitch) / NECK_SPEED, MIN_NECK_TRAJECTORY_TIME);
      
      initializeForSingleJointAngle(trajctoryTime, neckPitchJoint.getName(), pitch);
   }

   private void initializeFromJointAnglesPacket(JointAnglesPacket packet)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         areHandTaskspaceControlled.get(robotSide).set(false);
         areHandJointspaceControlled.get(robotSide).set(false);
      }
   
      initialTrajectoryTime = timeProvider.getDoubleValue();
      trajectoryTimeProvider.set(packet.getTrajectoryTime());
   
      if (armJointAngles == null)
      {
         armJointAngles = new SideDependentList<double[]>();
   
         for (RobotSide robotSide : RobotSide.values)
         {
            double[] jointAngles = new double[packet.getNumberOfArmJoints(robotSide)];
            armJointAngles.put(robotSide, jointAngles);
         }
      }
   
      if (this.legJointAngles == null)
      {
         legJointAngles = new SideDependentList<double[]>();
   
         for (RobotSide robotSide : RobotSide.values)
         {
            double[] jointAngles = new double[packet.getNumberOfLegJoints(robotSide)];
            legJointAngles.put(robotSide, jointAngles);
         }
      }
   
      if (spineJointAngles == null)
      {
         spineJointAngles = new double[packet.getNumberOfSpineJoints()];
      }
   
      for (RobotSide robotSide : RobotSide.values)
      {
         setFinalPositionArms(robotSide, packet);
         setFinalPositionLegs(robotSide, packet);
      }
   
      setFinalPositionSpineJoints(packet);
      setFinalPositionNeckJoint(packet);
   
      if (firstPacket)
      {
         firstPacket = false;
   
         for (OneDoFJoint joint : jointsBeingControlled)
         {
            trajectoryGenerator.get(joint).initialize();
            previousPosition.put(joint, joint.getQ());
         }
      }
      else
      {
         for (OneDoFJoint joint : jointsBeingControlled)
         {
           trajectoryGenerator.get(joint).initialize(previousPosition.get(joint), 0.0);
  //       	 trajectoryGenerator.get(joint).initialize(joint.getqDesired(), 0.0);
         }
      }
      
      flattenFeet = packet.flattenFeetAtTheEnd;
   }
   
   private void initializeFromSingleJointAnglePacket(SingleJointAnglePacket packet)
   {
      initializeForSingleJointAngle(packet.trajcetoryTime, packet.jointName, packet.angle);
   }

   private void initializeForSingleJointAngle(double trajectoryTime, String jointName, double jointAngle)
   {
      initialTrajectoryTime = timeProvider.getDoubleValue();
      trajectoryTimeProvider.set(trajectoryTime);
      
      for (OneDoFJoint joint : jointsBeingControlled)
      {
         if (jointName.equals(joint.getName()))
         {
            if (joint.getName().contains("l_arm"))
            {
               areHandTaskspaceControlled.get(RobotSide.LEFT).set(false);
               areHandJointspaceControlled.get(RobotSide.LEFT).set(false);
            }
            else if (joint.getName().contains("r_arm"))
            {
               areHandTaskspaceControlled.get(RobotSide.RIGHT).set(false);
               areHandJointspaceControlled.get(RobotSide.RIGHT).set(false);
            }
            
            double desiredPosition = MathTools.clipToMinMax(jointAngle, joint.getJointLimitLower(), joint.getJointLimitUpper());
            trajectoryGenerator.get(joint).setFinalPosition(desiredPosition);

            alternativeController.get(joint).setMaximumOutputLimit(Double.POSITIVE_INFINITY);
            trajectoryGenerator.get(joint).initialize();
         }
         else
         {
            trajectoryGenerator.get(joint).initialize(joint.getqDesired(), 0.0);
         }
         
      }
   }
   
   private void initializeFromMultiJointAnglePacket(MultiJointAnglePacket multiJointAnglePacket)
   {
	   SingleJointAnglePacket[] jointPackets = multiJointAnglePacket.singleJointAnglePackets;
	   if (jointPackets == null)
	   {
	      System.out.println("MultiJointAnglePacket was null");
	      return;
	   }
	   if (jointPackets.length == 0)
	   {
		   System.out.println("Recieved empty multi joint angle packet.");
		   return;
	   }
	   if (jointPackets[0].trajcetoryTime <= 0.0)
	   {
	      System.out.println("Multi joint angle packet has invalid trajectory time of " + jointPackets[0].trajcetoryTime);
	      return;
	   }
	   
	   initialTrajectoryTime = timeProvider.getDoubleValue();
	   trajectoryTimeProvider.set(jointPackets[0].trajcetoryTime);
	   
      for (OneDoFJoint joint : jointsBeingControlled)
      {
         boolean jointSpecified = false;
         
         for (int i = 0; i < jointPackets.length; i++)
         {
            String jointName = jointPackets[i].jointName;
            double jointAngle = jointPackets[i].angle;

            if (jointName.equals(joint.getName()))
            {
               jointSpecified = true;
               
               if (joint.getName().contains("l_arm"))
               {
                  areHandTaskspaceControlled.get(RobotSide.LEFT).set(false);
                  areHandJointspaceControlled.get(RobotSide.LEFT).set(false);
               }
               else if (joint.getName().contains("r_arm"))
               {
                  areHandTaskspaceControlled.get(RobotSide.RIGHT).set(false);
                  areHandJointspaceControlled.get(RobotSide.RIGHT).set(false);
               }

               double desiredPosition = MathTools.clipToMinMax(jointAngle, joint.getJointLimitLower(), joint.getJointLimitUpper());
               trajectoryGenerator.get(joint).setFinalPosition(desiredPosition);

               alternativeController.get(joint).setMaximumOutputLimit(Double.POSITIVE_INFINITY);
               trajectoryGenerator.get(joint).initialize();
            }
         }
         
         if (!jointSpecified)
         {
            trajectoryGenerator.get(joint).initialize(joint.getqDesired(), 0.0);
         }
      }
   }

   private void initializeFromJointMap(RobotSide side, Map<OneDoFJoint, Double> jointTarget, double trajectoryTime)
   {
      areHandTaskspaceControlled.get(side).set(false);
      areHandJointspaceControlled.get(side).set(true);

      initialTrajectoryTime = timeProvider.getDoubleValue();
      trajectoryTimeProvider.set( trajectoryTime );
      
      for (Entry<OneDoFJoint, Double> entry: jointTarget.entrySet())
      {
         OneDoFJoint oneDoFJoint = entry.getKey();
         double  desiredPosition = entry.getValue();
         trajectoryGenerator.get(oneDoFJoint).setFinalPosition(desiredPosition);
      }
      
      if (firstPacket)
      {
         firstPacket = false;

         for (OneDoFJoint joint : jointsBeingControlled)
         {
            trajectoryGenerator.get(joint).initialize();
            previousPosition.put(joint, joint.getQ());
            joint.setqDesired(joint.getQ());
         }
      }
      else
      {
         for (OneDoFJoint joint : jointsBeingControlled)
         {
            trajectoryGenerator.get(joint).initialize(previousPosition.get(joint), 0.0);
         }
      }
   }
   
   private void initializeFromTaskSpaceHandPosePacket(RobotSide robotSide, FramePose desiredHandPose, ReferenceFrame desiredReferenceFrame)
   {
      if (DEBUG) PrintTools.debug(this, "Received Hand Pose Packet in Task Space");

      TaskspaceToJointspaceCalculator handTaskspaceToJointspaceCalculator = handTaskspaceToJointspaceCalculators.get(robotSide);
      handTaskspaceToJointspaceCalculator.initializeFromDesiredJointAngles();
      ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(handControlFrame);
      handTaskspaceToJointspaceCalculator.setPrivilegedJointPositionsToMidRange();

      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);

      boolean[] controlledOrientationAxes = handPoseProvider.getControlledOrientationAxes(robotSide);
      if (controlledOrientationAxes != null)
      {
         for (int i = 2; i >= 0; i--)
         {
            if (!controlledOrientationAxes[i])
               MatrixTools.removeRow(selectionMatrix, i);
         }
      }

      StraightLinePoseTrajectoryGenerator handStraightLinePoseTrajectoryGenerator = handStraightLinePoseTrajectoryGenerators.get(robotSide);
      handStraightLinePoseTrajectoryGenerator.registerAndSwitchFrame(desiredReferenceFrame);
      computeHandCurrentDesiredFramePose(currentDesiredHandPose, robotSide, worldFrame, handControlFrame, HandTrajectoryType.STRAIGHT_LINE);
      handStraightLinePoseTrajectoryGenerator.setInitialPose(currentDesiredHandPose);
      handStraightLinePoseTrajectoryGenerator.setFinalPose(desiredHandPose);
      handStraightLinePoseTrajectoryGenerator.setTrajectoryTime(handPoseProvider.getTrajectoryTime());

      initializeTaskspaceHandTrajectory(robotSide, handStraightLinePoseTrajectoryGenerator, selectionMatrix);
   }

   private final FramePoint steeringWheelCenter = new FramePoint();
   private final FrameVector steeringWheelRotationAxis = new FrameVector();
   private final FrameVector steeringWheelZeroAxis = new FrameVector();

   private void updateSteeringWheelInformation(RobotSide robotSide)
   {
      desiredSteeringWheelProvider.getSteeringWheelPose(robotSide, steeringWheelCenter, steeringWheelRotationAxis, steeringWheelZeroAxis);
      SteeringPoseTrajectoryGenerator handSteeringPoseTrajectoryGenerator = handSteeringPoseTrajectoryGenerators.get(robotSide);
      handSteeringPoseTrajectoryGenerator.updateSteeringWheel(steeringWheelCenter, steeringWheelRotationAxis, steeringWheelZeroAxis);
      handSteeringPoseTrajectoryGenerator.setSteeringWheelRadius(desiredSteeringWheelProvider.getSteeringWheelRadius(robotSide));
      handSteeringPoseTrajectoryGenerator.setDesiredSteeringSpeed(desiredSteeringWheelProvider.getDesiredSteeringSpeed(robotSide));
   }

   private void initializeSteeringWheelTrajectory(RobotSide robotSide)
   {
      SteeringPoseTrajectoryGenerator handSteeringPoseTrajectoryGenerator = handSteeringPoseTrajectoryGenerators.get(robotSide);
      handSteeringPoseTrajectoryGenerator.setFinalSteeringAngle(desiredSteeringWheelProvider.getDesiredAbsoluteSteeringAngle(robotSide));

      PoseReferenceFrame optionalHandControlFrame = optionalHandControlFrames.get(robotSide);
      optionalHandControlFrame.setX(desiredSteeringWheelProvider.getGraspOffsetFromControlFrame(robotSide));
      optionalHandControlFrame.update();

      TaskspaceToJointspaceCalculator handTaskspaceToJointspaceCalculator = handTaskspaceToJointspaceCalculators.get(robotSide);
      handTaskspaceToJointspaceCalculator.setPrivilegedJointPositionsToMidRange();

      // Hack for driving to get away from the shz limit
      if (robotSide == RobotSide.LEFT)
      {
         handTaskspaceToJointspaceCalculator.setPrivilegedJointPosition(2, 1.0); // ely
         handTaskspaceToJointspaceCalculator.setPrivilegedJointPosition(4, -2.5); // wry
         handTaskspaceToJointspaceCalculator.setPrivilegedJointPosition(5, -0.8); // wrx
         handTaskspaceToJointspaceCalculator.setPrivilegedJointPosition(6, 0.0); // wry2
      }

      computeHandCurrentDesiredFramePose(currentDesiredHandPose, robotSide, worldFrame, optionalHandControlFrame, HandTrajectoryType.CIRCULAR);
      handSteeringPoseTrajectoryGenerator.setInitialPose(currentDesiredHandPose);
      handSteeringPoseTrajectoryGenerator.setControlledFrame(optionalHandControlFrame);

      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      
      selectionFrameMatrix.setToZero(handSteeringPoseTrajectoryGenerator.getSteeringWheelFrame());
      selectionFrameMatrix.setElement(0, 0, 1.0);
      selectionFrameMatrix.setElement(1, 1, 1.0);
      selectionFrameMatrix.setElement(2, 2, 0.0);
      selectionFrameMatrix.changeFrame(optionalHandControlFrame);

      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      selectionFrameMatrix.getDenseMatrix(selectionMatrix, 0, 0);

      initializeTaskspaceHandTrajectory(robotSide, handSteeringPoseTrajectoryGenerator, selectionMatrix);
      handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(optionalHandControlFrame);
      ReferenceFrame tangentialCircleFrame = handSteeringPoseTrajectoryGenerator.getTangentialSteeringFrame();
      handTaskspaceControllers.get(robotSide).setCompliantControlFrame(tangentialCircleFrame);
   }

   private void initializeFromRotateAboutAxisPacket(RobotSide robotSide, Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld, double rotationAngleRightHandRule, double graspOffsetFromControlFrame, boolean controlHandAngleAboutAxis)
   {
      if (DEBUG) PrintTools.debug(this, "Received Hand Rotate about Axis Packet");
      
      trajectoryTimeProvider.set(handPoseProvider.getTrajectoryTime());

      PoseReferenceFrame optionalHandControlFrame = optionalHandControlFrames.get(robotSide);
      optionalHandControlFrame.setX(graspOffsetFromControlFrame);
      optionalHandControlFrame.update();

      TaskspaceToJointspaceCalculator handTaskspaceToJointspaceCalculator = handTaskspaceToJointspaceCalculators.get(robotSide);
      handTaskspaceToJointspaceCalculator.setPrivilegedJointPositionsToMidRange();

      // Hack for driving to get away from the shz limit
      if (robotSide == RobotSide.LEFT)
      {
         handTaskspaceToJointspaceCalculator.setPrivilegedJointPosition(2, 1.0); // ely
         handTaskspaceToJointspaceCalculator.setPrivilegedJointPosition(4, -2.5); // wry
         handTaskspaceToJointspaceCalculator.setPrivilegedJointPosition(5, -0.8); // wrx
         handTaskspaceToJointspaceCalculator.setPrivilegedJointPosition(6, 0.0); // wry2
      }

      CirclePoseTrajectoryGenerator handCircularPoseTrajectoryGenerator = handCircularPoseTrajectoryGenerators.get(robotSide);
      handCircularPoseTrajectoryGenerator.setDesiredRotationAngle(rotationAngleRightHandRule);

      rotationAxisOrigin.setIncludingFrame(worldFrame, rotationAxisOriginInWorld);
      rotationAxis.setIncludingFrame(worldFrame, rotationAxisInWorld);

      handCircularPoseTrajectoryGenerator.updateCircleFrame(rotationAxisOrigin, rotationAxis);

      computeHandCurrentDesiredFramePose(currentDesiredHandPose, robotSide, worldFrame, optionalHandControlFrame, HandTrajectoryType.CIRCULAR);
      handCircularPoseTrajectoryGenerator.setInitialPose(currentDesiredHandPose);
      handCircularPoseTrajectoryGenerator.setControlledFrame(optionalHandControlFrame);
      handCircularPoseTrajectoryGenerator.setControlHandAngleAboutAxis(controlHandAngleAboutAxis);

      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      
      if (!controlHandAngleAboutAxis)
      {
         selectionFrameMatrix.setToZero(handCircularPoseTrajectoryGenerator.getCircleFrame());
         selectionFrameMatrix.setElement(0, 0, 1.0);
         selectionFrameMatrix.setElement(1, 1, 1.0);
         selectionFrameMatrix.setElement(2, 2, 0.0);
         selectionFrameMatrix.changeFrame(optionalHandControlFrame);

         selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
         CommonOps.setIdentity(selectionMatrix);
         selectionFrameMatrix.getDenseMatrix(selectionMatrix, 0, 0);
      }

      initializeTaskspaceHandTrajectory(robotSide, handCircularPoseTrajectoryGenerator, selectionMatrix);
      handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(optionalHandControlFrame);
      ReferenceFrame tangentialCircleFrame = handCircularPoseTrajectoryGenerator.getTangentialCircleFrame();
      handTaskspaceControllers.get(robotSide).setCompliantControlFrame(tangentialCircleFrame);
   }

   private void initializeTaskspaceHandTrajectory(RobotSide robotSide, PoseTrajectoryGenerator trajectory, DenseMatrix64F selectionMatrix)
   {
      areHandTaskspaceControlled.get(robotSide).set(true);
      areHandJointspaceControlled.get(robotSide).set(false);
      TaskspaceToJointspaceHandPositionControlState handTaskspaceController = handTaskspaceControllers.get(robotSide);
      handTaskspaceController.setTrajectory(trajectory);
      handTaskspaceController.setSelectionMatrix(selectionMatrix);
      handTaskspaceController.initializeWithDesiredJointAngles();
   }

   private void computeHandCurrentDesiredFramePose(FramePose desiredHandPoseToPack, RobotSide robotSide, ReferenceFrame trajectoryFrame, ReferenceFrame newControlFrame, HandTrajectoryType newTrajectoryType)
   {
      TaskspaceToJointspaceCalculator handTaskspaceToJointspaceCalculator = handTaskspaceToJointspaceCalculators.get(robotSide);
      TaskspaceToJointspaceHandPositionControlState handTaskspaceController = handTaskspaceControllers.get(robotSide);

      if (!areHandTaskspaceControlled.get(robotSide).getBooleanValue() || newTrajectoryType == HandTrajectoryType.STRAIGHT_LINE)
      {
         handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(newControlFrame);
         handTaskspaceToJointspaceCalculator.initializeFromDesiredJointAngles();
         handTaskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(desiredHandPoseToPack, trajectoryFrame);
         handTaskspaceController.resetCompliantControl();
      }
      else
      {
         desiredHandPoseToPack.setPoseIncludingFrame(handTaskspaceController.getDesiredPose());
         ReferenceFrame oldControlFrame = handTaskspaceToJointspaceCalculator.getControlFrame();
         if (oldControlFrame != newControlFrame)
         {
            handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(newControlFrame);
            handTaskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(desiredHandPoseToPack, trajectoryFrame);
            handTaskspaceController.resetCompliantControl();
         }
      }

      desiredHandPoseToPack.changeFrame(trajectoryFrame);
   }

   private void setFinalPositionSpineJoints(JointAnglesPacket packet)
   {
      if( packet.spineJointAngle == null ) return;
      
      packet.packSpineJointAngle(spineJointAngles);

      SpineJointName[] spineJointNames = fullRobotModel.getRobotSpecificJointNames().getSpineJointNames();
      
      for(int i=0; i<spineJointNames.length; i++)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getSpineJoint(spineJointNames[i]);
         double desiredPostion = spineJointAngles[i];
         
         //make sure that we do not command an arm joint outside the joint limits
         desiredPostion = MathTools.clipToMinMax(desiredPostion, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
         
         trajectoryGenerator.get(oneDoFJoint).setFinalPosition(desiredPostion);
      }
   }
   
   private void setFinalPositionNeckJoint(JointAnglesPacket packet)
   {     
      neckJointAngle = packet.getNeckJointAngle();
      
      OneDoFJoint neckJoint = fullRobotModel.getNeckJoint(NeckJointName.LOWER_NECK_PITCH);
      
      if( neckJoint == null) return;
      
      double desiredNeckJointAngle = MathTools.clipToMinMax(neckJointAngle, neckJoint.getJointLimitLower(), neckJoint.getJointLimitUpper());
      
      OneDoFJointQuinticTrajectoryGenerator trajectory = trajectoryGenerator.get(neckJoint);
      
      if( trajectory == null) return;
      
      trajectory.setFinalPosition(desiredNeckJointAngle);
   }
   

   private void setFinalPositionArms(RobotSide robotSide, JointAnglesPacket packet)
   {
      if( packet.leftArmJointAngle  == null && robotSide == RobotSide.LEFT)  return;
      if( packet.rightArmJointAngle == null && robotSide == RobotSide.RIGHT) return;
      
      packet.packArmJointAngle(robotSide, armJointAngles.get(robotSide));
      
      ArmJointName[] armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
      for(int i=0; i<armJointNames.length; i++)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getArmJoint(robotSide, armJointNames[i]);
         double desiredPostion = armJointAngles.get(robotSide)[i];
         
         //make sure that we do not command an arm joint outside the joint limits
         desiredPostion = MathTools.clipToMinMax(desiredPostion, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
                
         trajectoryGenerator.get(oneDoFJoint).setFinalPosition(desiredPostion);
         
         int jointTorqueLimit = 0;
         if( robotSide == RobotSide.LEFT ) {
            jointTorqueLimit = packet.leftArmTorqueLimit[i] ;
         }
         else{
            jointTorqueLimit = packet.rightArmTorqueLimit[i] ;
         }
         if( jointTorqueLimit > 0 )
         {
            //useAlternativeController.put( oneDoFJoint, useAlternative);
            alternativeController.get(oneDoFJoint).setMaximumOutputLimit( jointTorqueLimit );
         }
         else{
            //useAlternativeController.put( oneDoFJoint, dontUseAlternative);
            alternativeController.get(oneDoFJoint).setMaximumOutputLimit( Double.POSITIVE_INFINITY );
         }
      }
   }
   
   private void setFinalPositionLegs(RobotSide robotSide, JointAnglesPacket packet)
   {   
      if( packet.leftLegJointAngle  == null && robotSide == RobotSide.LEFT)  return;
      if( packet.rightLegJointAngle == null && robotSide == RobotSide.RIGHT) return;
      
      packet.packLegJointAngle(robotSide, legJointAngles.get(robotSide));
      
      LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
      for(int i=0; i<legJointNames.length; i++)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getLegJoint(robotSide, legJointNames[i]);
         double desiredPostion = legJointAngles.get(robotSide)[i];
         
         //make sure that we do not command an arm joint outside the joint limits
         desiredPostion = MathTools.clipToMinMax(desiredPostion, oneDoFJoint.getJointLimitLower(), oneDoFJoint.getJointLimitUpper());
         
         trajectoryGenerator.get(oneDoFJoint).setFinalPosition(desiredPostion);
         
         int jointTorqueLimit = 0;
         if( robotSide == RobotSide.LEFT ) {
            jointTorqueLimit = packet.leftLegTorqueLimit[i] ;
         }
         else{
            jointTorqueLimit = packet.rightLegTorqueLimit[i] ;
         }
         if( jointTorqueLimit > 0 )
         {
          //  useAlternativeController.put( oneDoFJoint, useAlternative);
            alternativeController.get(oneDoFJoint).setMaximumOutputLimit( jointTorqueLimit );
         }
         else{
          //  useAlternativeController.put( oneDoFJoint, dontUseAlternative);
            alternativeController.get(oneDoFJoint).setMaximumOutputLimit( Double.POSITIVE_INFINITY );
         }
      }      
   }
   
   private void setJointTrajectoriesToCurrent()
   {
      for (OneDoFJoint joint : jointsBeingControlled)
      {         
         double finalPosition = (Math.abs(joint.getQ() - joint.getqDesired()) < MAX_DELTA_TO_BELIEVE_DESIRED) ? joint.getqDesired() : joint.getQ();
                  
         trajectoryGenerator.get(joint).setFinalPosition(finalPosition);
         trajectoryGenerator.get(joint).initialize(finalPosition, 0.0);
      }      
   }
}
