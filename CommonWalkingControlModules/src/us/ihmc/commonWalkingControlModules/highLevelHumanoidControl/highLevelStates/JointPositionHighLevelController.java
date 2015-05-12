package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.Stoppable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.TaskspaceToJointspaceHandPositionControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredJointsPositionProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.HandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.SingleJointPositionProvider;
import us.ihmc.commonWalkingControlModules.packetProducers.HandPoseStatusProducer;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.DataType;
import us.ihmc.communication.packets.wholebody.JointAnglesPacket;
import us.ihmc.communication.packets.wholebody.SingleJointAnglePacket;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameMatrix3D;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.trajectories.CirclePoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class JointPositionHighLevelController extends HighLevelBehavior implements Stoppable
{   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean VISUALIZE_TASKSPACE_TRAJECTORIES = true;
   private static final boolean DEBUG = false;
   
   private final String name;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final HashSet<OneDoFJoint> jointsBeingControlled = new HashSet<OneDoFJoint>();
   private final FullRobotModel fullRobotModel;

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
   private boolean firstPacket = true;
   private final HandPoseStatusProducer handPoseStatusProducer;
   private boolean handTrajectoryDoneHasBeenSent;


   private final HashMap<OneDoFJoint, Double> previousPosition = new HashMap<OneDoFJoint, Double>();

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
   private final SideDependentList<TaskspaceToJointspaceCalculator> handTaskspaceToJointspaceCalculators;
   private final SideDependentList<PoseReferenceFrame> optionalHandControlFrames;

   private final MomentumBasedController momentumBasedController;
   private final ICPAndMomentumBasedController icpAndMomentumBasedController;
   
   private final static double MAX_DELTA_TO_BELIEVE_DESIRED = 0.05;

   public JointPositionHighLevelController(final MomentumBasedController momentumBasedController, ICPAndMomentumBasedController icpAndMomentumBasedController,
         VariousWalkingProviders variousWalkingProviders)
   {
      super(controllerState);
      
      name = getClass().getSimpleName();

      timeProvider = momentumBasedController.getYoTime();
      this.desiredJointsProvider = variousWalkingProviders.getDesiredJointsPositionProvider();
      this.handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      this.singleJointPositionProvider = variousWalkingProviders.getSingleJointPositionProvider();
      this.momentumBasedController = momentumBasedController;
      this.icpAndMomentumBasedController = icpAndMomentumBasedController;
      this.handPoseStatusProducer = variousWalkingProviders.getHandPoseStatusProducer();
      
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
      
      for (int i = 0; i < fullRobotModel.getOneDoFJoints().length; i++)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJoints()[i];
         String joinName = joint.getName();

         if (joinName.contains("finger"))
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
      
      YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      handTaskspaceControllers = new SideDependentList<>();
      areHandTaskspaceControlled = new SideDependentList<>();
      areHandJointspaceControlled = new SideDependentList<>();
      handStraightLinePoseTrajectoryGenerators = new SideDependentList<>();
      handCircularPoseTrajectoryGenerators = new SideDependentList<>();
      handTaskspaceToJointspaceCalculators = new SideDependentList<>();
      optionalHandControlFrames = new SideDependentList<PoseReferenceFrame>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = name + robotSide.getCamelCaseNameForMiddleOfExpression() + "Hand";
         
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
         CirclePoseTrajectoryGenerator handCircularPoseTrajectoryGenerator = new CirclePoseTrajectoryGenerator(namePrefix + "Circular", chest.getBodyFixedFrame(), trajectoryTimeProvider, registry, yoGraphicsListRegistry);
         
         handStraightLinePoseTrajectoryGenerators.put(robotSide, handStraightLinePoseTrajectoryGenerator);
         handCircularPoseTrajectoryGenerators.put(robotSide, handCircularPoseTrajectoryGenerator);

         optionalHandControlFrames.put(robotSide, new PoseReferenceFrame("optional" + robotSide.getCamelCaseNameForMiddleOfExpression() + "HandControlFrame", handControlFrame));
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
            
            for (ArmJointName jointName : ArmJointName.values())
            {
               OneDoFJoint joint = fullRobotModel.getArmJoint(robotSide, jointName);
               if (joint == null) continue;

               trajectoryGenerator.get(joint).setFinalPosition(joint.getqDesired());
               previousPosition.put(joint, joint.getqDesired());
            }
         }
      }

      momentumBasedController.callUpdatables();
      icpAndMomentumBasedController.update();
   }

   @Override
   public void doTransitionIntoAction()
   {
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
   
      if (singleJointPositionProvider != null && singleJointPositionProvider.checkForNewPacket())
         initializeFromSingleJointAnglePacket(singleJointPositionProvider.getNewPacket());
      
      for (RobotSide side : RobotSide.values)
      {
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
      }
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
      initialTrajectoryTime = timeProvider.getDoubleValue();
      trajectoryTimeProvider.set(packet.trajcetoryTime);
      
      for (OneDoFJoint joint : jointsBeingControlled)
      {
         if (packet.jointName.equals(joint.getName()))
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
            
            double desiredPosition = MathTools.clipToMinMax(packet.angle, joint.getJointLimitLower(), joint.getJointLimitUpper());
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
      computeHandCurrentDesiredFramePose(currentDesiredHandPose, robotSide, worldFrame, handControlFrame);
      handStraightLinePoseTrajectoryGenerator.setInitialPose(currentDesiredHandPose);
      handStraightLinePoseTrajectoryGenerator.setFinalPose(desiredHandPose);
      handStraightLinePoseTrajectoryGenerator.setTrajectoryTime(handPoseProvider.getTrajectoryTime());

      initializeTaskspaceHandTrajectory(robotSide, handStraightLinePoseTrajectoryGenerator, selectionMatrix);
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

      computeHandCurrentDesiredFramePose(currentDesiredHandPose, robotSide, worldFrame, optionalHandControlFrame);
      handCircularPoseTrajectoryGenerator.setInitialPose(currentDesiredHandPose);
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

   private void computeHandCurrentDesiredFramePose(FramePose desiredHandPoseToPack, RobotSide robotSide, ReferenceFrame trajectoryFrame, ReferenceFrame newControlFrame)
   {
      TaskspaceToJointspaceCalculator handTaskspaceToJointspaceCalculator = handTaskspaceToJointspaceCalculators.get(robotSide);
      if (areHandTaskspaceControlled.get(robotSide).getBooleanValue())
      {
         desiredHandPoseToPack.setPoseIncludingFrame(handTaskspaceControllers.get(robotSide).getDesiredPose());
         ReferenceFrame oldControlFrame = handTaskspaceToJointspaceCalculator.getControlFrame();
         if (oldControlFrame != newControlFrame)
         {
            handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(newControlFrame);
            handTaskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(desiredHandPoseToPack, trajectoryFrame);
         }
      }
      else
      {
         handTaskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(newControlFrame);
         handTaskspaceToJointspaceCalculator.initializeFromDesiredJointAngles();
         handTaskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(desiredHandPoseToPack, trajectoryFrame);
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
