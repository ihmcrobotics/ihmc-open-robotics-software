package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameQuaternion;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.CubicPolynomialTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoVariableDoubleProvider;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAngularAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHeadOrientationProvider;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class AbstractHighLevelHumanoidControlPattern extends State<HighLevelState>
{
   private final String name = getClass().getSimpleName();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);

   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final DoubleYoVariable yoTime;
   protected final double controlDT;
   protected final double gravity;
   protected final CommonWalkingReferenceFrames referenceFrames;

   protected final TwistCalculator twistCalculator;

   private final DesiredHeadOrientationProvider desiredHeadOrientationProvider;

   protected final ChestOrientationManager chestOrientationManager;
   protected final HeadOrientationManager headOrientationManager;
   protected final ManipulationControlModule manipulationControlModule;

   private final LidarControllerInterface lidarControllerInterface;

   private final OneDoFJoint jointForExtendedNeckPitchRange;
   private final List<OneDoFJoint> torqueControlJoints = new ArrayList<OneDoFJoint>();
   protected final OneDoFJoint[] positionControlJoints;

   protected final YoFrameQuaternion desiredPelvisOrientation = new YoFrameQuaternion("desiredPelvis", worldFrame, registry);
   protected final YoFrameVector desiredPelvisAngularVelocity = new YoFrameVector("desiredPelvisAngularVelocity", worldFrame, registry);

   protected final YoFrameVector desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", worldFrame, registry);
   protected final SideDependentList<Integer> legJacobianIds = new SideDependentList<Integer>();
   protected final SideDependentList<FootControlModule> footEndEffectorControlModules = new SideDependentList<FootControlModule>();
   protected final FullRobotModel fullRobotModel;
   protected final MomentumBasedController momentumBasedController;
   protected final WalkingControllerParameters walkingControllerParameters;

   private final DoubleYoVariable kpUpperBody = new DoubleYoVariable("kpUpperBody", registry);
   private final DoubleYoVariable zetaUpperBody = new DoubleYoVariable("zetaUpperBody", registry);
   private final DoubleYoVariable maxAccelerationUpperBody = new DoubleYoVariable("maxAccelerationUpperBody", registry);
   private final DoubleYoVariable maxJerkUpperBody = new DoubleYoVariable("maxJerkUpperBody", registry);

   protected final SideDependentList<? extends ContactablePlaneBody> feet, handPalms;

   protected final DoubleYoVariable coefficientOfFriction = new DoubleYoVariable("coefficientOfFriction", registry);
   private final RootJointAngularAccelerationControlModule rootJointAccelerationControlModule;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   private final VariousWalkingProviders variousWalkingProviders;

// private final VariousWalkingManagers variousWalkingManagers;

   protected final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;

   private final DoubleYoVariable kpPelvisOrientation = new DoubleYoVariable("kpPelvisOrientation", registry);
   private final DoubleYoVariable zetaPelvisOrientation = new DoubleYoVariable("zetaPelvisOrientation", registry);
   private final DoubleYoVariable maxAccelerationPelvisOrientation = new DoubleYoVariable("maxAccelerationPelvisOrientation", registry);
   private final DoubleYoVariable maxJerkPelvisOrientation = new DoubleYoVariable("maxJerkPelvisOrientation", registry);

   public AbstractHighLevelHumanoidControlPattern(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
           MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters,
           LidarControllerInterface lidarControllerInterface, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
           HighLevelState controllerState)
   {
      super(controllerState);

      this.variousWalkingProviders = variousWalkingProviders;

//    this.variousWalkingManagers = variousWalkingManagers;

      this.dynamicGraphicObjectsListRegistry = dynamicGraphicObjectsListRegistry;

      // Getting parameters from the momentumBasedController
      this.momentumBasedController = momentumBasedController;
      fullRobotModel = momentumBasedController.getFullRobotModel();
      yoTime = momentumBasedController.getYoTime();
      gravity = momentumBasedController.getGravityZ();
      controlDT = momentumBasedController.getControlDT();
      twistCalculator = momentumBasedController.getTwistCalculator();
      referenceFrames = momentumBasedController.getReferenceFrames();

      feet = momentumBasedController.getContactablePlaneFeet();
      handPalms = momentumBasedController.getContactablePlaneHandsWithFingersBentBack();

      this.desiredHeadOrientationProvider = variousWalkingProviders.getDesiredHeadOrientationProvider();
      this.headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      this.chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      this.manipulationControlModule = variousWalkingManagers.getManipulationControlModule();

      this.lidarControllerInterface = lidarControllerInterface;
      this.walkingControllerParameters = walkingControllerParameters;

      // Setup jacobians for legs and arms
      setupLegJacobians(fullRobotModel);
      coefficientOfFriction.set(1.0);

      setUpperBodyControlGains(walkingControllerParameters.getKpUpperBody(), walkingControllerParameters.getZetaUpperBody(),
                               walkingControllerParameters.getMaxAccelerationUpperBody(), walkingControllerParameters.getMaxJerkUpperBody());

      // Setup foot control modules:
//    setupFootControlModules(); //TODO: get rid of that?

//      DesiredHandPoseProvider handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
//      TorusPoseProvider torusPoseProvider = variousWalkingProviders.getTorusPoseProvider();
//      TorusManipulationProvider torusManipulationProvider = variousWalkingProviders.getTorusManipulationProvider();

      jointForExtendedNeckPitchRange = setupJointForExtendedNeckPitchRange();

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the RootJointAngularAccelerationControlModule for PelvisOrientation control ////////
      kpPelvisOrientation.set(walkingControllerParameters.getKpPelvisOrientation());
      zetaPelvisOrientation.set(walkingControllerParameters.getZetaPelvisOrientation());
      maxAccelerationPelvisOrientation.set(walkingControllerParameters.getMaxAccelerationPelvisOrientation());
      maxJerkPelvisOrientation.set(walkingControllerParameters.getMaxJerkPelvisOrientation());


      rootJointAccelerationControlModule = new RootJointAngularAccelerationControlModule(controlDT, momentumBasedController, registry);
      VariableChangedListener pelvisOrientationGainsChangedListener = createPelvisOrientationGainsChangedListener();
      pelvisOrientationGainsChangedListener.variableChanged(null);

      // Setup joint constraints
      positionControlJoints = setupJointConstraints();


      if ((jointForExtendedNeckPitchRange != null) && (desiredHeadOrientationProvider != null))
      {
         DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(walkingControllerParameters.getTrajectoryTimeHeadOrientation());
         extendedNeckPitchInitialAngle = new YoVariableDoubleProvider("extendedNeckPitchInitialAngle", registry);
         extendedNeckPitchFinalAngle = new YoVariableDoubleProvider("extendedNeckPitchFinalAngle", registry);
         extendedNeckPitchTrajectory = new CubicPolynomialTrajectoryGenerator("extendedNeckPitchTrajectory", extendedNeckPitchInitialAngle,
                 extendedNeckPitchFinalAngle, trajectoryTimeProvider, registry);
         extendedNeckPitchTrajectory.initialize();
         extendedNeckPitchReceivedTime = new DoubleYoVariable("extendedNeckPitchReceived", registry);
      }
      else
      {
         extendedNeckPitchInitialAngle = null;
         extendedNeckPitchFinalAngle = null;
         extendedNeckPitchTrajectory = null;
         extendedNeckPitchReceivedTime = null;
      }
   }

   private final CubicPolynomialTrajectoryGenerator extendedNeckPitchTrajectory;
   private final DoubleYoVariable extendedNeckPitchReceivedTime;
   private final DoubleYoVariable extendedNeckPitchDesiredAngle = new DoubleYoVariable("extendedNeckPitchDesiredAngle", registry);
   private final DoubleYoVariable extendedNeckPitchDesiredVelocity = new DoubleYoVariable("extendedNeckPitchDesiredVelocity", registry);
   private final YoVariableDoubleProvider extendedNeckPitchInitialAngle;
   private final YoVariableDoubleProvider extendedNeckPitchFinalAngle;

   public void setUpperBodyControlGains(double kpUpperBody, double zetaUpperBody, double maxAcceleration, double maxJerk)
   {
      this.kpUpperBody.set(kpUpperBody);
      this.zetaUpperBody.set(zetaUpperBody);
      this.maxAccelerationUpperBody.set(maxAcceleration);
      this.maxJerkUpperBody.set(maxJerk);
   }


   private VariableChangedListener createPelvisOrientationGainsChangedListener()
   {
      VariableChangedListener listener = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            double dPelvisOrientation = GainCalculator.computeDerivativeGain(kpPelvisOrientation.getDoubleValue(), zetaPelvisOrientation.getDoubleValue());
            rootJointAccelerationControlModule.setProportionalGains(kpPelvisOrientation.getDoubleValue(), kpPelvisOrientation.getDoubleValue(),
                    kpPelvisOrientation.getDoubleValue());
            rootJointAccelerationControlModule.setDerivativeGains(dPelvisOrientation, dPelvisOrientation, dPelvisOrientation);
            rootJointAccelerationControlModule.setMaxAccelerationAndJerk(maxAccelerationPelvisOrientation.getDoubleValue(),
                    maxJerkPelvisOrientation.getDoubleValue());
         }
      };

      kpPelvisOrientation.addVariableChangedListener(listener);
      zetaPelvisOrientation.addVariableChangedListener(listener);
      maxAccelerationPelvisOrientation.addVariableChangedListener(listener);
      maxJerkPelvisOrientation.addVariableChangedListener(listener);

      return listener;
   }


   protected void setupLegJacobians(FullRobotModel fullRobotModel)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody endEffector = fullRobotModel.getFoot(robotSide);
         int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(fullRobotModel.getPelvis(), endEffector, endEffector.getBodyFixedFrame());
         legJacobianIds.put(robotSide, jacobianId);
      }
   }

   protected OneDoFJoint setupJointForExtendedNeckPitchRange()
   {
      if (walkingControllerParameters.getJointNameForExtendedPitchRange() == null)
         return null;

      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());

      InverseDynamicsJoint[] inverseDynamicsJointForExtendedNeckPitchControl = ScrewTools.findJointsWithNames(allJoints,
                                                                                  walkingControllerParameters.getJointNameForExtendedPitchRange());
      OneDoFJoint[] jointForExtendedNeckPitchControl = ScrewTools.filterJoints(inverseDynamicsJointForExtendedNeckPitchControl, OneDoFJoint.class);

      if (jointForExtendedNeckPitchControl.length == 1)
         return jointForExtendedNeckPitchControl[0];
      else
         return null;
   }

   protected OneDoFJoint[] setupJointConstraints()
   {
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      String[] headOrientationControlJointNames = walkingControllerParameters.getDefaultHeadOrientationControlJointNames();
      String[] chestOrientationControlJointNames = walkingControllerParameters.getDefaultChestOrientationControlJointNames();
      
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] headOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, headOrientationControlJointNames);
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);

      List<InverseDynamicsJoint> unconstrainedJoints = new ArrayList<InverseDynamicsJoint>(Arrays.asList(allJoints));

      for (RobotSide robotSide : RobotSide.values)
      {
         // Leg joints
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         InverseDynamicsJoint[] legJoints = ScrewTools.createJointPath(pelvis, foot);
         unconstrainedJoints.removeAll(Arrays.asList(legJoints));

         // Arm joints
         if (fullRobotModel.getHand(robotSide) != null)
         {
            RigidBody hand = fullRobotModel.getHand(robotSide);
            InverseDynamicsJoint[] armJoints = ScrewTools.createJointPath(chest, hand);
            unconstrainedJoints.removeAll(Arrays.asList(armJoints));

            // Hand joints
            InverseDynamicsJoint[] handJoints = ScrewTools.computeSubtreeJoints(hand);
            OneDoFJoint[] handJointsArray = new OneDoFJoint[ScrewTools.computeNumberOfJointsOfType(OneDoFJoint.class, handJoints)];
            ScrewTools.filterJoints(handJoints, handJointsArray, OneDoFJoint.class);
            List<OneDoFJoint> handJointsList = Arrays.asList(handJointsArray);
            unconstrainedJoints.removeAll(handJointsList);
            torqueControlJoints.addAll(handJointsList);
         }

      }

      // Lidar joint
      if (lidarControllerInterface != null)
         unconstrainedJoints.remove(lidarControllerInterface.getLidarJoint());

      // Head joints
      unconstrainedJoints.removeAll(Arrays.asList(headOrientationControlJoints));
      if (jointForExtendedNeckPitchRange != null)
         unconstrainedJoints.remove(jointForExtendedNeckPitchRange);

      // Chest joints
      unconstrainedJoints.removeAll(Arrays.asList(chestOrientationControlJoints));

      unconstrainedJoints.remove(fullRobotModel.getRootJoint());
      InverseDynamicsJoint[] unconstrainedJointsArray = new InverseDynamicsJoint[unconstrainedJoints.size()];
      unconstrainedJoints.toArray(unconstrainedJointsArray);
      OneDoFJoint[] positionControlJoints = new OneDoFJoint[unconstrainedJointsArray.length];
      ScrewTools.filterJoints(unconstrainedJointsArray, positionControlJoints, OneDoFJoint.class);

      unconstrainedJoints.removeAll(Arrays.asList(positionControlJoints));

      if (unconstrainedJoints.size() > 0)
         throw new RuntimeException("Joints unconstrained: " + unconstrainedJoints);

      return positionControlJoints;
   }

   public void initialize()
   {
      variousWalkingProviders.clearPoseProviders();
      callUpdatables();
   }

   protected void callUpdatables()
   {
      double time = yoTime.getDoubleValue();
      for (int i = 0; i < updatables.size(); i++)
      {
         updatables.get(i).update(time);
      }
   }

   public void doMotionControl()
   {
      momentumBasedController.doPrioritaryControl();
      callUpdatables();

      doFootControl();
      doArmControl();
      doHeadControl();
      doLidarJointControl();
      doChestControl();
      doCoMControl();
      doPelvisControl();
      doJointPositionControl();

      setTorqueControlJointsToZeroDersiredAcceleration();

      momentumBasedController.doSecondaryControl();
   }

   protected void doLidarJointControl()
   {
      if ((lidarControllerInterface != null) && (lidarControllerInterface.getLidarJoint() != null))
         momentumBasedController.setOneDoFJointAcceleration(lidarControllerInterface.getLidarJoint(), 0.0);
   }

   protected void doHeadControl()
   {
      if (headOrientationManager != null)
      {
         headOrientationManager.compute();

         if (jointForExtendedNeckPitchRange != null)
         {
            double kpHead = this.kpUpperBody.getDoubleValue();
            double kdHead = GainCalculator.computeDerivativeGain(kpHead, zetaUpperBody.getDoubleValue());

            double maxAcceleration = maxAccelerationUpperBody.getDoubleValue();
            double maxJerk = maxJerkUpperBody.getDoubleValue();


            if ((desiredHeadOrientationProvider != null) && (extendedNeckPitchTrajectory != null))
            {
               double qDesired, qdDesired;
               double desiredExtendedNeckPitchJointAngle = desiredHeadOrientationProvider.getDesiredExtendedNeckPitchJointAngle();
               if (!Double.isNaN(desiredExtendedNeckPitchJointAngle))
               {
                  extendedNeckPitchInitialAngle.set(extendedNeckPitchTrajectory.getValue());
                  extendedNeckPitchFinalAngle.set(desiredExtendedNeckPitchJointAngle);
                  extendedNeckPitchTrajectory.initialize();
                  extendedNeckPitchTrajectory.compute(0.0);
                  extendedNeckPitchReceivedTime.set(yoTime.getDoubleValue());
               }
               else
               {
                  extendedNeckPitchTrajectory.compute(yoTime.getDoubleValue() - extendedNeckPitchReceivedTime.getDoubleValue());
               }

               qDesired = extendedNeckPitchTrajectory.getValue();
               qdDesired = extendedNeckPitchTrajectory.getVelocity();

               extendedNeckPitchDesiredAngle.set(qDesired);
               extendedNeckPitchDesiredVelocity.set(qdDesired);

               momentumBasedController.doPDControl(jointForExtendedNeckPitchRange, kpHead, kdHead, qDesired, qdDesired, maxAcceleration, maxJerk);
            }
            else
            {
               momentumBasedController.doPDControl(jointForExtendedNeckPitchRange, kpHead, kdHead, 0.0, 0.0, maxAcceleration, maxJerk);
            }
         }
      }
   }

   protected void doChestControl()
   {
      if (chestOrientationManager != null)
      {
         chestOrientationManager.compute();
      }
   }

   protected void doFootControl()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footEndEffectorControlModules.get(robotSide).doControl();
      }
   }

   protected void doArmControl()
   {
      if (manipulationControlModule != null)
         manipulationControlModule.doControl();
   }

   protected void doCoMControl()
   {
   }

   protected void doPelvisControl()
   {
      OrientationTrajectoryData pelvisOrientationTrajectoryData = new OrientationTrajectoryData();
      FrameOrientation orientation = new FrameOrientation(desiredPelvisOrientation.getReferenceFrame());
      desiredPelvisOrientation.get(orientation);
      pelvisOrientationTrajectoryData.set(orientation, desiredPelvisAngularVelocity.getFrameVectorCopy(),
              desiredPelvisAngularAcceleration.getFrameVectorCopy());

      rootJointAccelerationControlModule.doControl(pelvisOrientationTrajectoryData);
   }

   protected void doJointPositionControl()
   {
      double kpUpperBody = this.kpUpperBody.getDoubleValue();
      double kdUpperBody = GainCalculator.computeDerivativeGain(kpUpperBody, zetaUpperBody.getDoubleValue());
      double maxJerkUpperBody = this.maxJerkUpperBody.getDoubleValue();
      double maxAccelerationUpperBody = this.maxAccelerationUpperBody.getDoubleValue();

      momentumBasedController.doPDControl(positionControlJoints, kpUpperBody, kdUpperBody, maxAccelerationUpperBody, maxJerkUpperBody);
   }

   // TODO: New methods coming from extending State class
   @Override
   public void doAction()
   {
      doMotionControl();
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

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   protected void setTorqueControlJointsToZeroDersiredAcceleration()
   {
      for (OneDoFJoint joint : torqueControlJoints)
      {
         momentumBasedController.setOneDoFJointAcceleration(joint, 0.0);
      }
   }

   public void addUpdatables(ArrayList<Updatable> updatables)
   {
      this.updatables.addAll(updatables);
   }
}
