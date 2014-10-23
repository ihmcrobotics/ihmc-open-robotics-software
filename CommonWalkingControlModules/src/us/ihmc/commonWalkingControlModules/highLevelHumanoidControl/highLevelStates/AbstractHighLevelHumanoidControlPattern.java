package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.HeadOrientationProvider;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;


public abstract class AbstractHighLevelHumanoidControlPattern extends HighLevelBehavior
{
   private final String name = getClass().getSimpleName();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);

   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final DoubleYoVariable yoTime;
   protected final double controlDT;
   protected final double gravity;
   protected final CommonHumanoidReferenceFrames referenceFrames;

   protected final TwistCalculator twistCalculator;

   private final HeadOrientationProvider desiredHeadOrientationProvider;

   protected final PelvisOrientationManager pelvisOrientationManager;
   protected final ChestOrientationManager chestOrientationManager;
   protected final HeadOrientationManager headOrientationManager;
   protected final ManipulationControlModule manipulationControlModule;
   protected final FeetManager feetManager;

   private final OneDoFJoint jointForExtendedNeckPitchRange;
   private final List<OneDoFJoint> torqueControlJoints = new ArrayList<OneDoFJoint>();
   protected final OneDoFJoint[] positionControlJoints;

   protected final FullRobotModel fullRobotModel;
   protected final MomentumBasedController momentumBasedController;
   protected final WalkingControllerParameters walkingControllerParameters;

   private final DoubleYoVariable kpUpperBody = new DoubleYoVariable("kpUpperBody", registry);
   private final DoubleYoVariable zetaUpperBody = new DoubleYoVariable("zetaUpperBody", registry);
   private final DoubleYoVariable maxAccelerationUpperBody = new DoubleYoVariable("maxAccelerationUpperBody", registry);
   private final DoubleYoVariable maxJerkUpperBody = new DoubleYoVariable("maxJerkUpperBody", registry);

   protected final SideDependentList<? extends ContactablePlaneBody> feet, handPalms;

   protected final DoubleYoVariable coefficientOfFriction = new DoubleYoVariable("coefficientOfFriction", registry);

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   protected final VariousWalkingProviders variousWalkingProviders;

   protected final VariousWalkingManagers variousWalkingManagers;

   protected final YoGraphicsListRegistry yoGraphicsListRegistry;

   public AbstractHighLevelHumanoidControlPattern(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters, HighLevelState controllerState)
   {
      super(controllerState);

      this.variousWalkingProviders = variousWalkingProviders;

      this.variousWalkingManagers = variousWalkingManagers;

      this.yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();

      // Getting parameters from the momentumBasedController
      this.momentumBasedController = momentumBasedController;
      fullRobotModel = momentumBasedController.getFullRobotModel();
      yoTime = momentumBasedController.getYoTime();
      gravity = momentumBasedController.getGravityZ();
      controlDT = momentumBasedController.getControlDT();
      twistCalculator = momentumBasedController.getTwistCalculator();
      referenceFrames = momentumBasedController.getReferenceFrames();

      feet = momentumBasedController.getContactableFeet();
      handPalms = momentumBasedController.getContactableHands();

      this.pelvisOrientationManager = variousWalkingManagers.getPelvisOrientationManager();
      this.desiredHeadOrientationProvider = variousWalkingProviders.getDesiredHeadOrientationProvider();
      this.headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      this.chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      this.manipulationControlModule = variousWalkingManagers.getManipulationControlModule();
      this.feetManager = variousWalkingManagers.getFeetManager();

      this.walkingControllerParameters = walkingControllerParameters;

      coefficientOfFriction.set(1.0);

      setUpperBodyControlGains(walkingControllerParameters.getKpUpperBody(), walkingControllerParameters.getZetaUpperBody(),
                               walkingControllerParameters.getMaxAccelerationUpperBody(), walkingControllerParameters.getMaxJerkUpperBody());

      // Setup foot control modules:
//    setupFootControlModules(); //TODO: get rid of that?

      jointForExtendedNeckPitchRange = setupJointForExtendedNeckPitchRange();

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the RootJointAngularAccelerationControlModule for PelvisOrientation control ////////

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

   protected OneDoFJoint setupJointForExtendedNeckPitchRange()
   {
      if (walkingControllerParameters.getJointNameForExtendedPitchRange() == null)
         return null;

      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());

      InverseDynamicsJoint[] inverseDynamicsJointForExtendedNeckPitchControl = ScrewTools.findJointsWithNames(allJoints, walkingControllerParameters.getJointNameForExtendedPitchRange());
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

      List<InverseDynamicsJoint> unconstrainedJoints = new ArrayList<InverseDynamicsJoint>(Arrays.asList(momentumBasedController.getControlledJoints()));

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
      variousWalkingManagers.initializeManagers();
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
      doChestControl();
      doCoMControl();
      doPelvisControl();
      doJointPositionControl();

      setTorqueControlJointsToZeroDersiredAcceleration();

      momentumBasedController.doSecondaryControl();
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
      feetManager.compute();
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
      pelvisOrientationManager.compute();
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
      for (int i = 0; i < torqueControlJoints.size(); i++)
      {
         momentumBasedController.setOneDoFJointAcceleration(torqueControlJoints.get(i), 0.0);
      }
   }

   public void addUpdatables(ArrayList<Updatable> updatables)
   {
      this.updatables.addAll(updatables);
   }
}
