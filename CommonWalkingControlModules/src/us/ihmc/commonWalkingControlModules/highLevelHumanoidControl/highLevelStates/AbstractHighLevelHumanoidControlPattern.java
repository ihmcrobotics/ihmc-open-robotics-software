package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.ManipulationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.JointspaceAccelerationCommand;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public abstract class AbstractHighLevelHumanoidControlPattern extends HighLevelBehavior
{
   private final String name = getClass().getSimpleName();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);

   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final JointspaceFeedbackControlCommand unconstrainedJointsCommand = new JointspaceFeedbackControlCommand();
   
   protected final DoubleYoVariable yoTime;
   protected final double controlDT;
   protected final double gravity;
   protected final CommonHumanoidReferenceFrames referenceFrames;

   protected final TwistCalculator twistCalculator;

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> unconstrainedDesiredPositions = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> preRateLimitedDesiredAccelerations = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedDesiredAccelerations = new LinkedHashMap<>();

   protected final PelvisOrientationManager pelvisOrientationManager;
   protected final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;
   protected final ChestOrientationManager chestOrientationManager;
   protected final HeadOrientationManager headOrientationManager;
   protected final ManipulationControlModule manipulationControlModule;
   protected final FeetManager feetManager;

   private final List<OneDoFJoint> uncontrolledJoints = new ArrayList<OneDoFJoint>();
   private final JointspaceAccelerationCommand uncontrolledJointsCommand = new JointspaceAccelerationCommand();
   protected final OneDoFJoint[] unconstrainedJoints;
   protected final OneDoFJoint[] allOneDoFjoints;

   protected final FullHumanoidRobotModel fullRobotModel;
   protected final MomentumBasedController momentumBasedController;
   protected final WalkingControllerParameters walkingControllerParameters;

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

      allOneDoFjoints = fullRobotModel.getOneDoFJoints();

      this.pelvisOrientationManager = variousWalkingManagers.getPelvisOrientationManager();
      this.pelvisICPBasedTranslationManager = variousWalkingManagers.getPelvisICPBasedTranslationManager();
      this.headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      this.chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      this.manipulationControlModule = variousWalkingManagers.getManipulationControlModule();
      this.feetManager = variousWalkingManagers.getFeetManager();

      this.walkingControllerParameters = walkingControllerParameters;

      coefficientOfFriction.set(1.0);

      // Setup foot control modules:
      //    setupFootControlModules(); //TODO: get rid of that?

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the RootJointAngularAccelerationControlModule for PelvisOrientation control ////////

      // Setup joint constraints
      unconstrainedJoints = findUnconstrainedJoints();

      for (OneDoFJoint joint : unconstrainedJoints)
      {
         unconstrainedJointsCommand.addJoint(joint, 0.0, 0.0, 0.0);
         String jointName = joint.getName();
         unconstrainedDesiredPositions.put(joint, new DoubleYoVariable("unconstrained_q_d_" + jointName, registry));
         preRateLimitedDesiredAccelerations.put(joint, new DoubleYoVariable("prl_unconstrained_qdd_d_" + jointName, registry));
         rateLimitedDesiredAccelerations.put(joint, new RateLimitedYoVariable("rl_unconstrained_qdd_d_" + jointName, registry, Double.POSITIVE_INFINITY, controlDT));
      }

      YoPDGains unconstrainedJointsControlGains = walkingControllerParameters.createUnconstrainedJointsControlGains(registry);
      unconstrainedJointsCommand.setGains(unconstrainedJointsControlGains);

      for (int i = 0; i < uncontrolledJoints.size(); i++)
         uncontrolledJointsCommand.addJoint(uncontrolledJoints.get(i), 0.0);
   }

   protected OneDoFJoint[] findUnconstrainedJoints()
   {
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      String[] headOrientationControlJointNames = walkingControllerParameters.getDefaultHeadOrientationControlJointNames();
      String[] chestOrientationControlJointNames = walkingControllerParameters.getDefaultChestOrientationControlJointNames();

      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      InverseDynamicsJoint[] headOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, headOrientationControlJointNames);
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);

      List<InverseDynamicsJoint> unconstrainedJointList = new ArrayList<InverseDynamicsJoint>(Arrays.asList(momentumBasedController.getControlledJoints()));

      for (RobotSide robotSide : RobotSide.values)
      {
         // Leg joints
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         InverseDynamicsJoint[] legJoints = ScrewTools.createJointPath(pelvis, foot);
         unconstrainedJointList.removeAll(Arrays.asList(legJoints));

         // Arm joints
         if (fullRobotModel.getHand(robotSide) != null)
         {
            RigidBody hand = fullRobotModel.getHand(robotSide);
            InverseDynamicsJoint[] armJoints = ScrewTools.createJointPath(chest, hand);
            unconstrainedJointList.removeAll(Arrays.asList(armJoints));

            // Hand joints
            InverseDynamicsJoint[] handJoints = ScrewTools.computeSubtreeJoints(hand);
            OneDoFJoint[] handJointsArray = new OneDoFJoint[ScrewTools.computeNumberOfJointsOfType(OneDoFJoint.class, handJoints)];
            ScrewTools.filterJoints(handJoints, handJointsArray, OneDoFJoint.class);
            List<OneDoFJoint> handJointsList = Arrays.asList(handJointsArray);
            unconstrainedJointList.removeAll(handJointsList);
            uncontrolledJoints.addAll(handJointsList);
         }

      }

      // Head joints
      unconstrainedJointList.removeAll(Arrays.asList(headOrientationControlJoints));

      // Chest joints
      unconstrainedJointList.removeAll(Arrays.asList(chestOrientationControlJoints));

      unconstrainedJointList.remove(fullRobotModel.getRootJoint());
      InverseDynamicsJoint[] unconstrainedJointsArray = new InverseDynamicsJoint[unconstrainedJointList.size()];
      unconstrainedJointList.toArray(unconstrainedJointsArray);
      OneDoFJoint[] unconstrainedJointArray = new OneDoFJoint[unconstrainedJointsArray.length];
      ScrewTools.filterJoints(unconstrainedJointsArray, unconstrainedJointArray, OneDoFJoint.class);

      unconstrainedJointList.removeAll(Arrays.asList(unconstrainedJointArray));

      return unconstrainedJointArray;
   }

   public void initialize()
   {
      for (int i = 0; i < unconstrainedJoints.length; i++)
      {
         OneDoFJoint joint = unconstrainedJoints[i];
         unconstrainedDesiredPositions.get(joint).set(joint.getQ());
      }

      momentumBasedController.initialize();
      variousWalkingManagers.initializeManagers();
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
      doUnconstrainedJointControl();

      momentumBasedController.doSecondaryControl();
   }

   protected void doHeadControl()
   {
      if (headOrientationManager != null)
      {
         headOrientationManager.compute();
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

   protected JointspaceFeedbackControlCommand doUnconstrainedJointControl()
   {
      for (int i = 0; i < unconstrainedJoints.length; i++)
      {
         OneDoFJoint joint = unconstrainedJoints[i];
         double desiredPosition = unconstrainedDesiredPositions.get(joint).getDoubleValue();
         unconstrainedJointsCommand.setOneDoFJoint(i, desiredPosition, 0.0, 0.0);
      }

      return unconstrainedJointsCommand;
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
      for (int i = 0; i < allOneDoFjoints.length; i++)
      {
         allOneDoFjoints[i].resetDesiredAccelerationIntegrator();
         allOneDoFjoints[i].setQddDesired(0.0);
         allOneDoFjoints[i].setTau(0.0);
      }

      initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      for (int i = 0; i < allOneDoFjoints.length; i++)
      {
         allOneDoFjoints[i].resetDesiredAccelerationIntegrator();
         allOneDoFjoints[i].setQddDesired(0.0);
         allOneDoFjoints[i].setTau(0.0);
      }
   }

   protected JointspaceAccelerationCommand getUncontrolledJointCommand()
   {
      return uncontrolledJointsCommand;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void addUpdatables(ArrayList<Updatable> updatables)
   {
      this.updatables.addAll(updatables);
   }
}
