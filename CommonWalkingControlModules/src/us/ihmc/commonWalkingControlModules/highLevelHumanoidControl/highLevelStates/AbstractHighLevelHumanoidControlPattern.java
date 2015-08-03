package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;


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

   protected final PelvisOrientationManager pelvisOrientationManager;
   protected final PelvisICPBasedTranslationManager pelvisICPBasedTranslationManager;
   protected final ChestOrientationManager chestOrientationManager;
   protected final HeadOrientationManager headOrientationManager;
   protected final ManipulationControlModule manipulationControlModule;
   protected final FeetManager feetManager;

   private final List<OneDoFJoint> torqueControlJoints = new ArrayList<OneDoFJoint>();
   protected final OneDoFJoint[] unconstrainedJoints;
   protected final OneDoFJoint[] allOneDoFjoints;

   protected final FullRobotModel fullRobotModel;
   protected final MomentumBasedController momentumBasedController;
   protected final WalkingControllerParameters walkingControllerParameters;

   protected final SideDependentList<? extends ContactablePlaneBody> feet, handPalms;

   protected final DoubleYoVariable coefficientOfFriction = new DoubleYoVariable("coefficientOfFriction", registry);

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   protected final VariousWalkingProviders variousWalkingProviders;

   protected final VariousWalkingManagers variousWalkingManagers;

   protected final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final YoPDGains unconstrainedJointsControlGains;

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

      this.unconstrainedJointsControlGains = walkingControllerParameters.createUnconstrainedJointsControlGains(registry);

      coefficientOfFriction.set(1.0);

      // Setup foot control modules:
//    setupFootControlModules(); //TODO: get rid of that?

      /////////////////////////////////////////////////////////////////////////////////////////////
      // Setup the RootJointAngularAccelerationControlModule for PelvisOrientation control ////////

      // Setup joint constraints
      unconstrainedJoints = findUnconstrainedJoints();
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
            torqueControlJoints.addAll(handJointsList);
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
      momentumBasedController.initialize();
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
      doUnconstrainedJointControl();

      setTorqueControlJointsToZeroDersiredAcceleration();

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

   protected void doUnconstrainedJointControl()
   {
      momentumBasedController.doPDControl(unconstrainedJoints, unconstrainedJointsControlGains);
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
