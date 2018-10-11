package us.ihmc.avatar.warmup;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.KinematicsBasedFootSwitch;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.FrameMessageCommandConverter;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class HumanoidControllerWarmup
{
   private static final double gravityZ = 9.81;
   private static final double velocityDecay = 0.98;

   private final DRCRobotModel robotModel;
   private final double controlDT;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoDouble yoTime = new YoDouble("time", registry);

   private final StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());
   private final CommandInputManager commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
   private final List<ContactablePlaneBody> contactableBodies = new ArrayList<>();

   private final SideDependentList<YoEnum<ConstraintType>> footStates = new SideDependentList<>();

   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidReferenceFrames referenceFrames;
   private OneDoFJoint[] oneDoFJoints;

   private HighLevelControlManagerFactory managerFactory;
   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private WalkingHighLevelHumanoidController walkingController;
   private WholeBodyControllerCore controllerCore;
   private JointDesiredOutputList controllerOutput;

   private final List<Runnable> tickListeners = new ArrayList<>();

   public HumanoidControllerWarmup(DRCRobotModel robotModel)
   {
      this.robotModel = robotModel;
      controlDT = robotModel.getControllerDT();

      setupController();
      controllerToolbox.initialize();
      walkingController.initialize();
      controllerCore.initialize();
   }

   protected abstract void runWarmup();

   public void addTickListener(Runnable listener)
   {
      tickListeners.add(listener);
   }

   protected void simulate(double time)
   {
      double startTime = yoTime.getDoubleValue();
      while (yoTime.getDoubleValue() - startTime < time)
      {
         doSingleTimeUpdate();

         for (Runnable listener : tickListeners)
         {
            listener.run();
         }
      }
   }

   protected <M extends Settable<M>> void submitMessage(M message)
   {
      commandInputManager.submitMessage(message);
   }

   protected HumanoidReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public YoVariable<?> getYoVariable(String name)
   {
      return registry.getVariable(name);
   }

   private void doSingleTimeUpdate()
   {
      // (1) do control and compute desired accelerations
      controllerToolbox.update();
      walkingController.doAction();
      ControllerCoreCommand coreCommand = walkingController.getControllerCoreCommand();
      controllerCore.submitControllerCoreCommand(coreCommand);
      controllerCore.compute();

      // (2) integrate accelerations in full robot model
      integrate();

      // update viz and advance time
      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      yoTime.add(robotModel.getControllerDT());
   }

   private final Quaternion newOrientation = new Quaternion();
   private final Quaternion orientation = new Quaternion();
   private final Vector3D newAngularVelocity = new Vector3D();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D desiredAngularAcceleration = new Vector3D();

   private final Point3D position = new Point3D();
   private final Point3D newPosition = new Point3D();
   private final Vector3D linearVelocity = new Vector3D();
   private final Vector3D newLinearVelocity = new Vector3D();
   private final Vector3D desiredLinearAcceleration = new Vector3D();

   private final FrameVector3D frameLinearVelocity = new FrameVector3D();
   private final FrameVector3D frameAngularVelocity = new FrameVector3D();
   private final Twist rootJointTwist = new Twist();

   private void integrate()
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         JointDesiredOutputReadOnly jointDesireds = controllerOutput.getJointDesiredOutput(joint);

         if (jointDesireds == null)
         {
            continue;
         }

         if (jointDesireds.hasDesiredAcceleration())
         {
            double q = joint.getQ();
            double qd = joint.getQd();
            double qdd = jointDesireds.getDesiredAcceleration();

            double qNew = q + controlDT * qd + 0.5 * controlDT * controlDT * qdd;
            double qdNew = qd + controlDT * qdd;

            joint.setQ(qNew);
            joint.setQd(velocityDecay * qdNew);
         }
      }

      RootJointDesiredConfigurationDataReadOnly rootJointOutput = controllerCore.getOutputForRootJoint();
      DenseMatrix64F desiredAcceleration = rootJointOutput.getDesiredAcceleration();
      desiredAngularAcceleration.set(desiredAcceleration.get(0), desiredAcceleration.get(1), desiredAcceleration.get(2));
      desiredLinearAcceleration.set(desiredAcceleration.get(3), desiredAcceleration.get(4), desiredAcceleration.get(5));

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      rootJoint.getTranslation(position);
      rootJoint.getLinearVelocity(linearVelocity);

      newPosition.set(desiredLinearAcceleration);
      newPosition.scale(0.5 * controlDT);
      newPosition.add(linearVelocity);
      newPosition.scale(controlDT);
      newPosition.add(position);
      newLinearVelocity.set(desiredLinearAcceleration);
      newLinearVelocity.scale(controlDT);
      newLinearVelocity.add(linearVelocity);

      RotationTools.integrateAngularVelocity(angularVelocity, controlDT, newOrientation);
      newOrientation.preMultiply(orientation);
      newAngularVelocity.set(desiredAngularAcceleration);
      newAngularVelocity.scale(controlDT);
      newAngularVelocity.add(angularVelocity);

      rootJoint.setRotation(newOrientation);
      rootJoint.setPosition(newPosition);
      rootJoint.updateFramesRecursively();
      frameLinearVelocity.setIncludingFrame(ReferenceFrame.getWorldFrame(), newLinearVelocity);
      frameAngularVelocity.setIncludingFrame(ReferenceFrame.getWorldFrame(), newAngularVelocity);
      frameLinearVelocity.scale(velocityDecay);
      frameAngularVelocity.scale(velocityDecay);
      frameLinearVelocity.changeFrame(rootJoint.getFrameAfterJoint());
      frameAngularVelocity.changeFrame(rootJoint.getFrameAfterJoint());
      rootJointTwist.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(), frameLinearVelocity,
                         frameAngularVelocity);
      rootJoint.setJointTwist(rootJointTwist);
   }

   private void createControllerCore(YoVariableRegistry walkingControllerRegistry)
   {
      InverseDynamicsJoint[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());
      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, rootJoint, jointsToOptimizeFor, centerOfMassFrame,
                                                                            momentumOptimizationSettings, yoGraphicsListRegistry, walkingControllerRegistry);
      toolbox.setupForInverseDynamicsSolver(contactableBodies);

      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      toolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);

      FeedbackControlCommandList template = managerFactory.createFeedbackControlTemplate();
      controllerOutput = new JointDesiredOutputList(fullRobotModel.getControllableOneDoFJoints());
      controllerCore = new WholeBodyControllerCore(toolbox, template, controllerOutput, walkingControllerRegistry);
   }

   private void createWalkingControllerAndSetUpManagerFactory(YoVariableRegistry walkingControllerParent, YoVariableRegistry managerFactoryParent)
   {
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, referenceFrames);
      FrameMessageCommandConverter commandConversionHelper = new FrameMessageCommandConverter(referenceFrameHashCodeResolver);
      commandInputManager.registerConversionHelper(commandConversionHelper);

      double omega0 = walkingControllerParameters.getOmega0();

      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i), additionalContactNames.get(i),
                                                            additionalContactTransforms.get(i));
      contactableBodiesFactory.setFullRobotModel(fullRobotModel);
      contactableBodiesFactory.setReferenceFrames(referenceFrames);
      SideDependentList<ContactableFoot> feet = new SideDependentList<>(contactableBodiesFactory.createFootContactableFeet());
      List<ContactablePlaneBody> additionalContacts = contactableBodiesFactory.createAdditionalContactPoints();
      contactableBodiesFactory.disposeFactory();

      for (RobotSide robotSide : RobotSide.values)
         contactableBodies.add(feet.get(robotSide));
      contactableBodies.addAll(additionalContacts);

      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityZ;
      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(feet, totalRobotWeight, referenceFrames.getSoleZUpFrames());

      controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel, referenceFrames, footSwitches, null, null,
                                                                                                    yoTime, gravityZ, omega0, feet, controlDT, null,
                                                                                                    contactableBodies, yoGraphicsListRegistry);

      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultTouchdownTime = walkingControllerParameters.getDefaultTouchdownTime();
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double defaultFinalTransferTime = walkingControllerParameters.getDefaultFinalTransferTime();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime, defaultSwingTime, defaultTouchdownTime,
                                                                              defaultInitialTransferTime, defaultFinalTransferTime, feet, statusOutputManager,
                                                                              yoTime, yoGraphicsListRegistry, controllerToolbox.getYoVariableRegistry());
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);

      managerFactory = new HighLevelControlManagerFactory(statusOutputManager, managerFactoryParent);
      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);

      walkingController = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
                                                                 controllerToolbox);
      walkingControllerParent.addChild(walkingController.getYoVariableRegistry());
      walkingControllerParent.getParent().addChild(controllerToolbox.getYoVariableRegistry());
   }

   @SuppressWarnings("unchecked")
   public void setupController()
   {
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      oneDoFJoints = fullRobotModel.getOneDoFJoints();

      // Create registries to match controller so the XML gets loaded properly.
      YoVariableRegistry drcControllerThread = new YoVariableRegistry("DRCControllerThread");
      YoVariableRegistry drcMomentumBasedController = new YoVariableRegistry("DRCMomentumBasedController");
      YoVariableRegistry humanoidHighLevelControllerManager = new YoVariableRegistry("HumanoidHighLevelControllerManager");
      YoVariableRegistry highLevelHumanoidControllerFactory = new YoVariableRegistry("HighLevelHumanoidControllerFactory");
      YoVariableRegistry walkingControllerState = new YoVariableRegistry("WalkingControllerState");
      registry.addChild(drcControllerThread);
      drcControllerThread.addChild(drcMomentumBasedController);
      drcMomentumBasedController.addChild(humanoidHighLevelControllerManager);
      humanoidHighLevelControllerManager.addChild(walkingControllerState);
      humanoidHighLevelControllerManager.addChild(highLevelHumanoidControllerFactory);

      createWalkingControllerAndSetUpManagerFactory(walkingControllerState, highLevelHumanoidControllerFactory);
      createControllerCore(walkingControllerState);
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());

      for (RobotSide robotSide : RobotSide.values)
      {
         String name = robotSide.getLowerCaseName() + "FootAssumeCopOnEdge";
         YoBoolean variable = (YoBoolean) registry.getVariable(name);
         variable.set(true);

         name = robotSide.getLowerCaseName() + "FootAssumeFootBarelyLoaded";
         variable = (YoBoolean) registry.getVariable(name);
         variable.set(true);

         name = robotSide.getCamelCaseNameForStartOfExpression() + "FootCurrentState";
         YoEnum<ConstraintType> footState = (YoEnum<ConstraintType>) registry.getVariable(name);
         footStates.put(robotSide, footState);
      }

      ParameterLoaderHelper.loadParameters(this, robotModel.getWholeBodyControllerParametersFile(), drcControllerThread);

      YoVariable<?> defaultHeight = registry.getVariable(PelvisHeightControlState.class.getSimpleName(),
                                                         PelvisHeightControlState.class.getSimpleName() + "DefaultHeight");
      if (Double.isNaN(defaultHeight.getValueAsDouble()))
      {
         throw new RuntimeException("Need to load a default height.");
      }
   }

   private SideDependentList<FootSwitchInterface> createFootSwitches(SideDependentList<ContactableFoot> feet, double totalRobotWeight,
                                                                     SideDependentList<? extends ReferenceFrame> soleZupFrames)
   {
      SideDependentList<FootSwitchInterface> ret = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         FootSwitchInterface footSwitch = new KinematicsBasedFootSwitch(feet.get(robotSide).getName(), feet, () -> 0.0, totalRobotWeight, robotSide, registry);
         ret.put(robotSide, footSwitch);
      }
      return ret;
   }

   public DoubleProvider getTimeProvider()
   {
      return yoTime;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
