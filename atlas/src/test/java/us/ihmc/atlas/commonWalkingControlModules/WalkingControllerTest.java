package us.ihmc.atlas.commonWalkingControlModules;

import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.List;

import javax.management.MBeanServer;
import javax.management.MalformedObjectNameException;
import javax.management.ObjectName;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
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
import us.ihmc.sensorProcessing.outputData.LowLevelJointDataReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoLong;

public class WalkingControllerTest
{
   private static final boolean profile = false;
   private static final boolean showSCS = false;
   private static final double totalTime = 5.0;

   private static final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS, RobotTarget.SCS, false);
   private static final double gravityZ = 9.81;
   private static final double controlDT = robotModel.getControllerDT();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoDouble yoTime = new YoDouble("time", registry);

   private final YoLong allocatedInTick = new YoLong("AllocatedBytesInTick", registry);

   private final StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(ControllerAPIDefinition.getControllerSupportedStatusMessages());
   private final CommandInputManager commandInputManager = new CommandInputManager(ControllerAPIDefinition.getControllerSupportedCommands());
   private final HighLevelControlManagerFactory managerFactory = new HighLevelControlManagerFactory(statusOutputManager, registry);
   private final List<ContactablePlaneBody> contactableBodies = new ArrayList<>();

   private final SideDependentList<YoEnum<ConstraintType>> footStates = new SideDependentList<>();

   private SimulationConstructionSet scs;
   private SideDependentList<TestFootSwitch> updatableFootSwitches;
   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidReferenceFrames referenceFrames;
   private PerfectSimulatedOutputWriter writer;
   private OneDoFJoint[] oneDoFJoints;

   private WalkingHighLevelHumanoidController walkingController;
   private WholeBodyControllerCore controllerCore;
   private LowLevelOneDoFJointDesiredDataHolderList controllerOutput;

   @ContinuousIntegrationTest(estimatedDuration = 5.0)
   @Test
   public void testForGarbage() throws MalformedObjectNameException
   {
      walkingController.doTransitionIntoAction();

      // warm up and calibrate the allocation measurement
      MBeanServer mBeanServer = ManagementFactory.getPlatformMBeanServer();
      ObjectName name = new ObjectName(ManagementFactory.THREAD_MXBEAN_NAME);
      for (int i = 0; i < 1000; i++)
      {
         threadAllocatedBytes(mBeanServer, name);
      }
      long callibrate = threadAllocatedBytes(mBeanServer, name);
      long bytesUsedToMeasure = threadAllocatedBytes(mBeanServer, name) - callibrate;

      // measure multiple ticks
      PrintTools.info("Starting to loop.");
      while (yoTime.getDoubleValue() < totalTime || profile)
      {
         long allocatedBytesBefore = threadAllocatedBytes(mBeanServer, name);
         doSingleTimeUpdate();
         long allocatedBytesAfter = threadAllocatedBytes(mBeanServer, name);
         allocatedInTick.set(allocatedBytesAfter - allocatedBytesBefore - bytesUsedToMeasure);

         if (showSCS)
         {
            writer.updateRobotConfigurationBasedOnFullRobotModel();
            scs.setTime(yoTime.getDoubleValue());
            scs.tickAndUpdate();
         }
      }
   }

   private void doSingleTimeUpdate()
   {
      // (1) compute foot switches based on sole height
      for (RobotSide side : RobotSide.values)
      {
         updatableFootSwitches.get(side).update();
      }

      // (2) do control and compute desired accelerations
      walkingController.doAction();
      ControllerCoreCommand coreCommand = walkingController.getControllerCoreCommand();
      controllerCore.submitControllerCoreCommand(coreCommand);
      controllerCore.compute();

      // (3) integrate accelerations in full robot model
      integrate();

      // update viz and advance time
      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      yoTime.add(robotModel.getControllerDT());
   }

   private final String[] SIGNATURE = new String[]{long.class.getName()};
   private final Object[] PARAMS = new Object[]{Thread.currentThread().getId()};
   /**
    * See http://www.rationaljava.com/2015/07/measuring-allocations-programmatically.html
    */
   private long threadAllocatedBytes(MBeanServer mBeanServer, ObjectName name)
   {
      try
      {
         return (long) mBeanServer.invoke(name, "getThreadAllocatedBytes", PARAMS, SIGNATURE);
      }
      catch (Exception e)
      {
         throw new IllegalArgumentException(e);
      }
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

   private final FramePoint3D solePosition = new FramePoint3D();

   private void integrate()
   {
      // fix one foot to the ground:
      RobotSide robotSide = finSideInFullSupport(footStates);
      ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
      solePosition.setToZero(soleFrame);
      solePosition.changeFrame(ReferenceFrame.getWorldFrame());
      double zCorrection = solePosition.getZ();

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         LowLevelJointDataReadOnly jointDesireds = controllerOutput.getLowLevelJointData(joint);

         if (jointDesireds.hasControlMode())
         {
            double q = joint.getQ();
            double qd = joint.getQd();
            double qdd = jointDesireds.getDesiredAcceleration();

            double qNew = q + controlDT * qd + 0.5 * controlDT * controlDT * qdd;
            double qdNew = qd + controlDT * qdd;

            joint.setQ(qNew);
            joint.setQd(qdNew);
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

      newPosition.addZ(-zCorrection);

      rootJoint.setRotation(newOrientation);
      rootJoint.setPosition(newPosition);
      rootJoint.updateFramesRecursively();
      frameLinearVelocity.setIncludingFrame(ReferenceFrame.getWorldFrame(), newLinearVelocity);
      frameAngularVelocity.setIncludingFrame(ReferenceFrame.getWorldFrame(), newAngularVelocity);
      frameLinearVelocity.changeFrame(rootJoint.getFrameAfterJoint());
      frameAngularVelocity.changeFrame(rootJoint.getFrameAfterJoint());
      rootJointTwist.set(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint(), frameLinearVelocity,
                         frameAngularVelocity);
      rootJoint.setJointTwist(rootJointTwist);
   }

   private void createControllerCore()
   {
      InverseDynamicsJoint[] jointsToIgnore = DRCControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());
      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, rootJoint, jointsToOptimizeFor, centerOfMassFrame,
                                                                            momentumOptimizationSettings, yoGraphicsListRegistry, registry);
      toolbox.setupForInverseDynamicsSolver(contactableBodies);

      JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      toolbox.setJointPrivilegedConfigurationParameters(jointPrivilegedConfigurationParameters);

      FeedbackControlCommandList template = managerFactory.createFeedbackControlTemplate();
      controllerOutput = new LowLevelOneDoFJointDesiredDataHolderList(fullRobotModel.getControllableOneDoFJoints());
      controllerCore = new WholeBodyControllerCore(toolbox, template, controllerOutput, registry);
   }

   private void createWalkingControllerAndSetUpManagerFactory()
   {
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, referenceFrames);
      FrameMessageCommandConverter commandConversionHelper = new FrameMessageCommandConverter(referenceFrameHashCodeResolver);
      commandInputManager.registerConversionHelper(commandConversionHelper);

      double omega0 = walkingControllerParameters.getOmega0();

      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();
      SideDependentList<ContactableFoot> feet = contactableBodiesFactory.createFootContactableBodies(fullRobotModel, referenceFrames);
      SideDependentList<String> footNames = new SideDependentList<>();
      List<ContactablePlaneBody> addidionalContacts = contactableBodiesFactory.createAdditionalContactPoints(fullRobotModel);
      for (RobotSide robotSide : RobotSide.values)
      {
         contactableBodies.add(feet.get(robotSide));
         footNames.put(robotSide, feet.get(robotSide).getName());
      }
      contactableBodies.addAll(addidionalContacts);

      double totalRobotWeight = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator()) * gravityZ;
      updatableFootSwitches = TestFootSwitch.createFootSwitches(feet, totalRobotWeight);
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<>(updatableFootSwitches);

      HighLevelHumanoidControllerToolbox controllerToolbox = new HighLevelHumanoidControllerToolbox(fullRobotModel, referenceFrames, footSwitches, null, null,
                                                                                                    yoTime, gravityZ, omega0, feet, controlDT, null,
                                                                                                    contactableBodies, yoGraphicsListRegistry);

      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultInitialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      WalkingMessageHandler walkingMessageHandler = new WalkingMessageHandler(defaultTransferTime, defaultSwingTime, defaultInitialTransferTime, feet,
                                                                              statusOutputManager, yoTime, yoGraphicsListRegistry, registry);
      controllerToolbox.setWalkingMessageHandler(walkingMessageHandler);

      managerFactory.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      managerFactory.setWalkingControllerParameters(walkingControllerParameters);
      managerFactory.setCapturePointPlannerParameters(capturePointPlannerParameters);

      walkingController = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
                                                                 capturePointPlannerParameters, controllerToolbox);
   }

   private static RobotSide finSideInFullSupport(SideDependentList<YoEnum<ConstraintType>> footStates)
   {
      for (RobotSide side : RobotSide.values)
      {
         if (footStates.get(side).getEnumValue() == ConstraintType.FULL)
         {
            return side;
         }
      }
      throw new RuntimeException("One foot needs to be in full support at all times for this test.");
   }

   @SuppressWarnings("unchecked")
   @Before
   public void setupTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      oneDoFJoints = fullRobotModel.getOneDoFJoints();

      setupRobotAndCopyConfiguration(robot);
      createWalkingControllerAndSetUpManagerFactory();
      createControllerCore();

      registry.addChild(walkingController.getYoVariableRegistry());
      walkingController.setControllerCoreOutput(controllerCore.getOutputForHighLevelController());

      for (RobotSide robotSide : RobotSide.values)
      {
         String name = robotSide.getLowerCaseName() + "FootAssumeCopOnEdge";
         YoBoolean variable = (YoBoolean) registry.getVariable(name);
         variable.set(true);

         name = robotSide.getLowerCaseName() + "FootAssumeFootBarelyLoaded";
         variable = (YoBoolean) registry.getVariable(name);
         variable.set(true);

         name = robotSide.getCamelCaseNameForStartOfExpression() + "FootState";
         YoEnum<ConstraintType> footState = (YoEnum<ConstraintType>) registry.getVariable(name);
         footStates.put(robotSide, footState);
      }

      if (showSCS)
      {
         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         parameters.setCreateGUI(true);
         scs = new SimulationConstructionSet(robot);
         scs.setDT(robotModel.getControllerDT(), 1);
         SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
         plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
         plotterFactory.createOverheadPlotter();
         scs.addYoVariableRegistry(registry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setTime(0.0);
         scs.tickAndUpdate();
      }
   }

   private void setupRobotAndCopyConfiguration(HumanoidFloatingRootJointRobot robot)
   {
      robot.setDynamic(false);
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      initialSetup.initializeRobot(robot, robotModel.getJointMap());

      writer = new PerfectSimulatedOutputWriter(robot, fullRobotModel);

      for (OneDoFJoint revoluteJoint : fullRobotModel.getOneDoFJoints())
      {
         String name = revoluteJoint.getName();
         OneDegreeOfFreedomJoint oneDoFJoint = robot.getOneDegreeOfFreedomJoint(name);
         revoluteJoint.setQ(oneDoFJoint.getQ());
         oneDoFJoint.setQd(0.0);
         revoluteJoint.setQd(0.0);
      }

      FloatingJoint floatingJoint = robot.getRootJoint();
      FloatingInverseDynamicsJoint sixDoFJoint = fullRobotModel.getRootJoint();
      RigidBodyTransform transform = new RigidBodyTransform();
      floatingJoint.getTransformToWorld(transform);
      sixDoFJoint.setPositionAndRotation(transform);

      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();
   }

   @After
   public void tearDown()
   {
      if (showSCS)
      {
         scs.setIndex(1);
         scs.setInPoint();
         scs.cropBuffer();
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
