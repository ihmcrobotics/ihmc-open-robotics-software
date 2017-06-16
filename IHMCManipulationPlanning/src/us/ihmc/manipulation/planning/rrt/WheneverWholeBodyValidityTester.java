package us.ihmc.manipulation.planning.rrt;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class WheneverWholeBodyValidityTester
{
   private WheneverWholeBodyFunctions wholebodyFunctions = new WheneverWholeBodyFunctions();
   
   private static final int numberOfTicksToSendSolution = 10;
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double updateDT = 1.0e-3;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private static final String centerOfMassName = "CenterOfMass";
   
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
   
   private final WholeBodyControllerCore controllerCore;
   private final CommonHumanoidReferenceFrames referenceFrames;
   
   private final KinematicsToolboxOutputStatus inverseKinematicsSolution;
   
   private final YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("genericGains", registry);
   
   private FullHumanoidRobotModel desiredFullRobotModel;
   private final RigidBody rootBody;
   private final FloatingInverseDynamicsJoint rootJoint;
   private final OneDoFJoint[] oneDoFJoints;
   
   private final DoubleYoVariable solutionQuality = new DoubleYoVariable("solutionQuality", registry);
   
   private final FeedbackControllerDataReadOnly feedbackControllerDataHolder;
   
   private final CommandInputManager commandInputManager;
   
   private final List<RigidBody> listOfControllableRigidBodies = new ArrayList<>();
   private final EnumMap<LegJointName, DoubleYoVariable> legJointLimitReductionFactors = new EnumMap<>(LegJointName.class);
   private final BooleanYoVariable holdSupportFootPose = new BooleanYoVariable("holdSupportFootPose", registry);
   private final BooleanYoVariable holdCenterOfMassXYPosition = new BooleanYoVariable("holdCenterOfMassXYPosition", registry);
   private final YoFramePoint initialCenterOfMassPosition = new YoFramePoint("initialCenterOfMass", worldFrame, registry);
   private final SideDependentList<YoFramePoseUsingQuaternions> initialFootPoses = new SideDependentList<>();
   private final DoubleYoVariable privilegedWeight = new DoubleYoVariable("privilegedWeight", registry);
   private final DoubleYoVariable privilegedConfigurationGain = new DoubleYoVariable("privilegedConfigurationGain", registry);
   private final DoubleYoVariable privilegedMaxVelocity = new DoubleYoVariable("privilegedMaxVelocity", registry);
   
   private final IntegerYoVariable numberOfActiveCommands = new IntegerYoVariable("numberOfActiveCommands", registry);
   
   private final SideDependentList<BooleanYoVariable> isFootInSupport = new SideDependentList<>();
   
   private final AtomicReference<PrivilegedConfigurationCommand> privilegedConfigurationCommandReference = new AtomicReference<>(null);
   
   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);
   private final Map<String, FeedbackControlCommand<?>> userFeedbackCommands = new HashMap<>();
   
   private final DoubleYoVariable momentumWeight = new DoubleYoVariable("momentumWeight", registry);
      
   private final Map<Long, OneDoFJoint> jointNameBasedHashCodeMap = new HashMap<>();
   
   public WheneverWholeBodyValidityTester(FullHumanoidRobotModel desiredFullRobotModel)
   {
      this.desiredFullRobotModel = desiredFullRobotModel;

      referenceFrames = new HumanoidReferenceFrames(desiredFullRobotModel);
      rootBody = desiredFullRobotModel.getElevator();
      rootJoint = desiredFullRobotModel.getRootJoint();
      
      populateJointLimitReductionFactors();
      populateListOfControllableRigidBodies();
      
      controllerCore = createControllerCore();
      feedbackControllerDataHolder = controllerCore.getWholeBodyFeedbackControllerDataHolder();
      
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(desiredFullRobotModel);
      Arrays.stream(oneDoFJoints).forEach(joint -> jointNameBasedHashCodeMap.put(joint.getNameBasedHashCode(), joint));
      
      inverseKinematicsSolution = new KinematicsToolboxOutputStatus(oneDoFJoints);
      inverseKinematicsSolution.setDestination(-1);

      commandInputManager = new CommandInputManager(name, createListOfSupportedCommands());
      
      gains.setProportionalGain(800.0); // Gains used for everything. It is as high as possible to reduce the convergence time.
      
      for (RobotSide robotSide : RobotSide.values)
      {
         String side = robotSide.getCamelCaseNameForMiddleOfExpression();
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         isFootInSupport.put(robotSide, new BooleanYoVariable("is" + side + "FootInSupport", registry));
         initialFootPoses.put(robotSide, new YoFramePoseUsingQuaternions(sidePrefix + "FootInitial", worldFrame, registry));
      }
   }
   
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(KinematicsToolboxCenterOfMassCommand.class);
      commands.add(KinematicsToolboxRigidBodyCommand.class);
      commands.add(KinematicsToolboxConfigurationCommand.class);
      return commands;
   }
   
   private void populateJointLimitReductionFactors()
   {
      DoubleYoVariable hipReductionFactor = new DoubleYoVariable("hipLimitReductionFactor", registry);
      DoubleYoVariable kneeReductionFactor = new DoubleYoVariable("kneeLimitReductionFactor", registry);
      DoubleYoVariable ankleReductionFactor = new DoubleYoVariable("ankleLimitReductionFactor", registry);
      hipReductionFactor.set(0.05);

      legJointLimitReductionFactors.put(LegJointName.HIP_PITCH, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_ROLL, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.HIP_YAW, hipReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.KNEE_PITCH, kneeReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_PITCH, ankleReductionFactor);
      legJointLimitReductionFactors.put(LegJointName.ANKLE_ROLL, ankleReductionFactor);
   }
   
   private void populateListOfControllableRigidBodies()
   {
      listOfControllableRigidBodies.add(desiredFullRobotModel.getChest());
      listOfControllableRigidBodies.add(desiredFullRobotModel.getPelvis());

      for (RobotSide robotSide : RobotSide.values)
      {
         listOfControllableRigidBodies.add(desiredFullRobotModel.getHand(robotSide));
         listOfControllableRigidBodies.add(desiredFullRobotModel.getFoot(robotSide));
      }
   }
   
   private WholeBodyControllerCore createControllerCore()
   {
      InverseDynamicsJoint[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(desiredFullRobotModel);
      ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(updateDT, 0.0, rootJoint, controlledJoints, centerOfMassFrame, null,
                                                                            null, new YoVariableRegistry("WheneverWholeBodyTester"));
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());
      toolbox.setupForInverseKinematicsSolver();
      FeedbackControlCommandList controllerCoreTemplate = createControllerCoreTemplate();
      controllerCoreTemplate.addCommand(new CenterOfMassFeedbackControlCommand());
      return new WholeBodyControllerCore(toolbox, controllerCoreTemplate, registry);
   }
   
   private FeedbackControlCommandList createControllerCoreTemplate()
   {
      FeedbackControlCommandList template = new FeedbackControlCommandList();
      listOfControllableRigidBodies.stream().map(this::createFeedbackControlCommand).forEach(template::addCommand);
      return template;
   }
   
   private SpatialFeedbackControlCommand createFeedbackControlCommand(RigidBody endEffector)
   {
      SpatialFeedbackControlCommand command = new SpatialFeedbackControlCommand();
      command.set(rootBody, endEffector);
      return command;
   }
   
   public boolean initialize()
   {
      PrintTools.info("initialize phase Start");
      
      userFeedbackCommands.clear();

      RobotConfigurationData robotConfigurationData = latestRobotConfigurationDataReference.get();

      if (robotConfigurationData == null)
         return false;

      // Initializes this desired robot to the most recent robot configuration data received from the walking controller.
      wholebodyFunctions.setRobotStateFromRobotConfigurationData(robotConfigurationData, rootJoint, oneDoFJoints);
      
      RigidBodyTransform rooJointTransformToWorldFrame = rootJoint.getFrameBeforeJoint().getTransformToWorldFrame();

      for(int i=0;i<oneDoFJoints.length;i++)
      {
         PrintTools.info("oneDoFJoints[i].getQ() "+oneDoFJoints[i].getName()+" "+oneDoFJoints[i].getQ());
      }
      // Initialize the initialCenterOfMassPosition and initialFootPoses to match the current state of the robot.
      updateCoMPositionAndFootPoses();
    
      // By default, always hold the support foot/feet and center of mass in place. This can be changed on the fly by sending a KinematicsToolboxConfigurationMessage.
      holdSupportFootPose.set(true);
      holdCenterOfMassXYPosition.set(true);

      // Sets the privileged configuration to match the current robot configuration such that the solution will be as close as possible to the current robot configuration.
      snapPrivilegedConfigurationToCurrent();
      PrintTools.info("initialize phase END");
      
      return true;
   }
   
   public void updateInternal()
   {
      // Updating the reference frames and twist calculator.
      updateTools();

      // Compiling all the commands to be submitted to the controller core.
      controllerCoreCommand.clear();
      controllerCoreCommand.addFeedbackControlCommand(createHoldCenterOfMassXYCommand());
      controllerCoreCommand.addFeedbackControlCommand(createHoldSupportFootCommands());
      
      FeedbackControlCommandList userCommands = new FeedbackControlCommandList();      
      updateUserFeedbackCommands();               
      userFeedbackCommands.values().forEach(userCommands::addCommand);
      controllerCoreCommand.addFeedbackControlCommand(userCommands);

      controllerCoreCommand.addInverseKinematicsCommand(createJointLimitReductionCommand());
      controllerCoreCommand.addInverseKinematicsCommand(privilegedConfigurationCommandReference.getAndSet(null));

      // Save all commands used for this control tick for computing the solution quality.
      FeedbackControlCommandList allFeedbackControlCommands = new FeedbackControlCommandList(controllerCoreCommand.getFeedbackControlCommandList());

      /*
       * Submitting and requesting the controller core to run the feedback controllers, formulate
       * and solve the optimization problem for this control tick.
       */
      controllerCore.reset();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      // Calculating the solution quality based on sum of all the commands' tracking error.
      solutionQuality.set(wholebodyFunctions.calculateSolutionQuality(allFeedbackControlCommands, feedbackControllerDataHolder));
      PrintTools.info("solutionQuality "+ solutionQuality.getDoubleValue());
      
      // Updating the the robot state from the current solution, initializing the next control tick.
      wholebodyFunctions.setRobotStateFromControllerCoreOutput(controllerCore.getControllerCoreOutput(), rootJoint, oneDoFJoints);
      
      inverseKinematicsSolution.setDesiredJointState(rootJoint, oneDoFJoints);
      inverseKinematicsSolution.setSolutionQuality(solutionQuality.getDoubleValue());
      
   }
   
   public void updateTools()
   {
      desiredFullRobotModel.updateFrames();
      referenceFrames.updateFrames();
   }
   
   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
   }
   
   private JointLimitReductionCommand createJointLimitReductionCommand()
   {
      JointLimitReductionCommand jointLimitReductionCommand = new JointLimitReductionCommand();
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : desiredFullRobotModel.getRobotSpecificJointNames().getLegJointNames())
         {
            OneDoFJoint joint = desiredFullRobotModel.getLegJoint(robotSide, legJointName);
            double reductionFactor = legJointLimitReductionFactors.get(legJointName).getDoubleValue();
            jointLimitReductionCommand.addReductionFactor(joint, reductionFactor);
         }
      }
      return jointLimitReductionCommand;
   }
   
   private void updateCoMPositionAndFootPoses()
   {
      updateTools();

      initialCenterOfMassPosition.setFromReferenceFrame(referenceFrames.getCenterOfMassFrame());
      
      for (RobotSide robotSide : RobotSide.values)
      {         
         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);
         initialFootPoses.get(robotSide).setFromReferenceFrame(foot.getBodyFixedFrame());
      }
   }
   
   private void snapPrivilegedConfigurationToCurrent()
   {
      PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_CURRENT);
      privilegedConfigurationCommand.setDefaultWeight(privilegedWeight.getDoubleValue());
      privilegedConfigurationCommand.setDefaultConfigurationGain(privilegedConfigurationGain.getDoubleValue());
      privilegedConfigurationCommand.setDefaultMaxVelocity(privilegedMaxVelocity.getDoubleValue());
      privilegedConfigurationCommandReference.set(privilegedConfigurationCommand);
   }
   
   private FeedbackControlCommand<?> createHoldCenterOfMassXYCommand()
   {
      if (!holdCenterOfMassXYPosition.getBooleanValue())
         return null;

      // Do not hold the CoM position if the user is already controlling it.
      if (userFeedbackCommands.containsKey(centerOfMassName))
      {
         holdCenterOfMassXYPosition.set(false);
         return null;
      }

      FramePoint positionToHold = new FramePoint();
      initialCenterOfMassPosition.getFrameTupleIncludingFrame(positionToHold);

      CenterOfMassFeedbackControlCommand feedbackControlCommand = new CenterOfMassFeedbackControlCommand();
      feedbackControlCommand.setGains(gains);
      feedbackControlCommand.setWeightForSolver(momentumWeight.getDoubleValue());
      feedbackControlCommand.setSelectionMatrixForLinearXYControl();
      feedbackControlCommand.set(positionToHold);
      return feedbackControlCommand;
   }
   
   private FeedbackControlCommand<?> createHoldSupportFootCommands()
   {
      if (!holdSupportFootPose.getBooleanValue())
         return null;

      FeedbackControlCommandList inputs = new FeedbackControlCommandList();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!isFootInSupport.get(robotSide).getBooleanValue())
            continue;

         RigidBody foot = desiredFullRobotModel.getFoot(robotSide);

         // Do not hold the foot position if the user is already controlling it.
         if (userFeedbackCommands.containsKey(foot.getName()))
            continue;

         FramePose poseToHold = new FramePose();
         initialFootPoses.get(robotSide).getFramePoseIncludingFrame(poseToHold);

         SpatialFeedbackControlCommand feedbackControlCommand = new SpatialFeedbackControlCommand();
         feedbackControlCommand.set(rootBody, foot);
         feedbackControlCommand.setGains((SE3PIDGainsInterface) gains);
//         feedbackControlCommand.setWeightForSolver(footWeight.getDoubleValue());
         feedbackControlCommand.set(poseToHold);
         inputs.addCommand(feedbackControlCommand);
      }
      return inputs;
   }
   
   public void updateUserFeedbackCommands()
   {
//      List<KinematicsToolboxRigidBodyCommand> commands = commandInputManager.pollNewCommands(KinematicsToolboxRigidBodyCommand.class);
//      for (int i = 0; i < commands.size(); i++)
//      {
//         SpatialFeedbackControlCommand rigidBodyCommand = wholebodyFunctions.consumeRigidBodyCommand(commands.get(i), rootBody, gains);
//         String endEffectorName = rigidBodyCommand.getEndEffector().getName();
//         userFeedbackCommands.put(endEffectorName, rigidBodyCommand);
//      }
      
      
      
   }
   
}
