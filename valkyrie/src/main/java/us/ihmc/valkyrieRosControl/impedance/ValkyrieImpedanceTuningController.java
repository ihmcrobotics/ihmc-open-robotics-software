package us.ihmc.valkyrieRosControl.impedance;

import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.Conversions;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.JointImpedanceHandle;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieStandPrepSetpoints;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.HashMap;
import java.util.Map;

import static us.ihmc.valkyrieRosControl.impedance.ValkyrieJointList.allJoints;
import static us.ihmc.valkyrieRosControl.impedance.ValkyrieJointList.impedanceJoints;

public class ValkyrieImpedanceTuningController extends IHMCWholeRobotControlJavaBridge
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.ARM_MASS_SIM);
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final TimestampProvider monotonicTimeProvider = RealtimeThread::getCurrentMonotonicClockTime;
   private final YoDouble yoTime = new YoDouble("yoTime", registry);
   private final YoDouble masterGain = new YoDouble("masterGain", registry);

   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final WholeBodySetpointParameters jointHome;
   private final Map<String, AlphaFilteredYoVariable> nameToFilteredJointVelocity = new HashMap<>();
   private final ValkyrieImpedanceOutputWriter outputWriter;

   private final EffortJointHandle[] effortHandles = new EffortJointHandle[allJoints.size()];
   private final JointImpedanceHandle[] impedanceHandles = new JointImpedanceHandle[impedanceJoints.size()];

   private final Map<String, EffortJointHandle> nameToEffortHandleMap = new HashMap<>();
   private final Map<String, JointImpedanceHandle> nameToImpedanceHandleMap = new HashMap<>();

   private final YoEnum<ValkyrieJointList.ValkyrieImpedanceJoint> requestedTuningJoint = new YoEnum<>("requestedTuningJoint", registry, ValkyrieJointList.ValkyrieImpedanceJoint.class, true);
   private ValkyrieJointList.ValkyrieImpedanceJoint jointToTune = null;
   private final YoFunctionGeneratorNew[] functionGenerators;

   private final YoLowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList;
   private YoVariableServer yoVariableServer;

   public ValkyrieImpedanceTuningController()
   {
      fullRobotModel = robotModel.createFullRobotModel();

      JointBasics[] jointsToIgnore = AvatarControllerThread.createListOfJointsToIgnore(fullRobotModel, robotModel, robotModel.getSensorInformation());
      JointBasics[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);
      jointHome = new ValkyrieStandPrepSetpoints(robotModel.getJointMap());
      jointDesiredOutputList = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);
      outputWriter = new ValkyrieImpedanceOutputWriter(fullRobotModel, jointDesiredOutputList, nameToEffortHandleMap, nameToImpedanceHandleMap);
      functionGenerators = new YoFunctionGeneratorNew[controlledJoints.length];

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = controlledOneDoFJoints[i];
         String jointName = joint.getName();

         double alphaVelocity = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(25.0, robotModel.getControllerDT());
         AlphaFilteredYoVariable filteredVelocity = new AlphaFilteredYoVariable("qd_filt_" + jointName, registry, alphaVelocity);
         nameToFilteredJointVelocity.put(jointName, filteredVelocity);

         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         jointDesiredOutput.setDesiredPosition(jointHome.getSetpoint(jointName));
         jointDesiredOutput.setDesiredVelocity(0.0);

         if (impedanceJoints.contains(jointName))
         {
            jointDesiredOutput.setMasterGain(0.1);
            jointDesiredOutput.setStiffness(200.0);
            jointDesiredOutput.setDamping(35.0);
         }
         else
         {
            jointDesiredOutput.setMasterGain(0.6);
            jointDesiredOutput.setStiffness(55.0);
            jointDesiredOutput.setDamping(5.0);
         }

         YoFunctionGeneratorNew functionGenerator = new YoFunctionGeneratorNew(jointName, robotModel.getControllerDT(), registry);
         functionGenerator.setMode(YoFunctionGeneratorMode.SINE);
         functionGenerators[i] = functionGenerator;
      }

      new DefaultParameterReader().readParametersInRegistry(registry);
   }

   @Override
   protected void init()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         effortHandles[i] = createEffortJointHandle(allJoints.get(i));
         nameToEffortHandleMap.put(allJoints.get(i), effortHandles[i]);
      }

      for (int i = 0; i < impedanceJoints.size(); i++)
      {
         impedanceHandles[i] = createJointImpedanceHandle(impedanceJoints.get(i));
         nameToImpedanceHandleMap.put(impedanceJoints.get(i), impedanceHandles[i]);
      }

      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();

      yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getRootBody(), graphicsListRegistry);
      yoVariableServer.start();
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      yoTime.set(Conversions.nanosecondsToSeconds(monotonicTimeProvider.getTimestamp()));

      /* Perform state estimation */
      read();

      /* Process incoming trajectories */
      doControl();

      /* Write to effort and impedance handles */
      outputWriter.write();

      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), registry);
   }

   private void read()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(allJoints.get(i));
         AlphaFilteredYoVariable filteredVelocity = nameToFilteredJointVelocity.get(allJoints.get(i));
         filteredVelocity.update(effortHandles[i].getVelocity());

         joint.setQ(effortHandles[i].getPosition());
         joint.setQd(filteredVelocity.getDoubleValue());
      }

      fullRobotModel.getRootBody().updateFramesRecursively();
   }

   private void doControl()
   {
      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         String jointName = controlledOneDoFJoints[i].getName();
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(controlledOneDoFJoints[i]);
         YoFunctionGeneratorNew functionGenerator = functionGenerators[i];

         functionGenerator.update();
         jointDesiredOutput.setDesiredPosition(jointHome.getSetpoint(jointName) + functionGenerator.getValue());
         jointDesiredOutput.setDesiredVelocity(functionGenerator.getValueDot());

         /* Keep one master gain for all joints, easier to turn off */
         jointDesiredOutput.setMasterGain(masterGain.getValue());
      }
   }
}
