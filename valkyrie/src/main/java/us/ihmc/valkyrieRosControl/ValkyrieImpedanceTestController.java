package us.ihmc.valkyrieRosControl;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.JointImpedanceHandle;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ValkyrieImpedanceTestController extends IHMCWholeRobotControlJavaBridge
{
   private static final String[] jointNames;

   static
   {
      List<String> jointList = new ArrayList<>();
      jointList.addAll(Arrays.asList("leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch"));
      jointNames = jointList.toArray(new String[0]);
   }

   private boolean firstTick = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final TimestampProvider monotonicTimeProvider = RealtimeThread::getCurrentMonotonicClockTime;
   private FullHumanoidRobotModel fullRobotModel;

   private final JointImpedanceHandle[] jointImpedanceHandles = new JointImpedanceHandle[jointNames.length];

   private final YoDouble masterGain = new YoDouble("masterGain", registry);
   private final YoDouble desiredJointStiffness = new YoDouble("desiredJointStiffness", registry);
   private final YoDouble desiredJointDamping = new YoDouble("desiredJointDamping", registry);

   private final YoDouble[] desiredJointAngles = new YoDouble[jointNames.length];

   private YoVariableServer yoVariableServer;

   public ValkyrieImpedanceTestController()
   {
      for (int i = 0; i < desiredJointAngles.length; i++)
      {
         desiredJointAngles[i] = new YoDouble("qDesired_" + jointNames[i], registry);
      }

      desiredJointStiffness.set(40.0);
      desiredJointDamping.set(8.0);
   }

   @Override
   protected void init()
   {
      for (int i = 0; i < jointNames.length; i++)
      {
         jointImpedanceHandles[i] = createJointImpedanceHandle(jointNames[i]);
      }

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.ARM_MASS_SIM);
      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();
      double estimatorDT = robotModel.getEstimatorDT();
      fullRobotModel = robotModel.createFullRobotModel();

      yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getRootBody(), graphicsListRegistry);
      yoVariableServer.start();
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      if (firstTick)
      {
         for (int i = 0; i < jointImpedanceHandles.length; i++)
         {
            desiredJointAngles[i].set(jointImpedanceHandles[i].getPosition());
         }

         firstTick = false;
      }

      for (int i = 0; i < jointNames.length; i++)
      {
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(jointNames[i]);
         joint.setQ(jointImpedanceHandles[i].getPosition());
         joint.setQd(jointImpedanceHandles[i].getVelocity());
      }

      fullRobotModel.getRootBody().updateFramesRecursively();

      for (int i = 0; i < jointNames.length; i++)
      {
         jointImpedanceHandles[i].setStiffness(masterGain.getDoubleValue() * desiredJointStiffness.getDoubleValue());
         jointImpedanceHandles[i].setDamping(masterGain.getDoubleValue() * desiredJointDamping.getDoubleValue());

         jointImpedanceHandles[i].setPosition(desiredJointAngles[i].getDoubleValue());
         jointImpedanceHandles[i].setVelocity(0.0);
      }

      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), registry);
   }
}
