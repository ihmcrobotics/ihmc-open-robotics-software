package us.ihmc.valkyrieRosControl;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.JointImpedanceHandle;
import us.ihmc.rosControl.wholeRobot.PositionJointHandle;
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
   private static final double[] desiredJointAngles = new double[]{0.4, -1.0, 0.1, -1.0};

   static
   {
      List<String> jointList = new ArrayList<>();
      jointList.addAll(Arrays.asList("leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch"));
      jointNames = jointList.toArray(new String[0]);
   }

   private boolean firstTick = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final TimestampProvider monotonicTimeProvider = RealtimeThread::getCurrentMonotonicClockTime;

   private final JointImpedanceHandle[] jointImpedanceHandles = new JointImpedanceHandle[jointNames.length];

   private final YoDouble desiredJointStiffness = new YoDouble("desiredJointStiffness", registry);
   private final YoDouble desiredJointDamping = new YoDouble("desiredJointDamping", registry);
   private YoVariableServer yoVariableServer;

   public ValkyrieImpedanceTestController()
   {
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
      DataServerSettings logSettings = robotModel.getLogSettings();double estimatorDT = robotModel.getEstimatorDT();

      yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);
      yoVariableServer.start();
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      if (firstTick)
      {
         firstTick = false;
      }

      for (int i = 0; i < jointNames.length; i++)
      {
         jointImpedanceHandles[i].setStiffness(desiredJointStiffness.getDoubleValue());
         jointImpedanceHandles[i].setDamping(desiredJointDamping.getDoubleValue());

         jointImpedanceHandles[i].setPosition(desiredJointAngles[i]);
         jointImpedanceHandles[i].setVelocity(0.0);
      }

      yoVariableServer.update(monotonicTimeProvider.getTimestamp());
   }
}
