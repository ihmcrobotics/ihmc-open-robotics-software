package us.ihmc.valkyrieRosControl.impedance;

import us.ihmc.avatar.drcRobot.RobotTarget;
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
   private static final String[] allJoints;
   private static final String[] jointsToServo;

   static
   {
      List<String> jointList = new ArrayList<>();

      // Arms
      jointList.addAll(Arrays.asList("leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch"));
      jointList.addAll(Arrays.asList("rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch"));

      // Torso
      jointList.addAll(Arrays.asList("torsoYaw", "torsoPitch", "torsoRoll"));

      // Legs
      jointList.addAll(Arrays.asList("leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll"));
      jointList.addAll(Arrays.asList("rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"));

      allJoints = jointList.toArray(new String[0]);

      jointsToServo = Arrays.asList("leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch").toArray(new String[0]);
   }

   private boolean firstTick = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final TimestampProvider monotonicTimeProvider = RealtimeThread::getCurrentMonotonicClockTime;
   private FullHumanoidRobotModel fullRobotModel;

   private final EffortJointHandle[] effortHandles = new EffortJointHandle[allJoints.length];
   private final JointImpedanceHandle[] impedanceHandles = new JointImpedanceHandle[jointsToServo.length];

   private final YoDouble masterGain = new YoDouble("masterGain", registry);
   private final YoDouble desiredJointStiffness = new YoDouble("desiredJointStiffness", registry);
   private final YoDouble desiredJointDamping = new YoDouble("desiredJointDamping", registry);

   private final YoDouble[] desiredJointAngles = new YoDouble[jointsToServo.length];

   private YoVariableServer yoVariableServer;

   public ValkyrieImpedanceTestController()
   {
      for (int i = 0; i < desiredJointAngles.length; i++)
      {
         desiredJointAngles[i] = new YoDouble("qDesired_" + jointsToServo[i], registry);
      }

      desiredJointStiffness.set(40.0);
      desiredJointDamping.set(8.0);
   }

   @Override
   protected void init()
   {
      for (int i = 0; i < allJoints.length; i++)
      {
         effortHandles[i] = createEffortJointHandle(allJoints[i]);
      }

      for (int i = 0; i < jointsToServo.length; i++)
      {
         impedanceHandles[i] = createJointImpedanceHandle(jointsToServo[i]);
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
         for (int i = 0; i < desiredJointAngles.length; i++)
         {
            String jointName = jointsToServo[i];
            for (int j = 0; j < allJoints.length; j++)
            {
               if (allJoints[j].equals(jointName))
               {
                  desiredJointAngles[i].set(effortHandles[j].getPosition());
                  break;
               }
            }
         }

         firstTick = false;
      }

      for (int i = 0; i < allJoints.length; i++)
      {
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(allJoints[i]);
         joint.setQ(effortHandles[i].getPosition());
         joint.setQd(effortHandles[i].getVelocity());
      }

      fullRobotModel.getRootBody().updateFramesRecursively();

      for (int i = 0; i < jointsToServo.length; i++)
      {
         impedanceHandles[i].setStiffness(masterGain.getDoubleValue() * desiredJointStiffness.getDoubleValue());
         impedanceHandles[i].setDamping(masterGain.getDoubleValue() * desiredJointDamping.getDoubleValue());

         impedanceHandles[i].setPosition(desiredJointAngles[i].getDoubleValue());
         impedanceHandles[i].setVelocity(0.0);
      }

      yoVariableServer.update(monotonicTimeProvider.getTimestamp(), registry);
   }
}
