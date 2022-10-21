package us.ihmc.valkyrieRosControl;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.JointGainsHandle;
import us.ihmc.rosControl.wholeRobot.PositionJointHandle;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ValkyrieJointGainsTestController extends IHMCWholeRobotControlJavaBridge
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

   private final JointGainsHandle[] jointGainsHandles = new JointGainsHandle[jointNames.length];
   private final PositionJointHandle[] positionJointHandles = new PositionJointHandle[jointNames.length];

   private final YoDouble desiredJointStiffness = new YoDouble("desiredJointStiffness", registry);
   private final YoDouble desiredJointDamping = new YoDouble("desiredJointDamping", registry);

   public ValkyrieJointGainsTestController()
   {
   }

   @Override
   protected void init()
   {
      for (int i = 0; i < jointNames.length; i++)
      {
         jointGainsHandles[i] = createJointGainsHandle(jointNames[i]);
         positionJointHandles[i] = createPositionJointHandle(jointNames[i]);
      }

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.ARM_MASS_SIM);
      LogModelProvider logModelProvider = robotModel.getLogModelProvider();
      DataServerSettings logSettings = robotModel.getLogSettings();double estimatorDT = robotModel.getEstimatorDT();

      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), logModelProvider, logSettings, estimatorDT);

      yoVariableServer.start();
   }

   @Override
   protected void doControl(long rosTime, long duration)
   {
      if (firstTick)
      {
         for (int i = 0; i < jointNames.length; i++)
         {
            positionJointHandles[i].setDesiredPosition(positionJointHandles[i].getPosition());
         }

         firstTick = false;
      }

      for (int i = 0; i < jointNames.length; i++)
      {
         jointGainsHandles[i].setStiffness(desiredJointStiffness.getDoubleValue());
         jointGainsHandles[i].setDamping(desiredJointDamping.getDoubleValue());
      }
   }
}
