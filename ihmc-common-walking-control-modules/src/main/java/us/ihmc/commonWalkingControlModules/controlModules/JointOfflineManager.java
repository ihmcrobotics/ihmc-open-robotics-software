package us.ihmc.commonWalkingControlModules.controlModules;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class JointOfflineManager
{
   private static final double JOINT_OFFLINE_WEIGHT = 100.0;
   private static final double UNLOAD_DURATION = 0.6;

   private final TIntObjectHashMap<OneDoFJointBasics> jointHashCodeMap = new TIntObjectHashMap<>();
   private final YoBoolean isJointOffline;
   private final YoInteger offlineJointHashCode;
   private final DoubleProvider yoTime;

   private OneDoFJointBasics offlineJoint;
   private final YoDouble jointOfflineWeight;
   private final YoDouble timeJointWentOffline;

   private final JointTorqueCommand jointTorqueCommand = new JointTorqueCommand();

   public JointOfflineManager(OneDoFJointBasics[] oneDoFJoints, DoubleProvider yoTime, YoRegistry registry)
   {
      isJointOffline = new YoBoolean("isJointOffline", registry);
      offlineJointHashCode = new YoInteger("offlineJointHashCode", registry);
      jointOfflineWeight = new YoDouble("jointOfflineWeight", registry);
      timeJointWentOffline = new YoDouble("timeJointWentOffline", registry);

      this.yoTime = yoTime;

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointHashCodeMap.put(oneDoFJoints[i].hashCode(), oneDoFJoints[i]);
      }
   }

   public void setJointOffline(int offlineJointHashCode)
   {
      if (offlineJoint != null && offlineJoint.hashCode() != offlineJointHashCode)
      {
         throw new RuntimeException("Only one joint can be offline at a time.");
      }

      OneDoFJointBasics joint = jointHashCodeMap.get(offlineJointHashCode);
      if (joint == null)
      {
         throw new RuntimeException("Cannot find joint with hash-code: " + offlineJointHashCode);
      }

      offlineJoint = joint;
      isJointOffline.set(true);
      this.offlineJointHashCode.set(offlineJointHashCode);
      timeJointWentOffline.set(yoTime.getValue());

      jointTorqueCommand.clear();
      jointTorqueCommand.addJoint(offlineJoint, 0.0);

      offlineJoint = joint;
   }

   public boolean isJointOffline()
   {
      return offlineJoint != null;
   }

   public OneDoFJointBasics getOfflineJoint()
   {
      return offlineJoint;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      double alphaUnload = EuclidCoreTools.clamp((yoTime.getValue() - timeJointWentOffline.getValue()) / UNLOAD_DURATION, 0.0, 1.0);
      jointOfflineWeight.set(MathTools.square(alphaUnload) * JOINT_OFFLINE_WEIGHT);
      jointTorqueCommand.setWeight(jointOfflineWeight.getDoubleValue());

      return jointTorqueCommand;
   }

}
