package us.ihmc.valkyrieRosControl;

import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.rosControl.JointHandle;
import us.ihmc.rosControl.valkyrie.IHMCValkyrieControlJavaBridge;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieRosControlSliderBoard extends IHMCValkyrieControlJavaBridge
{

   private static final String[] controlledJoints = {"torsoYaw", "torsoPitch", "torsoRoll"};

//   private static final String[] controlledJoints = { "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
//       "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "torsoYaw", "torsoPitch", "torsoRoll",
//       "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch", "lowerNeckPitch",
//       "neckYaw", "upperNeckPitch", "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll",
//       "rightWristPitch" };
   
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, true);
   private final SDFFullHumanoidRobotModel sdfFullRobotModel = robotModel.createFullRobotModel();

   private final ArrayList<JointHolder> jointHolders = new ArrayList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName()),
         robotModel.getLogModelProvider(), LogSettings.SIMULATION, 0.001);

   private EnumYoVariable<?> selectedJoint;
   private final DoubleYoVariable qDesiredSelected = new DoubleYoVariable("qDesiredSelected", registry);
   private final DoubleYoVariable qdDesiredSelected = new DoubleYoVariable("qdDesiredSelected", registry);

   private final DoubleYoVariable kpSelected = new DoubleYoVariable("kpSelected", registry);
   private final DoubleYoVariable kdSelected = new DoubleYoVariable("kdSelected", registry);

   private final DoubleYoVariable qSelected = new DoubleYoVariable("qSelected", registry);
   private final DoubleYoVariable qdSelected = new DoubleYoVariable("qdSelected", registry);

   @Override
   protected void init()
   {
      ArrayList<String> jointNames = new ArrayList<>();
      for (String jointName : controlledJoints)
      {
         OneDoFJoint joint = sdfFullRobotModel.getOneDoFJointByName(jointName);
         JointHandle handle = createJointHandle(jointName);
         jointHolders.add(new JointHolder(joint, handle, registry));
         jointNames.add(joint.getName());
      }

      selectedJoint = new EnumYoVariable("selectedJoint", "", registry, false, jointNames.toArray(new String[jointNames.size()]));
      System.out.println(Arrays.toString(selectedJoint.getEnumValuesAsString()));

      selectedJoint.addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            JointHolder selected = jointHolders.get(selectedJoint.getOrdinal());
            qDesiredSelected.set(selected.q_d.getDoubleValue());
            qdDesiredSelected.set(selected.qd_d.getDoubleValue());

            kpSelected.set(selected.pdController.getProportionalGain());
            kdSelected.set(selected.pdController.getDerivativeGain());

         }
      });

      yoVariableServer.setMainRegistry(registry, sdfFullRobotModel, null);
      yoVariableServer.start();
   }

   @Override
   protected void doControl(long time, long duration)
   {
      JointHolder selected = jointHolders.get(selectedJoint.getOrdinal());
      selected.q_d.set(MathTools.clipToMinMax(qDesiredSelected.getDoubleValue(), selected.joint.getJointLimitLower(), selected.joint.getJointLimitUpper()));
      selected.qd_d.set(qdDesiredSelected.getDoubleValue());
      selected.pdController.setProportionalGain(kpSelected.getDoubleValue());
      selected.pdController.setDerivativeGain(kdSelected.getDoubleValue());

      for (int i = 0; i < jointHolders.size(); i++)
      {
         JointHolder holder = jointHolders.get(i);
         holder.update();
      }

      qSelected.set(selected.q.getDoubleValue());
      qdSelected.set(selected.qd.getDoubleValue());

      yoVariableServer.update(time);
   }

   private class JointHolder
   {
      private final YoVariableRegistry registry;
      private final OneDoFJoint joint;
      private final PDController pdController;
      private final JointHandle handle;

      private final DoubleYoVariable q;
      private final DoubleYoVariable qd;
      private final DoubleYoVariable tau;

      private final DoubleYoVariable q_d;
      private final DoubleYoVariable qd_d;
      private final DoubleYoVariable tau_d;

      public JointHolder(OneDoFJoint joint, JointHandle handle, YoVariableRegistry parentRegistry)
      {
         this.joint = joint;
         this.handle = handle;

         this.registry = new YoVariableRegistry(joint.getName());
         this.pdController = new PDController(joint.getName(), registry);

         q = new DoubleYoVariable(joint.getName() + "_q", registry);
         qd = new DoubleYoVariable(joint.getName() + "_qd", registry);
         tau = new DoubleYoVariable(joint.getName() + "_tau", registry);

         q_d = new DoubleYoVariable(joint.getName() + "_q_d", registry);
         qd_d = new DoubleYoVariable(joint.getName() + "_qd_d", registry);
         tau_d = new DoubleYoVariable(joint.getName() + "_tau_d", registry);

         parentRegistry.addChild(registry);
      }

      public void update()
      {
         joint.setQ(handle.getPosition());
         joint.setQd(handle.getVelocity());
         joint.setTauMeasured(handle.getEffort());

         q.set(joint.getQ());
         qd.set(joint.getQd());
         tau.set(joint.getTauMeasured());

         tau_d.set(pdController.compute(q.getDoubleValue(), q_d.getDoubleValue(), qd.getDoubleValue(), qd_d.getDoubleValue()));

         handle.setDesiredEffort(tau_d.getDoubleValue());

      }
   }
}
