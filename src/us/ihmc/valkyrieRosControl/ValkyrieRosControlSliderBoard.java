package us.ihmc.valkyrieRosControl;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.rosControl.JointHandle;
import us.ihmc.rosControl.valkyrie.IHMCValkyrieControlJavaBridge;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;

public class ValkyrieRosControlSliderBoard extends IHMCValkyrieControlJavaBridge
{

//   private static final String[] controlledJoints = {"torsoYaw", "torsoPitch", "torsoRoll"};

//   private static final String[] controlledJoints = { "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
//       "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "torsoYaw", "torsoPitch", "torsoRoll",
//       "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch", "lowerNeckPitch",
//       "neckYaw", "upperNeckPitch", "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll",
//       "rightWristPitch" };

   private static final String[] controlledJoints = {
         "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
         "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll",
         "torsoYaw", "torsoPitch", "torsoRoll",
         "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch",
         "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch"
         };

   public static final boolean LOAD_STAND_PREP_SETPOINTS = true;
   public static final boolean LOAD_TORQUE_OFFSETS = true;
   public static final double KP_DEFAULT = 30.0;
   public static final double KD_DEFAULT = 1.0;

   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, true);
   private final SDFFullHumanoidRobotModel sdfFullRobotModel = robotModel.createFullRobotModel();

   private final ArrayList<JointHolder> jointHolders = new ArrayList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName()),
         robotModel.getLogModelProvider(), robotModel.getLogSettings(ValkyrieConfigurationRoot.USE_CAMERAS_FOR_LOGGING), 0.001);

   private final DoubleYoVariable masterScaleFactor = new DoubleYoVariable("masterScaleFactor", registry);

   private EnumYoVariable<?> selectedJoint;
   private EnumYoVariable<?> previousSelectedJoint;
   private final DoubleYoVariable qDesiredSelected = new DoubleYoVariable("qDesiredSelected", registry);
   private final DoubleYoVariable qdDesiredSelected = new DoubleYoVariable("qdDesiredSelected", registry);

   private final DoubleYoVariable kpSelected = new DoubleYoVariable("kpSelected", registry);
   private final DoubleYoVariable kdSelected = new DoubleYoVariable("kdSelected", registry);

   private final DoubleYoVariable qSelected = new DoubleYoVariable("qSelected", registry);
   private final DoubleYoVariable qdSelected = new DoubleYoVariable("qdSelected", registry);

   private final DoubleYoVariable tauSelected = new DoubleYoVariable("tauSelected", registry);
   private final DoubleYoVariable tauOffsetSelected = new DoubleYoVariable("tauOffsetSelected", registry);
   private final DoubleYoVariable tauPDSelected = new DoubleYoVariable("tauPDSelected", registry);
   private final DoubleYoVariable tauFunctionSelected = new DoubleYoVariable("tauFunctionSelected", registry);
   private final DoubleYoVariable tauDesiredSelected = new DoubleYoVariable("tauDesiredSelected", registry);

   private long startTime = -1;
   private final DoubleYoVariable yoTime = new DoubleYoVariable("time", registry);
   private YoFunctionGenerator functionGenerator;

   @Override
   protected void init()
   {
      kpSelected.set(KP_DEFAULT);
      kdSelected.set(KD_DEFAULT);

      double dt = robotModel.getEstimatorDT();
      functionGenerator = new YoFunctionGenerator("FG", yoTime, registry, true, dt);
      functionGenerator.setAmplitude(0.0);
      functionGenerator.setFrequency(0.0);
      functionGenerator.setOffset(0.0);

      if (LOAD_STAND_PREP_SETPOINTS)
         loadStandPrepSetPoints();

      if (LOAD_TORQUE_OFFSETS)
         loadTorqueOffsets();

      ArrayList<String> jointNames = new ArrayList<>();
      for (String jointName : controlledJoints)
      {
         OneDoFJoint joint = sdfFullRobotModel.getOneDoFJointByName(jointName);
         JointHandle handle = createJointHandle(jointName);
         jointHolders.add(new JointHolder(joint, handle, registry));
         jointNames.add(joint.getName());
      }

      String[] jointNameArray = jointNames.toArray(new String[jointNames.size()]);
      selectedJoint = new EnumYoVariable<>("selectedJoint", "", registry, false, jointNameArray);
      System.out.println(Arrays.toString(selectedJoint.getEnumValuesAsString()));
      previousSelectedJoint = new EnumYoVariable<>("previousSelectedJoint", "", registry, true, jointNameArray);
      previousSelectedJoint.set(EnumYoVariable.NULL_VALUE);

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

            tauOffsetSelected.set(selected.tau_offset.getDoubleValue());

            if (previousSelectedJoint.getOrdinal() != EnumYoVariable.NULL_VALUE)
               jointHolders.get(previousSelectedJoint.getOrdinal()).tau_function.set(0.0);

            previousSelectedJoint.set(selectedJoint.getOrdinal());
         }
      });

      loadStandPrepSetPoints();
      selectedJoint.notifyVariableChangedListeners();
      
      yoVariableServer.setMainRegistry(registry, sdfFullRobotModel, null);
      yoVariableServer.start();
   }

   private Map<String, Double> setPointMap = null;

   @SuppressWarnings("unchecked")
   private void loadStandPrepSetPoints()
   {
      try
      {
         Yaml yaml = new Yaml();
         InputStream setpointsStream = getClass().getClassLoader().getResourceAsStream("standPrep/setpoints.yaml");
         setPointMap = (Map<String, Double>) yaml.load(setpointsStream);
         setpointsStream.close();
      }
      catch (Exception e)
      {
         PrintTools.error(this, "Could not load stand prep set points.");
         setPointMap = null;
      }
   }

   private Map<String, Double> torqueOffsetMap = null;

   @SuppressWarnings("unchecked")
   private void loadTorqueOffsets()
   {
      try
      {
         Yaml yaml = new Yaml();
         FileInputStream offsetsStream = new FileInputStream(new File(ValkyrieTorqueOffsetPrinter.IHMC_TORQUE_OFFSET_FILE));
         torqueOffsetMap = (Map<String, Double>) yaml.load(offsetsStream);
         offsetsStream.close();
      }
      catch (Exception e)
      {
         PrintTools.error(this, "Could not load joint torque offsets.");
         torqueOffsetMap = null;
      }
   }

   @Override
   protected void doControl(long time, long duration)
   {
      if (startTime == -1)
         startTime = time;
      yoTime.set(TimeTools.nanoSecondstoSeconds(time - startTime));

      tauFunctionSelected.set(functionGenerator.getValue());

      masterScaleFactor.set(MathTools.clipToMinMax(masterScaleFactor.getDoubleValue(), 0.0, 1.0));
      JointHolder selected = jointHolders.get(selectedJoint.getOrdinal());
      selected.q_d.set(MathTools.clipToMinMax(qDesiredSelected.getDoubleValue(), selected.joint.getJointLimitLower(), selected.joint.getJointLimitUpper()));
      selected.qd_d.set(qdDesiredSelected.getDoubleValue());
      selected.pdController.setProportionalGain(kpSelected.getDoubleValue());
      selected.pdController.setDerivativeGain(kdSelected.getDoubleValue());
      selected.tau_function.set(tauFunctionSelected.getDoubleValue());
      selected.tau_offset.set(tauOffsetSelected.getDoubleValue());

      for (int i = 0; i < jointHolders.size(); i++)
      {
         JointHolder holder = jointHolders.get(i);
         holder.update();
      }

      qSelected.set(selected.q.getDoubleValue());
      qdSelected.set(selected.qd.getDoubleValue());

      tauSelected.set(selected.tau.getDoubleValue());
      tauPDSelected.set(selected.tau_pd.getDoubleValue());
      tauDesiredSelected.set(selected.tau_d.getDoubleValue());

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
      private final DoubleYoVariable tau_offset;
      private final DoubleYoVariable tau_pd;
      private final DoubleYoVariable tau_function;
      private final DoubleYoVariable tau_d;

      public JointHolder(OneDoFJoint joint, JointHandle handle, YoVariableRegistry parentRegistry)
      {
         this.joint = joint;
         this.handle = handle;

         String jointName = joint.getName();
         this.registry = new YoVariableRegistry(jointName);
         this.pdController = new PDController(jointName, registry);
         pdController.setProportionalGain(KP_DEFAULT);
         pdController.setDerivativeGain(KD_DEFAULT);

         q = new DoubleYoVariable(jointName + "_q", registry);
         qd = new DoubleYoVariable(jointName + "_qd", registry);
         tau = new DoubleYoVariable(jointName + "_tau", registry);

         q_d = new DoubleYoVariable(jointName + "_q_d", registry);
         qd_d = new DoubleYoVariable(jointName + "_qd_d", registry);

         tau_offset = new DoubleYoVariable(jointName + "_tau_offset", parentRegistry);
         tau_d = new DoubleYoVariable(jointName + "_tau_d", registry);
         tau_pd = new DoubleYoVariable(jointName + "_tau_pd", registry);
         tau_function = new DoubleYoVariable(jointName + "_tau_function", registry);

         if (setPointMap != null && setPointMap.containsKey(jointName))
            q_d.set(setPointMap.get(jointName));

         if (torqueOffsetMap != null && torqueOffsetMap.containsKey(jointName))
            tau_offset.set(torqueOffsetMap.get(jointName));

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

         double pdOutput = pdController.compute(q.getDoubleValue(), q_d.getDoubleValue(), qd.getDoubleValue(), qd_d.getDoubleValue());
         tau_pd.set(pdOutput);
         tau_d.set(masterScaleFactor.getDoubleValue() * (tau_pd.getDoubleValue() + tau_function.getDoubleValue()) + tau_offset.getDoubleValue());

         handle.setDesiredEffort(tau_d.getDoubleValue());
      }
   }
}
