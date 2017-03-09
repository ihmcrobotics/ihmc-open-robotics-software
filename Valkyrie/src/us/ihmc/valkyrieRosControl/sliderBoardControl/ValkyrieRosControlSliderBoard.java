package us.ihmc.valkyrieRosControl.sliderBoardControl;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import org.yaml.snakeyaml.Yaml;

import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.rosControl.wholeRobot.IHMCWholeRobotControlJavaBridge;
import us.ihmc.rosControl.wholeRobot.PositionJointHandle;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrieRosControl.ValkyrieTorqueOffsetPrinter;

public class ValkyrieRosControlSliderBoard extends IHMCWholeRobotControlJavaBridge
{

//   private static final String[] torqueControlledJoints = {"torsoYaw", "torsoPitch", "torsoRoll"};

//   private static final String[] torqueControlledJoints = { "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
//       "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "torsoYaw", "torsoPitch", "torsoRoll",
//       "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch", "lowerNeckPitch",
//       "neckYaw", "upperNeckPitch", "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll",
//       "rightWristPitch" };

   private static final String[] torqueControlledJoints = {
         "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
         "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll",
         "torsoYaw", "torsoPitch", "torsoRoll",
         "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch",
         "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch"
         };

   private static final String[] positionControlledJoints = { "lowerNeckPitch", "neckYaw", "upperNeckPitch", };

   public static final boolean LOAD_STAND_PREP_SETPOINTS = true;
   public static final boolean LOAD_TORQUE_OFFSETS = true;
   public static final double KP_DEFAULT = 30.0;
   public static final double KD_DEFAULT = 1.0;

   private static final boolean RESET_FUNCTIONS_ON_JOINT_CHANGE = false;

   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, true);
   private final FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

   private final ArrayList<ValkyrieSliderBoardJointHolder> jointHolders = new ArrayList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName()),
         robotModel.getLogModelProvider(), robotModel.getLogSettings(ValkyrieConfigurationRoot.USE_CAMERAS_FOR_LOGGING), 0.001);

   /* package private */ final DoubleYoVariable masterScaleFactor = new DoubleYoVariable("masterScaleFactor", registry);

   /* package private */ final DoubleYoVariable jointVelocityAlphaFilter = new DoubleYoVariable("jointVelocityAlphaFilter", registry);
   /* package private */ final DoubleYoVariable jointVelocitySlopTime = new DoubleYoVariable("jointBacklashSlopTime", registry);

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
   private YoFunctionGenerator selectedFunctionGenerator;

   private YoFunctionGenerator secondaryFunctionGenerator;
   private EnumYoVariable<?> secondaryJoint;
   private final DoubleYoVariable tauFunctionSecondary = new DoubleYoVariable("tauFunctionSecondary", registry);

   @Override
   protected void init()
   {
      double dt = robotModel.getEstimatorDT();
      jointVelocityAlphaFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(20.0, dt));
      jointVelocitySlopTime.set(0.03);
      
      kpSelected.set(KP_DEFAULT);
      kdSelected.set(KD_DEFAULT);

      selectedFunctionGenerator = new YoFunctionGenerator("Selected", yoTime, registry, false, dt);
      selectedFunctionGenerator.setMode(YoFunctionGeneratorMode.SINE);
      selectedFunctionGenerator.setAmplitude(0.0);
      selectedFunctionGenerator.setFrequency(0.0);
      selectedFunctionGenerator.setOffset(0.0);

      secondaryFunctionGenerator = new YoFunctionGenerator("Secondary", yoTime, registry, false, dt);
      secondaryFunctionGenerator.setMode(YoFunctionGeneratorMode.SINE);
      secondaryFunctionGenerator.setAmplitude(0.0);
      secondaryFunctionGenerator.setFrequency(0.0);
      secondaryFunctionGenerator.setOffset(0.0);

      if (LOAD_STAND_PREP_SETPOINTS)
         loadStandPrepSetPoints();

      if (LOAD_TORQUE_OFFSETS)
         loadTorqueOffsets();

      ArrayList<String> jointNames = new ArrayList<>();
      for (String jointName : torqueControlledJoints)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
         EffortJointHandle handle = createEffortJointHandle(jointName);
         jointHolders.add(new EffortJointHolder(this, joint, handle, registry, dt));
         jointNames.add(joint.getName());
      }

      for (String jointName : positionControlledJoints)
      {
         OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
         PositionJointHandle handle = createPositionJointHandle(jointName);
         jointHolders.add(new PositionJointHolder(this, joint, handle, registry, dt));
         jointNames.add(joint.getName());
      }

      String[] jointNameArray = jointNames.toArray(new String[jointNames.size()]);
      selectedJoint = new EnumYoVariable<>("selectedJoint", "", registry, false, jointNameArray);
      System.out.println(Arrays.toString(selectedJoint.getEnumValuesAsString()));
      previousSelectedJoint = new EnumYoVariable<>("previousSelectedJoint", "", registry, true, jointNameArray);
      previousSelectedJoint.set(EnumYoVariable.NULL_VALUE);
      secondaryJoint = new EnumYoVariable<>("secondaryJoint", "", registry, true, jointNameArray);
      secondaryJoint.set(EnumYoVariable.NULL_VALUE);

      selectedJoint.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            ValkyrieSliderBoardJointHolder selected = jointHolders.get(selectedJoint.getOrdinal());
            qDesiredSelected.set(selected.q_d.getDoubleValue());
            qdDesiredSelected.set(selected.qd_d.getDoubleValue());

            kpSelected.set(selected.pdController.getProportionalGain());
            kdSelected.set(selected.pdController.getDerivativeGain());

            tauOffsetSelected.set(selected.tau_offset.getDoubleValue());

            if (previousSelectedJoint.getOrdinal() != EnumYoVariable.NULL_VALUE)
               jointHolders.get(previousSelectedJoint.getOrdinal()).jointCommand_function.set(0.0);

            if (RESET_FUNCTIONS_ON_JOINT_CHANGE || selectedJoint.getOrdinal() != secondaryJoint.getOrdinal() || previousSelectedJoint.getOrdinal() != EnumYoVariable.NULL_VALUE)
            {
               selectedFunctionGenerator.setAmplitude(0.0);
               selectedFunctionGenerator.setFrequency(0.0);
               selectedFunctionGenerator.setOffset(0.0);

               secondaryFunctionGenerator.setAmplitude(0.0);
               secondaryFunctionGenerator.setFrequency(0.0);
               secondaryFunctionGenerator.setOffset(0.0);
            }
            else
            {
               secondaryJoint.set(previousSelectedJoint.getOrdinal());
               double previousAmplitude = selectedFunctionGenerator.getAmplitude();
               double previousFrequency = selectedFunctionGenerator.getFrequency();
               double previousOffset = selectedFunctionGenerator.getOffset();
               double previousPhase = selectedFunctionGenerator.getPhase();

               selectedFunctionGenerator.setAmplitude(secondaryFunctionGenerator.getAmplitude());
               selectedFunctionGenerator.setFrequency(secondaryFunctionGenerator.getFrequency());
               selectedFunctionGenerator.setOffset(secondaryFunctionGenerator.getOffset());
               selectedFunctionGenerator.setPhase(secondaryFunctionGenerator.getPhase());

               secondaryFunctionGenerator.setAmplitude(previousAmplitude);
               secondaryFunctionGenerator.setFrequency(previousFrequency);
               secondaryFunctionGenerator.setOffset(previousOffset);
               secondaryFunctionGenerator.setPhase(previousPhase);
            }

            previousSelectedJoint.set(selectedJoint.getOrdinal());
         }
      });

      loadStandPrepSetPoints();
      selectedJoint.notifyVariableChangedListeners();
      
      yoVariableServer.setMainRegistry(registry, fullRobotModel, null);
      yoVariableServer.start();
   }

   /* package private */ Map<String, Double> setPointMap = null;

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

   /* package private */ Map<String, Double> torqueOffsetMap = null;

   private void loadTorqueOffsets()
   {
      torqueOffsetMap = ValkyrieTorqueOffsetPrinter.loadTorqueOffsetsFromFile();
   }

   @Override
   protected void doControl(long time, long duration)
   {
      if (startTime == -1)
         startTime = time;
      yoTime.set(Conversions.nanoSecondstoSeconds(time - startTime));

      tauFunctionSelected.set(selectedFunctionGenerator.getValue());

      masterScaleFactor.set(MathTools.clamp(masterScaleFactor.getDoubleValue(), 0.0, 1.0));
      ValkyrieSliderBoardJointHolder selected = jointHolders.get(selectedJoint.getOrdinal());
      selected.q_d.set(MathTools.clamp(qDesiredSelected.getDoubleValue(), selected.joint.getJointLimitLower(), selected.joint.getJointLimitUpper()));
      selected.qd_d.set(qdDesiredSelected.getDoubleValue());
      selected.pdController.setProportionalGain(kpSelected.getDoubleValue());
      selected.pdController.setDerivativeGain(kdSelected.getDoubleValue());
      selected.jointCommand_function.set(tauFunctionSelected.getDoubleValue());
      selected.tau_offset.set(tauOffsetSelected.getDoubleValue());

      if (secondaryJoint.getOrdinal() != EnumYoVariable.NULL_VALUE)
      {
         ValkyrieSliderBoardJointHolder secondary = jointHolders.get(secondaryJoint.getOrdinal());
         if (secondaryJoint.getOrdinal() != selectedJoint.getOrdinal())
            secondary.jointCommand_function.set(secondaryFunctionGenerator.getValue());
         tauFunctionSecondary.set(secondary.jointCommand_function.getDoubleValue());
      }

      for (int i = 0; i < jointHolders.size(); i++)
      {
         ValkyrieSliderBoardJointHolder holder = jointHolders.get(i);
         holder.update();
      }

      qSelected.set(selected.q.getDoubleValue());
      qdSelected.set(selected.qd.getDoubleValue());

      tauSelected.set(selected.tau.getDoubleValue());
      tauPDSelected.set(selected.jointCommand_pd.getDoubleValue());
      tauDesiredSelected.set(selected.tau_d.getDoubleValue());

      yoVariableServer.update(time);
   }

}
