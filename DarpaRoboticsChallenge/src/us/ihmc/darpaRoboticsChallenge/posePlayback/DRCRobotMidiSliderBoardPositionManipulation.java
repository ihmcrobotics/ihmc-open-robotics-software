package us.ihmc.darpaRoboticsChallenge.posePlayback;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap;
import us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMapCorrelation;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.GeometricJacobian;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicCoordinateSystem;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicLineSegment;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.inputdevices.MidiSliderBoard;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePose;

public class DRCRobotMidiSliderBoardPositionManipulation
{
   private final SideDependentList<String> sideString = new SideDependentList<String>("q_l_", "q_r_");
   private final LinkedHashMap<SpineJointName, String> spineJointStringNames = new LinkedHashMap<SpineJointName, String>();
   private final LinkedHashMap<SpineJointName, Double> spineJointLowerLimits = new LinkedHashMap<SpineJointName, Double>();
   private final LinkedHashMap<SpineJointName, Double> spineJointUpperLimits = new LinkedHashMap<SpineJointName, Double>();
   private final LinkedHashMap<LegJointName, String> legJointStringNames = new LinkedHashMap<LegJointName, String>();
   private final SideDependentList<Map<LegJointName, Double>> legJointLowerLimits = SideDependentList.createListOfHashMaps();
   private final SideDependentList<Map<LegJointName, Double>> legJointUpperLimits = SideDependentList.createListOfHashMaps();
   private final LinkedHashMap<ArmJointName, String> armJointStringNames = new LinkedHashMap<ArmJointName, String>();
   private final SideDependentList<Map<ArmJointName, Double>> armJointLowerLimits = SideDependentList.createListOfHashMaps();
   private final SideDependentList<Map<ArmJointName, Double>> armJointUpperLimits = SideDependentList.createListOfHashMaps();
   private final double fingerJointLimit = 1.57079632679;

   private final YoVariableRegistry registry = new YoVariableRegistry("SliderBoardRegistry");
   private final YoVariableRegistry dontRecordRegistry = new YoVariableRegistry("dontRecordRegistry");

   private final EnumYoVariable<SliderSpace> sliderSpace = new EnumYoVariable<SliderSpace>("sliderSpace", "", registry, SliderSpace.class, false);
   private final EnumYoVariable<SliderBodyPart> sliderBodyPart = new EnumYoVariable<SliderBodyPart>("sliderBodyPart", "", registry, SliderBodyPart.class, false);

   private final BooleanYoVariable isCaptureSnapshotRequested = new BooleanYoVariable("isCaptureSnapshotRequested", dontRecordRegistry);
   private final BooleanYoVariable isSaveSequenceRequested = new BooleanYoVariable("isSaveSequenceRequested", dontRecordRegistry);
   private final BooleanYoVariable isLoadSequenceRequested = new BooleanYoVariable("isLoadSequenceRequested", dontRecordRegistry);
   private final BooleanYoVariable isClearSequenceRequested = new BooleanYoVariable("isClearSequenceRequested", dontRecordRegistry);
   private final BooleanYoVariable isLoadFrameByFrameSequenceRequested = new BooleanYoVariable("isLoadFrameByFrameSequenceRequested", dontRecordRegistry);
   private final BooleanYoVariable isPlayPoseFromFrameByFrameSequenceRequested = new BooleanYoVariable("isPlayPoseFromFrameByFrameSequenceRequested",
         dontRecordRegistry);
   private final BooleanYoVariable isSymmetricModeRequested = new BooleanYoVariable("isSymmetricModeRequested", dontRecordRegistry);
   private final BooleanYoVariable isResetToBasePoseRequested = new BooleanYoVariable("isResetToBasePoseRequested", dontRecordRegistry);
   private final BooleanYoVariable isLoadLastSequenceRequested = new BooleanYoVariable("isLoadLastSequenceRequested", dontRecordRegistry);

   private final BooleanYoVariable isPelvisControlRequested = new BooleanYoVariable("isPelvisControlRequested", dontRecordRegistry);
   private final BooleanYoVariable isChestControlRequested = new BooleanYoVariable("isChestControlRequested", dontRecordRegistry);

   private final BooleanYoVariable isLeftLegControlRequested = new BooleanYoVariable("isLeftLegControlRequested", dontRecordRegistry);
   private final BooleanYoVariable isRightLegControlRequested = new BooleanYoVariable("isRightLegControlRequested", dontRecordRegistry);
   private final BooleanYoVariable isLeftArmControlRequested = new BooleanYoVariable("isLeftArmControlRequested", dontRecordRegistry);
   private final BooleanYoVariable isRightArmControlRequested = new BooleanYoVariable("isRightArmControlRequested", dontRecordRegistry);

   private final BooleanYoVariable isSupportBaseControlRequested = new BooleanYoVariable("isSupportBaseControlRequested", dontRecordRegistry);
   private final BooleanYoVariable isSupportBaseToggleRequested = new BooleanYoVariable("isSupportBaseToggleRequested", dontRecordRegistry);

   private final BooleanYoVariable isSupportBaseControlTargetRequested = new BooleanYoVariable("isSupportBaseControlTargetRequested", dontRecordRegistry);
   private final BooleanYoVariable isSupportBaseTargetToggleRequested = new BooleanYoVariable("isSupportBaseTargetToggleRequested", dontRecordRegistry);

   private final SideDependentList<BooleanYoVariable> isLegControlRequested = new SideDependentList<BooleanYoVariable>(isLeftLegControlRequested,
         isRightLegControlRequested);
   private final SideDependentList<BooleanYoVariable> isArmControlRequested = new SideDependentList<BooleanYoVariable>(isLeftArmControlRequested,
         isRightArmControlRequested);

   private final SimulationConstructionSet scs;
   private final MidiSliderBoard sliderBoard;

   private final DoubleYoVariable q_yaw = new DoubleYoVariable("q_yaw", registry);
   private final DoubleYoVariable q_pitch = new DoubleYoVariable("q_pitch", registry);
   private final DoubleYoVariable q_roll = new DoubleYoVariable("q_roll", registry);

   private final DoubleYoVariable q_left = new DoubleYoVariable("q_left", registry);
   private final DoubleYoVariable q_right = new DoubleYoVariable("q_right", registry);
   private final SideDependentList<DoubleYoVariable> q_hands = new SideDependentList<DoubleYoVariable>(q_left, q_right);
   private final SideDependentList<String> handSideString = new SideDependentList<String>("q_left_f", "q_right_f");

   private boolean symmetricMode = false;
   private RobotSide symmetricControlSide;
   private final DoubleYoVariable q_qs, q_qx, q_qy, q_qz;
   private Quat4d qprev;

   //   private final DoubleYoVariable BaseControlPoint = new DoubleYoVariable("BaseControlPoint", registry);
   private final YoFramePoint[] baseControlPoints = new YoFramePoint[4];
   private final ArrayList<DynamicGraphicObject> baseControlPointsList = new ArrayList<DynamicGraphicObject>();
   private final ArrayList<DynamicGraphicObject> baseControlLinesList = new ArrayList<DynamicGraphicObject>();

   private final YoFramePoint[] baseControlTargetPoints = new YoFramePoint[4];
   private final ArrayList<DynamicGraphicObject> baseControlTargetPointsList = new ArrayList<DynamicGraphicObject>();
   private final ArrayList<DynamicGraphicObject> baseControlTargetLinesList = new ArrayList<DynamicGraphicObject>();

   public DRCRobotMidiSliderBoardPositionManipulation(SimulationConstructionSet scs)
   {
      this(scs, null, null, null, null);
   }

   public DRCRobotMidiSliderBoardPositionManipulation(SimulationConstructionSet scs, SDFRobot sdfRobot, ReferenceFrames referenceFrames,
         SDFFullRobotModel fullRobotModel, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.scs = scs;

      if (sdfRobot != null)
         setupForInverseKinematics(sdfRobot, referenceFrames, fullRobotModel, dynamicGraphicObjectsListRegistry);

      sliderSpace.set(SliderSpace.JOINT);
      sliderBodyPart.set(SliderBodyPart.LEFT_LEG);

      scs.addYoVariableRegistry(registry);
      sliderBoard = new MidiSliderBoard(scs);
      init();

      UpdateInverseKinematicsListener updateInverseKinematicsListener = new UpdateInverseKinematicsListener();
      sliderBoard.attachVariableChangedListener(updateInverseKinematicsListener);

      q_qs = (DoubleYoVariable) scs.getVariable("q_qs");
      q_qx = (DoubleYoVariable) scs.getVariable("q_qx");
      q_qy = (DoubleYoVariable) scs.getVariable("q_qy");
      q_qz = (DoubleYoVariable) scs.getVariable("q_qz");

      ChangeCurrentPartBeingControlledListener listener = new ChangeCurrentPartBeingControlledListener();

      for (RobotSide robotSide : RobotSide.values)
      {
         isLegControlRequested.get(robotSide).addVariableChangedListener(listener);
         isArmControlRequested.get(robotSide).addVariableChangedListener(listener);
      }
      isPelvisControlRequested.addVariableChangedListener(listener);
      isChestControlRequested.addVariableChangedListener(listener);

      PelvisRotationListener pelvisListener = new PelvisRotationListener();
      q_yaw.addVariableChangedListener(pelvisListener);
      q_pitch.addVariableChangedListener(pelvisListener);
      q_roll.addVariableChangedListener(pelvisListener);

      for (RobotSide robotSide : RobotSide.values)
      {
         q_hands.get(robotSide).addVariableChangedListener(new HandListener(robotSide));
      }

      isLeftLegControlRequested.set(true);

      isSymmetricModeRequested.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable yoVariable)
         {
            symmetricMode = isSymmetricModeRequested.getBooleanValue();
         }
      });

      setupSymmetricModeListeners();

      setupSupportPolygonDisplayAndControl(dynamicGraphicObjectsListRegistry);
   }

   private void setupSupportPolygonDisplayAndControl(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      for (int i = 0; i < baseControlPoints.length; i++)
      {
         baseControlPoints[i] = new YoFramePoint("baseControlPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         baseControlTargetPoints[i] = new YoFramePoint("baseControlTargetPoint" + i, ReferenceFrame.getWorldFrame(), registry);
      }
      baseControlPoints[0].set(0.18, 0.13, 0.0);
      baseControlPoints[1].set(0.18, -0.13, 0.0);
      baseControlPoints[2].set(-0.08, -0.15, 0.0);
      baseControlPoints[3].set(-0.08, 0.15, 0.0);

      baseControlTargetPoints[0].set(0.18, 0.13, 0.0);
      baseControlTargetPoints[1].set(0.18, -0.13, 0.0);
      baseControlTargetPoints[2].set(-0.08, -0.15, 0.0);
      baseControlTargetPoints[3].set(-0.08, 0.15, 0.0);

      SupportBaseControlRequestedListener supportBaseControlRequestedListener = new SupportBaseControlRequestedListener();
      isSupportBaseControlRequested.addVariableChangedListener(supportBaseControlRequestedListener);
      isSupportBaseToggleRequested.addVariableChangedListener(supportBaseControlRequestedListener);

      SupportBaseControlTargetRequestedListener supportBaseControlTargetRequestedListener = new SupportBaseControlTargetRequestedListener();
      isSupportBaseControlTargetRequested.addVariableChangedListener(supportBaseControlTargetRequestedListener);
      isSupportBaseTargetToggleRequested.addVariableChangedListener(supportBaseControlTargetRequestedListener);

      addSupportBaseControlGraphics(dynamicGraphicObjectsListRegistry);
      addSupportBaseControlTargetGraphics(dynamicGraphicObjectsListRegistry);
   }

   public void addCaptureSnapshotListener(VariableChangedListener variableChangedListener)
   {
      isCaptureSnapshotRequested.addVariableChangedListener(variableChangedListener);
   }

   public void addSaveSequenceRequestedListener(VariableChangedListener variableChangedListener)
   {
      isSaveSequenceRequested.addVariableChangedListener(variableChangedListener);
   }

   public void addLoadSequenceRequestedListener(VariableChangedListener variableChangedListener)
   {
      isLoadSequenceRequested.addVariableChangedListener(variableChangedListener);
   }

   public void addClearSequenceRequestedListener(VariableChangedListener variableChangedListener)
   {
      isClearSequenceRequested.addVariableChangedListener(variableChangedListener);
   }

   public void addLoadFrameByFrameSequenceRequestedListener(VariableChangedListener variableChangedListener)
   {
      isLoadFrameByFrameSequenceRequested.addVariableChangedListener(variableChangedListener);
   }
   
   public void addLoadLastSequenceRequestedListener(VariableChangedListener variableChangedListener)
   {
      isLoadLastSequenceRequested.addVariableChangedListener(variableChangedListener);
   }

   public void addPlayPoseFromFrameByFrameSequenceRequestedListener(VariableChangedListener variableChangedListener)
   {
      isPlayPoseFromFrameByFrameSequenceRequested.addVariableChangedListener(variableChangedListener);
   }

   public void addResetToBasePoseRequestedListener(VariableChangedListener variableChangedListener)
   {
      isResetToBasePoseRequested.addVariableChangedListener(variableChangedListener);
   }

   private void init()
   {
      String prefix = "q_back_";
      spineJointStringNames.put(SpineJointName.SPINE_YAW, prefix + "lbz");
      spineJointStringNames.put(SpineJointName.SPINE_PITCH, prefix + "mby");
      spineJointStringNames.put(SpineJointName.SPINE_ROLL, prefix + "ubx");

      spineJointLowerLimits.put(SpineJointName.SPINE_YAW, -0.610865);
      spineJointLowerLimits.put(SpineJointName.SPINE_PITCH, -1.2);
      spineJointLowerLimits.put(SpineJointName.SPINE_ROLL, -0.790809);

      spineJointUpperLimits.put(SpineJointName.SPINE_YAW, 0.610865);
      spineJointUpperLimits.put(SpineJointName.SPINE_PITCH, 1.28);
      spineJointUpperLimits.put(SpineJointName.SPINE_ROLL, 0.790809);

      String legPrefix = "leg_";
      legJointStringNames.put(LegJointName.HIP_YAW, legPrefix + "uhz");
      legJointStringNames.put(LegJointName.HIP_ROLL, legPrefix + "mhx");
      legJointStringNames.put(LegJointName.HIP_PITCH, legPrefix + "lhy");
      legJointStringNames.put(LegJointName.KNEE, legPrefix + "kny");
      legJointStringNames.put(LegJointName.ANKLE_PITCH, legPrefix + "uay");
      legJointStringNames.put(LegJointName.ANKLE_ROLL, legPrefix + "lax");

      double hipYawMax = 1.14;
      double hipYawMin = -0.32;

      SideDependentList<Double> hipYawLowerLimit = new SideDependentList<Double>(hipYawMin, -hipYawMin);
      SideDependentList<Double> hipYawUpperLimit = new SideDependentList<Double>(hipYawMax, -hipYawMax);

      double hipRollMax = 0.495;
      double hipRollMin = -0.47;

      SideDependentList<Double> hipRollLowerLimit = new SideDependentList<Double>(hipRollMin, -hipRollMin);
      SideDependentList<Double> hipRollUpperLimit = new SideDependentList<Double>(hipRollMax, -hipRollMax);

      for (RobotSide robotSide : RobotSide.values())
      {
         legJointLowerLimits.get(robotSide).put(LegJointName.HIP_YAW, hipYawLowerLimit.get(robotSide));
         legJointLowerLimits.get(robotSide).put(LegJointName.HIP_ROLL, hipRollLowerLimit.get(robotSide));
         legJointLowerLimits.get(robotSide).put(LegJointName.HIP_PITCH, -1.75);
         legJointLowerLimits.get(robotSide).put(LegJointName.KNEE, 0.0);
         legJointLowerLimits.get(robotSide).put(LegJointName.ANKLE_PITCH, -0.698);
         legJointLowerLimits.get(robotSide).put(LegJointName.ANKLE_ROLL, -0.436);

         legJointUpperLimits.get(robotSide).put(LegJointName.HIP_YAW, hipYawUpperLimit.get(robotSide));
         legJointUpperLimits.get(robotSide).put(LegJointName.HIP_ROLL, hipRollUpperLimit.get(robotSide));
         legJointUpperLimits.get(robotSide).put(LegJointName.HIP_PITCH, 0.524);
         legJointUpperLimits.get(robotSide).put(LegJointName.KNEE, 2.45);
         legJointUpperLimits.get(robotSide).put(LegJointName.ANKLE_PITCH, 0.698);
         legJointUpperLimits.get(robotSide).put(LegJointName.ANKLE_ROLL, 0.436);
      }

      String armPrefix = "arm_";
      armJointStringNames.put(ArmJointName.SHOULDER_PITCH, armPrefix + "usy");
      armJointStringNames.put(ArmJointName.SHOULDER_ROLL, armPrefix + "shx");
      armJointStringNames.put(ArmJointName.ELBOW_PITCH, armPrefix + "ely");
      armJointStringNames.put(ArmJointName.ELBOW_ROLL, armPrefix + "elx");
      armJointStringNames.put(ArmJointName.WRIST_PITCH, armPrefix + "uwy");
      armJointStringNames.put(ArmJointName.WRIST_ROLL, armPrefix + "mwx");

      double shoulderRollMax = 1.74533;
      double shoulderRollMin = -1.39626;

      SideDependentList<Double> shoulderRollLowerLimit = new SideDependentList<Double>(shoulderRollMin, -shoulderRollMin);
      SideDependentList<Double> shoulderRollUpperLimit = new SideDependentList<Double>(shoulderRollMax, -shoulderRollMax);

      double elbowRollMin = 0.0;
      double elbowRollMax = 2.35619;
      SideDependentList<Double> elbowRollLowerLimit = new SideDependentList<Double>(elbowRollMin, -elbowRollMin);
      SideDependentList<Double> elbowRollUpperLimit = new SideDependentList<Double>(elbowRollMax, -elbowRollMax);

      double wristRollMin = -0.436;
      double wristRollMax = 1.571;
      SideDependentList<Double> wristRollLowerLimit = new SideDependentList<Double>(wristRollMin, -wristRollMin);
      SideDependentList<Double> wristRollUpperLimit = new SideDependentList<Double>(wristRollMax, -wristRollMax);

      for (RobotSide robotSide : RobotSide.values())
      {
         armJointLowerLimits.get(robotSide).put(ArmJointName.SHOULDER_PITCH, -1.9635);
         armJointLowerLimits.get(robotSide).put(ArmJointName.SHOULDER_ROLL, shoulderRollLowerLimit.get(robotSide));
         armJointLowerLimits.get(robotSide).put(ArmJointName.ELBOW_PITCH, 0.0);
         armJointLowerLimits.get(robotSide).put(ArmJointName.ELBOW_ROLL, elbowRollLowerLimit.get(robotSide));
         armJointLowerLimits.get(robotSide).put(ArmJointName.WRIST_PITCH, -1.571);
         armJointLowerLimits.get(robotSide).put(ArmJointName.WRIST_ROLL, wristRollLowerLimit.get(robotSide));

         armJointUpperLimits.get(robotSide).put(ArmJointName.SHOULDER_PITCH, 1.9635);
         armJointUpperLimits.get(robotSide).put(ArmJointName.SHOULDER_ROLL, shoulderRollUpperLimit.get(robotSide));
         armJointUpperLimits.get(robotSide).put(ArmJointName.ELBOW_PITCH, 3.14159);
         armJointUpperLimits.get(robotSide).put(ArmJointName.ELBOW_ROLL, elbowRollUpperLimit.get(robotSide));
         armJointUpperLimits.get(robotSide).put(ArmJointName.WRIST_PITCH, 1.571);
         armJointUpperLimits.get(robotSide).put(ArmJointName.WRIST_ROLL, wristRollUpperLimit.get(robotSide));
      }

      resetSliderBoard();
   }

   private void resetSliderBoard()
   {
      LinkedHashMap<SpineJointName, Double> spineJointPositions = new LinkedHashMap<SpineJointName, Double>();
      for (SpineJointName spineJointName : spineJointStringNames.keySet())
      {
         double q = ((DoubleYoVariable) scs.getVariable(spineJointStringNames.get(spineJointName))).getDoubleValue();
         spineJointPositions.put(spineJointName, q);
      }
      SideDependentList<Map<ArmJointName, Double>> armJointPositions = SideDependentList.createListOfHashMaps();
      SideDependentList<Map<LegJointName, Double>> legJointPositions = SideDependentList.createListOfHashMaps();

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = sideString.get(robotSide);
         LinkedHashMap<ArmJointName, Double> armTemp = new LinkedHashMap<ArmJointName, Double>();
         for (ArmJointName armJointName : armJointStringNames.keySet())
         {
            double q = ((DoubleYoVariable) scs.getVariable(prefix + armJointStringNames.get(armJointName))).getDoubleValue();
            armTemp.put(armJointName, q);
         }
         armJointPositions.put(robotSide, armTemp);

         EnumMap<LegJointName, Double> legTemp = new EnumMap<LegJointName, Double>(LegJointName.class);
         for (LegJointName legJointName : legJointStringNames.keySet())
         {
            double q = ((DoubleYoVariable) scs.getVariable(prefix + legJointStringNames.get(legJointName))).getDoubleValue();
            legTemp.put(legJointName, q);
         }
         legJointPositions.put(robotSide, legTemp);
      }

      double q_x = ((DoubleYoVariable) scs.getVariable("q_x")).getDoubleValue();
      double q_y = ((DoubleYoVariable) scs.getVariable("q_y")).getDoubleValue();
      double q_z = ((DoubleYoVariable) scs.getVariable("q_z")).getDoubleValue();
      double q_qs = ((DoubleYoVariable) scs.getVariable("q_qs")).getDoubleValue();
      double q_qx = ((DoubleYoVariable) scs.getVariable("q_qx")).getDoubleValue();
      double q_qy = ((DoubleYoVariable) scs.getVariable("q_qy")).getDoubleValue();
      double q_qz = ((DoubleYoVariable) scs.getVariable("q_qz")).getDoubleValue();

      sliderBoard.reset();

      for (SpineJointName spineJointName : spineJointStringNames.keySet())
      {
         double q = spineJointPositions.get(spineJointName);
         ((DoubleYoVariable) scs.getVariable(spineJointStringNames.get(spineJointName))).set(q);
      }
      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = sideString.get(robotSide);
         for (ArmJointName armJointName : armJointStringNames.keySet())
         {
            double q = armJointPositions.get(robotSide).get(armJointName);
            ((DoubleYoVariable) scs.getVariable(prefix + armJointStringNames.get(armJointName))).set(q);
         }
         for (LegJointName legJointName : legJointStringNames.keySet())
         {
            double q = legJointPositions.get(robotSide).get(legJointName);
            ((DoubleYoVariable) scs.getVariable(prefix + legJointStringNames.get(legJointName))).set(q);
         }
      }

      ((DoubleYoVariable) scs.getVariable("q_x")).set(q_x);
      ((DoubleYoVariable) scs.getVariable("q_y")).set(q_y);
      ((DoubleYoVariable) scs.getVariable("q_z")).set(q_z);
      ((DoubleYoVariable) scs.getVariable("q_qs")).set(q_qs);
      ((DoubleYoVariable) scs.getVariable("q_qx")).set(q_qx);
      ((DoubleYoVariable) scs.getVariable("q_qy")).set(q_qy);
      ((DoubleYoVariable) scs.getVariable("q_qz")).set(q_qz);

      int buttonChannel = 1;
      sliderBoard.setButton(buttonChannel++, isCaptureSnapshotRequested);
      sliderBoard.setButton(buttonChannel++, isSaveSequenceRequested);
      sliderBoard.setButton(buttonChannel++, isLoadSequenceRequested);
      sliderBoard.setButton(buttonChannel++, isClearSequenceRequested);
      buttonChannel = 6;
      sliderBoard.setButton(buttonChannel++, isLoadLastSequenceRequested);
      sliderBoard.setButton(buttonChannel++, isLoadFrameByFrameSequenceRequested);
      sliderBoard.setButton(buttonChannel++, isPlayPoseFromFrameByFrameSequenceRequested);

      buttonChannel = 9;
      sliderBoard.setButton(buttonChannel++, isPelvisControlRequested);
      sliderBoard.setButton(buttonChannel++, isChestControlRequested);
      sliderBoard.setButton(buttonChannel++, isSymmetricModeRequested);
      sliderBoard.setButton(buttonChannel++, isResetToBasePoseRequested);
      sliderBoard.setButton(buttonChannel++, isSupportBaseTargetToggleRequested);
      sliderBoard.setButton(buttonChannel++, isSupportBaseControlTargetRequested);
      sliderBoard.setButton(buttonChannel++, isSupportBaseToggleRequested);
      sliderBoard.setButton(buttonChannel++, isSupportBaseControlRequested);

      buttonChannel = 17;
      sliderBoard.setButton(buttonChannel++, isLeftArmControlRequested);
      sliderBoard.setButton(buttonChannel++, isRightArmControlRequested);
      sliderBoard.setButton(buttonChannel++, isLeftLegControlRequested);
      sliderBoard.setButton(buttonChannel++, isRightLegControlRequested);      
      
      setUpFingerKnobs();

   }

   private void setUpFingerKnobs()
   {
      int knobIndex=1;
      for(RobotSide robotSide : RobotSide.values)
      {
         for (int f = 0; f <= 3; f++)
         {
            for (int j = 1; j <= 2; j++)
            {
               sliderBoard.setKnob(knobIndex++, handSideString.get(robotSide) + f + "_j" + j , scs, -fingerJointLimit, fingerJointLimit);
            }
         }
      }
      
      for(RobotSide robotSide : RobotSide.values)
      {
         int j=0;
         for (int f = 0; f <= 3; f++)
         {
               sliderBoard.setKnob(knobIndex++, handSideString.get(robotSide) + f + "_j" + j , scs, -fingerJointLimit, fingerJointLimit);
         }
         knobIndex=25;
      }
   }

   private void setupSymmetricModeListeners()
   {
      //main joints
      for(int thisSideIndex=0;thisSideIndex<ROSAtlasJointMap.numberOfJoints;thisSideIndex++)
      {
         int otherSideIndex=ROSAtlasJointMapCorrelation.oppositeSideIndex[thisSideIndex];
         if( thisSideIndex !=  otherSideIndex)
         {
            YoVariable thisVariable         = scs.getVariable("q_" + ROSAtlasJointMap.jointNames[thisSideIndex]);
            YoVariable oppositeSideVariable = scs.getVariable("q_" + ROSAtlasJointMap.jointNames[otherSideIndex]);
            boolean setToOppositeSign = ROSAtlasJointMapCorrelation.symmetricSignChange[thisSideIndex];
            RobotSide robotSide;
            if(ROSAtlasJointMap.jointNames[thisSideIndex].startsWith("l_"))
               robotSide=RobotSide.LEFT;
            else
               robotSide=RobotSide.RIGHT;               
            SymmetricModeListener symmetricModeListener = new SymmetricModeListener((DoubleYoVariable) oppositeSideVariable, robotSide, setToOppositeSign);
            thisVariable.addVariableChangedListener(symmetricModeListener);            
         }
      }
      
      //fingers
      for (RobotSide robotSide : RobotSide.values)
      {
         String thisSidePrefix     = handSideString.get(robotSide);
         String oppositeSidePrefix = handSideString.get(robotSide.getOppositeSide());
         boolean setToOppositeSign = false;         
         for (int f = 0; f <= 3; f++)
         {
            for (int j = 0; j <= 2; j++)
            {
               YoVariable thisVariable         = scs.getVariable(thisSidePrefix     + f + "_j" + j);
               YoVariable oppositeSideVariable = scs.getVariable(oppositeSidePrefix + f + "_j" + j);
               SymmetricModeListener symmetricModeListener = new SymmetricModeListener((DoubleYoVariable) oppositeSideVariable, robotSide, setToOppositeSign);
               thisVariable.addVariableChangedListener(symmetricModeListener);
            }
         }
      }
      
      //TODO setup listeners to move the targets when in cartesian mode. currently just changing joint listeners is unreliable.
      //TODO do joint symmetry in joint mode and cartesian symmetry about global XZ plane in cartesian mode.

   }

   public void setupSliderForLegs(RobotSide robotSide)
   {
      //      resetSliderBoard();
      symmetricControlSide = robotSide;

      int sliderChannel = 1;
      String prefix = sideString.get(robotSide);

      switch (sliderSpace.getEnumValue())
      {
      case JOINT:
      {
         for (LegJointName legJointName : legJointStringNames.keySet())
         {
            Double min = legJointLowerLimits.get(robotSide).get(legJointName);
            Double max = legJointUpperLimits.get(robotSide).get(legJointName);
            sliderBoard.setSlider(sliderChannel++, prefix + legJointStringNames.get(legJointName), scs, min, max);
         }
         break;
      }
      case CARTESIAN:
      {
         placeCartesianTargetsAtActuals();

         YoFramePose yoFramePose = feetIKs.get(robotSide);

         FramePoint footPosition = new FramePoint(fullRobotModel.getFoot(robotSide).getBodyFixedFrame());
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());

         double xyRange = 1.0;
         double zRange = 2.0;

         sliderBoard.setSlider(sliderChannel++, yoFramePose.getPosition().getYoX(), footPosition.getX() - xyRange / 2.0, footPosition.getX() + xyRange / 2.0);
         sliderBoard.setSlider(sliderChannel++, yoFramePose.getPosition().getYoY(), footPosition.getY() - xyRange / 2.0, footPosition.getY() + xyRange / 2.0);
         sliderBoard.setSlider(sliderChannel++, yoFramePose.getPosition().getYoZ(), footPosition.getZ() - zRange / 2.0, footPosition.getZ() + zRange / 2.0);

         sliderBoard.setSlider(sliderChannel++, yoFramePose.getOrientation().getYaw(), -Math.PI, Math.PI);
         sliderBoard.setSlider(sliderChannel++, yoFramePose.getOrientation().getPitch(), -Math.PI, Math.PI);
         sliderBoard.setSlider(sliderChannel++, yoFramePose.getOrientation().getRoll(), -Math.PI, Math.PI);

         break;
      }
      }

   }

   private void setupSliderForArms(RobotSide robotSide)
   {
      symmetricControlSide = robotSide;

      int sliderChannel = 1;
      String prefix = sideString.get(robotSide);

      switch (sliderSpace.getEnumValue())
      {
      case JOINT:
      {
         for (ArmJointName armJointName : armJointStringNames.keySet())
         {
            Double min = armJointLowerLimits.get(robotSide).get(armJointName);
            Double max = armJointUpperLimits.get(robotSide).get(armJointName);
            sliderBoard.setSlider(sliderChannel++, prefix + armJointStringNames.get(armJointName), scs, min, max);
         }

         sliderBoard.setSlider(sliderChannel++, q_hands.get(robotSide), -fingerJointLimit, fingerJointLimit);//all fingers open/close
         sliderBoard.setSlider(sliderChannel++, handSideString.get(robotSide) + "3_j0", scs, -fingerJointLimit, fingerJointLimit);//thumb back and forth

         break;
      }
      case CARTESIAN:
      {
         placeCartesianTargetsAtActuals();

         YoFramePose yoFramePose = handIKs.get(robotSide);

         FramePoint handPosition = new FramePoint(fullRobotModel.getHand(robotSide).getBodyFixedFrame());
         handPosition.changeFrame(ReferenceFrame.getWorldFrame());

         double xyRange = 1.0;
         double zRange = 1.0;
         sliderBoard.setSlider(sliderChannel++, yoFramePose.getPosition().getYoX(), handPosition.getX() - xyRange / 2.0, handPosition.getX() + xyRange / 2.0);
         sliderBoard.setSlider(sliderChannel++, yoFramePose.getPosition().getYoY(), handPosition.getY() - xyRange / 2.0, handPosition.getY() + xyRange / 2.0);
         sliderBoard.setSlider(sliderChannel++, yoFramePose.getPosition().getYoZ(), handPosition.getZ() - zRange / 2.0, handPosition.getZ() + zRange / 2.0);

         sliderBoard.setSlider(sliderChannel++, yoFramePose.getOrientation().getYaw(), -Math.PI, Math.PI);
         sliderBoard.setSlider(sliderChannel++, yoFramePose.getOrientation().getPitch(), -Math.PI, Math.PI);
         sliderBoard.setSlider(sliderChannel++, yoFramePose.getOrientation().getRoll(), -Math.PI, Math.PI);

         break;
      }
      }

   }

   private void setupSliderForPelvis()
   {
      int sliderChannel = 1;
      double angle_min = -Math.PI;
      double angle_max = +Math.PI;

      FramePoint pelvisPosition = new FramePoint(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double xyRange = 2.0;
      double zRange = 2.0;

      sliderBoard.setSlider(sliderChannel++, "q_x", scs, pelvisPosition.getX() - xyRange / 2.0, pelvisPosition.getX() + xyRange / 2.0);
      sliderBoard.setSlider(sliderChannel++, "q_y", scs, pelvisPosition.getY() - xyRange / 2.0, pelvisPosition.getY() + xyRange / 2.0);
      sliderBoard.setSlider(sliderChannel++, "q_z", scs, pelvisPosition.getZ() - zRange / 2.0, pelvisPosition.getZ() + zRange / 2.0);

      //reset yaw pitch and roll so that it can be based off current 'global' yaw pitch and roll.
      qprev = new Quat4d(q_qx.getDoubleValue(), q_qy.getDoubleValue(), q_qz.getDoubleValue(), q_qs.getDoubleValue());
      q_yaw.set(0.0);
      q_pitch.set(0.0);
      q_roll.set(0.0);
      sliderBoard.setSlider(sliderChannel++, "q_yaw", scs, angle_min, angle_max);
      sliderBoard.setSlider(sliderChannel++, "q_pitch", scs, angle_min, angle_max);
      sliderBoard.setSlider(sliderChannel++, "q_roll", scs, angle_min, angle_max);

   }

   private void setupSliderForChest()
   {
      int sliderChannel = 1;

      for (SpineJointName spineJointName : spineJointStringNames.keySet())
         sliderBoard.setSlider(sliderChannel++, spineJointStringNames.get(spineJointName), scs, spineJointLowerLimits.get(spineJointName),
               spineJointUpperLimits.get(spineJointName));
      //neck
      sliderBoard.setSlider(sliderChannel++, "q_neck_ay", scs, -0.610865, 1.134460);
   }

   private void setupSlidersForSupportBaseControl()
   {
      int sliderChannel = 1;

      for (int i = 0; i < baseControlPoints.length; i++)
      {
         sliderBoard.setSlider(sliderChannel++, "baseControlPoint" + i + "X", scs, baseControlPoints[i].getX() - 2, baseControlPoints[i].getX() + 2);
         sliderBoard.setSlider(sliderChannel++, "baseControlPoint" + i + "Y", scs, baseControlPoints[i].getY() - 2, baseControlPoints[i].getY() + 2);
      }
   }

   private void setupSlidersForSupportBaseTargetControl()
   {
      int sliderChannel = 1;

      for (int i = 0; i < baseControlTargetPoints.length; i++)
      {
         sliderBoard.setSlider(sliderChannel++, "baseControlTargetPoint" + i + "X", scs, baseControlTargetPoints[i].getX() - 2, baseControlTargetPoints[i].getX() + 2);
         sliderBoard.setSlider(sliderChannel++, "baseControlTargetPoint" + i + "Y", scs, baseControlTargetPoints[i].getY() - 2, baseControlTargetPoints[i].getY() + 2);
      }
   }

   private void placeCartesianTargetsAtActuals()
   {
      reader.read();
      referenceFrames.updateFrames();

      for (RobotSide robotSide : RobotSide.values())
      {
         YoFramePose yoFramePose = feetIKs.get(robotSide);
         FramePose framePose = new FramePose(fullRobotModel.getFoot(robotSide).getBodyFixedFrame());
         framePose.changeFrame(ReferenceFrame.getWorldFrame());
         yoFramePose.set(framePose);

         yoFramePose = handIKs.get(robotSide);
         framePose = new FramePose(fullRobotModel.getHand(robotSide).getBodyFixedFrame());
         framePose.changeFrame(ReferenceFrame.getWorldFrame());
         yoFramePose.set(framePose);
      }

      ReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FramePose framePose = new FramePose(pelvisFrame);
      framePose.changeFrame(ReferenceFrame.getWorldFrame());

      System.out.println("Pelvis is at " + framePose);
   }

   private class ChangeCurrentPartBeingControlledListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable v)
      {
         if (!(v instanceof BooleanYoVariable))
            return;

         for (RobotSide robotSide : RobotSide.values)
         {
            if (v.equals(isLegControlRequested.get(robotSide)))
            {
               setupSliderForLegs(robotSide);
               sliderBodyPart.set(legBodyParts.get(robotSide));
            }

            if (v.equals(isArmControlRequested.get(robotSide)))
            {
               setupSliderForArms(robotSide);
               sliderBodyPart.set(armBodyParts.get(robotSide));
            }
         }

         if (v.equals(isPelvisControlRequested))
         {
            setupSliderForPelvis();
            sliderBodyPart.set(SliderBodyPart.PELVIS);
         }

         if (v.equals(isChestControlRequested))
         {
            setupSliderForChest();
            sliderBodyPart.set(SliderBodyPart.CHEST);
         }
      }
   }

   private class SupportBaseControlRequestedListener implements VariableChangedListener
   {
      private int displayState = 0;

      public void variableChanged(YoVariable v)
      {
         setupSlidersForSupportBaseControl();
         if(v.equals(isSupportBaseToggleRequested))
            toggleBaseControlPointDisplay();              
      }

      private void toggleBaseControlPointDisplay()
      {
         displayState++;
         if (displayState == 6)
            displayState = 0;
         switch (displayState)
         {
         case 0:
            for (DynamicGraphicObject baseControlPoint : baseControlPointsList)
            {
               baseControlPoint.hideGraphicObject();
            }
            for (DynamicGraphicObject baseControlLine : baseControlLinesList)
            {
               baseControlLine.hideGraphicObject();
            }
            break;

         case 1:
            for (DynamicGraphicObject baseControlPoint : baseControlPointsList)
            {
               baseControlPoint.showGraphicObject();
            }
            for (DynamicGraphicObject baseControlLine : baseControlLinesList)
            {
               baseControlLine.showGraphicObject();
            }
            break;

         case 2:
            baseControlPointsList.get(0).hideGraphicObject();

            baseControlLinesList.get(0).hideGraphicObject();
            baseControlLinesList.get(1).hideGraphicObject();
            baseControlLinesList.get(2).hideGraphicObject();
            break;

         case 3:
            baseControlPointsList.get(0).showGraphicObject();
            baseControlPointsList.get(1).hideGraphicObject();

            baseControlLinesList.get(1).showGraphicObject();
            baseControlLinesList.get(2).showGraphicObject();
            baseControlLinesList.get(3).hideGraphicObject();
            baseControlLinesList.get(4).hideGraphicObject();
            break;

         case 4:
            baseControlPointsList.get(1).showGraphicObject();
            baseControlPointsList.get(2).hideGraphicObject();

            baseControlLinesList.get(0).showGraphicObject();
            baseControlLinesList.get(4).showGraphicObject();
            baseControlLinesList.get(1).hideGraphicObject();
            baseControlLinesList.get(5).hideGraphicObject();
            break;

         default:
            baseControlPointsList.get(2).showGraphicObject();
            baseControlPointsList.get(3).hideGraphicObject();

            baseControlLinesList.get(1).showGraphicObject();
            baseControlLinesList.get(3).showGraphicObject();
            baseControlLinesList.get(2).hideGraphicObject();
            baseControlLinesList.get(4).hideGraphicObject();
         }
      }
   }

   private class SupportBaseControlTargetRequestedListener implements VariableChangedListener
   {
      private int displayState = 0;

      public void variableChanged(YoVariable v)
      {
         setupSlidersForSupportBaseTargetControl();
         if(v.equals(isSupportBaseTargetToggleRequested))
            toggleBaseControlTargetPointDisplay();              
      }

      private void toggleBaseControlTargetPointDisplay()
      {
         displayState++;
         if (displayState == 6)
            displayState = 0;
         switch (displayState)
         {
         case 0:
            for (DynamicGraphicObject baseControlTargetPoint : baseControlTargetPointsList)
            {
               baseControlTargetPoint.hideGraphicObject();
            }
            for (DynamicGraphicObject baseControlTargetLine : baseControlTargetLinesList)
            {
               baseControlTargetLine.hideGraphicObject();
            }
            break;

         case 1:
            for (DynamicGraphicObject baseControlTargetPoint : baseControlTargetPointsList)
            {
               baseControlTargetPoint.showGraphicObject();
            }
            for (DynamicGraphicObject baseControlTargetLine : baseControlTargetLinesList)
            {
               baseControlTargetLine.showGraphicObject();
            }
            break;

         case 2:
            baseControlTargetPointsList.get(0).hideGraphicObject();

            baseControlTargetLinesList.get(0).hideGraphicObject();
            baseControlTargetLinesList.get(1).hideGraphicObject();
            baseControlTargetLinesList.get(2).hideGraphicObject();
            break;

         case 3:
            baseControlTargetPointsList.get(0).showGraphicObject();
            baseControlTargetPointsList.get(1).hideGraphicObject();

            baseControlTargetLinesList.get(1).showGraphicObject();
            baseControlTargetLinesList.get(2).showGraphicObject();
            baseControlTargetLinesList.get(3).hideGraphicObject();
            baseControlTargetLinesList.get(4).hideGraphicObject();
            break;

         case 4:
            baseControlTargetPointsList.get(1).showGraphicObject();
            baseControlTargetPointsList.get(2).hideGraphicObject();

            baseControlTargetLinesList.get(0).showGraphicObject();
            baseControlTargetLinesList.get(4).showGraphicObject();
            baseControlTargetLinesList.get(1).hideGraphicObject();
            baseControlTargetLinesList.get(5).hideGraphicObject();
            break;

         default:
            baseControlTargetPointsList.get(2).showGraphicObject();
            baseControlTargetPointsList.get(3).hideGraphicObject();

            baseControlTargetLinesList.get(1).showGraphicObject();
            baseControlTargetLinesList.get(3).showGraphicObject();
            baseControlTargetLinesList.get(2).hideGraphicObject();
            baseControlTargetLinesList.get(4).hideGraphicObject();
         }
      }
   }

   private class UpdateInverseKinematicsListener implements VariableChangedListener
   {
      private final Transform3D desiredTransform = new Transform3D();

      public void variableChanged(YoVariable v)
      {
         if (legInverseKinematicsCalculators == null)
            return;

         reader.read();
         referenceFrames.updateFrames();
         sdfRobot.update();

         if (sliderSpace.getEnumValue() == SliderSpace.CARTESIAN)
         {
            for (RobotSide robotSide : RobotSide.values())
            {
               YoFramePose footIK = feetIKs.get(robotSide);
               FramePoint position = footIK.getPosition().getFramePointCopy();
               FrameOrientation orientation = footIK.getOrientation().getFrameOrientationCopy();
               FramePose framePose = new FramePose(position, orientation);
               framePose.changeFrame(fullRobotModel.getPelvis().getBodyFixedFrame());
               framePose.getTransformFromPoseToFrame(desiredTransform);
               legInverseKinematicsCalculators.get(robotSide).solve(desiredTransform);

               YoFramePose handIK = handIKs.get(robotSide);
               position = handIK.getPosition().getFramePointCopy();
               orientation = handIK.getOrientation().getFrameOrientationCopy();
               framePose = new FramePose(position, orientation);
               framePose.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
               framePose.getTransformFromPoseToFrame(desiredTransform);
               armInverseKinematicsCalculators.get(robotSide).solve(desiredTransform);
            }

            writer.updateRobotConfigurationBasedOnFullRobotModel();
         }
      }

   }

   private class SymmetricModeListener implements VariableChangedListener
   {
      private final DoubleYoVariable variableToSet;
      private final RobotSide robotSide;
      private final double respectiveSign;

      public SymmetricModeListener(DoubleYoVariable variableToSet, RobotSide robotSide, boolean setToOppositeSign)
      {
         this.variableToSet = variableToSet;
         this.robotSide = robotSide;
         this.respectiveSign = setToOppositeSign ? -1.0 : 1.0;
      }

      public void variableChanged(YoVariable yoVariable)
      {
         if (symmetricMode && (robotSide == symmetricControlSide))
         {
            variableToSet.set(respectiveSign * ((DoubleYoVariable) yoVariable).getDoubleValue());
         }
      }
   }

   private class PelvisRotationListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable v)
      {
         if (!(v instanceof DoubleYoVariable))
            return;

         if (v.equals(q_yaw) || v.equals(q_pitch) || v.equals(q_roll))
            setYawPitchRoll();
      }
   }

   private class HandListener implements VariableChangedListener
   {
      private final RobotSide robotSide;

      HandListener(RobotSide handSide)
      {
         robotSide = handSide;
      }

      public void variableChanged(YoVariable v)
      {
         double q_val = q_hands.get(robotSide).getDoubleValue();
         for (int f = 0; f <= 3; f++)
         {
            for (int j = 1; j <= 2; j++)
            {
               ((DoubleYoVariable) scs.getVariable(handSideString.get(robotSide) + f + "_j" + j)).set(q_val);
            }
         }
      }
   }

   private void setYawPitchRoll()
   {
      Quat4d q = new Quat4d();

      //This code has a singularity when yaw and roll line up (e.g. pitch is 90, can't rotate in one direction any more).
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(q, q_yaw.getDoubleValue(), q_pitch.getDoubleValue(), q_roll.getDoubleValue());

      //This code compounds the rotations so that on subsequent frames the ability to rotate in lost rotation directions is regained
      //This affectively uses global yaw pitch and roll each time.
      q.mul(qprev);

      q_qs.set(q.w);
      q_qx.set(q.x);
      q_qy.set(q.y);
      q_qz.set(q.z);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private final SideDependentList<SliderBodyPart> legBodyParts = new SideDependentList<DRCRobotMidiSliderBoardPositionManipulation.SliderBodyPart>(
         SliderBodyPart.LEFT_LEG, SliderBodyPart.RIGHT_LEG);
   private final SideDependentList<SliderBodyPart> armBodyParts = new SideDependentList<DRCRobotMidiSliderBoardPositionManipulation.SliderBodyPart>(
         SliderBodyPart.LEFT_ARM, SliderBodyPart.RIGHT_ARM);

   private SideDependentList<NumericalInverseKinematicsCalculator> legInverseKinematicsCalculators, armInverseKinematicsCalculators;

   private YoFramePose leftFootIK, rightFootIK, leftHandIK, rightHandIK;
   private SideDependentList<YoFramePose> feetIKs, handIKs;

   private SDFRobot sdfRobot;
   private ReferenceFrames referenceFrames;
   private SDFFullRobotModel fullRobotModel;
   private SDFPerfectSimulatedSensorReader reader;
   private SDFPerfectSimulatedOutputWriter writer;

   private enum SliderBodyPart
   {
      LEFT_LEG, RIGHT_LEG, LEFT_ARM, RIGHT_ARM, PELVIS, CHEST;
   }

   private enum SliderSpace
   {
      JOINT, CARTESIAN
   }

   private void setupForInverseKinematics(SDFRobot sdfRobot, ReferenceFrames referenceFrames, SDFFullRobotModel fullRobotModel,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      if (this.sdfRobot != null)
         throw new RuntimeException("Already set up for Inverse Kinematics");

      this.sdfRobot = sdfRobot;
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      this.reader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, referenceFrames);
      this.writer = new SDFPerfectSimulatedOutputWriter(sdfRobot, fullRobotModel);

      GeometricJacobian leftFootJacobian = new GeometricJacobian(fullRobotModel.getPelvis(), fullRobotModel.getFoot(RobotSide.LEFT), fullRobotModel.getFoot(
            RobotSide.LEFT).getBodyFixedFrame());
      GeometricJacobian rightFootJacobian = new GeometricJacobian(fullRobotModel.getPelvis(), fullRobotModel.getFoot(RobotSide.RIGHT), fullRobotModel.getFoot(
            RobotSide.RIGHT).getBodyFixedFrame());
      GeometricJacobian leftHandJacobian = new GeometricJacobian(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT), fullRobotModel.getHand(
            RobotSide.LEFT).getBodyFixedFrame());
      GeometricJacobian rightHandJacobian = new GeometricJacobian(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.RIGHT), fullRobotModel.getHand(
            RobotSide.RIGHT).getBodyFixedFrame());

      double tolerance = 1e-8;
      double maxStepSize = 1.0;
      double minRandomSearchScalar = -0.5;
      double maxRandomSearchScalar = 1.0;
      int maxIterations = 10;

      NumericalInverseKinematicsCalculator leftLegInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(leftFootJacobian, tolerance,
            maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
      NumericalInverseKinematicsCalculator rightLegInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(rightFootJacobian, tolerance,
            maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
      NumericalInverseKinematicsCalculator leftArmInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(leftHandJacobian, tolerance,
            maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
      NumericalInverseKinematicsCalculator rightArmInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(rightHandJacobian, tolerance,
            maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);

      legInverseKinematicsCalculators = new SideDependentList<NumericalInverseKinematicsCalculator>(leftLegInverseKinematicsCalculator,
            rightLegInverseKinematicsCalculator);
      armInverseKinematicsCalculators = new SideDependentList<NumericalInverseKinematicsCalculator>(leftArmInverseKinematicsCalculator,
            rightArmInverseKinematicsCalculator);

      leftFootIK = new YoFramePose("leftFootIK", "", ReferenceFrames.getWorldFrame(), registry);
      rightFootIK = new YoFramePose("rightFootIK", "", ReferenceFrames.getWorldFrame(), registry);

      feetIKs = new SideDependentList<YoFramePose>(leftFootIK, rightFootIK);

      leftHandIK = new YoFramePose("leftHandIK", "", ReferenceFrames.getWorldFrame(), registry);
      rightHandIK = new YoFramePose("rightHandIK", "", ReferenceFrames.getWorldFrame(), registry);

      handIKs = new SideDependentList<YoFramePose>(leftHandIK, rightHandIK);

      double scale = 0.2;
      DynamicGraphicCoordinateSystem leftFootViz = new DynamicGraphicCoordinateSystem("leftFootViz", leftFootIK, scale);
      DynamicGraphicCoordinateSystem rightFootViz = new DynamicGraphicCoordinateSystem("rightFootViz", rightFootIK, scale);

      DynamicGraphicCoordinateSystem leftHandViz = new DynamicGraphicCoordinateSystem("leftHandViz", leftHandIK, scale);
      DynamicGraphicCoordinateSystem rightHandViz = new DynamicGraphicCoordinateSystem("rightHandViz", rightHandIK, scale);

      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("InverseKinematics");
      dynamicGraphicObjectsList.add(leftFootViz);
      dynamicGraphicObjectsList.add(rightFootViz);
      dynamicGraphicObjectsList.add(leftHandViz);
      dynamicGraphicObjectsList.add(rightHandViz);

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
   }

   private void addSupportBaseControlGraphics(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      addSupportBaseGraphics(dynamicGraphicObjectsListRegistry,baseControlPoints,baseControlPointsList,baseControlLinesList,"baseControl",YoAppearance.Green());

   }

   private void addSupportBaseControlTargetGraphics(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      addSupportBaseGraphics(dynamicGraphicObjectsListRegistry,baseControlTargetPoints,baseControlTargetPointsList,baseControlTargetLinesList,"baseControlTarget",YoAppearance.Red());
   }

   private void addSupportBaseGraphics(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,YoFramePoint[] basePoints, ArrayList<DynamicGraphicObject> basePointsList, ArrayList<DynamicGraphicObject> linesList, String namePrefix,AppearanceDefinition appearance)
   {
      AppearanceDefinition[] colors = { YoAppearance.Red(), YoAppearance.Green(), YoAppearance.Blue(), YoAppearance.Yellow() };
      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList(namePrefix + "Points");
      for (int i = 0; i < basePoints.length; i++)
      {
         DynamicGraphicPosition baseControlPointViz = new DynamicGraphicPosition(namePrefix + "Point" + i, basePoints[i], 0.01, colors[i]);
         dynamicGraphicObjectsList.add(baseControlPointViz);
         basePointsList.add(baseControlPointViz);
         
         for (int j = i + 1; j < basePoints.length; j++)
         {
            DynamicGraphicLineSegment dynamicGraphicLineSegment = new DynamicGraphicLineSegment(namePrefix + "SupportLine", basePoints[i], basePoints[j],
                  1.0, appearance, false);
            dynamicGraphicObjectsList.add(dynamicGraphicLineSegment);
            linesList.add(dynamicGraphicLineSegment);
         }
      }

      if (dynamicGraphicObjectsListRegistry != null)
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
      dynamicGraphicObjectsList.hideDynamicGraphicObjects();
   }

}
