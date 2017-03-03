package us.ihmc.avatar.posePlayback;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.MidiSliderBoard;

public class DRCRobotMidiSliderBoardPositionManipulation
{
   private final ArrayList<OneDegreeOfFreedomJoint> allSCSJoints = new ArrayList<>();

   private final ArrayList<NeckJointName> neckJointNames = new ArrayList<>();
   private final EnumMap<NeckJointName, OneDegreeOfFreedomJoint> neckSCSJoints = new EnumMap<>(NeckJointName.class);
   
   private final ArrayList<SpineJointName> spineJointNames = new ArrayList<>();
   private final EnumMap<SpineJointName, OneDegreeOfFreedomJoint> spineSCSJoints = new EnumMap<SpineJointName, OneDegreeOfFreedomJoint>(SpineJointName.class);

   private final SideDependentList<ArrayList<LegJointName>> legJointNames = new SideDependentList<>(new ArrayList<LegJointName>(), new ArrayList<LegJointName>());
   private final SideDependentList<EnumMap<LegJointName, OneDegreeOfFreedomJoint>> legSCSJoints = SideDependentList.createListOfEnumMaps(LegJointName.class);
   
   private final SideDependentList<ArrayList<ArmJointName>> armJointNames = new SideDependentList<>(new ArrayList<ArmJointName>(), new ArrayList<ArmJointName>());
   private final SideDependentList<EnumMap<ArmJointName, OneDegreeOfFreedomJoint>> armSCSJoints = SideDependentList.createListOfEnumMaps(ArmJointName.class);
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
   private final BooleanYoVariable isPlayPoseFromFrameByFrameSequenceRequested = new BooleanYoVariable("isPlayPoseFromFrameByFrameSequenceRequested", dontRecordRegistry);
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

   private final SideDependentList<BooleanYoVariable> isLegControlRequested = new SideDependentList<BooleanYoVariable>(isLeftLegControlRequested, isRightLegControlRequested);
   private final SideDependentList<BooleanYoVariable> isArmControlRequested = new SideDependentList<BooleanYoVariable>(isLeftArmControlRequested, isRightArmControlRequested);

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
   private final DoubleYoVariable q_qs, q_qx, q_qy, q_qz, q_x, q_y, q_z;
   private Quaternion qprev;

   //   private final DoubleYoVariable BaseControlPoint = new DoubleYoVariable("BaseControlPoint", registry);
   private final YoFramePoint[] baseControlPoints = new YoFramePoint[4];
   private final ArrayList<YoGraphic> baseControlPointsList = new ArrayList<YoGraphic>();
   private final ArrayList<YoGraphic> baseControlLinesList = new ArrayList<YoGraphic>();

   private final YoFramePoint[] baseControlTargetPoints = new YoFramePoint[4];
   private final ArrayList<YoGraphic> baseControlTargetPointsList = new ArrayList<YoGraphic>();
   private final ArrayList<YoGraphic> baseControlTargetLinesList = new ArrayList<YoGraphic>();

   private final SideDependentList<SliderBodyPart> legBodyParts = new SideDependentList<DRCRobotMidiSliderBoardPositionManipulation.SliderBodyPart>(SliderBodyPart.LEFT_LEG, SliderBodyPart.RIGHT_LEG);
   private final SideDependentList<SliderBodyPart> armBodyParts = new SideDependentList<DRCRobotMidiSliderBoardPositionManipulation.SliderBodyPart>(SliderBodyPart.LEFT_ARM, SliderBodyPart.RIGHT_ARM);

   private final SideDependentList<NumericalInverseKinematicsCalculator> legInverseKinematicsCalculators = new SideDependentList<NumericalInverseKinematicsCalculator>();
   private final SideDependentList<NumericalInverseKinematicsCalculator> armInverseKinematicsCalculators = new SideDependentList<NumericalInverseKinematicsCalculator>();

   private final SideDependentList<YoFramePose> feetIKs = new SideDependentList<>();
   private final SideDependentList<YoFramePose> handIKs = new SideDependentList<>();

   private final FloatingRootJointRobot sdfRobot;
   private final FullHumanoidRobotModel fullRobotModel;
   private final SDFPerfectSimulatedSensorReader reader;
   private final PerfectSimulatedOutputWriter writer;

   private enum SliderBodyPart
   {
      LEFT_LEG, RIGHT_LEG, LEFT_ARM, RIGHT_ARM, PELVIS, CHEST;
   }

   private enum SliderSpace
   {
      JOINT, CARTESIAN
   }
   
   private final BooleanYoVariable controlFingers = new BooleanYoVariable("controlFingers", registry);

   public DRCRobotMidiSliderBoardPositionManipulation(SimulationConstructionSet scs, FloatingRootJointRobot sdfRobot, FullHumanoidRobotModel fullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(false, scs, sdfRobot, fullRobotModel, yoGraphicsListRegistry);
   }
   
   public DRCRobotMidiSliderBoardPositionManipulation(boolean controlFingers, SimulationConstructionSet scs, FloatingRootJointRobot sdfRobot, FullHumanoidRobotModel fullRobotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.scs = scs;
      this.fullRobotModel = fullRobotModel;
      this.sdfRobot = sdfRobot;
      this.reader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, null);
      this.writer = new PerfectSimulatedOutputWriter(sdfRobot, fullRobotModel);

      this.controlFingers.set(controlFingers);
      
      sdfRobot.getAllOneDegreeOfFreedomJoints(allSCSJoints);
      

      sliderSpace.set(SliderSpace.JOINT);
      sliderBodyPart.set(SliderBodyPart.LEFT_LEG);

      scs.addYoVariableRegistry(registry);
      sliderBoard = new MidiSliderBoard(scs);

      UpdateInverseKinematicsListener updateInverseKinematicsListener = new UpdateInverseKinematicsListener();
      sliderBoard.attachVariableChangedListener(updateInverseKinematicsListener);

      FloatingJoint rootJoint = sdfRobot.getRootJoint();
      q_x = rootJoint.getQx();
      q_y = rootJoint.getQy();
      q_z = rootJoint.getQz();
      q_qs = rootJoint.getQuaternionQs();
      q_qx = rootJoint.getQuaternionQx();
      q_qy = rootJoint.getQuaternionQy();
      q_qz = rootJoint.getQuaternionQz();

      init();

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

      if (this.controlFingers.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            q_hands.get(robotSide).addVariableChangedListener(new HandListener(robotSide));
         }
      }

      isLeftLegControlRequested.set(true);

      isSymmetricModeRequested.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> yoVariable)
         {
            symmetricMode = isSymmetricModeRequested.getBooleanValue();
         }
      });
      
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();
      
      double lambdaLeastSquares = 0.0009;
      double tolerance = 1e-8;
      double maxStepSize = 1.0;
      double minRandomSearchScalar = -0.5;
      double maxRandomSearchScalar = 1.0;
      int maxIterations = 10;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         RigidBody hand = fullRobotModel.getHand(robotSide);
         
         GeometricJacobian legJacobian = new GeometricJacobian(pelvis, foot, foot.getBodyFixedFrame());
         GeometricJacobian armJacobian = new GeometricJacobian(chest, hand, hand.getBodyFixedFrame());
         
         NumericalInverseKinematicsCalculator legInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(legJacobian, lambdaLeastSquares, tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
         NumericalInverseKinematicsCalculator armInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(armJacobian, lambdaLeastSquares, tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
         
         legInverseKinematicsCalculators.put(robotSide, legInverseKinematicsCalculator);
         armInverseKinematicsCalculators.put(robotSide, armInverseKinematicsCalculator);
      }

      String listName = "InverseKinematics";
      double scale = 0.2;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         YoFramePose footIK = new YoFramePose(sidePrefix + "FootIK", "", HumanoidReferenceFrames.getWorldFrame(), registry);
         feetIKs.put(robotSide, footIK);
         
         YoFramePose handIK = new YoFramePose(sidePrefix + "HandIK", "", HumanoidReferenceFrames.getWorldFrame(), registry);
         handIKs.put(robotSide, handIK);

         yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicCoordinateSystem(sidePrefix + "FootViz", footIK, scale));
         yoGraphicsListRegistry.registerYoGraphic(listName, new YoGraphicCoordinateSystem(sidePrefix + "HandViz", handIK, scale));
      }
      
      setupSymmetricModeListeners();

      setupSupportPolygonDisplayAndControl(yoGraphicsListRegistry);
   }

   private void setupSupportPolygonDisplayAndControl(YoGraphicsListRegistry yoGraphicsListRegistry)
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

      addSupportBaseControlGraphics(yoGraphicsListRegistry);
      addSupportBaseControlTargetGraphics(yoGraphicsListRegistry);
   }

   private void init()
   {
      for (NeckJointName jointName : NeckJointName.values)
      {
         OneDoFJoint neckJoint = fullRobotModel.getNeckJoint(jointName);
         if (neckJoint == null)
            continue;

         neckJointNames.add(jointName);
         neckSCSJoints.put(jointName, sdfRobot.getOneDegreeOfFreedomJoint(neckJoint.getName()));
      }

      for (SpineJointName jointName : SpineJointName.values)
      {
         OneDoFJoint spineJoint = fullRobotModel.getSpineJoint(jointName);
         if (spineJoint == null)
            continue;

         spineJointNames.add(jointName);
         spineSCSJoints.put(jointName, sdfRobot.getOneDegreeOfFreedomJoint(spineJoint.getName()));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName jointName : LegJointName.values)
         {
            OneDoFJoint legJoint = fullRobotModel.getLegJoint(robotSide, jointName);
            if (legJoint == null)
               continue;
            
            legJointNames.get(robotSide).add(jointName);
            legSCSJoints.get(robotSide).put(jointName, sdfRobot.getOneDegreeOfFreedomJoint(legJoint.getName()));
         }

         for (ArmJointName jointName : ArmJointName.values())
         {
            OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, jointName);
            if (armJoint == null)
               continue;
            
            armJointNames.get(robotSide).add(jointName);
            armSCSJoints.get(robotSide).put(jointName, sdfRobot.getOneDegreeOfFreedomJoint(armJoint.getName()));
         }
      }

      resetSliderBoard();
   }

   private void resetSliderBoard()
   {
      LinkedHashMap<OneDegreeOfFreedomJoint, Double> savedJointAngles = new LinkedHashMap<>();
      
      for (OneDegreeOfFreedomJoint joint : allSCSJoints)
         savedJointAngles.put(joint, joint.getQYoVariable().getDoubleValue());
      
      double temp_q_x = q_x.getDoubleValue();
      double temp_q_y = q_y.getDoubleValue();
      double temp_q_z = q_z.getDoubleValue();
      double temp_q_qs = q_qs.getDoubleValue();
      double temp_q_qx = q_qx.getDoubleValue();
      double temp_q_qy = q_qy.getDoubleValue();
      double temp_q_qz = q_qz.getDoubleValue();
      
      sliderBoard.reset();

      for (OneDegreeOfFreedomJoint joint : allSCSJoints)
         joint.setQ(savedJointAngles.get(joint));
      
      q_x.set(temp_q_x);
      q_y.set(temp_q_y);
      q_z.set(temp_q_z);
      q_qs.set(temp_q_qs);
      q_qx.set(temp_q_qx);
      q_qy.set(temp_q_qy);
      q_qz.set(temp_q_qz);

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
      
      if (controlFingers.getBooleanValue())
      {
         setUpFingerKnobs();
      }
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
      final Matrix3D oppositeSignForYawAndRollOnly = new Matrix3D();
      oppositeSignForYawAndRollOnly.setIdentity();
      oppositeSignForYawAndRollOnly.setM00(-1.0);
      oppositeSignForYawAndRollOnly.setM22(-1.0);
      
      final Vector3D unitVectorThisSide = new Vector3D();
      final Vector3D unitVectorOtherSide = new Vector3D();
      
      //main joints
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName jointName : armSCSJoints.get(robotSide).keySet())
         {
            OneDegreeOfFreedomJoint thisJoint = armSCSJoints.get(robotSide).get(jointName);
            OneDegreeOfFreedomJoint otherJoint = armSCSJoints.get(robotSide.getOppositeSide()).get(jointName);
            
            unitVectorThisSide.set(thisJoint.physics.getUnitVector());
            unitVectorOtherSide.set(otherJoint.physics.getUnitVector());
            
            oppositeSignForYawAndRollOnly.transform(unitVectorThisSide);
            double sign = unitVectorOtherSide.dot(unitVectorThisSide);

            SymmetricModeListener symmetricModeListener = new SymmetricModeListener(otherJoint.getQYoVariable(), robotSide, sign);
            thisJoint.getQYoVariable().addVariableChangedListener(symmetricModeListener);
         }

         for (LegJointName jointName : legSCSJoints.get(robotSide).keySet())
         {
            OneDegreeOfFreedomJoint thisJoint = legSCSJoints.get(robotSide).get(jointName);
            OneDegreeOfFreedomJoint otherJoint = legSCSJoints.get(robotSide.getOppositeSide()).get(jointName);
            
            unitVectorThisSide.set(thisJoint.physics.getUnitVector());
            unitVectorOtherSide.set(otherJoint.physics.getUnitVector());
            
            oppositeSignForYawAndRollOnly.transform(unitVectorThisSide);
            double sign = unitVectorOtherSide.dot(unitVectorThisSide);

            SymmetricModeListener symmetricModeListener = new SymmetricModeListener(otherJoint.getQYoVariable(), robotSide, sign);
            thisJoint.getQYoVariable().addVariableChangedListener(symmetricModeListener);
         }
      }
      
      //fingers
      if (controlFingers.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            String thisSidePrefix = handSideString.get(robotSide);
            String oppositeSidePrefix = handSideString.get(robotSide.getOppositeSide());
            
            for (int f = 0; f <= 3; f++)
            {
               for (int j = 0; j <= 2; j++)
               {
                  @SuppressWarnings("deprecation")
                  DoubleYoVariable thisVariable = (DoubleYoVariable) scs.getVariable(thisSidePrefix + f + "_j" + j);
                  @SuppressWarnings("deprecation")
                  DoubleYoVariable oppositeSideVariable = (DoubleYoVariable) scs.getVariable(oppositeSidePrefix + f + "_j" + j);
                  SymmetricModeListener symmetricModeListener = new SymmetricModeListener(oppositeSideVariable, robotSide, 1.0);
                  thisVariable.addVariableChangedListener(symmetricModeListener);
               }
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

      switch (sliderSpace.getEnumValue())
      {
      case JOINT:
      {
         for (LegJointName legJointName : legJointNames.get(robotSide))
         {
            OneDegreeOfFreedomJoint legSCSJoint = legSCSJoints.get(robotSide).get(legJointName);
            OneDoFJoint legIDJoint = fullRobotModel.getLegJoint(robotSide, legJointName);
            sliderBoard.setSlider(sliderChannel++, legSCSJoint.getQYoVariable(), legIDJoint.getJointLimitLower(), legIDJoint.getJointLimitUpper());
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

      switch (sliderSpace.getEnumValue())
      {
      case JOINT:
      {
         for (ArmJointName armJointName : armJointNames.get(robotSide))
         {
            OneDegreeOfFreedomJoint armSCSJoint = armSCSJoints.get(robotSide).get(armJointName);
            OneDoFJoint armIDJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
            sliderBoard.setSlider(sliderChannel++, armSCSJoint.getQYoVariable(), armIDJoint.getJointLimitLower(), armIDJoint.getJointLimitUpper());
         }

         if (controlFingers.getBooleanValue())
         {
            sliderBoard.setSlider(sliderChannel++, q_hands.get(robotSide), -fingerJointLimit, fingerJointLimit);//all fingers open/close
            sliderBoard.setSlider(sliderChannel++, handSideString.get(robotSide) + "3_j0", scs, -fingerJointLimit, fingerJointLimit);//thumb back and forth
         }
         
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
      qprev = new Quaternion(q_qx.getDoubleValue(), q_qy.getDoubleValue(), q_qz.getDoubleValue(), q_qs.getDoubleValue());
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

      for (SpineJointName spineJointName : spineJointNames)
      {
         OneDegreeOfFreedomJoint spineSCSJoint = spineSCSJoints.get(spineJointName);
         OneDoFJoint spineIDJoint = fullRobotModel.getSpineJoint(spineJointName);
         sliderBoard.setSlider(sliderChannel++, spineSCSJoint.getQYoVariable(), spineIDJoint.getJointLimitLower(), spineIDJoint.getJointLimitUpper());
      }

      for (NeckJointName neckJointName : neckJointNames)
      {
         OneDegreeOfFreedomJoint neckSCSJoint = neckSCSJoints.get(neckJointName);
         OneDoFJoint neckIDJoint = fullRobotModel.getNeckJoint(neckJointName);
         sliderBoard.setSlider(sliderChannel++, neckSCSJoint.getQYoVariable(), neckIDJoint.getJointLimitLower(), neckIDJoint.getJointLimitUpper());
      }
   }

   private void setupSlidersForSupportBaseControl()
   {
      int sliderChannel = 1;

      for (int i = 0; i < baseControlPoints.length; i++)
      {
         YoFramePoint baseControlPoint = baseControlPoints[i];
         sliderBoard.setSlider(sliderChannel++, baseControlPoint.getYoX(), baseControlPoint.getX() - 2, baseControlPoint.getX() + 2);
         sliderBoard.setSlider(sliderChannel++, baseControlPoint.getYoY(), baseControlPoint.getY() - 2, baseControlPoint.getY() + 2);
      }
   }

   private void setupSlidersForSupportBaseTargetControl()
   {
      int sliderChannel = 1;

      for (int i = 0; i < baseControlTargetPoints.length; i++)
      {
         YoFramePoint baseControlTargetPoint = baseControlTargetPoints[i];
         sliderBoard.setSlider(sliderChannel++, baseControlTargetPoint.getYoX(), baseControlTargetPoint.getX() - 2, baseControlTargetPoint.getX() + 2);
         sliderBoard.setSlider(sliderChannel++, baseControlTargetPoint.getYoY(), baseControlTargetPoint.getY() - 2, baseControlTargetPoint.getY() + 2);
      }
   }

   private void placeCartesianTargetsAtActuals()
   {
      reader.read();

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
      public void variableChanged(YoVariable<?> v)
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

      public void variableChanged(YoVariable<?> v)
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
            for (YoGraphic baseControlPoint : baseControlPointsList)
            {
               baseControlPoint.hideGraphicObject();
            }
            for (YoGraphic baseControlLine : baseControlLinesList)
            {
               baseControlLine.hideGraphicObject();
            }
            break;

         case 1:
            for (YoGraphic baseControlPoint : baseControlPointsList)
            {
               baseControlPoint.showGraphicObject();
            }
            for (YoGraphic baseControlLine : baseControlLinesList)
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

      public void variableChanged(YoVariable<?> v)
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
            for (YoGraphic baseControlTargetPoint : baseControlTargetPointsList)
            {
               baseControlTargetPoint.hideGraphicObject();
            }
            for (YoGraphic baseControlTargetLine : baseControlTargetLinesList)
            {
               baseControlTargetLine.hideGraphicObject();
            }
            break;

         case 1:
            for (YoGraphic baseControlTargetPoint : baseControlTargetPointsList)
            {
               baseControlTargetPoint.showGraphicObject();
            }
            for (YoGraphic baseControlTargetLine : baseControlTargetLinesList)
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
      private final RigidBodyTransform desiredTransform = new RigidBodyTransform();

      public void variableChanged(YoVariable<?> v)
      {
         if (legInverseKinematicsCalculators == null)
            return;

         reader.read();
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
               framePose.getPose(desiredTransform);
               legInverseKinematicsCalculators.get(robotSide).solve(desiredTransform);

               YoFramePose handIK = handIKs.get(robotSide);
               position = handIK.getPosition().getFramePointCopy();
               orientation = handIK.getOrientation().getFrameOrientationCopy();
               framePose = new FramePose(position, orientation);
               framePose.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
               framePose.getPose(desiredTransform);
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

      public SymmetricModeListener(DoubleYoVariable variableToSet, RobotSide robotSide, double sign)
      {
         this.variableToSet = variableToSet;
         this.robotSide = robotSide;
         this.respectiveSign = sign;
      }

      public void variableChanged(YoVariable<?> yoVariable)
      {
         if (symmetricMode && (robotSide == symmetricControlSide))
         {
            variableToSet.set(respectiveSign * ((DoubleYoVariable) yoVariable).getDoubleValue());
         }
      }
   }

   private class PelvisRotationListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable<?> v)
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

      @SuppressWarnings("deprecation")
      public void variableChanged(YoVariable<?> v)
      {
         if (!controlFingers.getBooleanValue())
            return;
         
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
      Quaternion q = new Quaternion();

      //This code has a singularity when yaw and roll line up (e.g. pitch is 90, can't rotate in one direction any more).
      q.setYawPitchRoll(q_yaw.getDoubleValue(), q_pitch.getDoubleValue(), q_roll.getDoubleValue());

      //This code compounds the rotations so that on subsequent frames the ability to rotate in lost rotation directions is regained
      //This affectively uses global yaw pitch and roll each time.
      q.multiply(qprev);

      q_qs.set(q.getS());
      q_qx.set(q.getX());
      q_qy.set(q.getY());
      q_qz.set(q.getZ());
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
   
   private void addSupportBaseControlGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addSupportBaseGraphics(yoGraphicsListRegistry,baseControlPoints,baseControlPointsList,baseControlLinesList,"baseControl",YoAppearance.Green());

   }

   private void addSupportBaseControlTargetGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addSupportBaseGraphics(yoGraphicsListRegistry,baseControlTargetPoints,baseControlTargetPointsList,baseControlTargetLinesList,"baseControlTarget",YoAppearance.Red());
   }

   private void addSupportBaseGraphics(YoGraphicsListRegistry yoGraphicsListRegistry,YoFramePoint[] basePoints, ArrayList<YoGraphic> basePointsList, ArrayList<YoGraphic> linesList, String namePrefix,AppearanceDefinition appearance)
   {
      AppearanceDefinition[] colors = { YoAppearance.Red(), YoAppearance.Green(), YoAppearance.Blue(), YoAppearance.Yellow() };
      YoGraphicsList yoGraphicsList = new YoGraphicsList(namePrefix + "Points");
      for (int i = 0; i < basePoints.length; i++)
      {
         YoGraphicPosition baseControlPointViz = new YoGraphicPosition(namePrefix + "Point" + i, basePoints[i], 0.01, colors[i]);
         yoGraphicsList.add(baseControlPointViz);
         basePointsList.add(baseControlPointViz);
         
         for (int j = i + 1; j < basePoints.length; j++)
         {
            YoGraphicLineSegment dynamicGraphicLineSegment = new YoGraphicLineSegment(namePrefix + "SupportLine", basePoints[i], basePoints[j],
                  1.0, appearance, false);
            yoGraphicsList.add(dynamicGraphicLineSegment);
            linesList.add(dynamicGraphicLineSegment);
         }
      }

      if (yoGraphicsListRegistry != null)
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsList.hideYoGraphics();
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
}
