package us.ihmc.utilities.kinematics;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Random;

import javax.media.j3d.Transform3D;

import org.junit.Test;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.test.JUnitTools;

public class NumericalInverseKinematicsCalculatorWithRobotTest
{
   private static final long seed = 12091092L;
   Random randomNumberGenerator = new Random(seed);
   private static final ArrayList<Double> shoulderRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> elbowRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> wristRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> shoulderPitchLimits = new ArrayList<Double>();
   private static final ArrayList<Double> elbowPitchLimits = new ArrayList<Double>();
   private static final ArrayList<Double> wristPitchLimits = new ArrayList<Double>();

   public enum JointNames
   {
      SHOULDER_PITCH, SHOULDER_ROLL, ELBOW_PITCH, ELBOW_ROLL, WRIST_PITCH, WRIST_ROLL;
   };

   public enum JointLimits
   {
      SHOULDER_PITCH_LIMIT, SHOULDER_ROLL_LIMIT, ELBOW_PITCH_LIMIT, ELBOW_ROLL_LIMIT, WRIST_PITCH_LIMIT, WRIST_ROLL_LIMIT;
   };

   private final EnumMap<JointNames, OneDoFJoint> oneDoFJoints = new EnumMap<JointNames, OneDoFJoint>(JointNames.class);
   private final EnumMap<JointNames, ArrayList<Double>> jointLimits = new EnumMap<JointNames, ArrayList<Double>>(JointNames.class);
   private final EnumMap<JointNames, Double> randomJointAngles = new EnumMap<JointNames, Double>(JointNames.class);

   private final SDFFullRobotModel fullRobotModel;
   private final ReferenceFrame worldFrame;
   private final GeometricJacobian leftHandJacobian;
   private final NumericalInverseKinematicsCalculator leftArmInverseKinematicsCalculator;

   private ReferenceFrames referenceFrames;

   @SuppressWarnings("static-access")
   public NumericalInverseKinematicsCalculatorWithRobotTest()
   {
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCConfigParameters.robotModelToUse, false);
      JaxbSDFLoader jaxbSDFLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap, true);
      SDFFullRobotModelFactory fullRobotModelFactory = new SDFFullRobotModelFactory(jaxbSDFLoader.getGeneralizedSDFRobotModel(jointMap.getModelName()),
            jointMap);
      fullRobotModel = fullRobotModelFactory.create();
      referenceFrames = new ReferenceFrames(fullRobotModel, jointMap, jointMap.getAnkleHeight());
      worldFrame = referenceFrames.getWorldFrame();

      leftHandJacobian = new GeometricJacobian(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT), fullRobotModel.getHand(RobotSide.LEFT)
            .getBodyFixedFrame());
      double tolerance = 1e-8;
      double maxStepSize = 1.0;
      double minRandomSearchScalar = -0.5;
      double maxRandomSearchScalar = 1.0;
      int maxIterations = 100000;
      leftArmInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(leftHandJacobian, tolerance, maxIterations, maxStepSize,
            minRandomSearchScalar, maxRandomSearchScalar);

      //*_*_Limits(*_*_Max, *_*_Min);
      // atlasJointLimits TODO: get these values from somewhere
      shoulderRollLimits.add(1.74533);
      shoulderRollLimits.add(-1.39626);
      shoulderPitchLimits.add(1.9635);
      shoulderPitchLimits.add(-1.9635);
      elbowRollLimits.add(2.35619);
      elbowRollLimits.add(0.0);
      elbowPitchLimits.add(3.14159);
      elbowPitchLimits.add(0.0);
      wristRollLimits.add(1.571);
      wristRollLimits.add(-0.436);
      wristPitchLimits.add(1.571);
      wristPitchLimits.add(-1.571);

      oneDoFJoints.put(JointNames.SHOULDER_PITCH, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_PITCH));
      oneDoFJoints.put(JointNames.SHOULDER_ROLL, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL));
      oneDoFJoints.put(JointNames.ELBOW_PITCH, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_PITCH));
      oneDoFJoints.put(JointNames.ELBOW_ROLL, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.ELBOW_ROLL));
      oneDoFJoints.put(JointNames.WRIST_PITCH, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.WRIST_PITCH));
      oneDoFJoints.put(JointNames.WRIST_ROLL, fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.WRIST_ROLL));

      jointLimits.put(JointNames.SHOULDER_PITCH, shoulderPitchLimits);
      jointLimits.put(JointNames.SHOULDER_ROLL, shoulderRollLimits);
      jointLimits.put(JointNames.ELBOW_PITCH, elbowPitchLimits);
      jointLimits.put(JointNames.ELBOW_ROLL, elbowRollLimits);
      jointLimits.put(JointNames.WRIST_PITCH, wristPitchLimits);
      jointLimits.put(JointNames.WRIST_ROLL, wristRollLimits);

   }

   @Test
   public void generateRandomFeasibleRobotPoses()
   {

      for(int i = 0; i<10; i++)
      {
//         System.out.println(i + "-------------------------------------------------------------------");
         randomArmPoseWithForwardKinematics();
         FramePoint handEndEffectorPositionFK = getHandEndEffectorPosition();
         FrameOrientation handEndEffectorOrientationFK = getHandEndEffectorOrientation();
         armPoseWithInverseKinematics(handEndEffectorOrientationFK, handEndEffectorPositionFK);
         FramePoint handEndEffectorPositionIK = getHandEndEffectorPosition();
         FrameOrientation handEndEffectorOrientationIK = getHandEndEffectorOrientation();

//         System.out.println(handEndEffectorPositionFK.toString());
//         System.out.println(handEndEffectorPositionIK.toString());
//         System.out.println(handEndEffectorOrientationFK.toStringAsYawPitchRoll());
//         System.out.println(handEndEffectorOrientationIK.toStringAsYawPitchRoll());
//         System.out.println("-------------------------------------------------------------------");
//         System.out.println(" ");

         JUnitTools.assertFramePointEquals(handEndEffectorPositionFK, handEndEffectorPositionIK, 0.1);
         JUnitTools.assertFrameOrientationEquals(handEndEffectorOrientationFK, handEndEffectorOrientationIK, 0.1);
         
      }
      
   }
   
   @Test
   public void generateRandomDoubleInRangeTest()
   {
      double min = -randomNumberGenerator.nextDouble();
      double max = randomNumberGenerator.nextDouble();
      for (int i = 0; i < 10000; i++)
      {
         double randomNumber = generateRandomDoubleInRange(max, min);
         assertTrue(randomNumber <= max && randomNumber >= min);
      }
   }

   private void armPoseWithInverseKinematics(FrameOrientation desiredOrientation, FramePoint desiredPosition)
   {
      FramePose handPose = new FramePose(worldFrame);
      handPose.setOrientation(desiredOrientation);
      handPose.setPosition(desiredPosition);
      handPose.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
      Transform3D transform = new Transform3D();
      handPose.getTransformFromPoseToFrame(transform);
      
      resetArmPoseToZero();
      fullRobotModel.updateFrames();
      leftArmInverseKinematicsCalculator.solve(transform);
   }

   private FrameOrientation getHandEndEffectorOrientation()
   {
      ReferenceFrame handEndEffectorFrame = fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame();
      FrameOrientation handEndEffectorOrientation = new FrameOrientation(handEndEffectorFrame);
      handEndEffectorOrientation.changeFrame(worldFrame);
      return handEndEffectorOrientation;
   }

   private FramePoint getHandEndEffectorPosition()
   {
      ReferenceFrame handEndEffectorFrame = fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame();
      FramePoint handEndEffectorPosition = new FramePoint(handEndEffectorFrame);
      handEndEffectorPosition.set(0, 0, 0);
      handEndEffectorPosition.changeFrame(worldFrame);
      return handEndEffectorPosition;
   }

   private void resetArmPoseToZero()
   {
      for (JointNames name : JointNames.values())
      {
         oneDoFJoints.get(name).setQ(0.1);
      }
   }

   private void randomArmPoseWithForwardKinematics()
   {
      for (JointNames name : JointNames.values())
      {
         randomJointAngles.put(name, generateRandomDoubleInRange(jointLimits.get(name).get(0) - 0.5, jointLimits.get(name).get(1) + 0.5));
         oneDoFJoints.get(name).setQ(randomJointAngles.get(name));
      }
   }

   private double generateRandomDoubleInRange(double max, double min)
   {
      return min + (max - min) * randomNumberGenerator.nextDouble();
   }
}
