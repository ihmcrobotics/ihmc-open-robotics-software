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
   private static final ArrayList<Double> shoulderRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> elbowRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> wristRollLimits = new ArrayList<Double>();
   private static final ArrayList<Double> shoulderPitchLimits = new ArrayList<Double>();
   private static final ArrayList<Double> elbowPitchLimits = new ArrayList<Double>();
   private static final ArrayList<Double> wristPitchLimits = new ArrayList<Double>();
   private final Random randomNumberGenerator = new Random(seed);
   private final EnumMap<JointNames, OneDoFJoint> oneDoFJoints = new EnumMap<JointNames, OneDoFJoint>(JointNames.class);
   private final EnumMap<JointNames, ArrayList<Double>> jointLimits = new EnumMap<JointNames, ArrayList<Double>>(JointNames.class);
   private final EnumMap<JointNames, Double> jointAngles = new EnumMap<JointNames, Double>(JointNames.class);
   private final EnumMap<JointNames, Double> jointAnglesInitial = new EnumMap<JointNames, Double>(JointNames.class);
   private final SDFFullRobotModel fullRobotModel;
   private final ReferenceFrame worldFrame;
   private final double tolerance, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar;
   private final int maxIterations;

   private GeometricJacobian leftHandJacobian;
   private NumericalInverseKinematicsCalculator leftArmInverseKinematicsCalculator;

   private enum JointNames
   {
      SHOULDER_PITCH, SHOULDER_ROLL, ELBOW_PITCH, ELBOW_ROLL, WRIST_PITCH, WRIST_ROLL;
   };

   public NumericalInverseKinematicsCalculatorWithRobotTest()
   {
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCConfigParameters.robotModelToUse, false);
      JaxbSDFLoader jaxbSDFLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap, true);
      SDFFullRobotModelFactory fullRobotModelFactory = new SDFFullRobotModelFactory(jaxbSDFLoader.getGeneralizedSDFRobotModel(jointMap.getModelName()),
            jointMap);

      fullRobotModel = fullRobotModelFactory.create();
      worldFrame = ReferenceFrame.getWorldFrame();
      tolerance = 1e-8;
      maxStepSize = 1.0;
      minRandomSearchScalar = -0.05;
      maxRandomSearchScalar = 1.0;
      maxIterations = 10000;

      
      // TODO: get these values from somewhere RobotJointLimitWatcher
      // *_*_Limits(*_*_Max, *_*_Min);

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

      shoulderRollLimits.add(oneDoFJoints.get(JointNames.SHOULDER_ROLL).getJointLimitUpper());
      shoulderRollLimits.add(oneDoFJoints.get(JointNames.SHOULDER_ROLL).getJointLimitLower());
      shoulderPitchLimits.add(oneDoFJoints.get(JointNames.SHOULDER_PITCH).getJointLimitUpper());
      shoulderPitchLimits.add(oneDoFJoints.get(JointNames.SHOULDER_PITCH).getJointLimitLower());
      
      elbowRollLimits.add(oneDoFJoints.get(JointNames.ELBOW_ROLL).getJointLimitUpper());
      elbowRollLimits.add(oneDoFJoints.get(JointNames.ELBOW_ROLL).getJointLimitLower());
      elbowPitchLimits.add(oneDoFJoints.get(JointNames.ELBOW_PITCH).getJointLimitUpper());
      elbowPitchLimits.add(oneDoFJoints.get(JointNames.ELBOW_PITCH).getJointLimitLower());
      
      wristRollLimits.add(oneDoFJoints.get(JointNames.WRIST_ROLL).getJointLimitUpper());
      wristRollLimits.add(oneDoFJoints.get(JointNames.WRIST_ROLL).getJointLimitLower());
      wristPitchLimits.add(oneDoFJoints.get(JointNames.WRIST_PITCH).getJointLimitUpper());
      wristPitchLimits.add(oneDoFJoints.get(JointNames.WRIST_PITCH).getJointLimitLower());

      jointAnglesInitial.put(JointNames.SHOULDER_PITCH, 0.346);
      jointAnglesInitial.put(JointNames.SHOULDER_ROLL, -1.3141);
      jointAnglesInitial.put(JointNames.ELBOW_PITCH, 1.9195);
      jointAnglesInitial.put(JointNames.ELBOW_ROLL, 1.1749);
      jointAnglesInitial.put(JointNames.WRIST_PITCH, -0.0068);
      jointAnglesInitial.put(JointNames.WRIST_ROLL, -0.0447);
      
   }

   @Test
   public void generateRandomFeasibleRobotPoses()
   {
      for (int i = 0; i < 1000; i++)
      {
         randomArmPoseWithForwardKinematics();

         FramePoint handEndEffectorPositionFK = getHandEndEffectorPosition();
         FrameOrientation handEndEffectorOrientationFK = getHandEndEffectorOrientation();

         armPoseWithInverseKinematics(handEndEffectorOrientationFK, handEndEffectorPositionFK);

         FramePoint handEndEffectorPositionIK = getHandEndEffectorPosition();
         FrameOrientation handEndEffectorOrientationIK = getHandEndEffectorOrientation();
         
//         for (JointNames name : JointNames.values())
//         {
//            System.out.println(name + " : " + jointAngles.get(name));
//         }
         
         JUnitTools.assertFramePointEquals(handEndEffectorPositionFK, handEndEffectorPositionIK, 0.1);
         JUnitTools.assertFrameOrientationEquals(handEndEffectorOrientationFK, handEndEffectorOrientationIK, 0.3);
         

//         System.out.println("FK: " + handEndEffectorPositionFK.toString());
//         System.out.println("IK: " + handEndEffectorPositionIK.toString());
//         System.out.println("--------------------------------------------------------------------------------------");
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

         assertTrue((randomNumber <= max) && (randomNumber >= min));
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
      initialArmPose();
      leftHandJacobian = new GeometricJacobian(fullRobotModel.getChest(), fullRobotModel.getHand(RobotSide.LEFT), fullRobotModel.getHand(RobotSide.LEFT)
            .getBodyFixedFrame());

      leftArmInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(leftHandJacobian, tolerance, maxIterations, maxStepSize,
            minRandomSearchScalar, maxRandomSearchScalar);

      leftHandJacobian.compute();
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
      handEndEffectorPosition.changeFrame(worldFrame);
      return handEndEffectorPosition;
   }

   private void initialArmPose()
   {
      for (JointNames name : JointNames.values())
      {
         oneDoFJoints.get(name).setQ(jointAnglesInitial.get(name));
      }
   }

   private void randomArmPoseWithForwardKinematics()
   {
      for (JointNames name : JointNames.values())
      {
         double def = 0.0;
         jointAngles.put(name, generateRandomDoubleInRange(jointLimits.get(name).get(0) - def, jointLimits.get(name).get(1) + def));
         oneDoFJoints.get(name).setQ(jointAngles.get(name));
      }
   }

   private double generateRandomDoubleInRange(double max, double min)
   {
      return min + (max - min) * randomNumberGenerator.nextDouble();
   }
}
