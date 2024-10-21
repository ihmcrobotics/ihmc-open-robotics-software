package us.ihmc.avatar.networkProcessor.referenceSpreading;

import controller_msgs.msg.dds.SpatialVectorMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.LinkedHashMap;

public class ReferenceSpreader
{
   private static final List<String> JOINT_NAMES = Arrays.asList("SHOULDER_Y", "SHOULDER_X", "SHOULDER_Z", "ELBOW_Y", "WRIST_Z", "WRIST_X", "GRIPPER_Z");

   private final YoRegistry registry;
   private final String baseFinalPath;

   private final FullHumanoidRobotModel fullRobotModel;
   private final DRCRobotModel robotModel;

   private final TrajectoryRecordReplay originalReference;
   private final TrajectoryRecordReplay preImpactReference;
   private final TrajectoryRecordReplay postImpactReference;
   private final TrajectoryRecordReplay blendedImpactReference;
   private List<String> keyMatrix = new ArrayList<>();

   private int impactIndex = -1;
   private double impactTime = Double.NaN;
   private final CollisionDetection collisionDetection;

   private HashMap<RobotSide, SpatialVectorMessage> handWrenches = new HashMap<>(RobotSide.values().length);
   private us.ihmc.idl.IDLSequence.Float jointVelocities = new us.ihmc.idl.IDLSequence.Float(50, "type_5");
   private List<String> jointNames = new ArrayList<>();
   private double[] postImpactData = null;

   private final QuaternionCalculus quaternionCalculus = new QuaternionCalculus();

   LinkedHashMap<String, Double> currentFrame = new LinkedHashMap<>();

   Double timeDiff = 0.0;
   private final LinkedHashMap<String, Double> tempFrame = new LinkedHashMap<>();
   private final Vector3D rotationVectorIntegrated = new Vector3D();
   private final Quaternion rotationIntegrated = new Quaternion();
   private final Quaternion rotation = new Quaternion();

   private final Quaternion preRotation = new Quaternion();
   private final Quaternion postRotation = new Quaternion();

   private double excludeInterval;
   private double blendInterval;

   ReferenceSpreader(String filePath, Double excludeInterval, Double blendInterval, DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, CollisionDetection collisionDetection, YoRegistry registry)
   {
      this.registry = registry;
      this.baseFinalPath = filePath.replace(".csv", "");
      this.fullRobotModel = fullRobotModel;
      this.robotModel = robotModel;

      this.originalReference = new TrajectoryRecordReplay(filePath, 1, true);
      this.preImpactReference = new TrajectoryRecordReplay(filePath+"_pre.csv", 1, false);
      this.postImpactReference = new TrajectoryRecordReplay(filePath+"_post.csv", 1, false);
      this.blendedImpactReference = new TrajectoryRecordReplay(filePath+"_blended.csv", 1, false);
      this.originalReference.importData(true);
      this.keyMatrix = new ArrayList<>(originalReference.getKeyMatrix());

      this.collisionDetection = collisionDetection;
      this.excludeInterval = excludeInterval;
      this.blendInterval = blendInterval;

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         jointNames.add(joint.getName());
         jointVelocities.add(0.0f);
      }

      detectImpact();
      extendPreImpactTrajectory();
      extendPostImpactTrajectory();
   }

   public void detectImpact(){
      impactIndex = -1;
      postImpactData = null;

      while (!originalReference.hasDoneReplay() && postImpactData == null)
      {
         double[] values = originalReference.play(true);
         makeMap(values, currentFrame);
//         Hard-coded with jointMap of `CollisionDetection`
         for (int i = 12; i <= 28; i++)
         {
            jointVelocities.set(i, currentFrame.get("qd_" + jointNames.get(i)).floatValue());
         }

         for (RobotSide robotSide : RobotSide.values())
         {
            SpatialVectorMessage spatialVectorMessage = new SpatialVectorMessage();
            spatialVectorMessage.getLinearPart().setX(currentFrame.get("filteredWrenchLinearPartX"+robotSide.getSideNameInAllCaps()));
            spatialVectorMessage.getLinearPart().setY(currentFrame.get("filteredWrenchLinearPartY"+robotSide.getSideNameInAllCaps()));
            spatialVectorMessage.getLinearPart().setZ(currentFrame.get("filteredWrenchLinearPartZ"+robotSide.getSideNameInAllCaps()));
            spatialVectorMessage.getAngularPart().setX(currentFrame.get("filteredWrenchAngularPartX"+robotSide.getSideNameInAllCaps()));
            spatialVectorMessage.getAngularPart().setY(currentFrame.get("filteredWrenchAngularPartY"+robotSide.getSideNameInAllCaps()));
            spatialVectorMessage.getAngularPart().setZ(currentFrame.get("filteredWrenchAngularPartZ"+robotSide.getSideNameInAllCaps()));
            handWrenches.put(robotSide, spatialVectorMessage);

            if (impactIndex ==-1 && collisionDetection.detectCollision(handWrenches, jointVelocities, currentFrame.get("time[sec]")))
            {
               impactIndex = originalReference.getTimeStepReplay();
               impactTime = currentFrame.get("time[sec]");
            }

            if (impactIndex!=-1 && currentFrame.get("time[sec]") > impactTime + excludeInterval)
            {
               postImpactData = new double[values.length];
               System.arraycopy(values, 0, postImpactData, 0, values.length);
               break;
            }
         }
      }
      if (impactIndex == -1)
         new AssertionError("No impact detected for ReferenceSpreading! (Change tuning or check the data)");
      LogTools.info("Impact detected at index: " + impactIndex + " time: " + impactTime + " Current time " + currentFrame.get("time[sec]"));
   }

   private void extendPreImpactTrajectory()
   {
      double[] preImpactData = null;
      String nameEndEffector = "_GRIPPER_YAW_LINKCurrent";

      originalReference.reset();
      originalReference.play(false);

      while (!originalReference.hasDoneReplay())
      {
         double[] values = originalReference.play(false);
         makeMap(values, currentFrame);

         if(currentFrame.get("time[sec]") < impactTime-excludeInterval)
         {
          preImpactReference.record(values);
         }
         else
         {
            if(preImpactData == null)
            {
               preImpactData = new double[values.length];
               System.arraycopy(values, 0, preImpactData, 0, values.length);
            }

            makeMap(preImpactData, tempFrame);
            tempFrame.put("time[sec]", currentFrame.get("time[sec]"));
            timeDiff = currentFrame.get("time[sec]") - impactTime;
            for (RobotSide robotSide : RobotSide.values())
            {
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "PositionX",
                             tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "PositionX")
                             + tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "LinearVelocityX")* timeDiff);
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "PositionY",
                             tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "PositionY")
                             + tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "LinearVelocityY")* timeDiff);
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "PositionZ",
                             tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "PositionZ")
                             + tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "LinearVelocityZ")* timeDiff);

               rotation.set(tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQx"),
                            tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQy"),
                            tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQz"),
                            tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQs"));
               rotationVectorIntegrated.set(tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "AngularVelocityX")* timeDiff,
                                           tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "AngularVelocityY")* timeDiff,
                                           tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "AngularVelocityZ")* timeDiff);
               quaternionCalculus.exp(rotationVectorIntegrated, rotationIntegrated);
               rotation.multiply(rotationIntegrated);
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQx", rotation.getX());
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQy", rotation.getY());
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQz", rotation.getZ());
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQs", rotation.getS());

               for (String jointName : JOINT_NAMES)
               {
                  tempFrame.put("q_" + robotSide.getUpperCaseName() + "_" + jointName, tempFrame.get("q_" + robotSide.getUpperCaseName() + "_" + jointName) + tempFrame.get("qd_" + robotSide.getUpperCaseName() + "_" + jointName) * timeDiff);
               }
            }

            preImpactReference.record(tempFrame.values().stream().mapToDouble(Double::doubleValue).toArray());
         }
      }
      preImpactReference.saveRecordingMemory();
      LogTools.info("PreImpact trajectory extended (Saved to Memory)");
   }

   private void extendPostImpactTrajectory()
   {
      String nameEndEffector = "_GRIPPER_YAW_LINKCurrent";

      originalReference.reset();
      originalReference.play(false);

      while (!originalReference.hasDoneReplay())
      {
         double[] values = originalReference.play(false);
         makeMap(values, currentFrame);

         if(currentFrame.get("time[sec]") >= impactTime-excludeInterval)
         {
            postImpactReference.record(values);
         }
         else
         {
            makeMap(postImpactData, tempFrame);
            tempFrame.put("time[sec]", currentFrame.get("time[sec]"));
            timeDiff = currentFrame.get("time[sec]") - impactTime;
            for (RobotSide robotSide : RobotSide.values())
            {
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "PositionX",
                             tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "PositionX")
                             + tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "LinearVelocityX")* timeDiff);
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "PositionY",
                             tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "PositionY")
                             + tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "LinearVelocityY")* timeDiff);
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "PositionZ",
                             tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "PositionZ")
                             + tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "LinearVelocityZ")* timeDiff);

               rotation.set(tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQx"),
                            tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQy"),
                            tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQz"),
                            tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQs"));
               rotationVectorIntegrated.set(tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "AngularVelocityX")* timeDiff,
                                            tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "AngularVelocityY")* timeDiff,
                                            tempFrame.get(robotSide.getUpperCaseName() + nameEndEffector + "AngularVelocityZ")* timeDiff);
               quaternionCalculus.exp(rotationVectorIntegrated, rotationIntegrated);
               rotation.multiply(rotationIntegrated);
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQx", rotation.getX());
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQy", rotation.getY());
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQz", rotation.getZ());
               tempFrame.put(robotSide.getUpperCaseName() + nameEndEffector + "OrientationQs", rotation.getS());

               for (String jointName : JOINT_NAMES)
               {
                  tempFrame.put("q_" + robotSide.getUpperCaseName() + "_" + jointName, tempFrame.get("q_" + robotSide.getUpperCaseName() + "_" + jointName) + tempFrame.get("qd_" + robotSide.getUpperCaseName() + "_" + jointName) * timeDiff);
               }
            }

            postImpactReference.record(tempFrame.values().stream().mapToDouble(Double::doubleValue).toArray());
         }
      }
      postImpactReference.saveRecordingMemory();
      LogTools.info("PostImpact trajectory extended (Saved to Memory)");
   }

   public void blendImpactTrajectory(double impactTime)
   {
      this.impactTime = impactTime;

      preImpactReference.reset();
      postImpactReference.reset();
      preImpactReference.play(false);
      postImpactReference.play(false);

      double[] preImpactValues;
      double[] postImpactValues;
      double blendFactor = 0.0;

      while (!preImpactReference.hasDoneReplay())
      {
         preImpactValues = preImpactReference.play(false);
         postImpactValues = postImpactReference.play(false);
         if (preImpactValues == null || postImpactValues == null)
         {
            break;
         }
         makeMap(postImpactValues, currentFrame);

         if(currentFrame.get("time[sec]") >= impactTime && currentFrame.get("time[sec]") < impactTime + blendInterval)
         {
            blendFactor = (currentFrame.get("time[sec]") - impactTime) / blendInterval;

            for (RobotSide robotSide : RobotSide.values())
            {
               for (String jointName : JOINT_NAMES)
               {
                  currentFrame.put("q_" + robotSide.getUpperCaseName() + "_" + jointName,
                                preImpactValues[getKeyIndex("q_" + robotSide.getUpperCaseName() + "_" + jointName)] * (1 - blendFactor)
                                + postImpactValues[getKeyIndex("q_" + robotSide.getUpperCaseName() + "_" + jointName)] * blendFactor);
               }

               currentFrame.put(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentPositionX",
                                 preImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentPositionX")] * (1 - blendFactor)
                                 + postImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentPositionX")] * blendFactor);
               currentFrame.put(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentPositionY",
                                 preImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentPositionY")] * (1 - blendFactor)
                                 + postImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentPositionY")] * blendFactor);
               currentFrame.put(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentPositionZ",
                                 preImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentPositionZ")] * (1 - blendFactor)
                                 + postImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentPositionZ")] * blendFactor);

               preRotation.set(preImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQx")],
                                 preImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQy")],
                                 preImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQz")],
                                 preImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQs")]);
               postRotation.set(postImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQx")],
                                 postImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQy")],
                                 postImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQz")],
                                 postImpactValues[getKeyIndex(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQs")]);
               quaternionCalculus.interpolate(blendFactor,preRotation, postRotation, rotation);
               currentFrame.put(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQx", rotation.getX());
               currentFrame.put(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQy", rotation.getY());
               currentFrame.put(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQz", rotation.getZ());
               currentFrame.put(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrentOrientationQs", rotation.getS());

               currentFrame.put("filteredWrenchLinearPartX"+robotSide.getSideNameInAllCaps(),
                                 preImpactValues[getKeyIndex("filteredWrenchLinearPartX"+robotSide.getSideNameInAllCaps())] * (1 - blendFactor)
                                 + postImpactValues[getKeyIndex("filteredWrenchLinearPartX"+robotSide.getSideNameInAllCaps())] * blendFactor);
               currentFrame.put("filteredWrenchLinearPartY"+robotSide.getSideNameInAllCaps(),
                                 preImpactValues[getKeyIndex("filteredWrenchLinearPartY"+robotSide.getSideNameInAllCaps())] * (1 - blendFactor)
                                 + postImpactValues[getKeyIndex("filteredWrenchLinearPartY"+robotSide.getSideNameInAllCaps())] * blendFactor);
               currentFrame.put("filteredWrenchLinearPartZ"+robotSide.getSideNameInAllCaps(),
                                 preImpactValues[getKeyIndex("filteredWrenchLinearPartZ"+robotSide.getSideNameInAllCaps())] * (1 - blendFactor)
                                 + postImpactValues[getKeyIndex("filteredWrenchLinearPartZ"+robotSide.getSideNameInAllCaps())] * blendFactor);
               currentFrame.put("filteredWrenchAngularPartX"+robotSide.getSideNameInAllCaps(),
                                 preImpactValues[getKeyIndex("filteredWrenchAngularPartX"+robotSide.getSideNameInAllCaps())] * (1 - blendFactor)
                                 + postImpactValues[getKeyIndex("filteredWrenchAngularPartX"+robotSide.getSideNameInAllCaps())] * blendFactor);
               currentFrame.put("filteredWrenchAngularPartY"+robotSide.getSideNameInAllCaps(),
                                 preImpactValues[getKeyIndex("filteredWrenchAngularPartY"+robotSide.getSideNameInAllCaps())] * (1 - blendFactor)
                                 + postImpactValues[getKeyIndex("filteredWrenchAngularPartY"+robotSide.getSideNameInAllCaps())] * blendFactor);
               currentFrame.put("filteredWrenchAngularPartZ"+robotSide.getSideNameInAllCaps(),
                                 preImpactValues[getKeyIndex("filteredWrenchAngularPartZ"+robotSide.getSideNameInAllCaps())] * (1 - blendFactor)
                                 + postImpactValues[getKeyIndex("filteredWrenchAngularPartZ"+robotSide.getSideNameInAllCaps())] * blendFactor);
            }
            currentFrame.put("blendFactor", blendFactor);
            blendedImpactReference.record(currentFrame.values().stream().mapToDouble(Double::doubleValue).toArray());
         }
         else if (currentFrame.get("time[sec]") >= impactTime + blendInterval)
         {
            currentFrame.put("blendFactor", 1.0);
            blendedImpactReference.record(currentFrame.values().stream().mapToDouble(Double::doubleValue).toArray());
         }
      }
      blendedImpactReference.saveRecordingMemory();
      LogTools.info("BlendedImpact trajectory extended (Saved to Memory)");
   }

   public ReferenceSpreadingTrajectory getOriginalReferenceTrajectory()
   {
      return new ReferenceSpreadingTrajectory(originalReference, new ArrayList<>(keyMatrix), robotModel, fullRobotModel, registry);
   }

   public ReferenceSpreadingTrajectory getPreImpactReferenceTrajectory()
   {
      return new ReferenceSpreadingTrajectory(preImpactReference, new ArrayList<>(keyMatrix), robotModel, fullRobotModel, registry);
   }

   public ReferenceSpreadingTrajectory getPostImpactReferenceTrajectory()
   {
      return new ReferenceSpreadingTrajectory(postImpactReference, new ArrayList<>(keyMatrix), robotModel, fullRobotModel, registry);
   }

   public ReferenceSpreadingTrajectory getBlendedReferenceTrajectory()
   {
      List<String> blendedKeyMatrix = new ArrayList<>(keyMatrix);
      blendedKeyMatrix.add("blendingFactor");
      return new ReferenceSpreadingTrajectory(blendedImpactReference, blendedKeyMatrix, robotModel, fullRobotModel, registry);
   }

   private int getKeyIndex(String key)
   {
      return keyMatrix.indexOf(key);
   }

   private void makeMap(double[] values, LinkedHashMap<String, Double> mapToPack)
   {
      for (int i = 0; i < values.length; i++)
      {
         mapToPack.put(keyMatrix.get(i), values[i]);
      }
   }
}
