package us.ihmc.avatar.networkProcessor.referenceSpreading;

import controller_msgs.msg.dds.SpatialVectorMessage;
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

   private double excludeInterval;

   ReferenceSpreader(String filePath, Double excludeInterval, FullHumanoidRobotModel fullRobotModel, CollisionDetection collisionDetection, YoRegistry registry)
   {
      this.registry = registry;
      this.baseFinalPath = filePath.replace(".csv", "");
      this.fullRobotModel = fullRobotModel;

      this.originalReference = new TrajectoryRecordReplay(filePath, 1, true);
      this.preImpactReference = new TrajectoryRecordReplay(filePath+"_pre.csv", 1, false);
      this.postImpactReference = new TrajectoryRecordReplay(filePath+"_post.csv", 1, false);
      this.blendedImpactReference = new TrajectoryRecordReplay(filePath+"_blended.csv", 1, false);
      this.originalReference.importData(true);
      this.keyMatrix = originalReference.getKeyMatrix();

      this.collisionDetection = collisionDetection;
      this.excludeInterval = excludeInterval;

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
      makeMap(originalReference.play(false), currentFrame);

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
            LogTools.info("tempFrame time: " + tempFrame.get("time[sec]") + " Current time " + currentFrame.get("time[sec]"));
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
      makeMap(originalReference.play(false), currentFrame);

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
            LogTools.info("tempFrame time: " + tempFrame.get("time[sec]") + " Current time " + currentFrame.get("time[sec]"));
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

   public ReferenceSpreadingTrajectory getOriginalReferenceTrajectory()
   {
      return new ReferenceSpreadingTrajectory(originalReference, keyMatrix, fullRobotModel, registry);
   }

   public ReferenceSpreadingTrajectory getPreImpactReferenceTrajectory()
   {
      return new ReferenceSpreadingTrajectory(preImpactReference, keyMatrix, fullRobotModel, registry);
   }

   public ReferenceSpreadingTrajectory getPostImpactReferenceTrajectory()
   {
      return new ReferenceSpreadingTrajectory(postImpactReference, keyMatrix, fullRobotModel, registry);
   }

   public ReferenceSpreadingTrajectory getBlendedImpactReferenceTrajectory()
   {
      return new ReferenceSpreadingTrajectory(blendedImpactReference, keyMatrix, fullRobotModel, registry);
   }

   private void makeMap(double[] values, LinkedHashMap<String, Double> mapToPack)
   {
      for (int i = 0; i < values.length; i++)
      {
         mapToPack.put(keyMatrix.get(i), values[i]);
      }
   }
}
