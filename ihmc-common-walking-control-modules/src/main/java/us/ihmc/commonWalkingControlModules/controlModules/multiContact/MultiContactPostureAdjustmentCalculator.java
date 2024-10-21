package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginRegionCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.SensitivityBasedCoMMarginCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class MultiContactPostureAdjustmentCalculator implements WholeBodyPostureAdjustmentProvider
{
   private static final int SPATIAL_DIMENSIONS = 6;
   private static final String NAME_PREFIX = "postureAdj_";
   private static final double MAX_JOINT_DECAY_RATE = 0.3;
   private static final double MAX_HEIGHT_DECAY_RATE = 0.1;
   private static final FrameQuaternion zeroOrientation = new FrameQuaternion();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final WholeBodyContactState contactState;
   private final CenterOfMassStabilityMarginRegionCalculator staticStabilityRegionCalculator;
   private final YoEnum<PostureAdjustmentMode> mode = new YoEnum<>(NAME_PREFIX + "Mode", registry, PostureAdjustmentMode.class);
   private final SensitivityBasedCoMMarginCalculator postureOptimizer;

   private final YoDouble[] qJointPostureAdjustment;
   private final YoDouble[] qdJointPostureAdjustment;
   private final YoDouble qPelvisHeightPostureAdjustment = new YoDouble(NAME_PREFIX + "pelvisZ", registry);
   private final YoDouble qdPelvisHeightPostureAdjustment = new YoDouble(NAME_PREFIX + "pelvisDz", registry);
   private final YoFrameQuaternion pelvisOrientationPostureAdjustment = new YoFrameQuaternion(NAME_PREFIX + "pelvisOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D pelvisAngularVelocityPostureAdjustment = new YoFrameVector3D(NAME_PREFIX + "pelvisAngVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final Vector3D pelvisRotationVectorAdjustment = new Vector3D();
   private final Quaternion pelvisRotationQuaternionAdjustment = new Quaternion();
   private final TObjectIntHashMap<OneDoFJointBasics> jointToIndexMap = new TObjectIntHashMap<>(30);

   private final double controlDT;
   private final YoDouble qdMultiplier = new YoDouble("qdMultiplier", registry);
   private final ExecutionTimer postureOptimizationTimer = new ExecutionTimer("postureOptimizationTimer", registry);

   private enum PostureAdjustmentMode
   {
      /* Integrates the setpoints based on the posture calculator */
      ENABLED,
      /* Smoothly decays the setpoints to zero */
      DISABLED,
      /* Maintains the setpoints when entering freeze mode, with zero velocity */
      FREEZE
   }

   public MultiContactPostureAdjustmentCalculator(CenterOfMassStabilityMarginRegionCalculator staticStabilityRegionCalculator,
                                                  WholeBodyContactState contactState,
                                                  FullHumanoidRobotModel fullRobotModel,
                                                  ReferenceFrame centerOfMassFrame,
                                                  double controlDT,
                                                  YoRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.contactState = contactState;
      this.staticStabilityRegionCalculator = staticStabilityRegionCalculator;

      this.postureOptimizer = new SensitivityBasedCoMMarginCalculator(centerOfMassFrame, fullRobotModel, contactState, staticStabilityRegionCalculator, registry);
      qJointPostureAdjustment = new YoDouble[contactState.getNumberOfJoints()];
      qdJointPostureAdjustment = new YoDouble[contactState.getNumberOfJoints()];

      for (int jointIdx = 0; jointIdx < contactState.getNumberOfJoints(); jointIdx++)
      {
         OneDoFJointBasics joint = contactState.getOneDoFJoints()[jointIdx];
         jointToIndexMap.put(joint, jointIdx);
         qJointPostureAdjustment[jointIdx] = new YoDouble(NAME_PREFIX + "q_" + joint.getName(), registry);
         qdJointPostureAdjustment[jointIdx] = new YoDouble(NAME_PREFIX + "qd_" + joint.getName(), registry);
      }

      mode.set(PostureAdjustmentMode.DISABLED);
      qdMultiplier.set(0.02);
      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {
      boolean success;

      if (staticStabilityRegionCalculator.hasSolvedWholeRegion())
      {
         postureOptimizationTimer.startMeasurement();
         success = postureOptimizer.updateAll();
         postureOptimizationTimer.stopMeasurement();
      }
      else
      {
         postureOptimizer.initialize();
         success = false;
      }

      if (success)
      {
         updateVelocities();
      }
      else
      {
         setVelocitiesToZero();
      }

      if (mode.getValue() == PostureAdjustmentMode.ENABLED)
      { // integrate up velocities
         doIntegration();
      }
      else if (mode.getValue() == PostureAdjustmentMode.DISABLED)
      { // decay position offsets
         for (int i = 0; i < qJointPostureAdjustment.length; i++)
         {
            updateJointPositionRateLimited(0.0, qJointPostureAdjustment[i], controlDT, MAX_JOINT_DECAY_RATE);
         }
//         updateJointPositionRateLimited(0.0, qPelvisHeightPostureAdjustment, controlDT, MAX_HEIGHT_DECAY_RATE);
         pelvisOrientationPostureAdjustment.interpolate(zeroOrientation, 0.01);
      }
      else
      { // frozen, do nothing
      }
   }

   private void setVelocitiesToZero()
   {
      for (int i = 0; i < qJointPostureAdjustment.length; i++)
      {
         qdJointPostureAdjustment[i].set(0.0);
      }

      qdPelvisHeightPostureAdjustment.set(0.0);
      pelvisAngularVelocityPostureAdjustment.setToZero();
   }

   private void updateVelocities()
   {
      DMatrixRMaj optimizedWholeBodyVelocity = postureOptimizer.getOptimizedWholeBodyVelocity();

      pelvisAngularVelocityPostureAdjustment.set(0, optimizedWholeBodyVelocity);
      qdPelvisHeightPostureAdjustment.set(optimizedWholeBodyVelocity.get(5, 0));

      for (int i = 0; i < contactState.getNumberOfJoints(); i++)
      {
         qdJointPostureAdjustment[i].set(optimizedWholeBodyVelocity.get(SPATIAL_DIMENSIONS + i, 0));
      }
   }

   private void doIntegration()
   {
//      qPelvisHeightPostureAdjustment.add(qdMultiplier.getValue() * controlDT * qdPelvisHeightPostureAdjustment.getValue());

      pelvisRotationVectorAdjustment.setAndScale(qdMultiplier.getValue() * controlDT, pelvisAngularVelocityPostureAdjustment);
      pelvisRotationQuaternionAdjustment.setRotationVector(pelvisRotationVectorAdjustment);
      pelvisOrientationPostureAdjustment.append(pelvisRotationQuaternionAdjustment);

      for (int jointIdx = 0; jointIdx < contactState.getNumberOfJoints(); jointIdx++)
      {
         qJointPostureAdjustment[jointIdx].add(qdMultiplier.getValue() * controlDT * qdJointPostureAdjustment[jointIdx].getValue());
      }
   }

   public void disable()
   {
      mode.set(PostureAdjustmentMode.DISABLED);
   }

   public void resetIntegrators()
   {
      for (int i = 0; i < qJointPostureAdjustment.length; i++)
      {
         qJointPostureAdjustment[i].set(0.0);
         qdJointPostureAdjustment[i].set(0.0);
      }

      qPelvisHeightPostureAdjustment.set(0.0);
      qdPelvisHeightPostureAdjustment.set(0.0);

      pelvisOrientationPostureAdjustment.setToZero();
      pelvisAngularVelocityPostureAdjustment.setToZero();
   }

   private static void updateJointPositionRateLimited(double currentPosition, YoDouble qJointPostureAdjustment, double controlDT, double maxRate)
   {
      double difference = currentPosition - qJointPostureAdjustment.getValue();

      if (Math.abs(difference) > maxRate * controlDT)
      {
         difference = Math.signum(difference) * maxRate * controlDT;
      }

      qJointPostureAdjustment.add(difference);
   }

   @Override
   public boolean isEnabled()
   {
      return true;
   }

   @Override
   public double getDesiredJointPositionOffset(OneDoFJointBasics joint)
   {
      return qJointPostureAdjustment[jointToIndexMap.get(joint)].getValue();
   }

   @Override
   public double getDesiredJointVelocityOffset(OneDoFJointBasics joint)
   {
      return 0.0;
//      return qdJointPostureAdjustment[jointToIndexMap.get(joint)].getValue();
   }

   @Override
   public double getDesiredJointAccelerationOffset(OneDoFJointBasics joint)
   {
      return 0.0;
   }

   @Override
   public double getFloatingBasePositionOffsetZ()
   {
      return qPelvisHeightPostureAdjustment.getValue();
   }

   @Override
   public double getFloatingBaseVelocityOffsetZ()
   {
      return qdPelvisHeightPostureAdjustment.getValue();
   }

   @Override
   public double getFloatingBaseAccelerationOffsetZ()
   {
      return 0.0;
   }

   @Override
   public void packFloatingBaseOrientationOffset(FrameQuaternionBasics orientationOffsetToPack)
   {
      orientationOffsetToPack.set(pelvisOrientationPostureAdjustment);
   }

   @Override
   public void packFloatingBaseAngularVelocityOffset(FrameVector3DBasics angularVelocityToPack)
   {
//      angularVelocityToPack.set(pelvisAngularVelocityPostureAdjustment);
   }

   @Override
   public void packFloatingBaseAngularAccelerationOffset(FrameVector3DBasics angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero();
   }
}
