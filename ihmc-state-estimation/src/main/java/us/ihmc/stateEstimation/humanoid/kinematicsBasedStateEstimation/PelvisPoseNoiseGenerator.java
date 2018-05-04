package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.Random;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class PelvisPoseNoiseGenerator
{
   private final YoVariableRegistry registry;
   
   private final FloatingInverseDynamicsJoint rootJoint;
   private final ReferenceFrame rootJointFrame;
   
   private final Random random = new Random();
   private final RigidBodyTransform pelvisPose = new RigidBodyTransform();
   private final RotationMatrix rotationError = new RotationMatrix();
   private final Vector3D translationError = new Vector3D();
   
   private final Vector3D translationNoise = new Vector3D();
   private final Vector3D pelvisTranslation = new Vector3D();
   
   private final Quaternion rot = new Quaternion();
   private final double[] tempRots = new double[3];
   private final RotationMatrix rotationNoise = new RotationMatrix();
   private final RotationMatrix pelvisRotation = new RotationMatrix();
   
   
   private final YoFramePoint3D nonProcessedRootJointPosition;
   private final YoFrameQuaternion nonProcessedRootJointQuaternion;
   private final YoDouble nonProcessedRootJointPitch;
   private final YoDouble nonProcessedRootJointRoll;
   private final YoDouble nonProcessedRootJointYaw;
   
   private final YoFramePoint3D processedRootJointPosition;
   private final YoFrameQuaternion processedRootJointQuaternion;
   private final YoDouble processedRootJointPitch;
   private final YoDouble processedRootJointRoll;
   private final YoDouble processedRootJointYaw;
   
   private final YoDouble error_x;
   private final YoDouble error_y;
   private final YoDouble error_z;
   private final YoDouble error_yaw;
   private final YoDouble error_pitch;
   private final YoDouble error_roll;

   private final YoDouble noiseBias_x;
   private final YoDouble noiseBias_y;
   private final YoDouble noiseBias_z;
                                       
   private final YoDouble noiseBias_roll;
   private final YoDouble noiseBias_pitch;
   private final YoDouble noiseBias_yaw;
   
   private final YoDouble noiseScalar_x;
   private final YoDouble noiseScalar_y;
   private final YoDouble noiseScalar_z;
   private final YoDouble noiseScalar_yaw;
   private final YoDouble noiseScalar_pitch;
   private final YoDouble noiseScalar_roll;
   
   public PelvisPoseNoiseGenerator(FullInverseDynamicsStructure inverseDynamicsStructure, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      registry = new YoVariableRegistry("PelvisPoseNoiseGenerator");
      parentRegistry.addChild(registry);
      
      rotationError.setIdentity();
      pelvisRotation.setIdentity();
      
      nonProcessedRootJointPosition = new YoFramePoint3D("PelvisPose_beforeNoise_position", ReferenceFrame.getWorldFrame(), registry);
      nonProcessedRootJointQuaternion = new YoFrameQuaternion("PelvisPose_beforeNoise_quaternion", ReferenceFrame.getWorldFrame(), registry);
      nonProcessedRootJointYaw = new YoDouble("PelvisPose_beforeNoise_yaw", registry);
      nonProcessedRootJointPitch = new YoDouble("PelvisPose_beforeNoise_pitch", registry);
      nonProcessedRootJointRoll = new YoDouble("PelvisPose_beforeNoise_roll", registry);
      
      processedRootJointPosition = new YoFramePoint3D("PelvisPose_afterNoise_position", ReferenceFrame.getWorldFrame(), registry);
      processedRootJointQuaternion = new YoFrameQuaternion("PelvisPose_afterNoise_quaternion", ReferenceFrame.getWorldFrame(), registry);
      processedRootJointYaw = new YoDouble("PelvisPose_afterNoise_yaw", registry);
      processedRootJointPitch = new YoDouble("PelvisPose_afterNoise_pitch", registry);
      processedRootJointRoll = new YoDouble("PelvisPose_afterNoise_roll", registry);
      
      error_x = new YoDouble("PelvisPose_noise_x", registry);
      error_y = new YoDouble("PelvisPose_noise_y", registry);
      error_z = new YoDouble("PelvisPose_noise_z", registry);
      
      error_yaw = new YoDouble("PelvisPose_noise_yaw", registry);
      error_pitch = new YoDouble("PelvisPose_noise_pitch", registry);
      error_roll = new YoDouble("PelvisPose_noise_roll", registry);
                        
      noiseBias_x = new YoDouble("PelvisPose_bias_x", registry);
      noiseBias_y = new YoDouble("PelvisPose_bias_y", registry);
      noiseBias_z = new YoDouble("PelvisPose_bias_z", registry);
                        
      noiseBias_roll = new YoDouble("PelvisPose_bias_roll", registry);
      noiseBias_pitch = new YoDouble("PelvisPose_bias_pitch", registry);
      noiseBias_yaw = new YoDouble("PelvisPose_bias_yaw", registry);
                        
      noiseScalar_x = new YoDouble("PelvisPose_NoiseScalar_x", registry);
      noiseScalar_y = new YoDouble("PelvisPose_NoiseScalar_y", registry);
      noiseScalar_z = new YoDouble("PelvisPose_NoiseScalar_z", registry);
      
      noiseScalar_yaw = new YoDouble("PelvisPose_NoiseScalar_yaw", registry);
      noiseScalar_pitch = new YoDouble("PelvisPose_NoiseScalar_pitch", registry);
      noiseScalar_roll = new YoDouble("PelvisPose_NoiseScalar_roll", registry);
   }
   
   
   public void addNoise()
   {
      rootJointFrame.getTransformToParent(pelvisPose);
      
      updateBeforeYoVariables();
      integrateError();
      
      pelvisPose.getRotation(pelvisRotation);
      pelvisRotation.multiply(rotationError);
      pelvisPose.setRotation(pelvisRotation);
      
      pelvisPose.getTranslation(pelvisTranslation);
      pelvisTranslation.add(translationError);
      pelvisPose.setTranslation(pelvisTranslation);
      
      updateAfterYoVariables();
      
      rootJoint.setPositionAndRotation(pelvisPose);
      rootJointFrame.update();
   }

   private void updateBeforeYoVariables()
   {
      pelvisPose.getTranslation(pelvisTranslation);
      nonProcessedRootJointPosition.set(pelvisTranslation);
      
      pelvisPose.getRotation(rot);
      nonProcessedRootJointQuaternion.set(rot);
      nonProcessedRootJointQuaternion.getYawPitchRoll(tempRots);
      nonProcessedRootJointYaw.set(tempRots[0]);
      nonProcessedRootJointPitch.set(tempRots[1]);
      nonProcessedRootJointRoll.set(tempRots[2]);
   }
   
   private void updateAfterYoVariables()
   {
      error_pitch.set(rotationError.getPitch());
      error_roll.set(rotationError.getRoll());
      error_yaw.set(rotationError.getYaw());
      error_x.set(translationError.getX()); 
      error_y.set(translationError.getY());  
      error_z.set(translationError.getZ()); 
      
      pelvisPose.getTranslation(pelvisTranslation);
      processedRootJointPosition.set(pelvisTranslation);
      
      pelvisPose.getRotation(rot);
      processedRootJointQuaternion.set(rot);
      processedRootJointQuaternion.getYawPitchRoll(tempRots);
      processedRootJointYaw.set(tempRots[0]);
      processedRootJointPitch.set(tempRots[1]);
      processedRootJointRoll.set(tempRots[2]);
   }
   
   private void integrateError()
   {
      double yawNoise = (random.nextDouble() - 0.5) * Math.PI * noiseScalar_yaw.getDoubleValue() + noiseBias_yaw.getDoubleValue();
      double pitchNoise = (random.nextDouble() - 0.5) * Math.PI * noiseScalar_pitch.getDoubleValue() + noiseBias_pitch.getDoubleValue();
      double rollNoise = (random.nextDouble() - 0.5) * Math.PI * noiseScalar_roll.getDoubleValue() + noiseBias_roll.getDoubleValue();
      
      rotationNoise.setYawPitchRoll(yawNoise, pitchNoise, rollNoise);
      rotationError.multiply(rotationNoise);
      
      double xNoise = (random.nextDouble() - 0.5) * noiseScalar_x.getDoubleValue() + noiseBias_x.getDoubleValue(); 
      double yNoise =  (random.nextDouble() - 0.5) * noiseScalar_y.getDoubleValue() + noiseBias_y.getDoubleValue(); 
      double zNoise = (random.nextDouble() - 0.5) * noiseScalar_z.getDoubleValue() + noiseBias_z.getDoubleValue(); 
      translationNoise.set(xNoise, yNoise, zNoise);
      
      
      pelvisPose.getRotation(pelvisRotation);
      
      pelvisRotation.multiply(rotationError);
      pelvisRotation.transform(translationNoise);
      
      translationError.add(translationNoise);
   }
}
