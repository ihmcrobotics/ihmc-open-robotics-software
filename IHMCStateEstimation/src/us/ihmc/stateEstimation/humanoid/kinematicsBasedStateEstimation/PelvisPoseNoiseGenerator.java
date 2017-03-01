package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.Random;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
   
   
   private final YoFramePoint nonProcessedRootJointPosition;
   private final YoFrameQuaternion nonProcessedRootJointQuaternion;
   private final DoubleYoVariable nonProcessedRootJointPitch;
   private final DoubleYoVariable nonProcessedRootJointRoll;
   private final DoubleYoVariable nonProcessedRootJointYaw;
   
   private final YoFramePoint processedRootJointPosition;
   private final YoFrameQuaternion processedRootJointQuaternion;
   private final DoubleYoVariable processedRootJointPitch;
   private final DoubleYoVariable processedRootJointRoll;
   private final DoubleYoVariable processedRootJointYaw;
   
   private final DoubleYoVariable error_x;
   private final DoubleYoVariable error_y;
   private final DoubleYoVariable error_z;
   private final DoubleYoVariable error_yaw;
   private final DoubleYoVariable error_pitch;
   private final DoubleYoVariable error_roll;

   private final DoubleYoVariable noiseBias_x;
   private final DoubleYoVariable noiseBias_y;
   private final DoubleYoVariable noiseBias_z;
                                       
   private final DoubleYoVariable noiseBias_roll;
   private final DoubleYoVariable noiseBias_pitch;
   private final DoubleYoVariable noiseBias_yaw;
   
   private final DoubleYoVariable noiseScalar_x;
   private final DoubleYoVariable noiseScalar_y;
   private final DoubleYoVariable noiseScalar_z;
   private final DoubleYoVariable noiseScalar_yaw;
   private final DoubleYoVariable noiseScalar_pitch;
   private final DoubleYoVariable noiseScalar_roll;
   
   public PelvisPoseNoiseGenerator(FullInverseDynamicsStructure inverseDynamicsStructure, YoVariableRegistry parentRegistry)
   {
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      registry = new YoVariableRegistry("PelvisPoseNoiseGenerator");
      parentRegistry.addChild(registry);
      
      rotationError.setIdentity();
      pelvisRotation.setIdentity();
      
      nonProcessedRootJointPosition = new YoFramePoint("PelvisPose_beforeNoise_position", ReferenceFrame.getWorldFrame(), registry);
      nonProcessedRootJointQuaternion = new YoFrameQuaternion("PelvisPose_beforeNoise_quaternion", ReferenceFrame.getWorldFrame(), registry);
      nonProcessedRootJointYaw = new DoubleYoVariable("PelvisPose_beforeNoise_yaw", registry);
      nonProcessedRootJointPitch = new DoubleYoVariable("PelvisPose_beforeNoise_pitch", registry);
      nonProcessedRootJointRoll = new DoubleYoVariable("PelvisPose_beforeNoise_roll", registry);
      
      processedRootJointPosition = new YoFramePoint("PelvisPose_afterNoise_position", ReferenceFrame.getWorldFrame(), registry);
      processedRootJointQuaternion = new YoFrameQuaternion("PelvisPose_afterNoise_quaternion", ReferenceFrame.getWorldFrame(), registry);
      processedRootJointYaw = new DoubleYoVariable("PelvisPose_afterNoise_yaw", registry);
      processedRootJointPitch = new DoubleYoVariable("PelvisPose_afterNoise_pitch", registry);
      processedRootJointRoll = new DoubleYoVariable("PelvisPose_afterNoise_roll", registry);
      
      error_x = new DoubleYoVariable("PelvisPose_noise_x", registry);            
      error_y = new DoubleYoVariable("PelvisPose_noise_y", registry);            
      error_z = new DoubleYoVariable("PelvisPose_noise_z", registry);
      
      error_yaw = new DoubleYoVariable("PelvisPose_noise_yaw", registry);           
      error_pitch = new DoubleYoVariable("PelvisPose_noise_pitch", registry);       
      error_roll = new DoubleYoVariable("PelvisPose_noise_roll", registry);        
                        
      noiseBias_x = new DoubleYoVariable("PelvisPose_bias_x", registry);       
      noiseBias_y = new DoubleYoVariable("PelvisPose_bias_y", registry);       
      noiseBias_z = new DoubleYoVariable("PelvisPose_bias_z", registry);       
                        
      noiseBias_roll = new DoubleYoVariable("PelvisPose_bias_roll", registry);   
      noiseBias_pitch = new DoubleYoVariable("PelvisPose_bias_pitch", registry);   
      noiseBias_yaw = new DoubleYoVariable("PelvisPose_bias_yaw", registry);   
                        
      noiseScalar_x = new DoubleYoVariable("PelvisPose_NoiseScalar_x", registry);       
      noiseScalar_y = new DoubleYoVariable("PelvisPose_NoiseScalar_y", registry);   
      noiseScalar_z = new DoubleYoVariable("PelvisPose_NoiseScalar_z", registry);
      
      noiseScalar_yaw = new DoubleYoVariable("PelvisPose_NoiseScalar_yaw", registry);       
      noiseScalar_pitch = new DoubleYoVariable("PelvisPose_NoiseScalar_pitch", registry);   
      noiseScalar_roll = new DoubleYoVariable("PelvisPose_NoiseScalar_roll", registry);     
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
