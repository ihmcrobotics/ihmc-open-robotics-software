package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;

import us.ihmc.sensorProcessing.stateEstimation.IMUSelectorAndDataConverter;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

/**
 * PelvisRotationalStateUpdater reads and transforms the orientation and angular velocity obtained from the IMU to update the pelvis orientation and angular velocity in world. 
 * (Based on {@link IMUSelectorAndDataConverter} and {@link OrientationStateRobotModelUpdater})
 * @author Sylvain
 *
 */
public class PelvisRotationalStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameOrientation yoRootJointFrameOrientation;

   private final YoFrameVector measurementFrameAngularVelocity;
   private final YoFrameVector measurementFrameAngularVelocityInWorld;
   
   private final SixDoFJoint rootJoint;
   private final ReferenceFrame rootJointFrame;
   private final TwistCalculator twistCalculator;
   
   private final OrientationSensorConfiguration selectedOrientationSensorConfiguration;
   private final AngularVelocitySensorConfiguration selectedAngularVelocitySensorConfiguration;
   
   private final ReferenceFrame orientationMeasurementFrame;
   private final ReferenceFrame angularVelocityMeasurementFrame;
   private final RigidBody angularVelocityMeasurementLink;

   public PelvisRotationalStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure,
         List<OrientationSensorConfiguration> orientationSensorConfigurations, List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations,
         YoVariableRegistry parentRegistry)
   {
      checkNumberOfSensors(orientationSensorConfigurations, angularVelocitySensorConfigurations);

      selectedOrientationSensorConfiguration = orientationSensorConfigurations.get(0);
      selectedAngularVelocitySensorConfiguration = angularVelocitySensorConfigurations.get(0);
      
      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();
      twistCalculator = inverseDynamicsStructure.getTwistCalculator();

      this.orientationMeasurementFrame = selectedOrientationSensorConfiguration.getMeasurementFrame();
      this.angularVelocityMeasurementFrame = selectedAngularVelocitySensorConfiguration.getMeasurementFrame();
      this.angularVelocityMeasurementLink = selectedAngularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();

      yoRootJointFrameOrientation = new YoFrameOrientation("estimatedRootJointFrame", worldFrame, registry);
      measurementFrameAngularVelocity = new YoFrameVector("measFrameAngularVelocity", angularVelocityMeasurementFrame, registry);
      measurementFrameAngularVelocityInWorld = new YoFrameVector("measFrameAngularVelocityWorld", worldFrame, registry);
      
      parentRegistry.addChild(registry);
      
      angularVelocityRootJointFrameRelativeToWorld = new FrameVector(rootJointFrame);
   }

   private void checkNumberOfSensors(List<OrientationSensorConfiguration> orientationSensorConfigurations,
         List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations)
   {
      if (orientationSensorConfigurations.size() > 1)
         throw new RuntimeException("We are assuming there is only 1 IMU for right now. Got " + orientationSensorConfigurations.size() + " orientation sensors.");
      
      if (angularVelocitySensorConfigurations.size() > 1)
         throw new RuntimeException("We are assuming there is only 1 IMU for right now. Got " + angularVelocitySensorConfigurations.size() + " sensors for angular velocity.");

      if (orientationSensorConfigurations.size() == 0)
         throw new RuntimeException("No sensor set up for the orientation.");
      
      if (angularVelocitySensorConfigurations.size() == 0)
         throw new RuntimeException("No sensor set up for the angular velocity.");
   }

   public void initialize()
   {
      updateRootJointOrientationAndAngularVelocity();
   }

   public void updateRootJointOrientationAndAngularVelocity()
   {
      updateRootJointRotation();
      updateRootJointTwistAngularPart();
      updateViz();
   }

   private final Transform3D transformFromMeasurementFrameToWorld = new Transform3D();
   
   private final Transform3D transformFromRootJointFrameToWorld = new Transform3D();
   private final Transform3D transformFromRootJointFrameToMeasurementFrame = new Transform3D();

   private final Matrix3d rotationFromRootJointFrameToWorld = new Matrix3d();

   private void updateRootJointRotation()
   {
      // R_{measurementFrame}^{world}
      transformFromMeasurementFrameToWorld.set(selectedOrientationSensorConfiguration.getOutputPort().getData());

      // R_{root}^{measurementFrame}
      rootJointFrame.getTransformToDesiredFrame(transformFromRootJointFrameToMeasurementFrame, orientationMeasurementFrame);

      // R_{root}^{world} = R_{estimationLink}^{world} * R_{root}^{estimationLink}
      transformFromRootJointFrameToWorld.mul(transformFromMeasurementFrameToWorld, transformFromRootJointFrameToMeasurementFrame);
      transformFromRootJointFrameToWorld.get(rotationFromRootJointFrameToWorld);

      rootJoint.setRotation(rotationFromRootJointFrameToWorld);
      rootJointFrame.update();
   }

   /** Angular velocity of the measurement link, with respect to world. */
   private final FrameVector angularVelocityMeasurementLinkRelativeToWorld = new FrameVector();

   /** Angular velocity of the estimation link, with respect to the measurement link. */
   private final FrameVector angularVelocityRootJointFrameRelativeToMeasurementLink = new FrameVector();

   /** Angular velocity of the root body, with respect to world. */
   private final FrameVector angularVelocityRootJointFrameRelativeToWorld;

   /** Twist of the estimation link, with respect to the measurement link. */
   private final Twist twistRootJointFrameRelativeToMeasurementLink = new Twist();
   /** Twist of the root body, with respect to world. */
   private final Twist twistRootBodyRelativeToWorld = new Twist();

   private void updateRootJointTwistAngularPart()
   {
      // T_{rootBody}^{rootBody, measurementLink}
      twistCalculator.packRelativeTwist(twistRootJointFrameRelativeToMeasurementLink, angularVelocityMeasurementLink, rootJoint.getSuccessor());
      // T_{rootBody}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.changeFrame(rootJointFrame);
      // T_{rootJointFrame}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.changeBodyFrameNoRelativeTwist(rootJointFrame);
      
      // omega_{rootJointFrame}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.packAngularPart(angularVelocityRootJointFrameRelativeToMeasurementLink);

      // omega_{measurementLink}^{measurementFrame, world}
      angularVelocityMeasurementLinkRelativeToWorld.setIncludingFrame(angularVelocityMeasurementFrame, selectedAngularVelocitySensorConfiguration.getOutputPort().getData()); 
      measurementFrameAngularVelocity.set(angularVelocityMeasurementLinkRelativeToWorld);

      // omega_{measurementLink}^{rootJointFrame, world}
      angularVelocityMeasurementLinkRelativeToWorld.changeFrame(rootJointFrame);

      // omega_{rootJointFrame}^{rootJointFrame, world} = omega_{rootJointFrame}^{rootJointFrame, measurementLink} + omega_{measurementLink}^{rootJointFrame, world}
      angularVelocityRootJointFrameRelativeToWorld.add(angularVelocityRootJointFrameRelativeToMeasurementLink, angularVelocityMeasurementLinkRelativeToWorld);
      
      angularVelocityMeasurementLinkRelativeToWorld.changeFrame(worldFrame);
      measurementFrameAngularVelocityInWorld.set(angularVelocityMeasurementLinkRelativeToWorld);

      rootJoint.packJointTwist(twistRootBodyRelativeToWorld);
      twistRootBodyRelativeToWorld.setAngularPart(angularVelocityRootJointFrameRelativeToWorld);
      rootJoint.setJointTwist(twistRootBodyRelativeToWorld);
      twistCalculator.compute();
   }

   private void updateViz()
   {
      yoRootJointFrameOrientation.checkReferenceFrameMatch(worldFrame);
      yoRootJointFrameOrientation.set(rotationFromRootJointFrameToWorld);
      
      yoRootJointFrameOrientation.checkReferenceFrameMatch(worldFrame);
      yoRootJointFrameOrientation.set(rotationFromRootJointFrameToWorld);
   }

   public void getEstimatedOrientation(FrameOrientation estimatedOrientation)
   {
      estimatedOrientation.set(rotationFromRootJointFrameToWorld);
   }

   public void getEstimatedAngularVelocity(FrameVector estimatedAngularVelocityToPack)
   {
      estimatedAngularVelocityToPack.setIncludingFrame(angularVelocityRootJointFrameRelativeToWorld);
   }
}
