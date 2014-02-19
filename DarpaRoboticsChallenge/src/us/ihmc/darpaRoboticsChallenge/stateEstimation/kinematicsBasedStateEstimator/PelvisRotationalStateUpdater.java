package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;

import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.OrientationSensorConfiguration;
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
 * (Based on {@link us.ihmc.sensorProcessing.stateEstimation.IMUSelectorAndDataConverter} and {@link us.ihmc.sensorProcessing.stateEstimation.OrientationStateRobotModelUpdater})
 * @author Sylvain
 *
 */
public class PelvisRotationalStateUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameOrientation yoEstimationLinkOrientation;
   private final YoFrameOrientation yoRootJointFrameOrientation;
   private final YoFrameVector yoEstimationLinkAngularVelocity;
   private final YoFrameVector yoRootJointFrameAngularVelocity;
   
   private final SixDoFJoint rootJoint;
   private final ReferenceFrame frameAfterRootJoint;
   private final TwistCalculator twistCalculator;
   
   private final OrientationSensorConfiguration selectedOrientationSensorConfiguration;
   private final AngularVelocitySensorConfiguration selectedAngularVelocitySensorConfiguration;
   
   private final ReferenceFrame estimationFrame;
   private final RigidBody estimationLink;
   
   private final ReferenceFrame orientationMeasurementFrame;
   private final ReferenceFrame angularVelocityMeasurementFrame;
   private final RigidBody angularVelocityMeasurementLink;

   public PelvisRotationalStateUpdater(List<OrientationSensorConfiguration> orientationSensorConfigurations,
         List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations, FullInverseDynamicsStructure inverseDynamicsStructure, YoVariableRegistry parentRegistry)
   {
      if (orientationSensorConfigurations.size() > 1 || angularVelocitySensorConfigurations.size() > 1)
         throw new RuntimeException("We are assuming there is only 1 IMU for right now.. Got " + orientationSensorConfigurations.size());

      selectedOrientationSensorConfiguration = orientationSensorConfigurations.get(0);
      selectedAngularVelocitySensorConfiguration = angularVelocitySensorConfigurations.get(0);
      
      rootJoint = inverseDynamicsStructure.getRootJoint();
      frameAfterRootJoint = rootJoint.getFrameAfterJoint();
      twistCalculator = inverseDynamicsStructure.getTwistCalculator();

      this.estimationFrame = inverseDynamicsStructure.getEstimationFrame();
      this.estimationLink = inverseDynamicsStructure.getEstimationLink();
      
      this.orientationMeasurementFrame = selectedOrientationSensorConfiguration.getMeasurementFrame();
      this.angularVelocityMeasurementFrame = selectedAngularVelocitySensorConfiguration.getMeasurementFrame();
      this.angularVelocityMeasurementLink = selectedAngularVelocitySensorConfiguration.getAngularVelocityMeasurementLink();

      yoEstimationLinkOrientation = new YoFrameOrientation("estimatedEstimationLink", worldFrame, registry);
      yoRootJointFrameOrientation = new YoFrameOrientation("estimatedRootJointFrame", worldFrame, registry);
      yoEstimationLinkAngularVelocity = new YoFrameVector("estimatedEstimationLinkAngularVelocity", frameAfterRootJoint, registry);
      yoRootJointFrameAngularVelocity = new YoFrameVector("estimatedRootJointFrameAngularVelocity", frameAfterRootJoint, registry);
      
      parentRegistry.addChild(registry);
      
      angularVelocityEstimationLinkRelativeToWorld = new FrameVector(estimationFrame);
      angularVelocityRootBodyRelativeToWorld = new FrameVector(frameAfterRootJoint);
   }

   public void initialize()
   {
      updatePelvisOrientationAndAngularVelocity();
   }

   public void updatePelvisOrientationAndAngularVelocity()
   {
      updateRootJointRotation();
      updateRootJointTwistAngularPart();
      updateViz();
   }

   private final Transform3D transformFromMeasurementFrameToWorld = new Transform3D();
   
   private final Transform3D transformFromEstimationLinkFrameToMeasurementFrame = new Transform3D();
   private final Transform3D transformFromEstimationLinkToWorld = new Transform3D();

   private final Transform3D transformFromRootJointFrameToWorld = new Transform3D();
   private final Transform3D transformFromRootJointFrameToEstimationFrame = new Transform3D();

   private final Matrix3d rotationFromEstimationLinkToWorld = new Matrix3d();
   private final Matrix3d rotationFromRootJointFrameToWorld = new Matrix3d();

   private void updateRootJointRotation()
   {
      // R_{measurementFrame}^{world}
      transformFromMeasurementFrameToWorld.set(selectedOrientationSensorConfiguration.getOutputPort().getData());

      // R_{estimationLink}^{measurementFrame}
      estimationFrame.getTransformToDesiredFrame(transformFromEstimationLinkFrameToMeasurementFrame, orientationMeasurementFrame);

      // R_{estimationLink}^{world} = R_{measurementFrame}^{world} * R_{estimationLink}^{measurementFrame}
      transformFromEstimationLinkToWorld.mul(transformFromMeasurementFrameToWorld, transformFromEstimationLinkFrameToMeasurementFrame);
      transformFromEstimationLinkToWorld.get(rotationFromEstimationLinkToWorld);

      // R_{root}^{estimationLink}
      frameAfterRootJoint.getTransformToDesiredFrame(transformFromRootJointFrameToEstimationFrame, estimationFrame);

      // R_{root}^{world} = R_{estimationLink}^{world} * R_{root}^{estimationLink}
      transformFromRootJointFrameToWorld.mul(transformFromEstimationLinkToWorld, transformFromRootJointFrameToEstimationFrame);
      transformFromRootJointFrameToWorld.get(rotationFromRootJointFrameToWorld);

      rootJoint.setRotation(rotationFromRootJointFrameToWorld);
      frameAfterRootJoint.update();
   }

   /**
    * Angular velocity of the measurement link, with respect to world.
    */
   private final FrameVector angularVelocityMeasurementLinkRelativeToWorld = new FrameVector();

   /**
    * Angular velocity of the estimation link, with respect to the measurement link.
    */
   private final FrameVector angularVelocityEstimationLinkRelativeToMeasurementLink = new FrameVector();
   /**
    * Angular velocity of the estimation link, with respect to world.
    */
   private final FrameVector angularVelocityEstimationLinkRelativeToWorld;

   /**
    * Angular velocity of the root body, with respect to the estimation link.
    */
   private final FrameVector angularVelocityRootBodyRelativeToEstimationLink = new FrameVector();
   /**
    * Angular velocity of the root body, with respect to world.
    */
   private final FrameVector angularVelocityRootBodyRelativeToWorld;

   /**
    * Twist of the estimation link, with respect to the measurement link.
    */
   private final Twist twistEstimationLinkRelativeToMeasurementLink = new Twist();
   /**
    * Twist of the root body, with respect to the estimation link.
    */
   private final Twist twistRootBodyRelativeToEstimationLink = new Twist();
   /**
    * Twist of the root body, with respect to world.
    */
   private final Twist twitRootBodyRelativeToWorld = new Twist();

   private void updateRootJointTwistAngularPart()
   {
      // T_{estimationLink}^{estimationLinkCoMFrame, measurementLink}
      twistCalculator.packRelativeTwist(twistEstimationLinkRelativeToMeasurementLink, angularVelocityMeasurementLink, estimationLink);
      // T_{estimationLink}^{estimationFrame, measurementLink}
      twistEstimationLinkRelativeToMeasurementLink.changeFrame(estimationFrame);
      
      // omega_{estimationLink}^{estimationFrame, measurementLink}
      twistEstimationLinkRelativeToMeasurementLink.packAngularPart(angularVelocityEstimationLinkRelativeToMeasurementLink);

      // omega_{measurementLink}^{measurementFrame, world}
      angularVelocityMeasurementLinkRelativeToWorld.set(angularVelocityMeasurementFrame, selectedAngularVelocitySensorConfiguration.getOutputPort().getData()); 

      // omega_{measurementLink}^{estimationFrame, world}
      angularVelocityMeasurementLinkRelativeToWorld.changeFrame(estimationFrame);
      
      // omega_{estimationLink}^{estimationFrame, world} = omega_{estimationLink}^{estimationFrame, measurementLink} + omega_{measurementLink}^{estimationFrame, world}
      angularVelocityEstimationLinkRelativeToWorld.add(angularVelocityEstimationLinkRelativeToMeasurementLink, angularVelocityMeasurementLinkRelativeToWorld);
      
      // T_{rootBody}^{rootBodyCoMFrame, estimationLink}
      twistCalculator.packRelativeTwist(twistRootBodyRelativeToEstimationLink, estimationLink, rootJoint.getSuccessor());
      // T_{rootBody}^{rootJointFrame, estimationLink}
      twistRootBodyRelativeToEstimationLink.changeFrame(frameAfterRootJoint);

      // omega_{rootBody}^{rootJointFrame, estimationLink}
      twistRootBodyRelativeToEstimationLink.packAngularPart(angularVelocityRootBodyRelativeToEstimationLink);

      // omega_{estimationLink}^{rootJointFrame, world}
      angularVelocityEstimationLinkRelativeToWorld.changeFrame(frameAfterRootJoint);

      // omega_{rootBody}^{rootJointFrame, world} = omega_{estimationLink}^{rootJointFrame, world} + omega_{rootBody}^{rootJointFrame, estimationLink}
      angularVelocityRootBodyRelativeToWorld.add(angularVelocityRootBodyRelativeToEstimationLink, angularVelocityEstimationLinkRelativeToWorld);

      rootJoint.packJointTwist(twitRootBodyRelativeToWorld);
      twitRootBodyRelativeToWorld.setAngularPart(angularVelocityRootBodyRelativeToWorld.getVector());
      rootJoint.setJointTwist(twitRootBodyRelativeToWorld);
      twistCalculator.compute();
   }

   private void updateViz()
   {
      yoEstimationLinkOrientation.checkReferenceFrameMatch(worldFrame);
      yoEstimationLinkOrientation.set(rotationFromEstimationLinkToWorld);
      
      yoRootJointFrameOrientation.checkReferenceFrameMatch(worldFrame);
      yoRootJointFrameOrientation.set(rotationFromRootJointFrameToWorld);
      
      yoEstimationLinkAngularVelocity.set(angularVelocityEstimationLinkRelativeToWorld);
      yoRootJointFrameAngularVelocity.set(angularVelocityRootBodyRelativeToWorld);
   }
}
