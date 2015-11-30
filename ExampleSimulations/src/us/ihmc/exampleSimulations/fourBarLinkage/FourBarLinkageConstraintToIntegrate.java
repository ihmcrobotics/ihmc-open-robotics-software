package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.simulationconstructionset.Joint;

public class FourBarLinkageConstraintToIntegrate implements FunctionToIntegrate
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final DoubleYoVariable radialStiffness;
   private final DoubleYoVariable radialDamping;
   private final DoubleYoVariable axialStiffness;
   private final DoubleYoVariable axialDamping;

   private final ExternalForcePoint connectionPointA;
   private final ExternalForcePoint connectionPointB;

   private final YoFramePoint yoConnectionAPosition;
   private final YoFramePoint yoConnectionBPosition;
   private final YoFrameVector yoConnectionPositionError;
   private final DoubleYoVariable yoConnectionPositionErrorMagnitude;

   private final YoFrameVector yoConnectionAVelocity;
   private final YoFrameVector yoConnectionBVelocity;
   private final YoFrameVector yoConnectionVelocityError;
   private final DoubleYoVariable yoConnectionVelocityErrorMagnitude;

   private final DoubleYoVariable yoTempDouble;

   private final ReferenceFrame jointAReferenceFrame;

   private final FrameVector radialDirection;
   private final FrameVector axialDirection;

   // Temporary variables:
   private final FramePoint connectionAPosition = new FramePoint(worldFrame);
   private final FramePoint connectionBPosition = new FramePoint(worldFrame);
   private final FrameVector connectionPositionError;
   private final FrameVector connectionAVelocity = new FrameVector(worldFrame);
   private final FrameVector connectionBVelocity = new FrameVector(worldFrame);
   private final FrameVector connectionVelocityError;
   private final FrameVector radialSpringForce;
   private final FrameVector radialDamperForce;
   private final FrameVector axialSpringForce;
   private final FrameVector axialDamperForce;
   private final FrameVector totalForce = new FrameVector(worldFrame);

   public FourBarLinkageConstraintToIntegrate(String name, ExternalForcePoint connectionPointA, ExternalForcePoint connectionPointB, final Joint jointA,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      radialStiffness = new DoubleYoVariable(name + "_RadialStiffnes", registry);
      radialDamping = new DoubleYoVariable(name + "_RadialDamping", registry);
      axialStiffness = new DoubleYoVariable(name + "_AxialStiffness", registry);
      axialDamping = new DoubleYoVariable(name + "_AxialDamping", registry);

      yoTempDouble = new DoubleYoVariable(name + "TempDouble", registry);

      String namePrefix = jointA.getLink().getName();

      jointAReferenceFrame = new ReferenceFrame(namePrefix + "ReferenceFrame", worldFrame)
      {
         private static final long serialVersionUID = -4645732307402843565L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            jointA.getTransformToWorld(transformToParent);
         }
      };

      this.connectionPointA = connectionPointA;
      this.connectionPointB = connectionPointB;

      yoConnectionAPosition = connectionPointA.getYoPosition();
      yoConnectionBPosition = connectionPointB.getYoPosition();
      yoConnectionPositionError = new YoFrameVector(name + "_ConnectionPositionError", jointAReferenceFrame, registry);
      yoConnectionPositionErrorMagnitude = new DoubleYoVariable(name + "_ConnectionPositionErrorMagnitude", registry);

      yoConnectionAVelocity = connectionPointA.getYoVelocity();
      yoConnectionBVelocity = connectionPointB.getYoVelocity();
      yoConnectionVelocityError = new YoFrameVector(name + "_ConnectionVelocityErrorMagnitude", jointAReferenceFrame, registry);
      yoConnectionVelocityErrorMagnitude = new DoubleYoVariable(name + "_ConnectionVelocityErrorMagnitude", registry);

      axialDirection = new FrameVector(jointAReferenceFrame);
      axialDirection.set(new Vector3d(0.0, 0.0, 1.0)); // TODO: clean this up
      axialDirection.normalize();

      FrameVector jointAxis = new FrameVector(jointAReferenceFrame);
      jointAxis.set(jointA.physics.getUnitVector());
      radialDirection = new FrameVector(jointAReferenceFrame);
      radialDirection.cross(axialDirection, jointAxis);
      radialDirection.normalize();

      parentRegistry.addChild(registry);

      connectionPositionError = new FrameVector(jointAReferenceFrame);
      connectionVelocityError = new FrameVector(jointAReferenceFrame);
      radialSpringForce = new FrameVector(jointAReferenceFrame);
      radialDamperForce = new FrameVector(jointAReferenceFrame);
      axialSpringForce = new FrameVector(jointAReferenceFrame);
      axialDamperForce = new FrameVector(jointAReferenceFrame);
   }

   public void setAxialStiffness(double axialStiffness)
   {
      this.axialStiffness.set(axialStiffness);
   }

   public void setRadialStiffness(double radialStiffness)
   {
      this.radialStiffness.set(radialStiffness);
   }

   public void setAxialDamping(double axialDamping)
   {
      this.axialDamping.set(axialDamping);
   }

   public void setRadialDamping(double radialDamping)
   {
      this.radialDamping.set(radialDamping);
   }

   private void updateClosedJoint()
   {
      updateFrameAndKinematics();

      changeFrame(jointAReferenceFrame);

      computeErrors();

      totalForce.setToZero(jointAReferenceFrame);

      radialSpringForce.scale(radialStiffness.getDoubleValue(), connectionPositionError);
      double radialPositionErrorForce = radialSpringForce.dot(radialDirection);

      radialDamperForce.scale(radialDamping.getDoubleValue(), connectionVelocityError);
      double radialVelocityErrorForce = radialDamperForce.dot(radialDirection);

      FrameVector radialErrorForce = new FrameVector(radialDirection);
      radialErrorForce.scale(radialPositionErrorForce + radialVelocityErrorForce);
      totalForce.add(radialErrorForce);

      axialSpringForce.scale(axialStiffness.getDoubleValue(), connectionPositionError);
      double axialPositionErrorForce = axialSpringForce.dot(axialDirection);
      yoTempDouble.set(axialPositionErrorForce);

      axialDamperForce.scale(axialDamping.getDoubleValue(), connectionVelocityError);
      double axialVelocityErrorForce = axialDamperForce.dot(axialDirection);

      FrameVector axialErrorForce = new FrameVector(axialDirection);
      axialErrorForce.scale(axialPositionErrorForce + axialVelocityErrorForce);
      totalForce.add(axialErrorForce);

      totalForce.changeFrame(worldFrame);

      connectionPointA.setForce(totalForce.getVector());
      totalForce.scale(-1.0);
      connectionPointB.setForce(totalForce.getVector());

      connectionAPosition.changeFrame(worldFrame);
      connectionAPosition.changeFrame(jointAReferenceFrame);
   }

   private void updateFrameAndKinematics()
   {
      jointAReferenceFrame.update();

      yoConnectionAPosition.getFrameTupleIncludingFrame(connectionAPosition);
      yoConnectionBPosition.getFrameTupleIncludingFrame(connectionBPosition);

      yoConnectionAVelocity.getFrameTupleIncludingFrame(connectionAVelocity);
      yoConnectionBVelocity.getFrameTupleIncludingFrame(connectionBVelocity);
   }

   private void changeFrame(ReferenceFrame frame)
   {
      connectionAPosition.changeFrame(frame);
      connectionBPosition.changeFrame(frame);
      connectionAVelocity.changeFrame(frame);
      connectionBVelocity.changeFrame(frame);
   }

   private void computeErrors()
   {
      connectionPositionError.sub(connectionBPosition, connectionAPosition);
      yoConnectionPositionError.set(connectionPositionError);
      yoConnectionPositionErrorMagnitude.set(connectionPositionError.length());

      connectionVelocityError.sub(connectionBVelocity, connectionAVelocity);
      yoConnectionVelocityError.set(connectionVelocityError);
      yoConnectionVelocityErrorMagnitude.set(connectionVelocityError.length());
   }

   @Override
   public double[] computeDerivativeVector()
   {
      updateClosedJoint();

      return null;
   }

   @Override
   public int getVectorSize()
   {
      return 0;
   }

   @Override
   public DoubleYoVariable[] getOutputVariables()
   {
      return null;
   }

}