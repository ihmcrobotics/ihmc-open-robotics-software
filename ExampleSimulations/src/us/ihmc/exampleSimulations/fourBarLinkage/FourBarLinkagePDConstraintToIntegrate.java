package us.ihmc.exampleSimulations.fourBarLinkage;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;

public class FourBarLinkagePDConstraintToIntegrate implements FunctionToIntegrate
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final DoubleYoVariable stiffness;
   private final DoubleYoVariable damping;

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

   // Temporary variables:
   private final FramePoint connectionAPosition = new FramePoint(worldFrame);
   private final FramePoint connectionBPosition = new FramePoint(worldFrame);
   private final FrameVector connectionPositionError;
   private final FrameVector connectionAVelocity = new FrameVector(worldFrame);
   private final FrameVector connectionBVelocity = new FrameVector(worldFrame);
   private final FrameVector connectionVelocityError;
   private final FrameVector springForce;
   private final FrameVector damperForce;
   private final FrameVector totalForce = new FrameVector(worldFrame);

   public FourBarLinkagePDConstraintToIntegrate(String name, ExternalForcePoint connectionPointA, ExternalForcePoint connectionPointB,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      stiffness = new DoubleYoVariable(name + "_Stiffnes", registry);
      damping = new DoubleYoVariable(name + "_Damping", registry);

      this.connectionPointA = connectionPointA;
      this.connectionPointB = connectionPointB;

      yoConnectionAPosition = connectionPointA.getYoPosition();
      yoConnectionBPosition = connectionPointB.getYoPosition();
      yoConnectionPositionError = new YoFrameVector(name + "_ConnectionPositionError", worldFrame, registry);
      yoConnectionPositionErrorMagnitude = new DoubleYoVariable(name + "_ConnectionPositionErrorMagnitude", registry);

      yoConnectionAVelocity = connectionPointA.getYoVelocity();
      yoConnectionBVelocity = connectionPointB.getYoVelocity();
      yoConnectionVelocityError = new YoFrameVector(name + "_ConnectionVelocityErrorMagnitude", worldFrame, registry);
      yoConnectionVelocityErrorMagnitude = new DoubleYoVariable(name + "_ConnectionVelocityErrorMagnitude", registry);

      parentRegistry.addChild(registry);

      connectionPositionError = new FrameVector(worldFrame);
      connectionVelocityError = new FrameVector(worldFrame);

      springForce = new FrameVector(worldFrame);
      damperForce = new FrameVector(worldFrame);
   }

   public void setStiffness(double stiffness)
   {
      this.stiffness.set(stiffness);
   }

   public void setDamping(double damping)
   {
      this.damping.set(damping);
   }

   private void updateClosedJoint()
   {
      updateFrameAndKinematics();

      computeErrors();

      totalForce.setToZero(worldFrame);

      springForce.scale(stiffness.getDoubleValue(), connectionPositionError);
      damperForce.scale(damping.getDoubleValue(), connectionVelocityError);

      totalForce.add(springForce);
      totalForce.add(damperForce);

      totalForce.changeFrame(worldFrame);

      connectionPointA.setForce(totalForce.getVector());
      totalForce.scale(-1.0);
      connectionPointB.setForce(totalForce.getVector());
   }

   private void updateFrameAndKinematics()
   {
      yoConnectionAPosition.getFrameTupleIncludingFrame(connectionAPosition);
      yoConnectionBPosition.getFrameTupleIncludingFrame(connectionBPosition);

      yoConnectionAVelocity.getFrameTupleIncludingFrame(connectionAVelocity);
      yoConnectionBVelocity.getFrameTupleIncludingFrame(connectionBVelocity);
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
