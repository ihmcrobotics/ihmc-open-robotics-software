package us.ihmc.simulationconstructionset.physics;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;

public class ExternalForcePointPDConstraintToIntegrate implements FunctionToIntegrate
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final YoVariableRegistry registry;

   private final YoDouble stiffness;
   private final YoDouble damping;

   protected final ExternalForcePoint connectionPointA;
   protected final ExternalForcePoint connectionPointB;

   private final YoFramePoint yoConnectionAPosition;
   private final YoFramePoint yoConnectionBPosition;
   protected final YoFrameVector yoConnectionPositionError;
   private final YoDouble yoConnectionPositionErrorMagnitude;

   private final YoFrameVector yoConnectionAVelocity;
   private final YoFrameVector yoConnectionBVelocity;
   private final YoFrameVector yoConnectionVelocityError;
   private final YoDouble yoConnectionVelocityErrorMagnitude;

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

   public ExternalForcePointPDConstraintToIntegrate(String name, ExternalForcePoint connectionPointA, ExternalForcePoint connectionPointB,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      stiffness = new YoDouble(name + "_Stiffness", registry);
      damping = new YoDouble(name + "_Damping", registry);

      this.connectionPointA = connectionPointA;
      this.connectionPointB = connectionPointB;

      yoConnectionAPosition = connectionPointA.getYoPosition();
      yoConnectionBPosition = connectionPointB.getYoPosition();
      yoConnectionPositionError = new YoFrameVector(name + "_ConnectionPositionError", worldFrame, registry);
      yoConnectionPositionErrorMagnitude = new YoDouble(name + "_ConnectionPositionErrorMagnitude", registry);

      yoConnectionAVelocity = connectionPointA.getYoVelocity();
      yoConnectionBVelocity = connectionPointB.getYoVelocity();
      yoConnectionVelocityError = new YoFrameVector(name + "_ConnectionVelocityError", worldFrame, registry);
      yoConnectionVelocityErrorMagnitude = new YoDouble(name + "_ConnectionVelocityErrorMagnitude", registry);

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

   protected void updateClosedJoint()
   {
      updateFrameAndKinematics();
      computeErrors();

      totalForce.setToZero(worldFrame);

      springForce.scale(stiffness.getDoubleValue(), connectionPositionError);
      damperForce.scale(damping.getDoubleValue(), connectionVelocityError);

      totalForce.add(springForce);
      totalForce.add(damperForce);

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
   public YoDouble[] getOutputVariables()
   {
      return null;
   }

}
