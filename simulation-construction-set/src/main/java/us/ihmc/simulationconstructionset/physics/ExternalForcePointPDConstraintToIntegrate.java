package us.ihmc.simulationconstructionset.physics;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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

   private final YoFramePoint3D yoConnectionAPosition;
   private final YoFramePoint3D yoConnectionBPosition;
   protected final YoFrameVector3D yoConnectionPositionError;
   private final YoDouble yoConnectionPositionErrorMagnitude;

   private final YoFrameVector3D yoConnectionAVelocity;
   private final YoFrameVector3D yoConnectionBVelocity;
   private final YoFrameVector3D yoConnectionVelocityError;
   private final YoDouble yoConnectionVelocityErrorMagnitude;

   // Temporary variables:
   private final FramePoint3D connectionAPosition = new FramePoint3D(worldFrame);
   private final FramePoint3D connectionBPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D connectionPositionError;
   private final FrameVector3D connectionAVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D connectionBVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D connectionVelocityError;
   private final FrameVector3D springForce;
   private final FrameVector3D damperForce;
   private final FrameVector3D totalForce = new FrameVector3D(worldFrame);

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
      yoConnectionPositionError = new YoFrameVector3D(name + "_ConnectionPositionError", worldFrame, registry);
      yoConnectionPositionErrorMagnitude = new YoDouble(name + "_ConnectionPositionErrorMagnitude", registry);

      yoConnectionAVelocity = connectionPointA.getYoVelocity();
      yoConnectionBVelocity = connectionPointB.getYoVelocity();
      yoConnectionVelocityError = new YoFrameVector3D(name + "_ConnectionVelocityError", worldFrame, registry);
      yoConnectionVelocityErrorMagnitude = new YoDouble(name + "_ConnectionVelocityErrorMagnitude", registry);

      parentRegistry.addChild(registry);

      connectionPositionError = new FrameVector3D(worldFrame);
      connectionVelocityError = new FrameVector3D(worldFrame);

      springForce = new FrameVector3D(worldFrame);
      damperForce = new FrameVector3D(worldFrame);
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

      springForce.setAndScale(stiffness.getDoubleValue(), connectionPositionError);
      damperForce.setAndScale(damping.getDoubleValue(), connectionVelocityError);

      totalForce.add(springForce);
      totalForce.add(damperForce);

      connectionPointA.setForce(totalForce);
      totalForce.scale(-1.0);
      connectionPointB.setForce(totalForce);
   }

   private void updateFrameAndKinematics()
   {
      connectionAPosition.setIncludingFrame(yoConnectionAPosition);
      connectionBPosition.setIncludingFrame(yoConnectionBPosition);

      connectionAVelocity.setIncludingFrame(yoConnectionAVelocity);
      connectionBVelocity.setIncludingFrame(yoConnectionBVelocity);
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
