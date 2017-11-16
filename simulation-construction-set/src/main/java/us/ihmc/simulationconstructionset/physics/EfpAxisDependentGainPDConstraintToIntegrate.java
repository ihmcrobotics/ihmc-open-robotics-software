package us.ihmc.simulationconstructionset.physics;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class runs a PD controller between two external force points with
 * distinct x, y, and z gains. The reference frame provided defines these axes
 */
public class EfpAxisDependentGainPDConstraintToIntegrate implements FunctionToIntegrate
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoDouble stiffnessX, stiffnessY, stiffnessZ;
   private final YoDouble dampingX, dampingY, dampingZ;

   private final ExternalForcePoint connectionPointA;
   private final ExternalForcePoint connectionPointB;

   private final YoFramePoint yoConnectionAPosition;
   private final YoFramePoint yoConnectionBPosition;
   private final YoFrameVector yoConnectionPositionError;

   private final YoFrameVector yoConnectionAVelocity;
   private final YoFrameVector yoConnectionBVelocity;
   private final YoFrameVector yoConnectionVelocityError;

   private final ReferenceFrame stiffnessFrame;

   // Temporary variables:
   // private final Transform3D stiffnessFrameToTuple
   private final FramePoint3D connectionAPosition = new FramePoint3D(worldFrame);
   private final FramePoint3D connectionBPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D connectionPositionError;
   private final FrameVector3D connectionAVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D connectionBVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D connectionVelocityError;
   private final Vector3D springForce = new Vector3D();
   private final Vector3D damperForce = new Vector3D();
   private final FrameVector3D totalForce = new FrameVector3D(worldFrame);
   private final RotationMatrix stiffnessFrameToWorldFrameRotation = new RotationMatrix();
   private final Matrix3D stiffnessGainMatrix = new Matrix3D();
   private final Matrix3D dampingGainMatrix = new Matrix3D();
   private final Matrix3D stiffnessMatrix = new Matrix3D();
   private final Matrix3D dampingMatrix = new Matrix3D();

   public EfpAxisDependentGainPDConstraintToIntegrate(String name, ExternalForcePoint connectionPointA, ExternalForcePoint connectionPointB,
         ReferenceFrame stiffnessFrame, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);

      stiffnessX = new YoDouble(name + "_StiffnessX", registry);
      stiffnessY = new YoDouble(name + "_StiffnessY", registry);
      stiffnessZ = new YoDouble(name + "_StiffnessZ", registry);

      dampingX = new YoDouble(name + "_DampingX", registry);
      dampingY = new YoDouble(name + "_DampingY", registry);
      dampingZ = new YoDouble(name + "_DampingZ", registry);

      this.connectionPointA = connectionPointA;
      this.connectionPointB = connectionPointB;

      yoConnectionAPosition = connectionPointA.getYoPosition();
      yoConnectionBPosition = connectionPointB.getYoPosition();
      yoConnectionPositionError = new YoFrameVector(name + "_ConnectionPositionError", worldFrame, registry);

      yoConnectionAVelocity = connectionPointA.getYoVelocity();
      yoConnectionBVelocity = connectionPointB.getYoVelocity();
      yoConnectionVelocityError = new YoFrameVector(name + "_ConnectionVelocityErrorMagnitude", worldFrame, registry);

      parentRegistry.addChild(registry);

      connectionPositionError = new FrameVector3D(worldFrame);
      connectionVelocityError = new FrameVector3D(worldFrame);

      this.stiffnessFrame = stiffnessFrame;
   }

   public void setStiffness(Axis axis, double stiffness)
   {
      switch (axis)
      {
      case X:
         stiffnessX.set(stiffness);
      case Y:
         stiffnessY.set(stiffness);
      case Z:
         stiffnessZ.set(stiffness);
      }
   }

   public void setDamping(Axis axis, double damping)
   {
      switch (axis)
      {
      case X:
         dampingX.set(damping);
      case Y:
         dampingY.set(damping);
      case Z:
         dampingZ.set(damping);
      }
   }

   private void updateClosedJoint()
   {
      updateFrameAndKinematics();

      computeErrors();

      stiffnessFrame.getTransformToDesiredFrame(worldFrame).getRotation(stiffnessFrameToWorldFrameRotation);

      stiffnessGainMatrix.setM00(stiffnessX.getDoubleValue());
      stiffnessGainMatrix.setM11(stiffnessY.getDoubleValue());
      stiffnessGainMatrix.setM22(stiffnessZ.getDoubleValue());

      dampingGainMatrix.setM00(dampingX.getDoubleValue());
      dampingGainMatrix.setM11(dampingY.getDoubleValue());
      dampingGainMatrix.setM22(dampingZ.getDoubleValue());

      stiffnessMatrix.set(stiffnessFrameToWorldFrameRotation);
      stiffnessMatrix.multiply(stiffnessGainMatrix);
      stiffnessFrameToWorldFrameRotation.transpose();
      stiffnessMatrix.multiply(stiffnessFrameToWorldFrameRotation);
      stiffnessMatrix.transform(connectionPositionError.getVector(), springForce);

      stiffnessMatrix.set(stiffnessFrameToWorldFrameRotation);
      stiffnessMatrix.multiply(stiffnessGainMatrix);
      stiffnessFrameToWorldFrameRotation.transpose();
      stiffnessMatrix.multiply(stiffnessFrameToWorldFrameRotation);
      stiffnessMatrix.transform(connectionPositionError.getVector(), springForce);

      stiffnessFrameToWorldFrameRotation.transpose();

      dampingMatrix.set(stiffnessFrameToWorldFrameRotation);
      dampingMatrix.multiply(dampingGainMatrix);
      stiffnessFrameToWorldFrameRotation.transpose();
      dampingMatrix.multiply(stiffnessFrameToWorldFrameRotation);
      dampingMatrix.transform(connectionVelocityError.getVector(), damperForce);

      totalForce.setToZero(worldFrame);
      totalForce.add(springForce);
      totalForce.add(damperForce);
      
      System.out.println("total force = " + totalForce);

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

      stiffnessFrame.update();
   }

   private void computeErrors()
   {
      connectionPositionError.sub(connectionBPosition, connectionAPosition);
      yoConnectionPositionError.set(connectionPositionError);

      connectionVelocityError.sub(connectionBVelocity, connectionAVelocity);
      yoConnectionVelocityError.set(connectionVelocityError);
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
