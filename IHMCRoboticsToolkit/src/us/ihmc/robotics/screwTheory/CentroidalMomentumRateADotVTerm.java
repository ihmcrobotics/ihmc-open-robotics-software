package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CentroidalMomentumRateADotVTerm
{
   private final InverseDynamicsJoint[] jointsInOrder;
   private final ReferenceFrame centerOfMassFrame;
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationVector rootAcceleration;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final SpatialAccelerationVector tempSpatialAcceleration;
   private final CentroidalMomentumMatrix centroidalMomentumMatrix;
   private final RigidBody rootBody;
   private final DenseMatrix64F v;
   private final double robotMass;
   private final DenseMatrix64F aDotV;

   private final Momentum tempMomentum;
   private final Twist tempTwist;
   private final Twist tempCoMTwist;
   private final Vector3D comVelocityVector;
   private final Twist comTwist;
   private final Vector3D tempVector;
   private final Momentum leftSide;

   public CentroidalMomentumRateADotVTerm(RigidBody rootBody, ReferenceFrame centerOfMassFrame, CentroidalMomentumMatrix centroidalMomentumMatrix,
         double robotMass, DenseMatrix64F v)
   {
      this.jointsInOrder = ScrewTools.computeSupportAndSubtreeJoints(rootBody);

      this.robotMass = robotMass;

      this.centerOfMassFrame = centerOfMassFrame;
      this.rootBody = rootBody;
      this.v = v;

      this.twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootBody);
      this.centroidalMomentumMatrix = centroidalMomentumMatrix;

      this.rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), rootBody.getBodyFixedFrame());
      this.spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, ReferenceFrame.getWorldFrame(), this.rootAcceleration,
            this.twistCalculator, true, false, false);

      this.comTwist = new Twist(centerOfMassFrame, rootBody.getBodyFixedFrame(), centerOfMassFrame);
      comTwist.setToZero();

      this.comVelocityVector = new Vector3D();
      this.aDotV = new DenseMatrix64F(6, 1);
      this.tempSpatialAcceleration = new SpatialAccelerationVector();
      this.tempMomentum = new Momentum();
      this.tempTwist = new Twist();
      this.tempCoMTwist = new Twist();
      this.tempVector = new Vector3D(0, 0, 0);
      this.leftSide = new Momentum(centerOfMassFrame);
   }

   public void compute()
   {
      DenseMatrix64F centroidalMomentumMatrixDense = centroidalMomentumMatrix.getMatrix();

      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      computeCoMVelocity(centroidalMomentumMatrixDense);

      aDotV.zero();
      tempMomentum.setToZero();

      for (int i = 0; i < jointsInOrder.length; i++)
      {
         RigidBody rigidBody = jointsInOrder[i].getSuccessor();

         RigidBodyInertia inertia = rigidBody.getInertia(); // I

         twistCalculator.getRelativeTwist(rootBody, rigidBody, tempTwist);

         tempTwist.changeFrame(inertia.getExpressedInFrame());

         tempMomentum.compute(inertia, tempTwist); // I * J * v expressed in frame attached to CoM of rowRigidBody expressed as momentum 
         tempMomentum.changeFrame(centerOfMassFrame); // Adj^{T} * I * J * v

         tempCoMTwist.set(comTwist);

         tempCoMTwist.changeFrame(tempTwist.getExpressedInFrame());
         tempCoMTwist.sub(tempTwist); //Twist of the CoM w.r.t. body i expressed in CoM frame.
         tempCoMTwist.changeFrame(centerOfMassFrame);

         // Left multiply by ad^{T} : this is separated out rather than creating matrix objects to save computation time.
         // See method below for creating adjoint from twist for reference.
         // ad^{T} * Ad^{T} * I * J * v
         // The order of these cross products are backwards because I need -a x b and cross products are anticommutative,
         // so -a x b = b x a.
         leftSide.setToZero();
         tempVector.cross(tempMomentum.getLinearPart(), tempCoMTwist.getLinearPart());
         leftSide.addAngularPart(tempVector);
         tempVector.cross(tempMomentum.getAngularPart(), tempCoMTwist.getAngularPart());
         leftSide.addAngularPart(tempVector);
         tempVector.cross(tempMomentum.getLinearPart(), tempCoMTwist.getAngularPart());
         leftSide.addLinearPart(tempVector);
         //
         CommonOps.add(aDotV, leftSide.toDenseMatrix(), aDotV);
         //Right Side
         // \dot{J} * v : Note: during creation of spatialAccelerationCalculator, the boolean for setting acceleration term to zero was set to true.
         spatialAccelerationCalculator.getAccelerationOfBody(rigidBody, tempSpatialAcceleration);

         inertia.changeFrame(tempSpatialAcceleration.getExpressedInFrame()); // easier to change the frame of inertia than the spatial acceleration

         // I * \dot{J} * v
         DenseMatrix64F inertiaTimesSpatialAccel = MatrixTools.mult(inertia.toMatrix(), tempSpatialAcceleration.toMatrix());

         //Express as momentum to make multiplying by adjoint easier
         tempMomentum.set(tempSpatialAcceleration.getExpressedInFrame(), inertiaTimesSpatialAccel);
         // Ad^{T} * I * \dot{J} * v
         tempMomentum.changeFrame(centerOfMassFrame);

         CommonOps.add(aDotV, tempMomentum.toDenseMatrix(), aDotV);
      }
   }

   /**
    * Compute the center of mass velocity given a centroidal momentum matrix.
    * Just takes linear portion of centroidal momentum matrix(bottom three rows),
    * multiplies by vector of joint velocities, and divide by the robots mass.
    */
   private void computeCoMVelocity(DenseMatrix64F centroidalMomentumMatrixDense)
   {
      comVelocityVector.set(0, 0, 0);

      for (int i = 0; i < centroidalMomentumMatrixDense.numCols; i++)
      {
         comVelocityVector.setX(comVelocityVector.getX() + centroidalMomentumMatrixDense.get(3, i) * v.get(i));
         comVelocityVector.setY(comVelocityVector.getY() + centroidalMomentumMatrixDense.get(4, i) * v.get(i));
         comVelocityVector.setZ(comVelocityVector.getZ() + centroidalMomentumMatrixDense.get(5, i) * v.get(i));
      }

      if (robotMass != 0.0)
      {
         comVelocityVector.scale(1 / robotMass);
      }
      else
      {
         throw new RuntimeException("Your robot has zero mass");
      }
      comTwist.setLinearPart(comVelocityVector); //Velocity of CoM w.r.t rootBody expressed in CoM frame. Angular part is zero.
   }

   public DenseMatrix64F getMatrix()
   {
      return aDotV;
   }
}
