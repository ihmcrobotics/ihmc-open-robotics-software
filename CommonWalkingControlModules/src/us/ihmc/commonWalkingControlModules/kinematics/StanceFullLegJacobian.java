package us.ihmc.commonWalkingControlModules.kinematics;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;
import Jama.Matrix;

public class StanceFullLegJacobian
{
   private final RobotSide robotSide;
   private final LegJointName[] legJointNames;

   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame footFrame;

   private final GeometricJacobian legJacobian;
   private final GeometricJacobian vtpJacobian;
   private final RevoluteJoint vtpXJoint;
   private final RevoluteJoint vtpYJoint;
   private final VTPXFrame vtpXFrame;
   private final VTPYFrame vtpYFrame;

   /**
    * Constructs a new StanceFullLegJacobian, for the given side of the robot
    * @param robotSpecificJointNames robot specific joint names
    * @param footHeight height of the origin of the foot frame above the sole of the foot
    */
   public StanceFullLegJacobian(RigidBody pelvis, RigidBody foot, RobotSide robotSide, CommonHumanoidReferenceFrames frames,
                                RobotSpecificJointNames robotSpecificJointNames, double footHeight)
   {
      this.robotSide = robotSide;
      this.legJointNames = robotSpecificJointNames.getLegJointNames();

      pelvisFrame = frames.getPelvisFrame();
      footFrame = frames.getFootFrame(robotSide);

      ReferenceFrame jacobianFrame = pelvisFrame;

      // create openChainJacobian
      legJacobian = new GeometricJacobian(pelvis, foot, jacobianFrame);

      // Build vtpJacobian
      vtpXFrame = new VTPXFrame(robotSide.getSideNameFirstLetter() + "Vtpx", footFrame, footHeight);
      vtpYFrame = new VTPYFrame(robotSide.getSideNameFirstLetter() + "Vtpy", vtpXFrame);
      RigidBody vtpJacobianBase = new RigidBody("vtpJacobianBase", footFrame);
      FrameVector x = new FrameVector(vtpYFrame, 1.0, 0.0, 0.0);
      FrameVector y = new FrameVector(vtpXFrame, 0.0, 1.0, 0.0);
      vtpXJoint = new RevoluteJoint("vtpX", vtpJacobianBase, vtpXFrame, y);
      RigidBody dummyBody = ScrewTools.addRigidBody("dummyBody", vtpXJoint, new Matrix3d(), 0.0, new Vector3d());
      vtpYJoint = new RevoluteJoint("vtpY", dummyBody, vtpYFrame, x);
      RigidBody vtpJacobianEndEffector = ScrewTools.addRigidBody("vtpJacobianBase", vtpYJoint, new Matrix3d(), 0.0, new Vector3d());

      // create openChainJacobian
      vtpJacobian = new GeometricJacobian(vtpJacobianBase, vtpJacobianEndEffector, jacobianFrame);
   }

   /**
    * Computes both the legJacobian and the vtpJacobian
    * @param vtpInFootFrame the vtp location, expressed in foot frame
    */
   public void computeJacobians(FramePoint2d vtpInFootFrame)
   {
      computeLegJacobianOnly();
      computeVTPJacobianOnly(vtpInFootFrame);
   }

   /**
    * Compute the vtpJacobian only, not the legJacobian.
    * @param vtpInFootFrame the vtp location, expressed in foot frame
    */
   public void computeVTPJacobianOnly(FramePoint2d vtpInFootFrame)
   {
      vtpInFootFrame.checkReferenceFrameMatch(footFrame);
      vtpXFrame.set(vtpInFootFrame.getX());
      vtpYFrame.set(vtpInFootFrame.getY());
      
      vtpXFrame.update();
      vtpYFrame.update();
//      vtpXJoint.getPredecessor().updateFramesRecursively();

      vtpJacobian.compute();
   }

   /**
    * Computes the legJointJacobian only, not the vtpJacobian.
    */
   public void computeLegJacobianOnly()
   {
      legJacobian.compute();
   }

   /**
    * Returns the twist of the pelvis frame with respect to the foot frame, expressed in the pelvis frame, corresponding to the given joint velocities.
    */
   public Twist getTwistOfPelvisWithRespectToFootInPelvisFrame(LegJointVelocities jointVelocities)
   {
      DenseMatrix64F jointVelocitiesVector = new DenseMatrix64F(legJointNames.length, 1);
      for (int i = 0; i < legJointNames.length; i++)
      {
         LegJointName legJointName = legJointNames[i];
         jointVelocitiesVector.set(i, 0, jointVelocities.getJointVelocity(legJointName));
      }

      DenseMatrix64F twistMatrix = new DenseMatrix64F(Twist.SIZE, 1);
      CommonOps.mult(legJacobian.getJacobianMatrix(), jointVelocitiesVector, twistMatrix);

      Twist ret = new Twist(legJacobian.getEndEffectorFrame(), legJacobian.getBaseFrame(), legJacobian.getJacobianFrame(), twistMatrix);
      ret.invert();
      ret.changeBaseFrameNoRelativeTwist(footFrame);
      ret.changeBodyFrameNoRelativeTwist(pelvisFrame);

      return ret;
   }

   /**
    * Computes the desired wrench on the pelvis, expressed in the pelvis frame, such that there are no torques at the vtp.
    * @param fZOnPelvisInPelvisFrame desired z-component of the force on the pelvis, expressed in pelvis frame
    * @param torqueOnPelvis desired torque on the pelvis, expressed in the pelvis frame
    * @return a wrench that requires no torque about the vtp, but still has the required fZ and torques.
    */
   public Wrench getWrenchInVTPNullSpace(double fZOnPelvisInPelvisFrame, FrameVector torqueOnPelvis)
   {
      /*
       * tauVTP = JVTPTranspose * FxyzNxyz
       *                                                [ Nx ]
       *                                                [ Ny ]
       * [ tauVTPx ] = [ J11 J21 J31 | J41 J51 | J61] * [ Nz ]
       * [ tauVTPy ]   [ J12 J22 J32 | J42 J52 | J62]    ----  = [ 0 ]
       *                      B1          A       B2    [ Fx ]   [ 0 ]
       *                                                [ Fy ]
       *                                                 ----
       *                                                [ Fz ]
       * A * Fxy + [B1 B2] * NxyzFz = 0
       * Fxy = -A^(-1) * [B1 B2] * FzNxyz
       */
      torqueOnPelvis.changeFrame(pelvisFrame);

      Matrix vtpJacobianMatrix = new Matrix(6, vtpJacobian.getNumberOfColumns());
      MatrixTools.convertEJMLToJama(vtpJacobian.getJacobianMatrix(), vtpJacobianMatrix);

      int[] columns = {0, 1};
      int[] aRows = {3, 4};
      Matrix A = vtpJacobianMatrix.getMatrix(aRows, columns).transpose();

      int[] bRows = {0, 1, 2, 5};
      Matrix B = vtpJacobianMatrix.getMatrix(bRows, columns).transpose();

      Matrix nxyzFZ = new Matrix(4, 1);
      nxyzFZ.set(0, 0, -torqueOnPelvis.getX());
      nxyzFZ.set(1, 0, -torqueOnPelvis.getY());
      nxyzFZ.set(2, 0, -torqueOnPelvis.getZ());
      nxyzFZ.set(3, 0, -fZOnPelvisInPelvisFrame);

      Matrix Fxy = (A.solve(B.times(nxyzFZ)));

      Vector3d forceOnPelvisInPelvisFrame = new Vector3d(Fxy.get(0, 0), Fxy.get(1, 0), fZOnPelvisInPelvisFrame);

      Wrench ret = new Wrench(pelvisFrame, pelvisFrame, forceOnPelvisInPelvisFrame, torqueOnPelvis.getVectorCopy());
      return ret;
   }

   /**
    * Packs a LegTorques object with the torques corresponding to the given wrench on the pelvis.
    */
   public void packLegTorques(LegTorques legTorquesToPack, Wrench wrenchOnPelvisInPelvisFrame)
   {
      // check that the LegTorques object we're packing has the correct RobotSide.
      if (this.robotSide != legTorquesToPack.getRobotSide())
      {
         throw new RuntimeException("legTorques object has the wrong RobotSide");
      }

      // the actual computation
      DenseMatrix64F jointTorques = legJacobian.computeJointTorques(wrenchOnPelvisInPelvisFrame);
      DenseMatrix64F vtpTorques = vtpJacobian.computeJointTorques(wrenchOnPelvisInPelvisFrame);

      CommonOps.scale(-1.0, jointTorques); // because pelvis is the base, not the end effector
      
      for (int i = 0; i < legJointNames.length; i++)
      {
         LegJointName legJointName = legJointNames[i];
         legTorquesToPack.setTorque(legJointName, jointTorques.get(i, 0));
      }

      // check if wrench is in null space of vtpJacobian.
      double vtpTorqueX = vtpTorques.get(0, 0);
      double vtpTorqueY = vtpTorques.get(1, 0);

      double epsilon = 1e-6;
      if ((vtpTorqueX > epsilon) || (vtpTorqueY > epsilon))
      {
         throw new RuntimeException("VTP torques are non-zero.\n" + "vtpTorqueX: " + vtpTorqueX + "\n" + "vtpTorqueY: " + vtpTorqueY);
      }
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   private static class VTPXFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = 1L;
      private double vtpX;
      private double footHeight;

      public VTPXFrame(String frameName, ReferenceFrame parentFrame, double footHeight)
      {
         super(frameName, parentFrame);
         this.vtpX = 0.0;
         this.footHeight = footHeight;
      }

      public void set(double vtpX)
      {
         this.vtpX = vtpX;
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setTranslation(new Vector3d(vtpX, 0.0, -footHeight));
      }
   }


   private static class VTPYFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = 1L;
      private double vtpY;

      public VTPYFrame(String frameName, ReferenceFrame parentFrame)
      {
         super(frameName, parentFrame);
         this.vtpY = 0.0;
      }

      public void set(double vtpY)
      {
         this.vtpY = vtpY;
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setTranslation(new Vector3d(0.0, vtpY, 0.0));
      }
   }
}
