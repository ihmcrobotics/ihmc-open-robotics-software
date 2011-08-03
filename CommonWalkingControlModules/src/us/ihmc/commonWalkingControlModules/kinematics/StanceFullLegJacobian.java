package us.ihmc.commonWalkingControlModules.kinematics;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OpenChainJacobian;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;

import com.mathworks.jama.Matrix;

public class StanceFullLegJacobian
{
   private final RobotSide robotSide;
   private final LegJointName[] legJointNames;
   
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame footFrame;
   private final VTPXFrame vtpXFrame;
   private final VTPYFrame vtpYFrame;

   private final OpenChainJacobian legJacobian;
   private final OpenChainJacobian vtpJacobian;

   /**
    * Constructs a new StanceFullLegJacobian, for the given side of the robot
    * @param robotSpecificJointNames robot specific joint names
    * @param footHeight height of the origin of the foot frame above the sole of the foot
    */
   public StanceFullLegJacobian(RobotSide robotSide, CommonWalkingReferenceFrames frames, RobotSpecificJointNames robotSpecificJointNames, double footHeight)
   {
      this.robotSide = robotSide;
      this.legJointNames = robotSpecificJointNames.getLegJointNames();

      pelvisFrame = frames.getPelvisFrame();
      footFrame = frames.getFootFrame(robotSide);
      
      vtpXFrame = new VTPXFrame("VTPXFrame", footFrame, footHeight);
      vtpYFrame = new VTPYFrame("VTPYFrame", vtpXFrame);

      Vector3d zero = new Vector3d();
      
      ArrayList<Twist> legTwists = new ArrayList<Twist>();
      ReferenceFrame baseFrame = pelvisFrame;
      for (LegJointName legJointName : legJointNames)
      {
         ReferenceFrame bodyFrame = frames.getLegJointFrame(robotSide, legJointName);
         ReferenceFrame expressedInFrame = bodyFrame;
         Twist twist = new Twist(bodyFrame, baseFrame, expressedInFrame, zero, legJointName.getJointAxis());
         
         legTwists.add(twist);
         
         baseFrame = bodyFrame; // for next iteration
      }

      for (Twist twist : legTwists)
      {
         twist.invert();
      }

      // define relevant frames
      ReferenceFrame legEndEffectorFrame = pelvisFrame;
      ReferenceFrame legJacobianBaseFrame = footFrame;
      ReferenceFrame jacobianFrame = pelvisFrame;

      // create openChainJacobian
      legJacobian = new OpenChainJacobian(legTwists, legEndEffectorFrame, legJacobianBaseFrame, jacobianFrame);


      // Build vtpJacobian
      Vector3d x = new Vector3d(1.0, 0.0, 0.0);
      Vector3d y = new Vector3d(0.0, 1.0, 0.0);
      
      Twist vtpXTwist = new Twist(vtpXFrame, footFrame, vtpXFrame, zero, y);
      Twist vtpYTwist = new Twist(vtpYFrame, vtpXFrame, vtpYFrame, zero, x);

      ArrayList<Twist> vtpTwists = new ArrayList<Twist>();
      vtpTwists.add(vtpXTwist);
      vtpTwists.add(vtpYTwist);

      for (Twist twist : vtpTwists)
      {
         twist.invert();
      }

      // define relevant frames
      ReferenceFrame vtpEndEffectorFrame = footFrame;
      ReferenceFrame vtpJacobianBaseFrame = vtpYFrame;

      // create openChainJacobian
      vtpJacobian = new OpenChainJacobian(vtpTwists, vtpEndEffectorFrame, vtpJacobianBaseFrame, jacobianFrame);

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
      Matrix jointVelocitiesVector = new Matrix(legJointNames.length, 1);
      for (int i = 0; i < legJointNames.length; i++)
      {
         LegJointName legJointName = legJointNames[i];
         jointVelocitiesVector.set(i, 0, jointVelocities.getJointVelocity(legJointName));
      }

      return legJacobian.getTwist(jointVelocitiesVector);
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
       *                                              [ Fx ]
       *                                              [ Fy ]
       *                                               ----
       * [ tauVTPx ] = [ J11 J21 | J31 J41 J51 J61] * [ Fz ] = [ 0 ]
       * [ tauVTPy ]   [ J12 J22 | J32 J42 J52 J62]   [ Nx ]   [ 0 ]
       *                    A             B           [ Ny ]
       *                                              [ Nz ]
       * A * Fxy + B * FzNxyz = 0
       * Fxy = -A^(-1) * B * FzNxyz
       */
      torqueOnPelvis = torqueOnPelvis.changeFrameCopy(pelvisFrame);

      Matrix vtpJacobianMatrix = vtpJacobian.getJacobianMatrix();

      Matrix A = vtpJacobianMatrix.getMatrix(0, 1, 0, 1).transpose();

      Matrix B = vtpJacobianMatrix.getMatrix(2, 5, 0, 1).transpose();

      Matrix FzNxyz = new Matrix(4, 1);
      FzNxyz.set(0, 0, fZOnPelvisInPelvisFrame);
      FzNxyz.set(1, 0, torqueOnPelvis.getX());
      FzNxyz.set(2, 0, torqueOnPelvis.getY());
      FzNxyz.set(3, 0, torqueOnPelvis.getZ());


      Matrix Fxy = (A.solve(B.times(FzNxyz))).times(-1.0);    //

      Vector3d forceOnPelvisInPelvisFrame = new Vector3d(Fxy.get(0, 0), Fxy.get(1, 0), fZOnPelvisInPelvisFrame);

      return new Wrench(pelvisFrame, pelvisFrame, forceOnPelvisInPelvisFrame, torqueOnPelvis.getVectorCopy());
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
      Matrix jointTorques = legJacobian.computeJointTorques(wrenchOnPelvisInPelvisFrame);
      Matrix vtpTorques = vtpJacobian.computeJointTorques(wrenchOnPelvisInPelvisFrame);
      
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
      public void updateTransformToParent(Transform3D transformToParent)
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
      public void updateTransformToParent(Transform3D transformToParent)
      {
         transformToParent.setTranslation(new Vector3d(0.0, vtpY, 0.0));
      }
   }
}
