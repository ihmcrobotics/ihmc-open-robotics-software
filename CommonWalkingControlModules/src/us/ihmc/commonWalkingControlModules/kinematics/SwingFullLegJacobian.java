package us.ihmc.commonWalkingControlModules.kinematics;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OpenChainJacobian;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;

import com.mathworks.jama.Matrix;

public class SwingFullLegJacobian
{
   private final RobotSide robotSide;
   private final OpenChainJacobian openChainJacobian;
   private final LegJointName[] legJointNames;
   
   /**
    * Constructs a new SwingFullLegJacobian, for the given side of the robot
    */
   public SwingFullLegJacobian(RobotSpecificJointNames robotJointNames, RobotSide robotSide, CommonWalkingReferenceFrames frames)
   {
      this.legJointNames = robotJointNames.getLegJointNames();
      this.robotSide = robotSide;

      // frames
      ReferenceFrame pelvisFrame = frames.getPelvisFrame();

      // unit twists in body frames
      Vector3d zero = new Vector3d();
      
      ArrayList<Twist> legTwists = new ArrayList<Twist>();
      ReferenceFrame twistBaseFrame = pelvisFrame;
      for (LegJointName legJointName : legJointNames)
      {
         ReferenceFrame twistBodyFrame = frames.getLegJointFrame(robotSide, legJointName);
         ReferenceFrame expressedInFrame = twistBodyFrame;
         Twist twist = new Twist(twistBodyFrame, twistBaseFrame, expressedInFrame, zero, legJointName.getJointAxis());
         
         legTwists.add(twist);
         
         twistBaseFrame = twistBodyFrame; // for next iteration
      }

      // frames
      ReferenceFrame endEffectorFrame = frames.getFootFrame(robotSide);
      ReferenceFrame baseFrame = pelvisFrame;
      ReferenceFrame jacobianFrame = endEffectorFrame;

      // create Jacobian
      openChainJacobian = new OpenChainJacobian(legTwists, endEffectorFrame, baseFrame, jacobianFrame);

   }

   /**
    * Computes the underlying openChainJacobian and the vtpJacobian
    */
   public void computeJacobian()
   {
      openChainJacobian.compute();
   }

   /**
    * @return the determinant of the Jacobian matrix
    */
   public double det()
   {
      return openChainJacobian.det();
   }

   /**
    * Returns the twist of the ankle pitch frame with respect to the pelvis frame, expressed in the ankle pitch frame,
    * corresponding to the given joint velocities.
    */
   public Twist getTwistOfFootWithRespectToPelvisInFootFrame(LegJointVelocities jointVelocities)
   {
      Matrix jointVelocitiesVector = new Matrix(legJointNames.length, 1);
      for (int i = 0; i < legJointNames.length; i++)
      {
         LegJointName legJointName = legJointNames[i];
         jointVelocitiesVector.set(i, 0, jointVelocities.getJointVelocity(legJointName));
      }

      return openChainJacobian.getTwist(jointVelocitiesVector);
   }

   /**
    * Returns the joint velocities corresponding to the twist of the foot, with respect to the pelvis, expressed in ankle pitch frame
    * @param anklePitchTwistInAnklePitchFrame
    * @return corresponding joint velocities
    */
   public LegJointVelocities getJointVelocitiesGivenTwist(Twist anklePitchTwistInAnklePitchFrame)
   {
      Matrix jointVelocities = openChainJacobian.getJointVelocities(anklePitchTwistInAnklePitchFrame);
      LegJointVelocities ret = new LegJointVelocities(legJointNames, robotSide);
      for (int i = 0; i < legJointNames.length; i++)
      {
         LegJointName legJointName = legJointNames[i];
         ret.setJointVelocity(legJointName, jointVelocities.get(i, 0));
      }

      return ret;
   }

   /**
    * Packs a LegTorques object with the torques corresponding to the given wrench on the foot.
    */
   public void packLegTorques(LegTorques legTorquesToPack, Wrench wrenchOnFootInFootFrame)
   {
      // check that the LegTorques object we're packing has the correct RobotSide.
      if (this.robotSide != legTorquesToPack.getRobotSide())
      {
         throw new RuntimeException("legTorques object has the wrong RobotSide");
      }

      // the actual computation
      Matrix jointTorques = openChainJacobian.getJointTorques(wrenchOnFootInFootFrame);

      for (int i = 0; i < legJointNames.length; i++)
      {
         LegJointName legJointName = legJointNames[i];
         legTorquesToPack.setTorque(legJointName, jointTorques.get(i, 0));
      }
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
}
