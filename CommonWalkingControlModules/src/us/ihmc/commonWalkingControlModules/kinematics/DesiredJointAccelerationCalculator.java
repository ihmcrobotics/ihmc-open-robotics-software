package us.ihmc.commonWalkingControlModules.kinematics;

import java.util.ArrayList;
import java.util.ListIterator;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;

import com.mathworks.jama.Matrix;

public class DesiredJointAccelerationCalculator
{
   private final RobotSide swingSide;
   private final SwingFullLegJacobian swingLegJacobian;
   private final FullRobotModel fullRobotModel;

   private final InverseDynamicsJoint rootJoint;
   private final ReferenceFrame footFrame;
   private final ReferenceFrame pelvisFrame;
   private final ArrayList<RevoluteJoint> legJointsList;
   private final LegJointName[] legJointNames;

   public DesiredJointAccelerationCalculator(LegJointName[] legJointNames, SwingFullLegJacobian swingLegJacobian, FullRobotModel fullRobotModel,
           CommonWalkingReferenceFrames referenceFrames, RobotSide robotSide)
   {
      this.legJointNames = legJointNames;

      swingLegJacobian.getRobotSide().checkRobotSideMatch(robotSide);

      this.swingSide = swingLegJacobian.getRobotSide();

      this.swingLegJacobian = swingLegJacobian;
      this.fullRobotModel = fullRobotModel;

      this.rootJoint = fullRobotModel.getRootJoint();
      this.footFrame = referenceFrames.getFootFrame(robotSide);
      this.pelvisFrame = referenceFrames.getPelvisFrame();
      this.legJointsList = fullRobotModel.getLegJointList(robotSide);
   }

   
   private final SpatialAccelerationVector jacobianDerivativeTerm = new SpatialAccelerationVector();
   private final SpatialAccelerationVector accelerationOfFootWithRespectToPelvis = new SpatialAccelerationVector();
   
   /**
    * Sets the accelerations for the RevoluteJoints in legJoints
    * Assumes that the swingLegJacobian is already updated
    * Assumes that the rootJoint's acceleration has already been set
    */
   public void compute(SpatialAccelerationVector desiredAccelerationOfFootWithRespectToWorld)
   {
      computeDesiredAccelerationOfFootWithRespectToPelvis(accelerationOfFootWithRespectToPelvis, desiredAccelerationOfFootWithRespectToWorld);

      computeJacobianDerivativeTerm(jacobianDerivativeTerm);
      computeJointAccelerations(accelerationOfFootWithRespectToPelvis, jacobianDerivativeTerm);
   }

   private final Twist twistOfPelvisWithRespectToElevator = new Twist();

   private void computeDesiredAccelerationOfFootWithRespectToPelvis(SpatialAccelerationVector accelerationOfFootWithRespectToPelvis, SpatialAccelerationVector desiredAccelerationOfFootWithRespectToElevator)
   {
//      SpatialAccelerationVector accelerationOfFootWithRespectToPelvis = new SpatialAccelerationVector();
      rootJoint.packDesiredJointAcceleration(accelerationOfFootWithRespectToPelvis);    // acceleration of imu with respect to elevator
      accelerationOfFootWithRespectToPelvis.changeBodyFrameNoRelativeAcceleration(pelvisFrame);    // acceleration of body with respect to elevator
      accelerationOfFootWithRespectToPelvis.changeFrameNoRelativeMotion(pelvisFrame);

      Twist twistOfPelvisWithRespectToFoot = computeTwistOfPelvisWithRespectToFoot();

      rootJoint.packJointTwist(twistOfPelvisWithRespectToElevator);    // twist of imu with respect to elevator
      twistOfPelvisWithRespectToElevator.changeBodyFrameNoRelativeTwist(pelvisFrame);    // twist of body with respect to elevator
      twistOfPelvisWithRespectToElevator.changeFrame(pelvisFrame);

      accelerationOfFootWithRespectToPelvis.changeFrame(footFrame, twistOfPelvisWithRespectToFoot, twistOfPelvisWithRespectToElevator);
      accelerationOfFootWithRespectToPelvis.invert();    // acceleration of elevator with respect to body
      accelerationOfFootWithRespectToPelvis.add(desiredAccelerationOfFootWithRespectToElevator);    // acceleration of foot with respect to body

//      return accelerationOfFootWithRespectToPelvis;
   }

   private Twist computeTwistOfPelvisWithRespectToFoot()
   {
      LegJointVelocities jointVelocities = fullRobotModel.getLegJointVelocities(swingSide);
      Twist twistOfPelvisWithRespectToFoot = swingLegJacobian.getTwistOfFootWithRespectToPelvisInFootFrame(jointVelocities);    // twist of foot with respect to body
      twistOfPelvisWithRespectToFoot.invert();    // twist of body with respect to foot
      twistOfPelvisWithRespectToFoot.changeFrame(pelvisFrame);

      return twistOfPelvisWithRespectToFoot;
   }

   private final SpatialAccelerationVector unitTwistDerivative = new SpatialAccelerationVector();
   private final Twist twistOfCurrentWithRespectToFoot = new Twist(); //footFrame, footFrame, footFrame);
   private final Twist unitJointTwist = new Twist();
   private final Twist jointTwist = new Twist();
   
   private SpatialAccelerationVector computeJacobianDerivativeTerm(SpatialAccelerationVector ret)
   {
//      Twist twistOfCurrentWithRespectToFoot = new Twist(footFrame, footFrame, footFrame);
//      Twist unitJointTwist = new Twist();
//      Twist jointTwist = new Twist();
//      SpatialAccelerationVector ret = new SpatialAccelerationVector(footFrame, footFrame, footFrame);
      
      twistOfCurrentWithRespectToFoot.setToZero(footFrame, footFrame, footFrame);
      ret.setToZero(footFrame, footFrame, footFrame);
      
      ListIterator<RevoluteJoint> iterator = legJointsList.listIterator(legJointsList.size());
      while (iterator.hasPrevious())
      {
         // scale twistOfCurrentWithRespectToFoot by qd
         // add to ret

         RevoluteJoint joint = iterator.previous();
         InverseDynamicsJoint parentJoint = joint.getPredecessor().getParentJoint();
         ReferenceFrame parentJointFrame = parentJoint.getFrameAfterJoint();

         joint.packUnitJointTwist(unitJointTwist);
         unitJointTwist.changeBaseFrameNoRelativeTwist(parentJointFrame);

         unitTwistDerivative.setToZero(unitJointTwist.getBodyFrame(), unitJointTwist.getBaseFrame(), unitJointTwist.getExpressedInFrame());

         unitTwistDerivative.changeFrame(footFrame, twistOfCurrentWithRespectToFoot, unitJointTwist);    // RESULT: column of Jdot
         unitTwistDerivative.scale(joint.getQd());
         unitTwistDerivative.add(ret);
         ret.set(unitTwistDerivative);    // kind of sucks, but ok

         joint.packJointTwist(jointTwist);
         jointTwist.changeBaseFrameNoRelativeTwist(parentJointFrame);
         jointTwist.invert();
         twistOfCurrentWithRespectToFoot.add(jointTwist);
         twistOfCurrentWithRespectToFoot.changeFrame(parentJointFrame);
      }

      return ret;
   }

   private void computeJointAccelerations(SpatialAccelerationVector accelerationOfFootWithRespectToPelvis, SpatialAccelerationVector jacobianDerivativeTerm)
   {
      Matrix jointAccelerations = swingLegJacobian.computeJointAccelerations(accelerationOfFootWithRespectToPelvis, jacobianDerivativeTerm);


      for (int i = 0; i < legJointNames.length; i++)
      {
         LegJointName legJointName = legJointNames[i];

         double qdd = jointAccelerations.get(i, 0);
         fullRobotModel.getLegJoint(swingSide, legJointName).setQddDesired(qdd);
      }
   }
}
