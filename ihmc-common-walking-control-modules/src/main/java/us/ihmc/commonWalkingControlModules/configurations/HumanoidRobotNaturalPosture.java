package us.ihmc.commonWalkingControlModules.configurations;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Natural Posture deals with the center of mass orientation or angular center of mass. It is a conceptualization of the average orientation of the
 * robot, similar to how center of mass is the average position of all mass in the robot. It short, it is the angle to which the robot is servoed.
 * REF: Y.-M. Chen, G. Nelson, R. Griffin, M. Posa, and J. Pratt, "Angular Center of Mass for Humanoid Robots." 2022. http://arxiv.org/abs/2210.08111
 */
public interface HumanoidRobotNaturalPosture
{
   /**
    * This is the main method of this class. An implementation of this method should contain the calculation which computes the Angular Center of Mass (ACOM).
    * Input "q" is an array of joint positions, and input "baseOrientation" is the orientation of the base body frame relative to world (frame B in the angular
    * center of mass paper). The implementation should have logic to handle when compute() is called without the joint positions using the default method (such
    * as in NaturalPostureController). See NadiaNaturalPosture for an example.
    */
   void compute(double[] q, Orientation3DReadOnly baseOrientation);

   /**
    * NaturalPostureController needs a compute without the joint positions in order to be robot agnostic. The implementation should have logic to handle when
    * compute() is called without the joint positions (such as in NaturalPostureController). See NadiaNaturalPosture for an example.
    */
   default void compute(Orientation3DReadOnly baseOrientation)
   {
      compute(null, baseOrientation);
   }

   /**
    * This gets the center of mass orientation (angular center of mass) relative to the world frame. It is a conceptualization of the average orientation of the
    * robot, similar to how center of mass is the average position of all mass in the robot. It short, it is the angle to which the robot is servoed.
    */
   Quaternion getCenterOfMassOrientation();

   /**
    * This gets the center of mass orientation relative to the root body frame (frame B in the angular center of mass paper), e.g. the pelvis frame.
    */
   Quaternion getCenterOfMassOrientationRelativeToBase();

   /**
    * This is the task jacobian for the center of mass orientation objective relative to the world frame. It is used to calculate the angular velocity of the
    * angular center of mass (ACOM) frame relative to world. This is the method the QP needs for the ACOM tracking task, so you should use this method.
    */
   DMatrixRMaj getCenterOfMassOrientationJacobian();

   /**
    * This is the task jacobian for the center of mass orientation objective relative to the base frame and expressed with respect to the base frame. It is used
    * to calculate the angular velocity of the angular center of mass (ACOM) frame relative to the base frame. The base frame is the frame of the root body
    * (e.g. the pelvis). The QP doesn't need this method, but you could use it for debugging.
    */
   DMatrixRMaj getCenterOfMassOrientationJacobianRelativeToBase();

   default YoRegistry getRegistry()
   {
      return null;
   }

   void createVisuals(YoGraphicsListRegistry yoGraphicsListRegistry);
}