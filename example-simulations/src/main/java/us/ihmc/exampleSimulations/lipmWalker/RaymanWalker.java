package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;

import java.util.ArrayList;
import java.util.List;

public class RaymanWalker
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   /**
    * This is the robot the controller uses.
    */
   private final MultiBodySystemBasics controllerRobot;
   /**
    * In this example, the center of mass reference frame will become handy as it allows to control the
    * robot's center of mass position.
    */
   private final ReferenceFrame centerOfMassFrame;
   /**
    * We will define additional reference frames located at the center of the foot soles. This will
    * allow to use them as reference when positioning the center of mass and will also be used as
    * control frames during the swing phase.
    */
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   /**
    * A {@code ContactablePlaneBody} allows to attach to a rigid-body a list of points that can be used
    * to distribute the ground reaction forces on the foot when it is in support.
    */
   private final SideDependentList<ContactablePlaneBody> footContactableBodies = new SideDependentList<>();
   /**
    * To generate the robot model we need the controller input and the robot definition
    */
   private final ControllerInput controllerInput;
   private final RaymanDefinition robotDefintion;
   /**
    * We need to be able to write to the controller output
    */
   private final ControllerOutput controllerOutput;

   public RaymanWalker(ControllerInput controllerInput, ControllerOutput controllerOutput, RaymanDefinition robotDefinition)
   {
      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;
      this.robotDefintion = robotDefinition;

      // Generate an instance of the robot for the controller (independent of the simulation robot). Make sure to use the same coordinate frame.
      controllerRobot = MultiBodySystemBasics.toMultiBodySystemBasics(robotDefinition.newInstance(WORLD_FRAME));
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", WORLD_FRAME, getElevator());

      /*
       * Left and right feet
       */
      for (RobotSide robotSide : RobotSide.values)
      {
         double footWidth = LIPMWalkerRobotDefinition.FOOT_WIDTH;
         double footLength = LIPMWalkerRobotDefinition.FOOT_LENGTH;
         double footBack = LIPMWalkerRobotDefinition.FOOT_BACK;

         RigidBodyBasics foot = getFoot(robotSide);

         /*
          * Let's first create a reference frame located at the bottom of the foot.
          */
         ReferenceFrame frameAfterAnkleJoint = foot.getParentJoint().getFrameAfterJoint();
         String soleFrameName = robotSide.getCamelCaseName() + "SoleFrame";
         RigidBodyTransform transformToAnkle = new RigidBodyTransform();
         transformToAnkle.getTranslation().setZ(-LIPMWalkerRobotDefinition.FOOT_HEIGHT); // Offset to be at the bottom of the foot.
         transformToAnkle.getTranslation().setX(-footBack + 0.5 * footLength); // Offset to center the frame in the middle of the sole.
         ReferenceFrame soleFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(soleFrameName, frameAfterAnkleJoint, transformToAnkle);
         soleFrames.put(robotSide, soleFrame);

         /*
          * Now we define the corner points of the foot. They can be used to apply forces on the ground.
          */
         List<Point2D> contactPoints = new ArrayList<>();
         contactPoints.add(new Point2D(0.5 * footLength, -0.5 * footWidth));
         contactPoints.add(new Point2D(0.5 * footLength, 0.5 * footWidth));
         contactPoints.add(new Point2D(-0.5 * footLength, -0.5 * footWidth));
         contactPoints.add(new Point2D(-0.5 * footLength, 0.5 * footWidth));
         ContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(foot, soleFrame, contactPoints);
         footContactableBodies.put(robotSide, footContactableBody);
      }
   }

   public void updateInverseDynamicsRobotState()
   {
      // We update the configuration state of our inverse dynamics robot model from the latest state of the simulated robot.
      MultiBodySystemTools.copyJointsState(controllerInput.getInput().getAllJoints(), controllerRobot.getAllJoints(), JointStateType.CONFIGURATION);
      MultiBodySystemTools.copyJointsState(controllerInput.getInput().getAllJoints(), controllerRobot.getAllJoints(), JointStateType.VELOCITY);
      MultiBodySystemTools.copyJointsState(controllerInput.getInput().getAllJoints(), controllerRobot.getAllJoints(), JointStateType.ACCELERATION);
      MultiBodySystemTools.copyJointsState(controllerInput.getInput().getAllJoints(), controllerRobot.getAllJoints(), JointStateType.EFFORT);

      // Need to update frames
      controllerRobot.getRootBody().updateFramesRecursively();
      centerOfMassFrame.update();
   }

   /**
    * Gets the root body of this robot.
    * <p>
    * The elevator is a massless, sizeless rigid-body fixed in world to which the first joint of the
    * robot is attached. The name comes from the use of this rigid-body to add the gravity effect to
    * the robot by making it accelerate like an elevator when it starts moving. However, this elevator
    * is always fixed in world with no velocity.
    * </p>
    *
    * @return the elevator.
    */
   public RigidBodyBasics getElevator()
   {
      // Note: the controller and simulation hold and maintain their own versions of frames (because they might run on separate threads). So we have to make sure to use the frame of the controller here and not of the simulated robot
      return controllerRobot.getRootBody();
   }

   /**
    * Gets the reference frame which is origin is located at the robot's center of mass.
    * <p>
    * Its axes are aligned with the world frame.
    * </p>
    *
    * @return the center of mass reference frame.
    */
   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   /**
    * Gets the floating joint of this robot.
    * <p>
    * The floating joint is the 6 degrees of freedom joint that represent the unactuated virtual
    * connection between the robot and the world.
    * </p>
    *
    * @return the robot floating joint.
    */
   public FloatingJointBasics getRootJoint()
   {
      return (FloatingJointBasics) controllerRobot.findJoint(robotDefintion.getRootJointName());
   }

   /**
    * Gets the pelvis.
    *
    * @return the pelvis.
    */
   public RigidBodyBasics getPelvis()
   {
      return getRootJoint().getSuccessor();
   }

   /**
    * Gets the left or right foot given the {@code RobotSide}.
    *
    * @param robotSide whether this method should return the left or right foot.
    * @return the foot.
    */
   public RigidBodyBasics getFoot(RobotSide robotSide)
   {
      return controllerRobot.findRigidBody(robotDefintion.getFootParentJoint(robotSide).getSuccessor().getName());
   }

   /**
    * Gets the foot with its contact points.
    *
    * @param robotSide whether this method should return the left or right contactable foot.
    * @return the contactable foot.
    */
   public ContactablePlaneBody getFootContactableBody(RobotSide robotSide)
   {
      return footContactableBodies.get(robotSide);
   }

   /**
    * Gets the sole frame of one of the feet.
    *
    * @param robotSide whether this method should return the left or right sole frame.
    * @return the sole frame.
    */
   public ReferenceFrame getSoleFrame(RobotSide robotSide)
   {
      return soleFrames.get(robotSide);
   }

   /**
    * Finds the corresponding simulated joint to the given {@code inverseDynamicsJoint} and set its
    * desired torque.
    *
    * @param jointName     the name of the joint of interest.
    * @param desiredEffort the new effort value.
    */
   public void setDesiredEffort(String jointName, double desiredEffort)
   {
      controllerOutput.getOneDoFJointOutput(jointName).setEffort(desiredEffort);
   }

   public double getCOMPositionZ()
   {
      FramePoint3D centerOfMassToPack = new FramePoint3D();
      controllerRobot.getRootBody().getCenterOfMass(centerOfMassToPack);
      return centerOfMassToPack.getZ();
   }

   public FramePoint3DReadOnly getCenterOfMassPosition()
   {
      FramePoint3D centerOfMassToPack = new FramePoint3D();
      controllerRobot.getRootBody().getCenterOfMass(centerOfMassToPack);
      return centerOfMassToPack;
   }

   public OneDoFJointBasics[] getAnkleJoints(RobotSide side)
   {
      OneDoFJointBasics ankleRollJoint = (OneDoFJointBasics) controllerRobot.findJoint(side + "_ankle_roll");
      OneDoFJointBasics anklePitchJoint = (OneDoFJointBasics) controllerRobot.findJoint(side + "_ankle_pitch");

      return new OneDoFJointBasics[] {ankleRollJoint, anklePitchJoint};
   }

   public ReferenceFrame getMidFeetFrame()
   {
      MidFrameZUpFrame midFeetZUpFrame;
      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZUp", ReferenceFrame.getWorldFrame(), getSoleFrame(RobotSide.LEFT), getSoleFrame(RobotSide.RIGHT));
      return midFeetZUpFrame;
   }

   public ReferenceFrame getHipBodyReferenceFrame(RobotSide side)
   {
      RigidBodyBasics hip = MultiBodySystemTools.findRigidBody(controllerRobot.getRootBody(), side.getCamelCaseName() + "Hip");
      return hip.getBodyFixedFrame();
   }
}
