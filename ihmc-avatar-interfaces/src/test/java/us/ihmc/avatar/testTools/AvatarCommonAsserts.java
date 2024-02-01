package us.ihmc.avatar.testTools;

import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.heightPlanning.HeightOffsetHandler;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class AvatarCommonAsserts
{
   public static void assertArmPositions(double[] expected, RobotSide robotSide, FullHumanoidRobotModel robot, double epsilon)
   {
      OneDoFJointBasics[] armJoints = getArmJoints(robotSide, robot);

      int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);
      Assert.assertEquals("Unexpected number of joints.", expected.length, numberOfJoints);

      for (int i = 0; i < armJoints.length; i++)
      {
         Assert.assertEquals(armJoints[i].getName() + " at unexpected position.", expected[i], armJoints[i].getQ(), epsilon);
      }
   }

   public static void assertArmVelocities(double[] expected, RobotSide robotSide, FullHumanoidRobotModel robot, double epsilon)
   {
      OneDoFJointBasics[] armJoints = getArmJoints(robotSide, robot);

      int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);
      Assert.assertEquals("Unexpected number of joints.", expected.length, numberOfJoints);

      for (int i = 0; i < armJoints.length; i++)
      {
         Assert.assertEquals(armJoints[i].getName() + " at unexpected velocity.", expected[i], armJoints[i].getQd(), epsilon);
      }
   }

   public static void assertArmVelocitiesZero(RobotSide robotSide, FullHumanoidRobotModel robot, double epsilon)
   {
      OneDoFJointBasics[] armJoints = getArmJoints(robotSide, robot);

      for (int i = 0; i < armJoints.length; i++)
      {
         Assert.assertEquals(armJoints[i].getName() + " at unexpected velocity.", 0.0, armJoints[i].getQd(), epsilon);
      }
   }

   public static void assertDesiredArmPositions(double[] expected, RobotSide robotSide, FullHumanoidRobotModel robot, YoVariableHolder scs, double epsilon)
   {
      OneDoFJointBasics[] armJoints = getArmJoints(robotSide, robot);

      int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);
      Assert.assertEquals("Unexpected number of joints.", expected.length, numberOfJoints);

      for (int i = 0; i < armJoints.length; i++)
      {
         Assert.assertEquals(armJoints[i].getName() + " at unexpected desired position.", expected[i], findJointDesiredPosition(scs, armJoints[i]), epsilon);
      }
   }

   public static void assertDesiredArmVelocities(double[] expected, RobotSide robotSide, FullHumanoidRobotModel robot, YoVariableHolder scs, double epsilon)
   {
      OneDoFJointBasics[] armJoints = getArmJoints(robotSide, robot);

      int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);
      Assert.assertEquals("Unexpected number of joints.", expected.length, numberOfJoints);

      for (int i = 0; i < armJoints.length; i++)
      {
         Assert.assertEquals(armJoints[i].getName() + " at unexpected desired velocity.", expected[i], findJointDesiredVelocity(scs, armJoints[i]), epsilon);
      }
   }

   public static void assertDesiredArmVelocitiesZero(RobotSide robotSide, FullHumanoidRobotModel robot, YoVariableHolder scs, double epsilon)
   {
      OneDoFJointBasics[] armJoints = getArmJoints(robotSide, robot);

      for (int i = 0; i < armJoints.length; i++)
      {
         Assert.assertEquals(armJoints[i].getName() + " at unexpected desired velocity.", 0.0, findJointDesiredVelocity(scs, armJoints[i]), epsilon);
      }
   }

   private static OneDoFJointBasics[] getArmJoints(RobotSide robotSide, FullHumanoidRobotModel robot)
   {
      RigidBodyBasics chest = robot.getChest();
      RigidBodyBasics hand = robot.getHand(robotSide);
      return MultiBodySystemTools.createOneDoFJointPath(chest, hand);
   }

   public static void assertChestOrientation(FrameQuaternionReadOnly expected, FullHumanoidRobotModel robot, double epsilon)
   {
      assertBodyOrientation(expected, robot.getChest(), epsilon);
   }

   public static void assertPelvisOrientation(FrameQuaternionReadOnly expected, FullHumanoidRobotModel robot, double epsilon)
   {
      assertBodyOrientation(expected, robot.getPelvis(), epsilon);
   }

   public static void assertPelvisPosition(FramePoint3DReadOnly expected, FullHumanoidRobotModel robot, double epsilon)
   {
      assertBodyPosition(expected, robot.getPelvis(), epsilon);
   }

   public static void assertBodyOrientation(FrameQuaternionReadOnly expected, RigidBodyBasics body, double epsilon)
   {
      FrameQuaternion currentBodyOrientation = new FrameQuaternion(body.getBodyFixedFrame());
      currentBodyOrientation.changeFrame(expected.getReferenceFrame());
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expected, currentBodyOrientation, epsilon);
   }

   public static void assertBodyPosition(FramePoint3DReadOnly expected, RigidBodyBasics body, double epsilon)
   {
      FramePoint3D currentBodyPosition = new FramePoint3D(body.getBodyFixedFrame());
      currentBodyPosition.changeFrame(expected.getReferenceFrame());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expected, currentBodyPosition, epsilon);
   }

   public static void assertDesiredChestOrientation(FrameQuaternionReadOnly expected, FullHumanoidRobotModel robot, YoVariableHolder scs, double epsilon)
   {
      assertDesiredBodyOrientation(expected, robot.getChest(), scs, epsilon);
   }

   public static void assertDesiredPelvisOrientation(FrameQuaternionReadOnly expected, FullHumanoidRobotModel robot, YoVariableHolder scs, double epsilon)
   {
      assertDesiredBodyOrientation(expected, robot.getPelvis(), scs, epsilon);
   }

   public static void assertDesiredBodyOrientation(FrameQuaternionReadOnly expected, RigidBodyBasics body, YoVariableHolder scs, double epsilon)
   {
      FrameQuaternion controllerDesiredOrientation = findDesiredOrientation(scs, body);
      controllerDesiredOrientation.changeFrame(expected.getReferenceFrame());
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expected, controllerDesiredOrientation, epsilon);
   }

   public static void assertDesiredChestAngularVelocityZero(FullHumanoidRobotModel robot, YoVariableHolder scs, double epsilon)
   {
      assertDesiredBodyAngularVelocityZero(robot.getChest(), scs, epsilon);
   }

   public static void assertDesiredPelvisAngularVelocityZero(FullHumanoidRobotModel robot, YoVariableHolder scs, double epsilon)
   {
      assertDesiredBodyAngularVelocityZero(robot.getPelvis(), scs, epsilon);
   }

   public static void assertDesiredBodyAngularVelocityZero(RigidBodyBasics body, YoVariableHolder scs, double epsilon)
   {
      FrameVector3D controllerDesiredAngularVelocity = findDesiredAngularVelocity(scs, body);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), controllerDesiredAngularVelocity, epsilon);
   }

   public static void assertDesiredICPOffsetZero(YoVariableHolder scs, double epsilon)
   {
      String namespace = PelvisICPBasedTranslationManager.class.getSimpleName();
      Tuple2DReadOnly desiredICPOffset = findTuple2d(namespace, "desiredICPOffset", scs);
      EuclidCoreTestTools.assertEquals(new Vector2D(), desiredICPOffset, epsilon);
   }

   public static void assertDesiredPelvisHeightOffsetZero(YoVariableHolder scs, double epsilon)
   {
      String namespace = HeightOffsetHandler.class.getSimpleName();
      double offset = getDoubleYoVariable(scs, "offsetHeightAboveGroundTrajectoryOutput", namespace).getValue();
      Assert.assertEquals(0.0, offset, epsilon);
   }

   private static double findJointDesiredPosition(YoVariableHolder scs, OneDoFJointBasics joint)
   {
      String jointName = joint.getName();
      String namespace = FeedbackControllerToolbox.class.getSimpleName();
      String variable = "q_d_" + jointName;
      return getDoubleYoVariable(scs, variable, namespace).getValue();
   }

   private static double findJointDesiredVelocity(YoVariableHolder scs, OneDoFJointBasics joint)
   {
      String jointName = joint.getName();
      String namespace = FeedbackControllerToolbox.class.getSimpleName();
      String variable = "qd_d_" + jointName;
      return getDoubleYoVariable(scs, variable, namespace).getValue();
   }

   private static FrameQuaternion findDesiredOrientation(YoVariableHolder scs, RigidBodyBasics body)
   {
      Quaternion desiredOrientation = findQuat4d(FeedbackControllerToolbox.class.getSimpleName(), body.getName() + "DesiredOrientation", scs);
      return new FrameQuaternion(ReferenceFrame.getWorldFrame(), desiredOrientation);
   }

   private static FrameVector3D findDesiredAngularVelocity(YoVariableHolder scs, RigidBodyBasics body)
   {
      Tuple3DBasics desiredAngularVelocity = findTuple3d(FeedbackControllerToolbox.class.getSimpleName(), body.getName() + "DesiredAngularVelocity", scs);
      return new FrameVector3D(ReferenceFrame.getWorldFrame(), desiredAngularVelocity);
   }

   private static Quaternion findQuat4d(String namespace, String varname, YoVariableHolder scs)
   {
      return findQuat4d(namespace, varname, "", scs);
   }

   private static Quaternion findQuat4d(String namespace, String prefix, String suffix, YoVariableHolder scs)
   {
      double x = scs.findVariable(namespace, YoGeometryNameTools.createQxName(prefix, suffix)).getValueAsDouble();
      double y = scs.findVariable(namespace, YoGeometryNameTools.createQyName(prefix, suffix)).getValueAsDouble();
      double z = scs.findVariable(namespace, YoGeometryNameTools.createQzName(prefix, suffix)).getValueAsDouble();
      double s = scs.findVariable(namespace, YoGeometryNameTools.createQsName(prefix, suffix)).getValueAsDouble();
      return new Quaternion(x, y, z, s);
   }

   private static Tuple3DBasics findTuple3d(String namespace, String varname, YoVariableHolder scs)
   {
      return findTuple3d(namespace, varname, "", scs);
   }

   private static Tuple3DBasics findTuple3d(String namespace, String prefix, String suffix, YoVariableHolder scs)
   {
      Tuple3DBasics tuple3d = new Point3D();
      tuple3d.setX(scs.findVariable(namespace, YoGeometryNameTools.createXName(prefix, suffix)).getValueAsDouble());
      tuple3d.setY(scs.findVariable(namespace, YoGeometryNameTools.createYName(prefix, suffix)).getValueAsDouble());
      tuple3d.setZ(scs.findVariable(namespace, YoGeometryNameTools.createZName(prefix, suffix)).getValueAsDouble());
      return tuple3d;
   }

   private static Tuple2DBasics findTuple2d(String namespace, String varname, YoVariableHolder scs)
   {
      return findTuple2d(namespace, varname, "", scs);
   }

   private static Tuple2DBasics findTuple2d(String namespace, String prefix, String suffix, YoVariableHolder scs)
   {
      Tuple2DBasics tuple2d = new Point2D();
      tuple2d.setX(scs.findVariable(namespace, YoGeometryNameTools.createXName(prefix, suffix)).getValueAsDouble());
      tuple2d.setY(scs.findVariable(namespace, YoGeometryNameTools.createYName(prefix, suffix)).getValueAsDouble());
      return tuple2d;
   }

   public static YoDouble getDoubleYoVariable(YoVariableHolder scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, YoDouble.class);
   }

   public static YoBoolean getBooleanYoVariable(YoVariableHolder scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, YoBoolean.class);
   }

   private static <T extends YoVariable> T getYoVariable(YoVariableHolder scs, String name, String namespace, Class<T> clazz)
   {
      YoVariable uncheckedVariable = scs.findVariable(namespace, name);
      if (uncheckedVariable == null)
         throw new RuntimeException("Could not find yo variable: " + namespace + "/" + name + ".");
      if (!clazz.isInstance(uncheckedVariable))
         throw new RuntimeException("YoVariable " + name + " is not of type " + clazz.getSimpleName());
      return clazz.cast(uncheckedVariable);
   }
}
