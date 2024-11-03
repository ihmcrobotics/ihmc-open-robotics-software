package us.ihmc.commonWalkingControlModules.sensors;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class ProvidedMassMatrixToolRigidBodyTest
{
   private final YoRegistry registry = new YoRegistry("ProvidedMassMatrixToolRigidBodyTest");
   private final RobotSide robotSide = RobotSide.LEFT;
   private final double gravity = 9.81;
   private final double mass = 2.0;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testprovidedMassMatrixToolRigidBody()
   {
      FullHumanoidRobotModel fullRobotModel = getFullRobotModel();

      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();

      ProvidedMassMatrixToolRigidBody providedMassMatrixToolRigidBody = new ProvidedMassMatrixToolRigidBody(robotSide, fullRobotModel, gravity, registry, null);
      providedMassMatrixToolRigidBody.setMass(mass);

      SpatialAcceleration handSpatialAccelerationVector = new SpatialAcceleration(elevatorFrame, elevatorFrame, elevatorFrame);
      Wrench toolWrench = new Wrench();

      providedMassMatrixToolRigidBody.control(handSpatialAccelerationVector, toolWrench);

      toolWrench.changeFrame(ReferenceFrame.getWorldFrame());

      assertTrue(toolWrench.getLinearPart().epsilonEquals(new Vector3D(0.0, 0.0, -mass * gravity), 10e-5));
   }

   public abstract FullHumanoidRobotModel getFullRobotModel();
}
