package us.ihmc.exampleSimulations.sphereICPControl.model;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.yoVariables.variable.YoDouble;

public class SphereRobot
{
   private static final double radius = 0.1;

   public static RobotTools.SCSRobotFromInverseDynamicsRobotModel createSphereRobot(String name, Vector3D initialPosition, RigidBody elevator,
         YoGraphicsListRegistry yoGraphicsListRegistry, double gravity)
   {
      RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot = new RobotTools.SCSRobotFromInverseDynamicsRobotModel(name, elevator.getChildrenJoints().get(0));

      scsRobot.setGravity(0.0, 0.0, -gravity);

      Joint floatingJoint = scsRobot.getRootJoints().get(0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(radius/2.0, YoAppearance.EarthTexture());
      floatingJoint.getLink().setLinkGraphics(linkGraphics);

      String gcName = "gc";
      GroundContactPoint gc = new GroundContactPoint(gcName, new Vector3D(0.0, 0.0, 0.0), scsRobot);
      floatingJoint.addGroundContactPoint(gc);

      ExternalForcePoint externalForcePoint = new ExternalForcePoint("forcePoint", new Vector3D(), scsRobot);
      floatingJoint.addExternalForcePoint(externalForcePoint);


      YoGraphicPosition dynamicGraphicPosition = new YoGraphicPosition(gcName + "Position", gc.getYoPosition(), 0.01, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("SphereGCPoints", dynamicGraphicPosition);

      YoGraphicVector dynamicGraphicVector = new YoGraphicVector(gcName + "Force", gc.getYoPosition(), gc.getYoForce(), 1.0/50.0);
      yoGraphicsListRegistry.registerYoGraphic("SphereForces", dynamicGraphicVector);

      initRobot(scsRobot, initialPosition);
      
      scsRobot.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scsRobot.update();

      return scsRobot;
   }

   public static void initRobot(Robot scsRobot, Vector3D initialPosition)
   {
      scsRobot.getYoTime().set(0.0);

      YoDouble q_x = (YoDouble) scsRobot.getVariable("q_x");
      YoDouble q_y = (YoDouble) scsRobot.getVariable("q_y");
      YoDouble q_z = (YoDouble) scsRobot.getVariable("q_z");
      YoDouble qd_x = (YoDouble) scsRobot.getVariable("qd_x");
      YoDouble qd_y = (YoDouble) scsRobot.getVariable("qd_y");
      YoDouble qd_z = (YoDouble) scsRobot.getVariable("qd_z");

      YoDouble q_qs = (YoDouble) scsRobot.getVariable("q_qs");
      YoDouble q_qx = (YoDouble) scsRobot.getVariable("q_qx");
      YoDouble q_qy = (YoDouble) scsRobot.getVariable("q_qy");
      YoDouble q_qz = (YoDouble) scsRobot.getVariable("q_qz");
      YoDouble qd_wx = (YoDouble) scsRobot.getVariable("qd_wx");
      YoDouble qd_wy = (YoDouble) scsRobot.getVariable("qd_wy");
      YoDouble qd_wz = (YoDouble) scsRobot.getVariable("qd_wz");

      q_x.set(initialPosition.getX());
      q_y.set(initialPosition.getY());
      q_z.set(initialPosition.getZ());

      qd_x.set(0.0);
      qd_y.set(0.0);
      qd_z.set(-0.10);

      q_qs.set(0.707106);
      q_qx.set(0.0);
      q_qy.set(0.707106);
      q_qz.set(0.0);

      qd_wx.set(0.0);
      qd_wy.set(0.0);
      qd_wz.set(0.0);
   }
}
