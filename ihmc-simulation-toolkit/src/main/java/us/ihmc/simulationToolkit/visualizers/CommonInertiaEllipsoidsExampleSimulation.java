package us.ihmc.simulationToolkit.visualizers;

import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaEllipsoidsVisualizer;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.InertiaTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class CommonInertiaEllipsoidsExampleSimulation
{
   public static void main(String[] args)
   {
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint rootJoint = new SixDoFJoint("sixdof", elevator);
      RigidBodyTransform inertiaPose = new RigidBodyTransform();
      Matrix3D momentOfInertia = new Matrix3D();

      double mass = 0.7854;
      double a = 0.1;
      double b = 0.1;
      double c = 1.0;
      double ixx = 1.0 / 5.0 * mass * (b * b + c * c);
      double iyy = 1.0 / 5.0 * mass * (c * c + a * a);
      double izz = 1.0 / 5.0 * mass * (a * a + b * b);
      //      double R = 0.025;
      //      double h = 0.4;
      //
      //      double ixx = 1.0 / 12.0 * mass * h * h + 1.0 / 4.0 * mass * R * R;
      //      double iyy = 1.0 / 12.0 * mass * h * h + 1.0 / 4.0 * mass * R * R;
      //      double izz = 1.0 / 2.0 * mass * R * R;

      momentOfInertia.setM00(ixx);
      momentOfInertia.setM11(iyy);
      momentOfInertia.setM22(izz);

      System.out.println(momentOfInertia);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(-Math.PI / 2.0);
      System.out.println(transform);
      momentOfInertia = InertiaTools.rotate(transform, momentOfInertia);
      System.out.println(momentOfInertia);
      RigidBody aBody = ScrewTools.addRigidBody("test", rootJoint, momentOfInertia, mass, inertiaPose);
      rootJoint.updateFramesRecursively();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      CommonInertiaEllipsoidsVisualizer viz = new CommonInertiaEllipsoidsVisualizer(aBody, yoGraphicsListRegistry);
      viz.update();

      YoGraphicCoordinateSystem coordinateSystem = new YoGraphicCoordinateSystem("coord", new YoFramePoseUsingYawPitchRoll("World", ReferenceFrame.getWorldFrame(), null), 1);

      SimulationConstructionSet scs = new SimulationConstructionSet();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.addYoGraphic(coordinateSystem);

      Graphics3DObject obj = new Graphics3DObject();
      obj.addEllipsoid(a, b, c, YoAppearance.Orange());
      obj.rotate(-Math.PI / 4.0, Axis.X);
      //      obj.translate(0, 0, -h/2.0);
      //      obj.addCylinder(h, R, YoAppearance.Orange());
      scs.addStaticLinkGraphics(obj);

      scs.setGroundVisible(false);
      scs.startOnAThread();

   }
}
