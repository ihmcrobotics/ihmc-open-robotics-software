package us.ihmc.commonWalkingControlModules.visualizer;

import javax.vecmath.Matrix3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.InertiaTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.yoUtilities.graphics.YoGraphicCoordinateSystem;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

public class CommonInertiaEllipsoidsExampleSimulation
{
   public static void main(String[] args)
   {
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint rootJoint = new SixDoFJoint("sixdof", elevator, ReferenceFrame.getWorldFrame());
      RigidBodyTransform inertiaPose = new RigidBodyTransform();
      Matrix3d momentOfInertia = new Matrix3d();

      double mass = 1.0;
      double a = 0.1;
      double b = 1;
      double c = 0.1;
      double ixx = 1.0 / 5.0 * mass * (b * b + c * c);
      double iyy = 1.0 / 5.0 * mass * (c * c + a * a);
      double izz = 1.0 / 5.0 * mass * (a * a + b * b);
      momentOfInertia.setM00(ixx);
      momentOfInertia.setM11(iyy);
      momentOfInertia.setM22(izz);
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.rotX(3.0*Math.PI/4.0);
      momentOfInertia = InertiaTools.rotate(transform, momentOfInertia);
      RigidBody aBody = ScrewTools.addRigidBody("test", rootJoint, momentOfInertia, mass, inertiaPose);
      rootJoint.updateFramesRecursively();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      CommonInertiaEllipsoidsVisualizer viz = new CommonInertiaEllipsoidsVisualizer(aBody, yoGraphicsListRegistry);
      viz.update();

      YoGraphicCoordinateSystem coordinateSystem = new YoGraphicCoordinateSystem("coord", new YoFramePose("World", ReferenceFrame.getWorldFrame(), null), 1);

      SimulationConstructionSet scs = new SimulationConstructionSet();
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.addYoGraphic(coordinateSystem);
     
      Graphics3DObject obj = new Graphics3DObject();
      obj.addEllipsoid(a, b, c, YoAppearance.Orange());
      scs.addStaticLinkGraphics(obj);
      
      scs.setGroundVisible(false);
      scs.startOnAThread();

   }
}
