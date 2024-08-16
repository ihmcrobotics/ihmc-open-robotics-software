package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

//TODO: Implement this test
public class InertiaVisualizationToolsTest
{
   private static final YoRegistry registry = new YoRegistry("test");

   @Test
   public void testCreateYoInertiaEllipsoids()
   {
      Random random = new Random(234234L);
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      FloatingJointBasics rootJoint = new SixDoFJoint("rootJoint", elevator);
      RigidBody body1 = MultiBodySystemRandomTools.nextRigidBody(random, "body1", rootJoint);
      JointBasics joint1 = MultiBodySystemRandomTools.nextJoint(random, "joint1", body1);
      RigidBody body2 = MultiBodySystemRandomTools.nextRigidBody(random, "body2", joint1);

      // This method actually wants the first physical body (body1), not the elevator, but it should handle this case.
      ArrayList<YoInertiaEllipsoid> inertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(elevator, registry);
      YoGraphicDefinition ellipsoidGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(inertiaEllipsoids);

      // Test that correct rigid body is stored. The elevator should have been skipped.
      assertEquals(inertiaEllipsoids.get(0).getRigidBody(), body1);
      assertEquals(inertiaEllipsoids.get(1).getRigidBody(), body2);

      // TODO: Test that the inertia ellipsoids have the correct radii for their rigid bodies.

      // Test the color setter and getter.
      int rgbInt = ColorDefinitions.toRGBA(17, 34, 51, 0.5);
      inertiaEllipsoids.get(0).setColor(rgbInt);
      assertEquals(inertiaEllipsoids.get(0).getColorInteger().getIntegerValue(), rgbInt);

      // Test the radii setter and getter.
      Vector3D radii = new Vector3D(1.0, 2.0, 3.0);
      inertiaEllipsoids.get(0).setRadii(radii);
      assertEquals(inertiaEllipsoids.get(0).getRadii().getX(), radii.getX());
      assertEquals(inertiaEllipsoids.get(0).getRadii().getY(), radii.getY());
      assertEquals(inertiaEllipsoids.get(0).getRadii().getZ(), radii.getZ());
   }
}
