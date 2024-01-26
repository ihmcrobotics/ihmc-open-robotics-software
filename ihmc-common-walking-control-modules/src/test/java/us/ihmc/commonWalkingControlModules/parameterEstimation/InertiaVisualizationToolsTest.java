package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Random;

//TODO: Implement this test
public class InertiaVisualizationToolsTest
{
   private static final double EPSILON = 1e-9;
   private static final int ITERATIONS = 1000;
   private static final YoRegistry registry = new YoRegistry("test");

   @Test
   public void testCreateYoInertiaEllipsoids()
   {
      Random random = new Random(234234L);
      String bodyName = "rootBody";
      RevoluteJoint parentJoint = null;
      double Ixx = 1.0;
      double Iyy = 1.0;
      double Izz = 1.0;
      double mass = 1.0;
      Vector3D centerOfMassOffset = new Vector3D();
      RigidBody rootBody = new RigidBody(bodyName, parentJoint, Ixx, Iyy, Izz, mass, centerOfMassOffset);
      JointBasics joint = MultiBodySystemRandomTools.nextJoint(random, bodyName, rootBody);
      //      RigidBody body = MultiBodySystemRandomTools.nextRigidBody(random, bodyName, joint);
      RigidBody body = new RigidBody(bodyName, joint, 0.5, 0.4, 0.3, 0.2, centerOfMassOffset);

      ArrayList<YoInertiaEllipsoid> inertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(body, registry);
      YoGraphicDefinition ellipsoidGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(rootBody, inertiaEllipsoids);

      //      assertEquals(inertiaEllipsoids.get(0).getRadii().getX(),rootBody.getInertia().getMomentOfInertia().get)
   }
}
