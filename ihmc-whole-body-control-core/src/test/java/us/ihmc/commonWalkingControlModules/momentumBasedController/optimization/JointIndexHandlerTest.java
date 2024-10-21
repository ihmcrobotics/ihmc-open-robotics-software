package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import gnu.trove.list.array.TIntArrayList;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class JointIndexHandlerTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   private Random random;

   @BeforeEach
   public void setUp()
   {
      random = new Random(1986L);
   }

   @Test
   public void testComputeIndicesForJoint()
   {
      Vector3D[] jointAxes = {X, Y, Z, Y, X};
      MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain chain = new MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain(random, jointAxes);
      JointBasics[] jointsArr = MultiBodySystemTools.collectSubtreeJoints(chain.getElevator());
      JointBasics rootJoint = jointsArr[0];
      JointBasics testJoint4 = jointsArr[5];


      TIntArrayList indices = new TIntArrayList();
      JointIndexHandler.computeIndicesForJoint(jointsArr, indices, testJoint4, rootJoint);
      assertEquals(7, indices.size());

      for(int i = 0; i < rootJoint.getDegreesOfFreedom(); i++)
      {
         assertEquals(i, indices.get(i));
      }
      assertEquals(10, indices.get(indices.size() - 1));
   }
}
