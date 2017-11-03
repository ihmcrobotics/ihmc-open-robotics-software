package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import org.apache.commons.lang3.text.WordUtils;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.DCELPolytope.CollisionDetection.HybridGJKEPACollisionDetector;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeVertex;

public class FrameConvexPolytopeTest
{
   private static final double epsilon = Epsilons.ONE_BILLIONTH;
   private RigidBodyTransform transformFromParent;
   private ReferenceFrame boxFrame;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Before
   public void setupTest()
   {
      transformFromParent = new RigidBodyTransform();
      transformFromParent.setTranslation(1, 1, 1);
      boxFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("BoxFrame", ReferenceFrame.getWorldFrame(), transformFromParent);
   }
   
   @After
   public void tearDownTest()
   {
      
   }
   
   @Test
   public void testCollisions()
   {
      ReferenceFrame polytope1Frame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("Polytope1Frame", worldFrame, transformFromParent);
      ReferenceFrame polytope2Frame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("Polytope2Frame", worldFrame, transformFromParent);
      FrameConvexPolytope polytope1 = new FrameConvexPolytope(polytope1Frame);
      FrameConvexPolytope polytope2 = new FrameConvexPolytope(polytope2Frame);
      HybridGJKEPACollisionDetector detector = new HybridGJKEPACollisionDetector();
      //detector.checkCollisionBetweenTwoPolytopes(polytope1, polytope2, new Vector3D(0.0, 0.0, 1.0));
   }
   
   @Test
   public void testConstructor()
   {
      FrameConvexPolytope framePolytope = new FrameConvexPolytope(boxFrame);
      framePolytope.addVertex(0, 0, 0, epsilon);
      framePolytope.addVertex(1, 0, 0, epsilon);
      framePolytope.addVertex(1, 1, 0, epsilon);
      framePolytope.addVertex(0, 1, 0, epsilon);

      framePolytope.addVertex(0, 0, 1, epsilon);
      framePolytope.addVertex(1, 0, 1, epsilon);
      framePolytope.addVertex(1, 1, 1, epsilon);
      framePolytope.addVertex(0, 1, 1, epsilon);
      
      PrintTools.debug("");
   }
   
   @Test
   public void testVertexConstruction()
   {
      FramePolytopeVertex vertex1 = new FramePolytopeVertex(boxFrame);
      assertTrue(vertex1 != null);
      assertTrue(vertex1.getReferenceFrame() == boxFrame);
      assertTrue(vertex1.getPosition().getX() == 0.0);
      assertTrue(vertex1.getPosition().getY() == 0.0);
      assertTrue(vertex1.getPosition().getZ() == 0.0);
   }
}
