package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.map.hash.THashMap;
import us.ihmc.convexOptimization.quadraticProgram.OASESConstrainedQPSolver;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.ExpandingPolytopeAlgorithm;
import us.ihmc.geometry.polytope.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.geometry.polytope.SimplexPolytope;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;

public class SimpleCollisionDetector implements ScsCollisionDetector
{
   private final ArrayList<CollisionShape> collisionObjects = new ArrayList<CollisionShape>();

   // Temporary variables for computation:
//   private final RigidBodyTransform transformOne = new RigidBodyTransform();
//   private final RigidBodyTransform transformTwo = new RigidBodyTransform();
   private final Point3d centerOne = new Point3d();
   private final Point3d centerTwo = new Point3d();
   private final Vector3d centerToCenterVector = new Vector3d();

   private final Vector3d tempVector = new Vector3d();

   private double objectSmoothingRadius = 0.00003;

   public SimpleCollisionDetector()
   {
      this(0.00003);
   }

   public SimpleCollisionDetector(double objectSmoothingRadius)
   {
      this.objectSmoothingRadius = objectSmoothingRadius;
   }

   public void setObjectSmoothingRadius(double objectSmoothingRadius)
   {
      this.objectSmoothingRadius = objectSmoothingRadius;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public CollisionShapeFactory getShapeFactory()
   {
      return new SimpleCollisionShapeFactory(this);
   }

   @Override
   public void removeShape(Link link)
   {
   }

   @Override
   public CollisionShape lookupCollisionShape(Link link)
   {
      return null;
   }

   private final Random random = new Random(1776L);
   
//   private final THashMap<CollisionShapeFactory, ArrayList<CollisionShape>> collidingPairs = new THashMap<>();
   private boolean[][] haveCollided = null;
   
   @Override
   public void performCollisionDetection(CollisionDetectionResult result)
   {
      int numberOfObjects = collisionObjects.size();

      if (haveCollided == null) 
      {
         haveCollided = new boolean[numberOfObjects][numberOfObjects];
      }

      for (int i = 0; i < numberOfObjects; i++)
      {
         CollisionShape collisionShape = collisionObjects.get(i);
         collisionShape.computeTransformedCollisionShape();
      }
      
      
      for (int i = 0; i < numberOfObjects; i++)
      {         
         CollisionShape objectOne = collisionObjects.get(i);
         CollisionShapeDescription descriptionOne = objectOne.getTransformedCollisionShapeDescription();

         for (int j = i + 1; j < numberOfObjects; j++)
         {
            if ((!haveCollided[i][j]) && (random.nextDouble() < 0.9)) continue;

            CollisionShape objectTwo = collisionObjects.get(j);
            CollisionShapeDescription descriptionTwo = objectTwo.getTransformedCollisionShapeDescription();

            boolean areColliding = false;
            
            if ((descriptionOne instanceof SphereShapeDescription) && (descriptionTwo instanceof SphereShapeDescription))
            {
               doSphereSphereCollisionDetection(objectOne, (SphereShapeDescription) descriptionOne, objectTwo, (SphereShapeDescription) descriptionTwo, result);
            }
            else if ((descriptionOne instanceof PolytopeShapeDescription) && (descriptionTwo instanceof PolytopeShapeDescription))
            {
               areColliding = doPolytopePolytopeCollisionDetection(objectOne, (PolytopeShapeDescription) descriptionOne, objectTwo, (PolytopeShapeDescription) descriptionTwo,
                     result);
            }
            else if ((descriptionOne instanceof BoxShapeDescription) && (descriptionTwo instanceof BoxShapeDescription))
            {
               doBoxBoxCollisionDetection(objectOne, (BoxShapeDescription) descriptionOne, objectTwo, (BoxShapeDescription) descriptionTwo, result);
            }
            
            if (areColliding)
            {
               haveCollided[i][j] = true;
//               ArrayList<CollisionShape> arrayList = collidingPairs.get(objectOne);
//               if (arrayList == null)
//               {
//                  arrayList = new ArrayList<>();
//                  collidingPairs.put(objectOne, arrayList);
//               }
//               
//               if (!arrayList.contains(objectTwo))
//               {
//                  arrayList.add(objectTwo);
//               }
            }
         }
      }
   }

   //   private final SimpleActiveSetQPStandaloneSolver solver = new SimpleActiveSetQPStandaloneSolver();
   //   private final QuadProgSolver solver = new QuadProgSolver(16, 2, 16);
   private final OASESConstrainedQPSolver solver = new OASESConstrainedQPSolver(null);

   private final DenseMatrix64F GMatrix = new DenseMatrix64F(3, 8);
   private final DenseMatrix64F HMatrix = new DenseMatrix64F(3, 8);

   private final DenseMatrix64F GTransposeMatrix = new DenseMatrix64F(8, 3);
   private final DenseMatrix64F HTransposeMatrix = new DenseMatrix64F(8, 3);

   private final DenseMatrix64F GTransposeGMatrix = new DenseMatrix64F(8, 8);
   private final DenseMatrix64F HTransposeHMatrix = new DenseMatrix64F(8, 8);

   private final DenseMatrix64F HTransposeGMatrix = new DenseMatrix64F(8, 8);
   private final DenseMatrix64F GTransposeHMatrix = new DenseMatrix64F(8, 8);

   private final DenseMatrix64F quadraticCostGMatrix = new DenseMatrix64F(16, 16);
   private final DenseMatrix64F quadraticCostFVector = new DenseMatrix64F(16, 1);

   private final DenseMatrix64F linearEqualityConstraintA = new DenseMatrix64F(2, 16);
   private final DenseMatrix64F linearEqualityConstraintB = new DenseMatrix64F(2, 1);

   private final DenseMatrix64F linearInequalityConstraintA = new DenseMatrix64F(16, 16);
   private final DenseMatrix64F linearInequalityConstraintB = new DenseMatrix64F(16, 1);
   private final boolean[] linearInequalityActiveSet = new boolean[16];
   private final DenseMatrix64F solutionVector = new DenseMatrix64F(16, 1);
   private final Point3d vertex = new Point3d();

   private void doBoxBoxCollisionDetection(CollisionShape objectOne, BoxShapeDescription descriptionOne, CollisionShape objectTwo,
         BoxShapeDescription descriptionTwo, CollisionDetectionResult result)
   {
////      objectOne.getTransformToWorld(transformOne);
////      objectOne.getTransformToWorld(transformTwo);
//
//      populateMatrixWithBoxVertices(GMatrix, descriptionOne);
//      populateMatrixWithBoxVertices(HMatrix, descriptionTwo);
//
//      CommonOps.transpose(GMatrix, GTransposeMatrix);
//      CommonOps.transpose(HMatrix, HTransposeMatrix);
//
//      CommonOps.mult(GTransposeMatrix, GMatrix, GTransposeGMatrix);
//      CommonOps.mult(HTransposeMatrix, HMatrix, HTransposeHMatrix);
//      CommonOps.mult(HTransposeMatrix, GMatrix, HTransposeGMatrix);
//
//      CommonOps.transpose(HTransposeGMatrix, GTransposeHMatrix);
//
//      for (int i = 0; i < 8; i++)
//      {
//         for (int j = 0; j < 8; j++)
//         {
//            quadraticCostGMatrix.set(i, j, GTransposeGMatrix.get(i, j));
//            quadraticCostGMatrix.set(i + 8, j + 8, HTransposeHMatrix.get(i, j));
//
//            quadraticCostGMatrix.set(i + 8, j, -HTransposeGMatrix.get(i, j));
//            quadraticCostGMatrix.set(i, j + 8, -GTransposeHMatrix.get(i, j));
//
//         }
//
//         linearInequalityActiveSet[i] = false;
//         linearInequalityActiveSet[i + 8] = false;
//
//         linearEqualityConstraintA.set(0, i, 1.0);
//         linearEqualityConstraintA.set(1, i + 8, 1.0);
//
//         linearInequalityConstraintA.set(i, i, 1.0);
//         linearInequalityConstraintB.set(i, 0, 0.0);
//         linearInequalityConstraintA.set(i + 8, i + 8, 1.0);
//         linearInequalityConstraintB.set(i + 8, 0, 0.0);
//      }
//
//      linearEqualityConstraintB.set(0, 0, 1.0);
//      linearEqualityConstraintB.set(1, 0, 1.0);
//
//      boolean initialize = false;
//      try
//      {
//         solver.solve(quadraticCostGMatrix, quadraticCostFVector, linearEqualityConstraintA, linearEqualityConstraintB, linearInequalityConstraintA,
//               linearInequalityConstraintB, solutionVector, initialize);
//         System.out.println("solutionVector = " + solutionVector);
//
//      }
//      catch (NoConvergenceException exception)
//      {
//
//      }
   }

//   private void populateMatrixWithBoxVertices(DenseMatrix64F matrix, BoxShapeDescription descriptionOne)
//   {
//      double halfLengthXOne = descriptionOne.getHalfLengthX();
//      double halfWidthYOne = descriptionOne.getHalfWidthY();
//      double halfHeightZOne = descriptionOne.getHalfHeightZ();
//
//      int index = 0;
//
//      for (double multX = -1.0; multX < 1.1; multX += 2.0)
//      {
//         for (double multY = -1.0; multY < 1.1; multY += 2.0)
//         {
//            for (double multZ = -1.0; multZ < 1.1; multZ += 2.0)
//            {
//               vertex.set(halfLengthXOne * multX, halfWidthYOne * multY, halfHeightZOne * multZ);
//               transformOne.transform(vertex);
//               matrix.set(0, index, vertex.getX());
//               matrix.set(1, index, vertex.getY());
//               matrix.set(2, index, vertex.getZ());
//
//               index++;
//            }
//         }
//      }
//   }

   private void doSphereSphereCollisionDetection(CollisionShape objectOne, SphereShapeDescription descriptionOne, CollisionShape objectTwo,
         SphereShapeDescription descriptionTwo, CollisionDetectionResult result)
   {
      double radiusOne = descriptionOne.getRadius();
      double radiusTwo = descriptionTwo.getRadius();

      descriptionOne.getCenter(centerOne);
      descriptionTwo.getCenter(centerTwo);
      
//      transformOne.getTranslation(centerOne);
//      transformTwo.getTranslation(centerTwo);

      double distanceSquared = centerOne.distanceSquared(centerTwo);

      if (distanceSquared <= (radiusOne + radiusTwo) * (radiusOne + radiusTwo))
      {
         Vector3d normalVector = new Vector3d();

         centerToCenterVector.sub(centerTwo, centerOne);

         Point3d pointOnOne = new Point3d(centerOne);
         normalVector.set(centerToCenterVector);
         normalVector.normalize();

         tempVector.set(normalVector);
         tempVector.scale(radiusOne);
         pointOnOne.add(tempVector);

         Point3d pointOnTwo = new Point3d(centerTwo);
         tempVector.set(normalVector);
         tempVector.scale(-radiusTwo);
         pointOnTwo.add(tempVector);

         double distance = Math.sqrt(distanceSquared) - radiusOne - radiusTwo;

         SimpleContactWrapper contacts = new SimpleContactWrapper(objectOne, objectTwo);
         contacts.addContact(pointOnOne, pointOnTwo, normalVector, distance);

         result.addContact(contacts);
      }
   }

   private final GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private double epsilonRelative = 1e-4;
   private final ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm(epsilonRelative);
   private final Point3d pointOnAToPack = new Point3d();
   private final Point3d pointOnBToPack = new Point3d();

   private boolean doPolytopePolytopeCollisionDetection(CollisionShape objectOne, PolytopeShapeDescription descriptionOne, CollisionShape objectTwo,
         PolytopeShapeDescription descriptionTwo, CollisionDetectionResult result)
   {
      ConvexPolytope polytopeOne = descriptionOne.getPolytope();
      ConvexPolytope polytopeTwo = descriptionTwo.getPolytope();

      boolean areColliding = gjkCollisionDetector.arePolytopesColliding(polytopeOne, polytopeTwo, pointOnAToPack, pointOnBToPack);

      if (!areColliding)
      {
         double distanceSquared = pointOnAToPack.distanceSquared(pointOnBToPack);
         //         System.out.println(distanceSquared);
         if (distanceSquared < objectSmoothingRadius * objectSmoothingRadius) //TODO: Consider the objects instead of one constant.
         {
            //TODO: Find more than one point per object...

            SimpleContactWrapper contacts = new SimpleContactWrapper(objectOne, objectTwo);
            Vector3d normalVector = new Vector3d();
            normalVector.sub(pointOnBToPack, pointOnAToPack);

            //TODO: Magic distance number...
            if (normalVector.lengthSquared() > 1e-6)
            {
               normalVector.normalize();
               double distanceToReport = -pointOnAToPack.distance(pointOnBToPack); //0.001; //TODO: Do we even need this?
               contacts.addContact(new Point3d(pointOnAToPack), new Point3d(pointOnBToPack), normalVector, distanceToReport);
               result.addContact(contacts);
               return true;
            }
         }
      }
      else
      {
//         System.out.println("Colliding!");
         //TODO: Deal with collision of the sharp objects...
         // Need to first compute the intersecting regions...

         SimplexPolytope simplex = gjkCollisionDetector.getSimplex();
         //TODO: Add points to the simplex when it is not a tetrahedral...
         //TODO: Find more than one point per object...

//         System.out.println("simplex points = " + simplex.getNumberOfPoints());

         if (simplex.getNumberOfPoints() == 4)
         {
            expandingPolytopeAlgorithm.setPolytopes(simplex, polytopeOne, polytopeTwo);
            try
            {
               //TODO: Reduce trash here...
               Vector3d collisionNormal = new Vector3d();
               expandingPolytopeAlgorithm.computeExpandedPolytope(collisionNormal, pointOnAToPack, pointOnBToPack);

               //TODO: Magic number for normalize
               if (collisionNormal.lengthSquared() > 1e-6)
               {
                  collisionNormal.normalize();
                  SimpleContactWrapper contacts = new SimpleContactWrapper(objectOne, objectTwo);

                  double distanceToReport = -pointOnAToPack.distance(pointOnBToPack); //TODO: Do we even need this?
                  contacts.addContact(new Point3d(pointOnAToPack), new Point3d(pointOnBToPack), collisionNormal, distanceToReport);
                  result.addContact(contacts);
               }
            }
            catch (RuntimeException e)
            {
               System.err.println("\n-------------------\nTroublesome Polytopes!: ");
               System.err.println("simplex = ");
               System.err.println(simplex);
               System.err.println("polytopeOneCopy = ");
               System.err.println(polytopeOne);
               System.err.println("polytopeTwoCopy = ");
               System.err.println(polytopeTwo);
            }
         }
         
         return true;
      }

      return false;
   }

   public void addShape(CollisionShape collisionShape)
   {
      collisionObjects.add(collisionShape);
   }

}
