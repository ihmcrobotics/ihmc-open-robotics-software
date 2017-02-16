package us.ihmc.simulationconstructionset.physics.collision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.ContactingExternalForcePoint;
import us.ihmc.simulationconstructionset.ContactingExternalForcePointsVisualizer;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeWithLink;
import us.ihmc.simulationconstructionset.physics.Contacts;

public class HybridImpulseSpringDamperCollisionHandler implements CollisionHandler
{
   private boolean visualize = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final BagOfBalls externalForcePointBalls;
   private final BagOfBalls newCollisionBalls;
   private final ContactingExternalForcePointsVisualizer contactingExternalForcePointsVisualizer;

   private final DoubleYoVariable kpCollision = new DoubleYoVariable("kpCollision", registry);
   private final DoubleYoVariable kdCollision = new DoubleYoVariable("kdCollision", registry);
   
   private double velocityForMicrocollision = 0.01; //0.1; //0.1;//0.01;
   private int numberOfCyclesPerContactPair = 1;///4
   private double minDistanceToConsiderDifferent = 0.003; //0.003; //0.002; //0.02;
   private double percentMoveTowardTouchdownWhenSamePoint = 0.2; //0.2; //0.05; //1.0; //0.05; 
   
   private static final boolean DEBUG = false;

   //TODO: Get maximumPenetrationToStart implemented for when useAverageNewCollisionTouchdownPoints = true;
   private static final boolean useAverageNewCollisionTouchdownPoints = false; //true; //false;
   private double maximumPenetrationToStart = 0.002;

   private static final boolean resolveCollisionWithAnImpact = false;
   private static final boolean allowMicroCollisions = false;

   private static final boolean performSpringDamper = true;
   private static final boolean createNewContactPairs = true;
   private static final boolean slipTowardEachOtherIfSlipping = true;

   private static final boolean allowRecyclingOfPointsInUse = true;


   private final Random random;

   private final Vector3d normal = new Vector3d();
   private final Vector3d negative_normal = new Vector3d();

   private final Point3d point1 = new Point3d();
   private final Point3d point2 = new Point3d();
   private final Point3d tempPoint = new Point3d();

   private List<CollisionHandlerListener> listeners = new ArrayList<CollisionHandlerListener>();

   // Coefficent of restitution.
   private final double epsilon;

   // Coefficient of friction
   private final double mu;

   public HybridImpulseSpringDamperCollisionHandler(double epsilon, double mu,  YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(new Random(), epsilon, mu, parentRegistry, yoGraphicsListRegistry);
   }

   /**
    *
    * @param epsilon coefficent of restitution.
    * @param mu coefficient of friction
    * @param robot Robot model
    */
   public HybridImpulseSpringDamperCollisionHandler(Random random, double epsilon, double mu, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.random = random;
      this.epsilon = epsilon;
      this.mu = mu;
      
      kpCollision.set(2000.0);
      kdCollision.set(200.0);
      
      if (yoGraphicsListRegistry == null)
      {
         visualize = false;
      }
      
      if (visualize)
      {
         externalForcePointBalls = BagOfBalls.createPatrioticBag(500, 0.0008, "contactBalls", parentRegistry, yoGraphicsListRegistry);
         newCollisionBalls = new BagOfBalls(500, 0.001, "newCollisionBalls", YoAppearance.Black(), parentRegistry, yoGraphicsListRegistry);
         int maxNumberOfDynamicGraphicPositions = 500;
         contactingExternalForcePointsVisualizer = new ContactingExternalForcePointsVisualizer(maxNumberOfDynamicGraphicPositions, yoGraphicsListRegistry, parentRegistry);
         contactingExternalForcePointsVisualizer.setForceVectorScale(0.25);
      }
      else
      {
         externalForcePointBalls = null;
         newCollisionBalls = null;
         contactingExternalForcePointsVisualizer = null;
      }
      
      parentRegistry.addChild(registry);
   }
   
   @Override
   public void maintenanceBeforeCollisionDetection()
   {
      shapesInContactList.clear();
   }

   @Override
   public void maintenanceAfterCollisionDetection()
   {
      int numberOfCollisions = shapesInContactList.size();

      Collections.shuffle(shapesInContactList, random);

      if (DEBUG) System.out.println("Resolving " + numberOfCollisions + " collisions....");

      if (visualize)
      {
         newCollisionBalls.reset();
         externalForcePointBalls.reset();
      }

      for (int i = 0; i < numberOfCollisions; i++)
      {
         Contacts shapesInContact = shapesInContactList.get(i);

         //TODO: Get rid of Type cast here...
         CollisionShapeWithLink shape1 = (CollisionShapeWithLink) shapesInContact.getShapeA();
         CollisionShapeWithLink shape2 = (CollisionShapeWithLink) shapesInContact.getShapeB();
         handleLocal(shape1, shape2, shapesInContact);
         
         if (visualize)
         {
            int numberOfContacts = shapesInContact.getNumberOfContacts();
            for (int j=0; j<numberOfContacts; j++)
            {
               Point3d locationA = new Point3d();
               shapesInContact.getWorldA(j, locationA);
               newCollisionBalls.setBall(locationA);
            }
         }
         
      }
      
      for (int index=0; index<allContactingExternalForcePoints.size(); index++)
      {
         ContactingExternalForcePoint contactingExternalForcePointOne = allContactingExternalForcePoints.get(index);
         contactingExternalForcePointOne.setForce(0.0, 0.0, 0.0);
         contactingExternalForcePointOne.setImpulse(0.0, 0.0, 0.0);
         
         if (visualize)
         {
            if (contactingExternalForcePointOne.isInContact())
            {
               externalForcePointBalls.setBall(contactingExternalForcePointOne.getPositionPoint());
            }
         }
      }
      
//      System.out.println("pairsToProcess.size() = " + pairsToProcess.size());
      for (int index=0; index<allContactingExternalForcePoints.size(); index++)
      {
         ContactingExternalForcePoint contactingExternalForcePointOne = allContactingExternalForcePoints.get(index);
         
         int indexOfContactingPair = contactingExternalForcePointOne.getIndexOfContactingPair();
         if (indexOfContactingPair == -1)
         {
            continue;
         }

         ContactingExternalForcePoint contactingExternalForcePointTwo = allContactingExternalForcePoints.get(indexOfContactingPair);
  
         if (index == indexOfContactingPair)
         {
            throw new RuntimeException();
         }
         
         if (allContactingExternalForcePoints.get(contactingExternalForcePointTwo.getIndexOfContactingPair()) != contactingExternalForcePointOne)
         {
            throw new RuntimeException();
         }
         
         if (performSpringDamper)
         {
            if (index < indexOfContactingPair)
               performSpringDamper(contactingExternalForcePointOne, contactingExternalForcePointTwo);
         }
         
         if (slipTowardEachOtherIfSlipping)
         {
            if (index < indexOfContactingPair)
               slipTowardEachOtherIfSlipping(contactingExternalForcePointOne, contactingExternalForcePointTwo);
         }
      }
      
      if (visualize)
      {
         contactingExternalForcePointsVisualizer.update();
      }
   }

   private final Point3d positionOne = new Point3d();
   private final Point3d positionTwo = new Point3d();
   private final Vector3d slipVector = new Vector3d();
   private final Vector3d tempNormal = new Vector3d();
   
   private void slipTowardEachOtherIfSlipping(ContactingExternalForcePoint contactingExternalForcePointOne,
                                              ContactingExternalForcePoint contactingExternalForcePointTwo)
   {
      boolean areSlipping = areSlipping(contactingExternalForcePointOne, contactingExternalForcePointTwo);

      if (areSlipping)
      {
         CollisionShapeWithLink collisionShapeOne = contactingExternalForcePointOne.getCollisionShape();
         CollisionShapeWithLink collisionShapeTwo = contactingExternalForcePointTwo.getCollisionShape();

         contactingExternalForcePointOne.getPosition(positionOne);
         contactingExternalForcePointTwo.getPosition(positionTwo);   
 
         boolean isPointOneInside = collisionShapeTwo.getTransformedCollisionShapeDescription().isPointInside(positionOne);
         boolean isPointTwoInside = collisionShapeOne.getTransformedCollisionShapeDescription().isPointInside(positionTwo);

         if ((!isPointOneInside) && (!isPointTwoInside))
         {
            contactingExternalForcePointOne.setIndexOfContactingPair(-1);
            contactingExternalForcePointTwo.setIndexOfContactingPair(-1);
            return;
         }

         slipVector.set(positionTwo);
         slipVector.sub(positionOne);

         contactingExternalForcePointOne.getSurfaceNormalInWorld(tempNormal);
         subtractOffNormalComponent(tempNormal, slipVector);
         slipVector.scale(0.05);

         positionOne.add(slipVector);
         positionTwo.sub(slipVector);

         if (isPointOneInside)
         {
            contactingExternalForcePointTwo.setOffsetWorld(positionTwo);
         }

         if (isPointTwoInside)
         {
            contactingExternalForcePointOne.setOffsetWorld(positionOne);
         }
      }
   }

   private boolean areSlipping(ContactingExternalForcePoint contactingExternalForcePointOne, ContactingExternalForcePoint contactingExternalForcePointTwo)
   {
      boolean isSlippingOne = contactingExternalForcePointOne.getIsSlipping();
      boolean isSlippingTwo = contactingExternalForcePointOne.getIsSlipping();

      if (isSlippingOne != isSlippingTwo)
      {
         throw new RuntimeException("Inconsistent isSlipping states!?");
      }
      
      return isSlippingOne;
   }

   private void performSpringDamper(ContactingExternalForcePoint contactingExternalForcePointOne, ContactingExternalForcePoint contactingExternalForcePointTwo)
   {
      Point3d position = new Point3d();
      Vector3d velocity = new Vector3d();
      Vector3d normal = new Vector3d();

      Point3d matchingPosition = new Point3d();
      Vector3d matchingVelocity = new Vector3d();
      Vector3d matchingNormal = new Vector3d();
 
      contactingExternalForcePointOne.getPosition(position);
      contactingExternalForcePointOne.getVelocity(velocity);
      contactingExternalForcePointOne.getSurfaceNormalInWorld(normal);
      
      contactingExternalForcePointTwo.getPosition(matchingPosition);
      contactingExternalForcePointTwo.getVelocity(matchingVelocity);
      contactingExternalForcePointTwo.getSurfaceNormalInWorld(matchingNormal);

      Vector3d positionDifference = new Vector3d();
      Vector3d velocityDifference = new Vector3d();
      
      positionDifference.set(matchingPosition);
      positionDifference.sub(position);
      
      velocityDifference.set(matchingVelocity);
      velocityDifference.sub(velocity);
      
      Vector3d springForce = new Vector3d();
      Vector3d damperForce = new Vector3d();

      springForce.set(positionDifference);
      springForce.scale(kpCollision.getDoubleValue());
      
      damperForce.set(velocityDifference);
      damperForce.scale(kdCollision.getDoubleValue());
      
      Vector3d totalForce = new Vector3d();
      totalForce.set(springForce);
      totalForce.add(damperForce);
      
      Vector3d forceAlongNormal = new Vector3d(normal);
      forceAlongNormal.scale(totalForce.dot(normal)/(normal.dot(normal)));
      
      Vector3d forcePerpendicularToNormal = new Vector3d(totalForce);
      forcePerpendicularToNormal.sub(forceAlongNormal);
      
//      System.out.println("forceAlongNormal = " + forceAlongNormal);
//      System.out.println("forcePerpendicularToNormal = " + forcePerpendicularToNormal);
      
      double forceRatio = (forcePerpendicularToNormal.length() / forceAlongNormal.length());

      if (forceAlongNormal.dot(normal) >= 0.0)
      {
         contactingExternalForcePointOne.setForce(0.0, 0.0, 0.0);
         contactingExternalForcePointTwo.setForce(0.0, 0.0, 0.0);

         contactingExternalForcePointOne.setIsSlipping(true);
         contactingExternalForcePointTwo.setIsSlipping(true);
//         System.out.println("Pulling. Let's let go!");
//         contactingExternalForcePointOne.setIndexOfContactingPair(-1);
//         contactingExternalForcePointTwo.setIndexOfContactingPair(-1);
      }

      else if (forceRatio > mu)
      {
//         System.out.println("forceRatio = " + forceRatio);

         forcePerpendicularToNormal.scale(mu/forceRatio);
         totalForce.set(forceAlongNormal);
         totalForce.add(forcePerpendicularToNormal);
         
         contactingExternalForcePointOne.setForce(totalForce);
         totalForce.negate();
         contactingExternalForcePointTwo.setForce(totalForce);

         contactingExternalForcePointOne.setIsSlipping(true);
         contactingExternalForcePointTwo.setIsSlipping(true);
//         System.out.println("Slipping. Let's let go!");
//         contactingExternalForcePointOne.setIndexOfContactingPair(-1);
//         contactingExternalForcePointTwo.setIndexOfContactingPair(-1);
      }

      else
      {
         contactingExternalForcePointOne.setForce(totalForce);
         totalForce.negate();
         contactingExternalForcePointTwo.setForce(totalForce);
         contactingExternalForcePointOne.setIsSlipping(false);
         contactingExternalForcePointTwo.setIsSlipping(false);
      }
      
      contactingExternalForcePointOne.setImpulse(0.0, 0.0, 0.0);
      contactingExternalForcePointTwo.setImpulse(0.0, 0.0, 0.0);
   }
   
   private final ArrayList<Contacts> shapesInContactList = new ArrayList<Contacts>();

   @Override
   public void handle(Contacts contacts)
   {
      shapesInContactList.add(contacts);
      //      handleLocal(shape1, shape2, contacts);
   }

   private final ArrayList<Integer> indices = new ArrayList<Integer>();
   
   private final LinkedHashSet<Robot> robotsThatAreInContactntact = new LinkedHashSet<>();

   private void handleLocal(CollisionShapeWithLink shape1, CollisionShapeWithLink shape2, Contacts contacts)
   {
      boolean shapeOneIsGround = shape1.isGround();
      boolean shapeTwoIsGround = shape2.isGround();
      if (shapeOneIsGround && shapeTwoIsGround)
      {
         // TODO: Make sure ground shapes never get checked for collisions at all...
//         throw new RuntimeException("Both shapes are ground. Shouldn't be contacting!!");
         return;
      }

      Link linkOne = shape1.getLink();
      Link linkTwo = shape2.getLink();

      //TODO: Messy train wreck here...
//      robotsThatAreInContact.add(linkOne.getParentJoint().getRobot());
//      robotsThatAreInContact.add(linkTwo.getParentJoint().getRobot());

      int numberOfContacts = contacts.getNumberOfContacts();
      indices.clear();

//      System.out.println("NumberOfContacts = " + numberOfContacts);
      for (int i = 0; i < numberOfContacts; i++)
      {
         indices.add(i);
      }

      // TODO: Smarter way of doing number of cycles.
      // Perhaps prioritize based on velocities or something.
      // Or keep track of graph of collision dependencies...

      for (int cycle = 0; cycle < numberOfCyclesPerContactPair; cycle++)
      {
         // TODO: Sims won't sim same way twice, but I don't think they do anyway...
         Collections.shuffle(indices, random);
         
         if (numberOfContacts > 1)
         {
            throw new RuntimeException("Only expecting one deepest contact each time...");
         }
         
      for (int j = 0; j < numberOfContacts; j++)
      {
         int i = indices.get(j);
         double distance = contacts.getDistance(i);

         if (distance > 0.0)
            continue;

         contacts.getWorldA(i, point1);
         contacts.getWorldB(i, point2);

         contacts.getWorldNormal(i, normal);

         if (!contacts.isNormalOnA())
         {
            normal.scale(-1.0);
         }

         // TODO handle the case where the object is embedded inside the object and the normal is invalid
         if (Double.isNaN(normal.getX()))
            throw new RuntimeException("Normal is invalid. Contains NaN!");

         negative_normal.set(normal);
         negative_normal.scale(-1.0);

         ArrayList<ContactingExternalForcePoint> contactingExternalForcePointsOne = linkOne.getContactingExternalForcePoints();
         ArrayList<ContactingExternalForcePoint> contactingExternalForcePointsTwo = linkTwo.getContactingExternalForcePoints();

         if (contactingExternalForcePointsOne.isEmpty())
         {
            throw new RuntimeException("No force points on link " + linkOne);
         }
         if (contactingExternalForcePointsTwo.isEmpty())
         {
            throw new RuntimeException("No force points on link " + linkTwo);
         }
         // Find first one attached to other part:
         boolean contactPairAlreadyExists = false;
         ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo = getPointsThatAreContactingOtherLink(contactingExternalForcePointsOne, linkTwo);
         
         ContactingExternalForcePoint externalForcePointOne = null;
         ContactingExternalForcePoint externalForcePointTwo = null;

         setSurfaceNormalToMatchNewCollision(pointsThatAreContactingShapeTwo, normal, negative_normal);
         removeContactOnPointsThatAreOutsideCollisionSandwhich(pointsThatAreContactingShapeTwo, point1, normal, point2, negative_normal);
         rollContactPointsIfRolling(pointsThatAreContactingShapeTwo);
         
         // Pick the existing pair that is close enough to the contacts:
         for (int k=0; k<pointsThatAreContactingShapeTwo.size(); k++)
         {
            ContactingExternalForcePoint contactPointToConsiderOne = pointsThatAreContactingShapeTwo.get(k);
            ContactingExternalForcePoint contactPointToConsiderTwo = allContactingExternalForcePoints.get(contactPointToConsiderOne.getIndexOfContactingPair());
 
            Vector3d deltaVectorRemovingNormalComponentsOne = new Vector3d(contactPointToConsiderOne.getPositionPoint());
            deltaVectorRemovingNormalComponentsOne.sub(point1);
            subtractOffNormalComponent(normal, deltaVectorRemovingNormalComponentsOne);
            double distanceToConsiderOne = deltaVectorRemovingNormalComponentsOne.length();
            
            Vector3d deltaVectorRemovingNormalComponentsTwo = new Vector3d(contactPointToConsiderTwo.getPositionPoint());
            deltaVectorRemovingNormalComponentsTwo.sub(point2);
            subtractOffNormalComponent(normal, deltaVectorRemovingNormalComponentsTwo);            
            double distanceToConsiderTwo = deltaVectorRemovingNormalComponentsTwo.length();

//            System.out.println("distanceToConsiderOne = " + distanceToConsiderOne);
//            System.out.println("distanceToConsiderTwo = " + distanceToConsiderTwo);

            if ((distanceToConsiderOne < minDistanceToConsiderDifferent) || (distanceToConsiderTwo < minDistanceToConsiderDifferent))
            {
               externalForcePointOne = contactPointToConsiderOne;
               externalForcePointTwo = contactPointToConsiderTwo;
               contactPairAlreadyExists = true;
               
               boolean areSlipping = true; //areSlipping(contactPointToConsiderOne, contactPointToConsiderTwo);

               if (areSlipping)
               {
                  contactPointToConsiderOne.getPosition(positionOne);
                  slipVector.set(deltaVectorRemovingNormalComponentsOne);
//                  subtractOffNormalComponent(normal, slipVector);
                  slipVector.scale(percentMoveTowardTouchdownWhenSamePoint);
                  positionOne.sub(slipVector);
                  contactPointToConsiderOne.setOffsetWorld(positionOne);

                  contactPointToConsiderTwo.getPosition(positionTwo);         
                  slipVector.set(deltaVectorRemovingNormalComponentsTwo);
//                  subtractOffNormalComponent(normal, slipVector);
                  slipVector.scale(percentMoveTowardTouchdownWhenSamePoint);
                  positionTwo.sub(slipVector);
                  contactPointToConsiderTwo.setOffsetWorld(positionTwo);
                  
//                  contactPointToConsiderOne.setOffsetWorld(point1);
//                  contactPointToConsiderTwo.setOffsetWorld(point2);

               }

               break;
            }
         }
            
         // If didn't find a pair, then create a new pair:
         if (!contactPairAlreadyExists)
         {
            externalForcePointOne = getAvailableContactingExternalForcePoint(contactingExternalForcePointsOne);
            externalForcePointTwo = getAvailableContactingExternalForcePoint(contactingExternalForcePointsTwo);
            
            if ((externalForcePointOne != null) && (externalForcePointTwo != null))
            {
               if (createNewContactPairs)
               {
                  externalForcePointOne.setIndexOfContactingPair(externalForcePointTwo.getIndex());
                  externalForcePointTwo.setIndexOfContactingPair(externalForcePointOne.getIndex());
                  
                  externalForcePointOne.setCollisionShape(shape1);
                  externalForcePointTwo.setCollisionShape(shape2);
               }
            }
            else
            {
               throw new RuntimeException("No more contact pairs are available!");
            }
         }
            
//            externalForcePointOne = pointsThatAreContactingShapeTwo.get(0);
//            externalForcePointTwo = allContactingExternalForcePoints.get(externalForcePointOne.getIndexOfContactingPair());
            
         int indexOfOne = externalForcePointOne.getIndex();
         int indexOfTwo = externalForcePointTwo.getIndex();
         
         int indexOfContactingPairOne = externalForcePointOne.getIndexOfContactingPair();
         int indexOfContactingPairTwo = externalForcePointTwo.getIndexOfContactingPair();
         
         if (createNewContactPairs)
         {
         if (indexOfOne != indexOfContactingPairTwo)
         {
            throw new RuntimeException("");
         }
         
         if (indexOfTwo != indexOfContactingPairOne)
         {
            throw new RuntimeException("");
         }
         
         if (allContactingExternalForcePoints.get(indexOfOne) != externalForcePointOne)
         {
            throw new RuntimeException("Contacting pair indices are not consistent!!!");
         }
         
         if (allContactingExternalForcePoints.get(indexOfTwo) != externalForcePointTwo)
         {
            throw new RuntimeException("Contacting pair indices are not consistent!!!");
         }
         }
         
         if (!contactPairAlreadyExists)
         {
            externalForcePointOne.setSurfaceNormalInWorld(normal);
            externalForcePointTwo.setSurfaceNormalInWorld(negative_normal);

            //TODO: What's best, setting the average of the collision points, or the actuals?
            if (useAverageNewCollisionTouchdownPoints)
            {
               tempPoint.set(point1);
               tempPoint.add(point2);
               tempPoint.scale(0.5);

               externalForcePointOne.setOffsetWorld(tempPoint);
               externalForcePointTwo.setOffsetWorld(tempPoint);
            }
            else
            {
               externalForcePointOne.setOffsetWorld(point1);
               externalForcePointTwo.setOffsetWorld(point2);
            }
         }
         
         // Update the robot and its velocity:
         //TODO: Should this be done here or somewhere else???
         Robot robot1 = linkOne.getParentJoint().getRobot();
         Robot robot2 = linkTwo.getParentJoint().getRobot();

         robot1.update();
         robot1.updateVelocities();

         if (robot2 != robot1)
         {
            robot2.update();
            robot2.updateVelocities();
         }

         if (DEBUG)
         {
            System.out.println("numberOfContacts = " + numberOfContacts);
            System.out.println("normal = " + normal);
            System.out.println("negative_normal = " + negative_normal);
            System.out.println("point1 = " + point1);
            System.out.println("point2 = " + point2);
            System.out.println("externalForcePointOne = " + externalForcePointOne);
            System.out.println("externalForcePointTwo = " + externalForcePointTwo);
         }
         
         if ((resolveCollisionWithAnImpact) && (!contactPairAlreadyExists || !performSpringDamper))
         {
            resolveCollisionWithAnImpact(shape1, shape2, shapeOneIsGround, shapeTwoIsGround, externalForcePointOne, externalForcePointTwo, allowMicroCollisions);
         }

      }
      }
   }

   private final Vector3d normalComponent = new Vector3d();

   private Vector3d subtractOffNormalComponent(Vector3d normal, Vector3d vectorToRemoveNormalComponent)
   {
      //TODO: If normal is already unit vector, don't need to divide by normal.dot(normal);
      double percentOfNormalComponent = vectorToRemoveNormalComponent.dot(normal)/(normal.dot(normal));
      normalComponent.set(normal);
      normalComponent.scale(percentOfNormalComponent);
      vectorToRemoveNormalComponent.sub(normalComponent);
      return vectorToRemoveNormalComponent;
   }

   private final Point3d tempPositionForRollingOne = new Point3d();
   private final Vector3d tempSurfaceNormalForRolllingOne = new Vector3d();
   private final Point3d tempPositionForRollingTwo = new Point3d();
   private final Vector3d tempSurfaceNormalForRolllingTwo = new Vector3d();
   private final Vector3d tempVectorForRolling = new Vector3d();

   private void rollContactPointsIfRolling(ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo)
   {
      for (int k=0; k<pointsThatAreContactingShapeTwo.size(); k++)
      {
         ContactingExternalForcePoint contactPointToConsiderOne = pointsThatAreContactingShapeTwo.get(k);
         ContactingExternalForcePoint contactPointToConsiderTwo = allContactingExternalForcePoints.get(contactPointToConsiderOne.getIndexOfContactingPair());

         contactPointToConsiderOne.getPosition(tempPositionForRollingOne);
         contactPointToConsiderOne.getSurfaceNormalInWorld(tempSurfaceNormalForRolllingOne);
         CollisionShapeWithLink collisionShapeOne = contactPointToConsiderOne.getCollisionShape();
         CollisionShapeDescription<?> collisionShapeDescriptionOne = collisionShapeOne.getTransformedCollisionShapeDescription();
         boolean wasRollingOne = collisionShapeDescriptionOne.rollContactIfRolling(tempSurfaceNormalForRolllingOne, tempPositionForRollingOne);
         contactPointToConsiderOne.setOffsetWorld(tempPositionForRollingOne);
         
         contactPointToConsiderTwo.getPosition(tempPositionForRollingTwo);
         contactPointToConsiderTwo.getSurfaceNormalInWorld(tempSurfaceNormalForRolllingTwo);
         CollisionShapeWithLink collisionShapeTwo = contactPointToConsiderTwo.getCollisionShape();
         CollisionShapeDescription<?> collisionShapeDescriptionTwo = collisionShapeTwo.getTransformedCollisionShapeDescription();
         boolean wasRollingTwo = collisionShapeDescriptionTwo.rollContactIfRolling(tempSurfaceNormalForRolllingTwo, tempPositionForRollingTwo);
         contactPointToConsiderTwo.setOffsetWorld(tempPositionForRollingTwo);
         
         if (wasRollingOne && wasRollingTwo)
         {
            return;
         }
         
         if (!wasRollingOne && !wasRollingTwo)
         {
            return;
         }
         
         if (wasRollingOne)
         {
            tempVectorForRolling.set(tempPositionForRollingOne);
            tempVectorForRolling.sub(tempPositionForRollingTwo);
            subtractOffNormalComponent(tempSurfaceNormalForRolllingOne, tempVectorForRolling);
            
            tempPositionForRollingTwo.add(tempVectorForRolling);
            contactPointToConsiderTwo.setOffsetWorld(tempPositionForRollingTwo);
         }
         
         if (wasRollingTwo)
         {
            tempVectorForRolling.set(tempPositionForRollingTwo);
            tempVectorForRolling.sub(tempPositionForRollingOne);
            subtractOffNormalComponent(tempSurfaceNormalForRolllingTwo, tempVectorForRolling);
            
            tempPositionForRollingOne.add(tempVectorForRolling);
            contactPointToConsiderOne.setOffsetWorld(tempPositionForRollingOne);
         }
      }
      
   }

   private final ArrayList<ContactingExternalForcePoint> pointsToRemove = new ArrayList<>();
   private final Point3d positionOneToConsider = new Point3d();
   private final Point3d positionTwoToConsider = new Point3d();
   private final Vector3d tempVector = new Vector3d();

   private void removeContactOnPointsThatAreOutsideCollisionSandwhich(ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo, Point3d point1,
                                                                      Vector3d normal, Point3d point2, Vector3d negativeNormal)
   {
      pointsToRemove.clear();
      
      for (int k=0; k<pointsThatAreContactingShapeTwo.size(); k++)
      {
         ContactingExternalForcePoint contactPointToConsiderOne = pointsThatAreContactingShapeTwo.get(k);
         ContactingExternalForcePoint contactPointToConsiderTwo = allContactingExternalForcePoints.get(contactPointToConsiderOne.getIndexOfContactingPair());

         contactPointToConsiderOne.getPosition(positionOneToConsider);
         contactPointToConsiderTwo.getPosition(positionTwoToConsider);
         
         tempVector.set(positionTwoToConsider);
         tempVector.sub(point1);
         if (tempVector.dot(normal) > 0.0)
         {
            contactPointToConsiderOne.setIndexOfContactingPair(-1);
            contactPointToConsiderTwo.setIndexOfContactingPair(-1);
            
            pointsToRemove.add(contactPointToConsiderOne);
         }
         else
         {
            tempVector.set(positionOneToConsider);
            tempVector.sub(point2);
            if (tempVector.dot(negativeNormal) > 0.0)
            {
               contactPointToConsiderOne.setIndexOfContactingPair(-1);
               contactPointToConsiderTwo.setIndexOfContactingPair(-1);
               
               pointsToRemove.add(contactPointToConsiderOne);
            }
         }
      }
      
      pointsThatAreContactingShapeTwo.removeAll(pointsToRemove);
   }

   private void setSurfaceNormalToMatchNewCollision(ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo, Vector3d normal, Vector3d negativeNormal)
   {
      for (int k=0; k<pointsThatAreContactingShapeTwo.size(); k++)
      {
         ContactingExternalForcePoint contactPointToConsiderOne = pointsThatAreContactingShapeTwo.get(k);
         ContactingExternalForcePoint contactPointToConsiderTwo = allContactingExternalForcePoints.get(contactPointToConsiderOne.getIndexOfContactingPair());

         contactPointToConsiderOne.setSurfaceNormalInWorld(normal);
         contactPointToConsiderTwo.setSurfaceNormalInWorld(negativeNormal); 
      }
   }

   private void resolveCollisionWithAnImpact(CollisionShapeWithLink shape1, CollisionShapeWithLink shape2, boolean shapeOneIsGround, boolean shapeTwoIsGround,
                                             ContactingExternalForcePoint externalForcePointOne, ContactingExternalForcePoint externalForcePointTwo, boolean allowMicroCollisions)
   {
      Vector3d p_world = new Vector3d();
      boolean collisionOccurred;

      if (shapeTwoIsGround)
      {
         //            System.out.println("shapeTwoIsGround");
         Vector3d velocityWorld = new Vector3d(0.0, 0.0, 0.0);

         if ((!allowMicroCollisions) || (externalForcePointOne.getVelocityVector().lengthSquared() > velocityForMicrocollision * velocityForMicrocollision))
         {
            collisionOccurred = externalForcePointOne.resolveCollision(velocityWorld, negative_normal, epsilon, mu, p_world); // link1.epsilon, link1.mu, p_world);
         }

         else
         {
            //               System.out.println("Microcollision");
            double penetrationSquared = point1.distanceSquared(point2);
            externalForcePointOne.resolveMicroCollision(penetrationSquared, velocityWorld, negative_normal, epsilon, mu, p_world);
            collisionOccurred = true;
         }
      }
      else if (shapeOneIsGround)
      {
         //            System.out.println("shapeOneIsGround");
         Vector3d velocityWorld = new Vector3d(0.0, 0.0, 0.0);
         if ((!allowMicroCollisions) || (externalForcePointTwo.getVelocityVector().lengthSquared() > velocityForMicrocollision * velocityForMicrocollision))
         {
            collisionOccurred = externalForcePointTwo.resolveCollision(velocityWorld, normal, epsilon, mu, p_world); // link1.epsilon, link1.mu, p_world);
         }

         else
         {
            //               System.out.println("Microcollision");
            double penetrationSquared = point1.distanceSquared(point2);
            externalForcePointTwo.resolveMicroCollision(penetrationSquared, velocityWorld, normal, epsilon, mu, p_world);
            collisionOccurred = true;
         }

      }
      else
      {
         //            System.out.println("Two ef points");
         Vector3d velocityVectorOne = externalForcePointOne.getVelocityVector();
         Vector3d velocityVectorTwo = externalForcePointTwo.getVelocityVector();

         Vector3d velocityDifference = new Vector3d();
         velocityDifference.sub(velocityVectorTwo, velocityVectorOne);

         if ((!allowMicroCollisions) || (velocityDifference.lengthSquared() > velocityForMicrocollision * velocityForMicrocollision))
         {
            //               System.out.println("Normal Collision");
            collisionOccurred = externalForcePointOne.resolveCollision(externalForcePointTwo, negative_normal, epsilon, mu, p_world); // link1.epsilon, link1.mu, p_world);
         }

         else
         {
            //               System.out.println("MicroCollision");
            double penetrationSquared = point1.distanceSquared(point2);
            collisionOccurred = externalForcePointOne.resolveMicroCollision(penetrationSquared, externalForcePointTwo, negative_normal, epsilon, mu, p_world); // link1.epsilon, link1.mu, p_world);
         }
      }

      if (collisionOccurred)
      {
         for (CollisionHandlerListener listener : listeners)
         {
            //               System.out.println("collision occured. Visualizing it...");
            //               System.out.println("externalForcePointOne = " + externalForcePointOne);
            //               System.out.println("externalForcePointTwo = " + externalForcePointTwo);

            listener.collision(shape1, shape2, externalForcePointOne, externalForcePointTwo, null, null);
         }
      }

   }

   private ArrayList<ContactingExternalForcePoint> getPointsThatAreContactingOtherLink(ArrayList<ContactingExternalForcePoint> contactingExternalForcePointsOne, Link linkTwo)
   {
      ArrayList<ContactingExternalForcePoint> pointsThatAreContactingShapeTwo = new ArrayList<>();
      
      for (int k=0; k<contactingExternalForcePointsOne.size(); k++)
      {
         ContactingExternalForcePoint contactingExternalForcePointOne = contactingExternalForcePointsOne.get(k);
         int indexOfContactingPair = contactingExternalForcePointOne.getIndexOfContactingPair();
         
         if (indexOfContactingPair != -1)
         {
            ContactingExternalForcePoint brotherContactingExternalForcePointTwo = allContactingExternalForcePoints.get(indexOfContactingPair);
            if (brotherContactingExternalForcePointTwo.getLink() == linkTwo)
            {
               pointsThatAreContactingShapeTwo.add(contactingExternalForcePointOne);
//               System.out.println("Found match " + pointsThatAreContactingShapeTwo.size());
            }
         } 
      }
      return pointsThatAreContactingShapeTwo;
   }

   private ContactingExternalForcePoint getAvailableContactingExternalForcePoint(ArrayList<ContactingExternalForcePoint> contactingExternalForcePoints)
   {
      for (int i=0; i<contactingExternalForcePoints.size(); i++)
      {
         ContactingExternalForcePoint contactingExternalForcePoint = contactingExternalForcePoints.get(i);
         if (contactingExternalForcePoint.getIndexOfContactingPair() == -1)
         {
            return contactingExternalForcePoint;
         }
      }

      if (allowRecyclingOfPointsInUse)
      {
         //System.err.println("Warning. Recycling a point that is in use...");
         ContactingExternalForcePoint contactingExternalForcePointToRecycleOne = contactingExternalForcePoints.get(random.nextInt(contactingExternalForcePoints.size()));
         int indexOfContactingPair = contactingExternalForcePointToRecycleOne.getIndexOfContactingPair();
         ContactingExternalForcePoint contactingExternalForcePointToRecycleTwo = allContactingExternalForcePoints.get(indexOfContactingPair);

         contactingExternalForcePointToRecycleOne.setIndexOfContactingPair(-1);
         contactingExternalForcePointToRecycleTwo.setIndexOfContactingPair(-1);

         return contactingExternalForcePointToRecycleOne;
      }
      else
      {
         System.err.println("No more contact pairs are available!");
         System.err.println("contactingExternalForcePoints.size() = " + contactingExternalForcePoints.size());

         for (int i=0; i<contactingExternalForcePoints.size(); i++)
         {
            ContactingExternalForcePoint contactingExternalForcePoint = contactingExternalForcePoints.get(i);
            System.err.println("contactingExternalForcePoint = " + contactingExternalForcePoint.getPositionPoint());
         }
      }
      
      return null;
   }

   @Override
   public void addListener(CollisionHandlerListener listener)
   {
      listeners.add(listener);
   }

   @Override
   public void handleCollisions(CollisionDetectionResult results)
   {
      //TODO: Iterate until no collisions left for stacking problems...
//      for (int j=0; j<10; j++)
      {
      this.maintenanceBeforeCollisionDetection();

      for (int i = 0; i < results.getNumberOfCollisions(); i++)
      {
         Contacts collision = results.getCollision(i);
         handle(collision);
      }

      this.maintenanceAfterCollisionDetection();
      }
   }

   
   private final ArrayList<ContactingExternalForcePoint> allContactingExternalForcePoints = new ArrayList<>();
   @Override
   public void addContactingExternalForcePoints(Link link, ArrayList<ContactingExternalForcePoint> contactingExternalForcePoints)
   {
      int index = allContactingExternalForcePoints.size();
      
      for (int i=0; i<contactingExternalForcePoints.size(); i++)
      {
         ContactingExternalForcePoint contactingExternalForcePoint = contactingExternalForcePoints.get(i);
         contactingExternalForcePoint.setIndex(index);
         allContactingExternalForcePoints.add(contactingExternalForcePoint);
         index++;
      }
      
      if (visualize)
      {
         contactingExternalForcePointsVisualizer.addPoints(contactingExternalForcePoints);
      }
   }
}
