package us.ihmc.robotics.hierarchicalKinematics;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DenseMatrix64F;
import org.xml.sax.SAXException;
import us.ihmc.robotics.dataStructures.Vector64F;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import javax.xml.parsers.ParserConfigurationException;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Map;

/**
 * The Class CollisionAvoidanceConstraint is used to build a constraint matrix that
 * can be used by the QP solver to avoid self collision.
 * It is very limited in terms of geometry, since only capsules can be used; but it 
 * is also extremely fast for the same reason.
 */
public class CollisionAvoidanceConstraint {

   final public  CapsuleCollisionCheck capsuleCheker = new CapsuleCollisionCheck();
   final private Vector3d      jacobianPointOffset   = new Vector3d();
   final private ForwardKinematicSolver forward_kinematic_solver;
   final public  ArrayList<ImmutablePair<String, String>> bodyPairs;
   final private RobotModel    model;
   private int  numConstraints;
   final private Vector64F jointWeights;

   private DenseMatrix64F jacobian_p1;
   private DenseMatrix64F jacobian_p2;

   private boolean enabled = true;

   public boolean isEnabled(){
      return enabled;
   }

   public void setEnabled(boolean enable)
   {
      enabled = enable;
   }

   /**
    * Sets the joint weights.
    * <p>
    * The value 1 means enabled, 0 is disabled and anything in between can be
    * used, but it usually make little sense.
    * A disabled joint means that the QP solver will not use that joint to avoid 
    * collisions. 
    * <p>
    * In practice, these values are applied to the column of the constraint
    * matrix returned by computeConstraints(...)
    *
    * @param weights the new joint weights.
    */
   public void setJointWeights(Vector64F weights ){
      jointWeights.set( weights );
   }

   public Vector64F getJointWeights(){
      return jointWeights;
   }

   /**
    * Load list of capsules from URDF.
    * <p> 
    * Must be the same URDF used to build the RobotModel passed in the constructor.
    *
    * @param filein the file.
    */
   public void loadURDF(File filein ) throws ParserConfigurationException, SAXException, IOException{
      capsuleCheker.loadFromURDF(filein);
   }

   
   /**
    * Instantiates a new collision avoidance constraint.
    * <p>
    * It must be noted that the instance of ForwardKinematicSolver provides the state
    * of the robot.
    *
    * @param _solver the instance of ForwardKinematicSolver. Must be up-to-date.
    * @param _model the RobotModel.
    */
   public CollisionAvoidanceConstraint(ForwardKinematicSolver _solver, RobotModel _model)
   {
      bodyPairs = new ArrayList<ImmutablePair<String, String>>();
      forward_kinematic_solver = _solver;
      model = _model;
      jacobianPointOffset.set(0,0,0); 
      numConstraints = 0;
      int numJoints = forward_kinematic_solver.getNumberOfJoints();
      jointWeights = new Vector64F( numJoints );
      jointWeights.setOnes();

      jacobian_p1 = new DenseMatrix64F( 6, numJoints );
      jacobian_p2 = new DenseMatrix64F( 6, numJoints );
   }

   /**
    * Gets the number of constraints. It corresponds to the number of pairs of bodies
    * passed using the method addBodyPair.
    *
    * @return the number of constraints
    */
   public int getNumConstraints(){
      return enabled ? numConstraints : 0;
   }

   /**
    * Add a pair of bodies that need to be checked.
    *
    * @param bodyA the name of body A
    * @param bodyB the name of body B
    */
   public void addBodyPair(String bodyA, String bodyB){

      if( model.getBodyId(bodyA) < 0) {
         throw new RuntimeException( "cant find body with name: " + bodyA);
      }

      if( model.getBodyId(bodyB) < 0) {
         throw new RuntimeException( "cant find body with name: " + bodyB);
      }

      if( bodyA.equals(bodyB)){
         return;
      }

      if( bodyA.compareTo(bodyB) < 0) {
         String temp = bodyA;
         bodyA = bodyB;
         bodyB = temp;      
      }

      boolean alreadyPresent = false;

      for (ImmutablePair<String, String> pair: bodyPairs) {
         String first  = pair.getLeft();
         String second = pair.getRight();
         if( first.equals(bodyA) && second.equals(bodyB))
         {
            alreadyPresent = true;
         }
      }

      if( !alreadyPresent )  {
         bodyPairs.add( new ImmutablePair<String, String>(bodyA, bodyB) );
         numConstraints += 1;
      }     
   }

   
   /**
    * Checks for collisions among the capsules.
    * <p>
    * You need to provide the transform yourself. 
    *
    * @param bodyTransforms limit the checks to this list of bodies and their transforms.
    * @return true, if collision was detected.
    */
   public boolean hasCollision( Map<String, RigidBodyTransform> bodyTransforms )
   {
      ImmutablePair<Point3d, Point3d> closestPoints = new ImmutablePair<Point3d, Point3d>( new Point3d(), new Point3d() );

      for (ImmutablePair<String, String> pair : bodyPairs) 
      {
         String nameBodyA = pair.getLeft();
         String nameBodyB = pair.getRight();
         
         RigidBodyTransform transformA = bodyTransforms.get(nameBodyA);
         if( transformA == null){
            continue;
         }
         
         RigidBodyTransform transformB = bodyTransforms.get(nameBodyB);
         if( transformB == null){
            continue;
         }
         
         try{            
            double distance = capsuleCheker.getClosestPoints( 
                  nameBodyA,  transformA, 
                  nameBodyB,  transformB, 
                  closestPoints);

            if (distance < 0){
               return true;
            }
         }
         catch (Exception e){
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
      }
      return false;
   }
   
   /**
    * Checks for collisions among the capsules.
    * <p>
    * The transform of each of the robot bodies is provided by the instance of ForwardKinematicSolver
    * passed to the constructor.  
    *
    * @return true, if collision was detected.
    */
   public boolean hasCollision()
   {
      if( !enabled) {
         return false;
      }
      RigidBodyTransform transformA = new RigidBodyTransform();
      RigidBodyTransform transformB = new RigidBodyTransform();
      ImmutablePair<Point3d, Point3d> closestPoints = new ImmutablePair<Point3d, Point3d>( new Point3d(), new Point3d() );

      for (ImmutablePair<String, String> pair : bodyPairs) 
      {
         String nameBodyA = pair.getLeft();
         String nameBodyB = pair.getRight();

         int bodyA =  model.getBodyId( nameBodyA );
         forward_kinematic_solver.getBodyPose( bodyA , transformA);

         int bodyB =  model.getBodyId( nameBodyB );
         forward_kinematic_solver.getBodyPose( bodyB , transformB);

         try{
            double distance = capsuleCheker.getClosestPoints( 
                  nameBodyA, transformA, 
                  nameBodyB, transformB, 
                  closestPoints);

            if (distance < 0){
               return true;
            }
         }
         catch (Exception e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }
      }
      return false;
   }

   /**
    * Checks for constraint matrix and the constraint bounds.
    * <p>
    * The transform of each of the robot bodies is provided by the instance of ForwardKinematicSolver
    * passed to the constructor.  
    *
    * @param constraintMatrix [OUTPUT] the constraint matrix. Must be [numConstraints X numJoints].
    * @param lowerBounds [OUTPUT] the lower bounds. Must be [numJoints X 1].
    */
   public void computeConstraints(DenseMatrix64F constraintMatrix, 
         Vector64F lowerBounds) throws Exception
   {
      if( !enabled) {
         return;
      }
      int numJoints = forward_kinematic_solver.getNumberOfJoints();

      Vector3d pointOffset_p1 = new Vector3d();
      Vector3d pointOffset_p2 = new Vector3d();

      RigidBodyTransform transformA = new RigidBodyTransform();
      RigidBodyTransform transformB = new RigidBodyTransform();

      Vector3d vectorN = new Vector3d();
      Point3d p1 =  new Point3d();
      Point3d p2 =  new Point3d();
      ImmutablePair<Point3d, Point3d> closestPoints = new ImmutablePair<Point3d, Point3d>( p1, p2 );

      int row = 0;

      for (ImmutablePair<String, String> pair : bodyPairs) 
      {
         String nameBodyA = pair.getLeft();
         String nameBodyB = pair.getRight();

         int bodyA =  model.getBodyId( nameBodyA );
         forward_kinematic_solver.getBodyPose( bodyA , transformA);

         int bodyB =  model.getBodyId( nameBodyB );
         forward_kinematic_solver.getBodyPose( bodyB , transformB);

         double distance = capsuleCheker.getClosestPoints( 
               nameBodyA, transformA, 
               nameBodyB, transformB, 
               closestPoints);

         vectorN.set( p2 );
         vectorN.sub( p1  );
         vectorN.normalize();
         double vectorData[] = {0,0,0};
         vectorN.get(vectorData);

         // move back into local coordinates
         RigidBodyTransform tempTrans = new RigidBodyTransform( transformA );
         tempTrans.invert();
         tempTrans.transform( p1 );

         pointOffset_p1.set( p1);

         forward_kinematic_solver.computeJacobian(bodyA, pointOffset_p1, jacobian_p1 );

         // move back into local coordinates
         tempTrans.set( transformB );
         tempTrans.invert();
         tempTrans.transform( p2 );

         pointOffset_p2.set( p2);

         forward_kinematic_solver.computeJacobian(bodyB, pointOffset_p2, jacobian_p2 );

         for (int c=0; c < numJoints; c++)
         {
            double val = 0;
            for (int r=0; r < 3; r++)
            {
               val += (jacobian_p2.get(r, c) - jacobian_p1.get(r, c)) * vectorData[r];
            }
            val *= jointWeights.get(c);
            constraintMatrix.set( row,c, val);
            lowerBounds.set( row, distance);
         }
         row++;
      }
   }

}


