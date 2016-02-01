package us.ihmc.robotics.hierarchicalKinematics;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.dataStructures.Vector64F;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class HierarchicalTaskBodyPose extends HierarchicalTask {

   final private Vector3d  goalPosition = new Vector3d();
   final private Quat4d    goalRotation = new Quat4d();

   private int     bodyId;
   private String  bodyName;
   final private RobotModel robotModel;
   final public Vector3d jacobianPointOffset = new Vector3d();
   final public Vector3d axisToDisable = new Vector3d();

   private double positionErrorTolerance = 0.001;
   private double orientationErrorTolerance = 0.01;
   
   public HierarchicalTaskBodyPose(String name, ForwardKinematicSolver _solver, RobotModel robotModel, String bodyName)
   {
      super(name, _solver, 6);
      this.bodyName = bodyName;
      this.robotModel = robotModel;
      bodyId = robotModel.getBodyId( bodyName );
      jacobianPointOffset.set(0,0,0);
   }

   public int getBodyId(){
      return bodyId;
   }

   public void setBodyToControl(String bodyName)
   {
      this.bodyName = bodyName;
      bodyId = robotModel.getBodyId( bodyName );
   }

   public String getBodyToControl()
   {
      return bodyName;
   }

   public void getError( Vector3d positionError, Vector3d orientationError){

      Vector64F error = getError();
      positionError.set( error.get(0), error.get(1), error.get(2) );
      orientationError.set( error.get(3), error.get(4), error.get(5) );
   }

   @Override
   public void  setWeightsJointSpace(Vector64F _weights)  {

      weights_J.set(_weights);

      boolean[] relevant = forward_kinematic_solver.getRelevantJoints( bodyId ) ;

      for (int j=0 ; j< getNumJoints(); j++) {
         if( !relevant[j])  weights_J.set(j, 0);
      }
   }

   @Override
   public void setErrorTolerance(double tolerance) 
   {
      positionErrorTolerance = tolerance;
      orientationErrorTolerance = tolerance;
   };

   public void setErrorTolerance(double positionTolerance, double orientationTolerance) 
   {
      positionErrorTolerance = positionTolerance;
      orientationErrorTolerance = orientationTolerance;
   };

   @Override
   public boolean isErrorLessThanTolerance()
   {
      Vector3d positionError = new Vector3d();
      Vector3d orientationError= new Vector3d();
      getError(positionError, orientationError);

      return ( positionError.length() < positionErrorTolerance && orientationError.length() < orientationErrorTolerance );
   }


   public void disableAxisInTaskSpace(double rotationOverallWeight, Vector3d ignoreAxis)
   {
      axisToDisable.set( ignoreAxis );
      
  /*    Vector3d position = new Vector3d();
      Quat4d rotation = new Quat4d();
      this.getTarget(position, rotation);
      Matrix3d rotationMatrix = new Matrix3d();
      rotationMatrix.set( rotation );
      rotationMatrix.transform( ignoreAxis );
      
      Plane3d plane = new Plane3d( new Point3d(0,0,0), ignoreAxis);

      double W = rotationOverallWeight;

      Point3d pointX = new Point3d(W, 0, 0);
      Point3d pointY = new Point3d(0, W, 0);
      Point3d pointZ = new Point3d(0, 0, W);
      plane.orthogonalProjection(pointX);
      plane.orthogonalProjection(pointY);
      plane.orthogonalProjection(pointZ);

      weight_matrix_T.set(3,3, pointX.x );
      weight_matrix_T.set(4,3, pointX.y );
      weight_matrix_T.set(5,3, pointX.z );

      weight_matrix_T.set(3,4, pointY.x );
      weight_matrix_T.set(4,4, pointY.y );
      weight_matrix_T.set(5,4, pointY.z );

      weight_matrix_T.set(3,5, pointZ.x );
      weight_matrix_T.set(4,5, pointZ.y );
      weight_matrix_T.set(5,5, pointZ.z );  */ 
   }

   public void setTarget(Quat4d goal_quat, Vector3d pos) {
      axisToDisable.set(0,0,0);
      goalPosition.set(pos);
      goalRotation.set( goal_quat );
   }

   @Override
   public void setTarget(Vector64F goal) {
      axisToDisable.set(0,0,0);
      goalPosition.set( goal.get(0),  goal.get(1),  goal.get(2) );
      goalRotation.set( goal.get(3),  goal.get(4),   goal.get(5),  goal.get(6) );
   }

   private void getCurrent( Vector3d position, Quat4d rotation)
   {   
      RigidBodyTransform endEffectorPose = new RigidBodyTransform();
      forward_kinematic_solver.getBodyPose( bodyId, endEffectorPose );
      
      Point3d tempPos = new Point3d( jacobianPointOffset);

      endEffectorPose.transform( tempPos );  
      position.set( tempPos );
      
      endEffectorPose.get( rotation );
   }

   @Override
   public Vector64F getCurrent()
   {
      Vector3d position = new Vector3d();
      Quat4d rotation   = new Quat4d();
      Vector64F  out    = new Vector64F(7);  

      getCurrent(position, rotation);

      out.zero();
      out.set( 0, position.getX() );
      out.set( 1, position.getY() );
      out.set( 2, position.getZ() );

      out.set( 3, rotation.getX() );
      out.set( 4, rotation.getY() );
      out.set( 5, rotation.getZ() );
      out.set( 6, rotation.getW() );  

      return out;
   }

   @Override
   public Vector64F getTarget() {
      return new Vector64F( 7,
            goalPosition.x, goalPosition.y, goalPosition.z,
            goalRotation.x, goalRotation.y, goalRotation.z, goalRotation.w   );
   }

   public void getTarget(Vector3d position, Quat4d rotation ) {
      position.set( goalPosition );
      rotation.set( goalRotation );
   }
   
   final private Vector64F error = new  Vector64F(6);

   @Override
   protected Vector64F getErrorImpl() 
   {
      Vector3d  actualPos  = new Vector3d();
      Quat4d    actualRot  = new Quat4d();

      getCurrent( actualPos, actualRot);

      //----------------------------------
      error.set(0,     goalPosition.getX() - actualPos.getX() );
      error.set(1,     goalPosition.getY() - actualPos.getY() );
      error.set(2,     goalPosition.getZ() - actualPos.getZ() );

      //----------------------------------
      Vector3d deltaRot = RigidBodyTransform.getRotationDifference( actualRot,  goalRotation);

   /*   if( axisToDisable.length() > 0.001 )
      {
         // first  rotate the target to reduce to zero the error
         
         Matrix3d oldRot = new Matrix3d( );
         oldRot.set( goalRotation );
         Vector3d ignoreAxis = new Vector3d( axisToDisable );
         oldRot.transform(ignoreAxis);

         Matrix3d newRot = new Matrix3d( );
         Matrix3d counterTransform = new Matrix3d( );
      
         counterTransform.set ( new AxisAngle4d (axisToDisable , ignoreAxis.dot( deltaRot ) ) );
         newRot.mul( oldRot, counterTransform );

         Quat4d newGoalRotation = new Quat4d();
         newGoalRotation.set(newRot);

         deltaRot = RigidBodyTransform.getRotationDifference( actualRot,  newGoalRotation);

         // further try to zero the error on the disabled axis using projection      
         Plane3d plane = new Plane3d( new Point3d(0,0,0), ignoreAxis);
         Point3d projectedError = new Point3d( deltaRot );
         plane.orthogonalProjection( projectedError );
         deltaRot.set( projectedError );
      }
*/
      error.set(3, - deltaRot.getX() );
      error.set(4, - deltaRot.getY() );
      error.set(5, - deltaRot.getZ() );

      return error;
   }

   @Override
   public DenseMatrix64F computeJacobian()
   {
      forward_kinematic_solver.computeJacobian(bodyId, jacobianPointOffset, Jacobian );	
      return Jacobian;
   }

}


