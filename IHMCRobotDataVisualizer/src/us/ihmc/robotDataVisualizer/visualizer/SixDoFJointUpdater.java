package us.ihmc.robotDataVisualizer.visualizer;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotDataLogger.jointState.SixDoFState;
import us.ihmc.simulationconstructionset.FloatingJoint;

public class SixDoFJointUpdater extends JointUpdater
{
   
   private final FloatingJoint joint;
   private final SixDoFState jointState;

   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final Vector3D tempVector = new Vector3D();

   public SixDoFJointUpdater(FloatingJoint joint, SixDoFState jointState)
   {
      this.joint = joint;
      this.jointState = jointState;
   }

   @Override
   public void update()
   {

      
      jointState.getRotation(rotationMatrix);
      jointState.getTranslation(tempVector);
      joint.setRotation(rotationMatrix);
      joint.setPosition(tempVector);

      
      jointState.getTwistAngularPart(tempVector);
      joint.setAngularVelocityInBody(tempVector);
      
      jointState.getTwistLinearPart(tempVector);
      joint.setVelocity(tempVector);
   }
}
