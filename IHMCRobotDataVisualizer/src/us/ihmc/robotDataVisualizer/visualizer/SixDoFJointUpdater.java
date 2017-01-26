package us.ihmc.robotDataVisualizer.visualizer;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.jointState.SixDoFState;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SixDoFJointUpdater extends JointUpdater
{
   
   private final FloatingJoint joint;
   private final SixDoFState jointState;

   private final Matrix3d rotationMatrix = new Matrix3d();
   private final Vector3d tempVector = new Vector3d();

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
