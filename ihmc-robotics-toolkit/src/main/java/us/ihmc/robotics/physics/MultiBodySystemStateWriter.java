package us.ihmc.robotics.physics;

import java.util.function.Consumer;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;

public interface MultiBodySystemStateWriter
{
   void setMultiBodySystem(MultiBodySystemBasics multiBodySystem);

   void write();

   public static <T extends JointBasics> MultiBodySystemStateWriter singleJointStateWriter(String jointName, Consumer<T> jointStateWriter)
   {
      return new MultiBodySystemStateWriter()
      {
         private T joint;
         
         @Override
         public void write()
         {
            jointStateWriter.accept(joint);
         }
         
         @SuppressWarnings("unchecked")
         @Override
         public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
         {
            joint = (T) RobotCollisionModel.findJoint(jointName, multiBodySystem);
         }
      };
   }
}
