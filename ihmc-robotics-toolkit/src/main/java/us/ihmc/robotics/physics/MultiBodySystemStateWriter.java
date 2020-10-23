package us.ihmc.robotics.physics;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;

public interface MultiBodySystemStateWriter
{
   /**
    * Sets the multi-body system that this writer is to update the state of.
    * 
    * @param multiBodySystem the system to update the state of in {@link #write()}.
    */
   void setMultiBodySystem(MultiBodySystemBasics multiBodySystem);

   /**
    * Update the state of the multi-body system and returns whether the state was actually modified.
    * 
    * @return {@code true} if the state changed, {@code false} otherwise.
    */
   boolean write();

   public static <T extends JointBasics> MultiBodySystemStateWriter singleJointStateWriter(String jointName, Consumer<T> jointStateWriter)
   {
      return new MultiBodySystemStateWriter()
      {
         private T joint;

         @Override
         public boolean write()
         {
            jointStateWriter.accept(joint);
            return true;
         }

         @SuppressWarnings("unchecked")
         @Override
         public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
         {
            joint = (T) RobotCollisionModel.findJoint(jointName, multiBodySystem);
         }
      };
   }

   public static abstract class MapBasedJointStateWriter implements MultiBodySystemStateWriter
   {
      private final Map<String, JointBasics> jointMap = new HashMap<>();

      @SuppressWarnings("unchecked")
      public <T extends JointBasics> T getJoint(String jointName)
      {
         return (T) jointMap.get(jointName);
      }

      @Override
      public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
      {
         for (JointBasics joint : multiBodySystem.getAllJoints())
         {
            jointMap.put(joint.getName(), joint);
         }
      }
   }
}
