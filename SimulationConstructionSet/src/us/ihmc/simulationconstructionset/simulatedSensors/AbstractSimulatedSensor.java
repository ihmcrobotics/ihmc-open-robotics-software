package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.simulationconstructionset.SimulatedSensor;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public abstract class AbstractSimulatedSensor implements SimulatedSensor
{
   protected RigidBodyTransform transformToHere = new RigidBodyTransform();
   protected RigidBodyTransform transformFromJoint = new RigidBodyTransform();
   protected Graphics3DAdapter graphics3dAdapter;
   protected boolean dataIsLocked;

   @Override
   public void setWorld(Graphics3DAdapter graphics3dAdapter)
   {
      this.graphics3dAdapter = graphics3dAdapter;
   }

   @Override
   public void updateTransform(RigidBodyTransform transformToHere, double time)
   {
      this.transformToHere.set(transformToHere);
      this.transformToHere.multiply(transformFromJoint);
   }
}
