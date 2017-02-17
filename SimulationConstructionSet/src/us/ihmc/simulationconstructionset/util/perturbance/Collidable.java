package us.ihmc.simulationconstructionset.util.perturbance;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;



public interface Collidable
{
   public abstract void handleCollision(Vector3D ballVelocity, double ballMass, DoubleYoVariable coefficientOfRestitution);
}