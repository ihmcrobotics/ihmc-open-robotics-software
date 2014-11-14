package us.ihmc.simulationconstructionset.util.perturbance;

import javax.vecmath.Vector3d;

import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;



public interface Collidable
{
   public abstract void handleCollision(Vector3d ballVelocity, double ballMass, DoubleYoVariable coefficientOfRestitution);
}