package us.ihmc.robotics.robotDefinition;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

public class LinkDefinition
{
   private double mass;
   private DenseMatrix64F momentOfInertia = new DenseMatrix64F(3, 3);
   private Vector3d centerOfMassLocation = new Vector3d();
   private LinkGraphicsDefinition linkGraphicsDefinition;
   
   
   
}
