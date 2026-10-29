package us.ihmc.rdx.simulation.environment.object;

import com.badlogic.gdx.math.Matrix4;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.robot.RobotDefinition;

/**
 * TODO: This class probably needs renaming and setup as layers of interfaces.
 *
 * TODO: This object can be added to SCS 2 as a robot or a terrian definition.
 */
public class RDXSCS2EnvironmentObject extends RDXSCS2SimpleObject
{
   private final Matrix4 tempGDXTransform = new Matrix4();
   private float mass = 0.0f;
   private final Point3D centerOfMassInModelFrame = new Point3D();
   private boolean isSelected = false;
   private RobotDefinition robotDefinition;

   public RDXSCS2EnvironmentObject(String titleCasedName, RDXSCS2EnvironmentObjectFactory factory)
   {
      super(titleCasedName, factory);
   }

   public RDXSCS2EnvironmentObject duplicate()
   {
      return factory.getSupplier().get();
   }

   public void setMass(float mass)
   {
      this.mass = mass;
   }

   public float getMass()
   {
      return mass;
   }

   public Point3D getCenterOfMassInModelFrame()
   {
      return centerOfMassInModelFrame;
   }

   public void setSelected(boolean selected)
   {
      isSelected = selected;
   }
}
