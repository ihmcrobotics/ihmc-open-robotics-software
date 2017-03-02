package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class YoGraphicCylinder extends YoGraphic implements RemoteYoGraphic
{
   private DoubleYoVariable baseX, baseY, baseZ, x, y, z;
   private double lineThickness;
   private final AppearanceDefinition appearance;

   public YoGraphicCylinder(String name, YoFramePoint startPoint, YoFrameVector frameVector, AppearanceDefinition appearance)
   {
      this(name, startPoint, frameVector, appearance, -1.0);
   }

   public YoGraphicCylinder(String name, YoFramePoint startPoint, YoFrameVector frameVector, AppearanceDefinition appearance, double lineThickness)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), startPoint.getYoZ(), frameVector.getYoX(), frameVector.getYoY(), frameVector.getYoZ(), appearance,
            lineThickness);

      if ((!startPoint.getReferenceFrame().isWorldFrame()) || (!frameVector.getReferenceFrame().isWorldFrame()))
      {
         System.err.println("Warning: Should be in a World Frame to create a DynamicGraphicCylinder. startPoint = " + startPoint + ", frameVector = "
               + frameVector);
      }
   }

   public YoGraphicCylinder(String name, DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable x, DoubleYoVariable y,
         DoubleYoVariable z, AppearanceDefinition appearance)
   {
      this(name, baseX, baseY, baseZ, x, y, z, appearance, -1.0);
   }

   public YoGraphicCylinder(String name, DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable x, DoubleYoVariable y,
         DoubleYoVariable z, AppearanceDefinition appearance, double lineThickness)
   {
      super(name);

      this.baseX = baseX;
      this.baseY = baseY;
      this.baseZ = baseZ;
      this.x = x;
      this.y = y;
      this.z = z;
      this.lineThickness = lineThickness;
      this.appearance = appearance;
   }

   public void getBasePosition(Point3D point3d)
   {
      point3d.setX(this.baseX.getDoubleValue());
      point3d.setY(this.baseY.getDoubleValue());
      point3d.setZ(this.baseZ.getDoubleValue());
   }

   public void getBasePosition(FramePoint framePoint)
   {
      framePoint.setX(this.baseX.getDoubleValue());
      framePoint.setY(this.baseY.getDoubleValue());
      framePoint.setZ(this.baseZ.getDoubleValue());
   }

   public void getVector(Vector3D vector3d)
   {
      vector3d.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
   }

   public void getVector(FrameVector frameVector)
   {
      frameVector.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
   }

   private Vector3D translationVector = new Vector3D();
   private Vector3D z_rot = new Vector3D(), y_rot = new Vector3D(), x_rot = new Vector3D();
   private RotationMatrix rotMatrix = new RotationMatrix();

   protected void computeRotationTranslation(AffineTransform transform3D)
   {
      transform3D.setIdentity();

      z_rot.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
      double length = z_rot.length();
      if (length < 1e-7)
         z_rot.set(0.0, 0.0, 1.0);
      else
         z_rot.normalize();

      if (Math.abs(z_rot.getX()) <= 0.99)
         x_rot.set(1.0, 0.0, 0.0);
      else
         x_rot.set(0.0, 1.0, 0.0);

      y_rot.cross(z_rot, x_rot);
      y_rot.normalize();

      x_rot.cross(y_rot, z_rot);
      x_rot.normalize();

      rotMatrix.setColumns(x_rot, y_rot, z_rot);

      translationVector.set(baseX.getDoubleValue(), baseY.getDoubleValue(), baseZ.getDoubleValue());

      transform3D.setScale(lineThickness, lineThickness, length);
      transform3D.setTranslation(translationVector);
      transform3D.setRotation(rotMatrix);

   }

   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }

   @Override
   protected boolean containsNaN()
   {
      if (baseX.isNaN())
         return true;
      if (baseY.isNaN())
         return true;
      if (baseZ.isNaN())
         return true;
      if (x.isNaN())
         return true;
      if (y.isNaN())
         return true;
      if (z.isNaN())
         return true;

      return false;
   }

   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.CYLINDER_DGO;
   }

   public YoVariable<?>[] getVariables()
   {
      return new DoubleYoVariable[] { baseX, baseY, baseZ, x, y, z };
   }

   public double[] getConstants()
   {
      return new double[] { lineThickness };
   }

   public Graphics3DObject getLinkGraphics()
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(1.0, 1.0, appearance);

      return linkGraphics;
   }

   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }
}
