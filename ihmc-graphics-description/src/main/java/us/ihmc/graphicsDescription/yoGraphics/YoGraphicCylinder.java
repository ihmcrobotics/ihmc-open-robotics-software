package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoGraphicCylinder extends YoGraphic implements RemoteYoGraphic
{
   private final YoFramePoint3D base;
   private final YoFrameVector3D vector;
   private double lineThickness;
   private final AppearanceDefinition appearance;

   public YoGraphicCylinder(String name, YoFramePoint3D startPoint, YoFrameVector3D frameVector, AppearanceDefinition appearance)
   {
      this(name, startPoint, frameVector, appearance, -1.0);
   }

   public YoGraphicCylinder(String name, YoFramePoint3D startPoint, YoFrameVector3D frameVector, AppearanceDefinition appearance, double lineThickness)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), startPoint.getYoZ(), frameVector.getYoX(), frameVector.getYoY(), frameVector.getYoZ(), appearance,
           lineThickness);

      if ((!startPoint.getReferenceFrame().isWorldFrame()) || (!frameVector.getReferenceFrame().isWorldFrame()))
      {
         System.err.println("Warning: Should be in a World Frame to create a YoGraphicCylinder. startPoint = " + startPoint + ", frameVector = " + frameVector);
      }
   }

   public YoGraphicCylinder(String name, YoDouble baseX, YoDouble baseY, YoDouble baseZ, YoDouble x, YoDouble y, YoDouble z, AppearanceDefinition appearance)
   {
      this(name, baseX, baseY, baseZ, x, y, z, appearance, -1.0);
   }

   public YoGraphicCylinder(String name, YoDouble baseX, YoDouble baseY, YoDouble baseZ, YoDouble x, YoDouble y, YoDouble z, AppearanceDefinition appearance,
                            double lineThickness)
   {
      super(name);

      base = new YoFramePoint3D(baseX, baseY, baseZ, ReferenceFrame.getWorldFrame());
      vector = new YoFrameVector3D(x, y, z, ReferenceFrame.getWorldFrame());
      this.lineThickness = lineThickness;
      this.appearance = appearance;
   }

   public void getBasePosition(Point3DBasics point3D)
   {
      point3D.set(base);
   }

   public void getBasePosition(FramePoint3DBasics framePoint3D)
   {
      framePoint3D.setIncludingFrame(base);
   }

   public void getVector(Vector3DBasics vector3D)
   {
      vector3D.set(vector3D);
   }

   public void getVector(FrameVector3DBasics frameVector3D)
   {
      frameVector3D.setIncludingFrame(vector);
   }

   private Vector3D translationVector = new Vector3D();
   private Vector3D z_rot = new Vector3D(), y_rot = new Vector3D(), x_rot = new Vector3D();
   private RotationMatrix rotMatrix = new RotationMatrix();

   protected void computeRotationTranslation(AffineTransform transform3D)
   {
      transform3D.setIdentity();

      z_rot.set(vector);
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

      translationVector.set(base);

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
      return base.containsNaN() || vector.containsNaN();
   }

   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.CYLINDER_DGO;
   }

   public YoVariable<?>[] getVariables()
   {
      return new YoDouble[] {base.getYoX(), base.getYoY(), base.getYoZ(), vector.getYoX(), vector.getYoY(), vector.getYoZ()};
   }

   public double[] getConstants()
   {
      return new double[] {lineThickness};
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
