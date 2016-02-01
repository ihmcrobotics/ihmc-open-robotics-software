package us.ihmc.simulationconstructionset.yoUtilities.graphics;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.plotting.Artifact;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.Transform3d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class YoGraphicVector extends YoGraphic implements RemoteYoGraphic
{
   private static final double DEFAULT_LINE_THICKNESS_RATIO = 0.02;

   private DoubleYoVariable baseX, baseY, baseZ, x, y, z;
   protected double scaleFactor;
   private boolean drawArrowhead;
   private double lineThicknessRatio;
   private final AppearanceDefinition appearance;

   public YoGraphicVector(String name, YoFramePoint startPoint, YoFrameVector frameVector, AppearanceDefinition appearance)
   {
      this(name, startPoint, frameVector, 1.0, appearance);
   }

   public YoGraphicVector(String name, YoFramePoint startPoint, YoFrameVector frameVector, double scale)
   {
      this(name, startPoint, frameVector, scale, YoAppearance.Black(), true);
   }

   public YoGraphicVector(String name, YoFramePoint startPoint, YoFrameVector frameVector, double scale, AppearanceDefinition appearance)
   {
      this(name, startPoint, frameVector, scale, appearance, true);
   }

   public YoGraphicVector(String name, YoFramePoint startPoint, YoFrameVector frameVector, double scale, AppearanceDefinition appearance,
         boolean drawArrowhead)
   {
      this(name, startPoint, frameVector, scale, appearance, drawArrowhead, -1.0);
   }

   public YoGraphicVector(String name, YoFramePoint startPoint, YoFrameVector frameVector, double scale, AppearanceDefinition appearance,
         boolean drawArrowhead, double lineThicknessRatio)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), startPoint.getYoZ(), frameVector.getYoX(), frameVector.getYoY(), frameVector.getYoZ(), scale,
            appearance, drawArrowhead, lineThicknessRatio);

      if ((!startPoint.getReferenceFrame().isWorldFrame()) || (!frameVector.getReferenceFrame().isWorldFrame()))
      {
         System.err.println("Warning: Should be in a World Frame to create a DynamicGraphicVector. startPoint = " + startPoint + ", frameVector = "
               + frameVector);
      }
   }

   public YoGraphicVector(String name, DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable x, DoubleYoVariable y,
         DoubleYoVariable z, double scaleFactor, AppearanceDefinition appearance)
   {
      this(name, baseX, baseY, baseZ, x, y, z, scaleFactor, appearance, true);
   }

   public YoGraphicVector(String name, DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable x, DoubleYoVariable y,
         DoubleYoVariable z, double scaleFactor, AppearanceDefinition appearance, boolean drawArrowhead)
   {
      this(name, baseX, baseY, baseZ, x, y, z, scaleFactor, appearance, drawArrowhead, -1.0);
   }

   public YoGraphicVector(String name, DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable x, DoubleYoVariable y,
         DoubleYoVariable z, double scaleFactor, AppearanceDefinition appearance, boolean drawArrowhead, double lineThicknessRatio)
   {
      super(name);

      this.baseX = baseX;
      this.baseY = baseY;
      this.baseZ = baseZ;
      this.x = x;
      this.y = y;
      this.z = z;
      this.drawArrowhead = drawArrowhead;
      this.scaleFactor = scaleFactor;
      this.appearance = appearance;

      if (lineThicknessRatio < 0.0)
      {
         this.lineThicknessRatio = DEFAULT_LINE_THICKNESS_RATIO;
      }
      else
      {
         this.lineThicknessRatio = lineThicknessRatio;
      }
   }

   public void getBasePosition(Point3d point3d)
   {
      point3d.x = this.baseX.getDoubleValue();
      point3d.y = this.baseY.getDoubleValue();
      point3d.z = this.baseZ.getDoubleValue();
   }

   public void getBasePosition(FramePoint framePoint)
   {
      framePoint.setX(this.baseX.getDoubleValue());
      framePoint.setY(this.baseY.getDoubleValue());
      framePoint.setZ(this.baseZ.getDoubleValue());
   }

   public void getVector(Vector3d vector3d)
   {
      vector3d.x = x.getDoubleValue();
      vector3d.y = y.getDoubleValue();
      vector3d.z = z.getDoubleValue();
   }

   public void getVector(FrameVector frameVector)
   {
      frameVector.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
   }

   public double getScale()
   {
      return scaleFactor;
   }

   private Vector3d translationVector = new Vector3d();
   private Vector3d z_rot = new Vector3d(), y_rot = new Vector3d(), x_rot = new Vector3d();
   private Matrix3d rotMatrix = new Matrix3d();

   protected void computeRotationTranslation(Transform3d transform3D)
   {
      transform3D.setIdentity();

      z_rot.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
      double length = z_rot.length();
      if (length < 1e-7)
         z_rot.set(0.0, 0.0, 1.0);
      else
         z_rot.normalize();

      if (Math.abs(z_rot.x) <= 0.99)
         x_rot.set(1.0, 0.0, 0.0);
      else
         x_rot.set(0.0, 1.0, 0.0);

      y_rot.cross(z_rot, x_rot);
      y_rot.normalize();

      x_rot.cross(y_rot, z_rot);
      x_rot.normalize();

      rotMatrix.setColumn(0, x_rot);
      rotMatrix.setColumn(1, y_rot);
      rotMatrix.setColumn(2, z_rot);

      translationVector.set(baseX.getDoubleValue(), baseY.getDoubleValue(), baseZ.getDoubleValue());

      transform3D.setScale(length * scaleFactor);
      transform3D.setTranslation(translationVector);
      transform3D.setRotation(rotMatrix);

   }

   public void set(DoubleYoVariable baseX, DoubleYoVariable baseY, DoubleYoVariable baseZ, DoubleYoVariable x, DoubleYoVariable y, DoubleYoVariable z)
   {
      this.baseX.set(baseX.getDoubleValue());
      this.baseY.set(baseY.getDoubleValue());
      this.baseZ.set(baseZ.getDoubleValue());
      this.x.set(x.getDoubleValue());
      this.y.set(y.getDoubleValue());
      this.z.set(z.getDoubleValue());
   }

   public void set(YoFramePoint base, YoFrameVector vector)
   {
      this.baseX.set(base.getX());
      this.baseY.set(base.getY());
      this.baseZ.set(base.getZ());
      this.x.set(vector.getX());
      this.y.set(vector.getY());
      this.z.set(vector.getZ());
   }

   public void set(double baseX, double baseY, double baseZ, double x, double y, double z)
   {
      this.baseX.set(baseX);
      this.baseY.set(baseY);
      this.baseZ.set(baseZ);
      this.x.set(x);
      this.y.set(y);
      this.z.set(z);
   }

   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }

   @Override
   public boolean containsNaN()
   {
      if (x.isNaN())
         return true;
      if (y.isNaN())
         return true;
      if (z.isNaN())
         return true;

      if (baseX.isNaN())
         return true;
      if (baseY.isNaN())
         return true;
      if (baseZ.isNaN())
         return true;

      return false;
   }

   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.VECTOR_DGO;
   }

   public DoubleYoVariable[] getVariables()
   {
      return new DoubleYoVariable[] { baseX, baseY, baseZ, x, y, z };
   }

   public double[] getConstants()
   {
      return new double[] { scaleFactor };
   }

   public boolean getDrawArrowhead()
   {
      return drawArrowhead;
   }

   public Graphics3DObject getLinkGraphics()
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();

      if (drawArrowhead)
      {
         linkGraphics.addCylinder(0.9, lineThicknessRatio, appearance);
         linkGraphics.translate(0.0, 0.0, 0.9);
         linkGraphics.addCone(0.1, (0.05 / 0.02) * lineThicknessRatio, appearance);
      }
      else
      {
         linkGraphics.addCylinder(1.0, lineThicknessRatio, appearance);
      }

      return linkGraphics;
   }

   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }
}
