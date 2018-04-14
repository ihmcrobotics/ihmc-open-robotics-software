package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.color.MutableColor;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class YoGraphicVector extends YoGraphic implements RemoteYoGraphic, GraphicsUpdatable
{
   private double lineRadiusWhenOneMeterLong = 0.015;
   private double minRadiusScaleFactor = 0.3;
   private double maxRadiusScaleFactor = 3.0;

   private final YoFramePoint3D base;
   private final YoFrameVector3D vector;
   protected final double scaleFactor;
   private boolean drawArrowhead;
   private final AppearanceDefinition appearance;

   public YoGraphicVector(String name, YoFramePoint3D startPoint, YoFrameVector3D frameVector, AppearanceDefinition appearance)
   {
      this(name, startPoint, frameVector, 1.0, appearance);
   }

   public YoGraphicVector(String name, YoFramePoint3D startPoint, YoFrameVector3D frameVector, double scale)
   {
      this(name, startPoint, frameVector, scale, YoAppearance.Black(), true);
   }

   public YoGraphicVector(String name, YoFramePoint3D startPoint, YoFrameVector3D frameVector, double scale, AppearanceDefinition appearance)
   {
      this(name, startPoint, frameVector, scale, appearance, true);
   }

   public YoGraphicVector(String name, YoFramePoint3D startPoint, YoFrameVector3D directionVector, double scale, AppearanceDefinition appearance,
                          boolean drawArrowhead, double lineRadiusWhenOneMeterLong)
   {
      this(name, startPoint, directionVector, scale, appearance, drawArrowhead);
      this.setLineRadiusWhenOneMeterLong(lineRadiusWhenOneMeterLong);
   }

   public YoGraphicVector(String name, YoFramePoint3D startPoint, YoFrameVector3D frameVector, double scale, AppearanceDefinition appearance,
                          boolean drawArrowhead)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), startPoint.getYoZ(), frameVector.getYoX(), frameVector.getYoY(), frameVector.getYoZ(), scale,
           appearance, drawArrowhead);

      if ((!startPoint.getReferenceFrame().isWorldFrame()) || (!frameVector.getReferenceFrame().isWorldFrame()))
      {
         System.err.println("Warning: Should be in a World Frame to create a YoGraphicVector. startPoint = " + startPoint + ", frameVector = " + frameVector);
      }
   }

   public YoGraphicVector(String name, YoDouble baseX, YoDouble baseY, YoDouble baseZ, YoDouble x, YoDouble y, YoDouble z, double scaleFactor,
                          AppearanceDefinition appearance)
   {
      this(name, baseX, baseY, baseZ, x, y, z, scaleFactor, appearance, true);
   }

   public YoGraphicVector(String name, YoDouble baseX, YoDouble baseY, YoDouble baseZ, YoDouble x, YoDouble y, YoDouble z, double scaleFactor,
                          AppearanceDefinition appearance, boolean drawArrowhead)
   {
      super(name);

      base = new YoFramePoint3D(baseX, baseY, baseZ, ReferenceFrame.getWorldFrame());
      vector = new YoFrameVector3D(x, y, z, ReferenceFrame.getWorldFrame());
      this.drawArrowhead = drawArrowhead;
      this.scaleFactor = scaleFactor;
      this.appearance = appearance;
   }

   public void setLineRadiusWhenOneMeterLong(double lineRadiusWhenOneMeterLong)
   {
      this.lineRadiusWhenOneMeterLong = lineRadiusWhenOneMeterLong;
   }

   public void setMinAndMaxRadiusScaleFactors(double minRadiusScaleFactor, double maxRadiusScaleFactor)
   {
      this.minRadiusScaleFactor = minRadiusScaleFactor;
      this.maxRadiusScaleFactor = maxRadiusScaleFactor;
   }

   public void setDrawArrowhead(boolean drawArrowhead)
   {
      this.drawArrowhead = drawArrowhead;
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
      vector3D.set(vector);
   }

   public void getVector(FrameVector3DBasics frameVector3D)
   {
      frameVector3D.setIncludingFrame(vector);
   }

   public double getScale()
   {
      return scaleFactor;
   }

   private Vector3D translationVector = new Vector3D();
   private Vector3D z_rot = new Vector3D(), y_rot = new Vector3D(), x_rot = new Vector3D();
   private RotationMatrix rotMatrix = new RotationMatrix();

   @Override
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

      double globalScale = 1.0;
      if (globalScaleProvider != null)
      {
         globalScale = globalScaleProvider.getValue();
      }

      double xyScaleFactor = length * scaleFactor * globalScale;

      if (xyScaleFactor < minRadiusScaleFactor)
      {
         xyScaleFactor = minRadiusScaleFactor;
      }
      if (xyScaleFactor > maxRadiusScaleFactor)
      {
         xyScaleFactor = maxRadiusScaleFactor;
      }

      transform3D.setScale(xyScaleFactor, xyScaleFactor, length * scaleFactor * globalScale);
      transform3D.setTranslation(translationVector);
      transform3D.setRotation(rotMatrix);
   }

   public void set(YoDouble baseX, YoDouble baseY, YoDouble baseZ, YoDouble x, YoDouble y, YoDouble z)
   {
      base.set(baseX.getValue(), baseY.getValue(), baseZ.getValue());
      vector.set(x.getValue(), y.getValue(), z.getValue());
   }

   public void set(FramePoint3DReadOnly base, FrameVector3DReadOnly vector)
   {
      this.base.set(base);
      this.vector.set(vector);
   }

   public void set(double baseX, double baseY, double baseZ, double x, double y, double z)
   {
      base.set(baseX, baseY, baseZ);
      vector.set(x, y, z);
   }

   @Override
   public Artifact createArtifact()
   {
      MutableColor color3f = appearance.getColor();
      YoDouble endPointX = new YoDouble(getName() + "ArtifactEndPointX", base.getYoX().getYoVariableRegistry());
      YoDouble endPointY = new YoDouble(getName() + "ArtifactEndPointY", base.getYoY().getYoVariableRegistry());

      base.getYoX().addVariableChangedListener(v -> endPointX.set(base.getX() + vector.getX()));
      base.getYoY().addVariableChangedListener(v -> endPointY.set(base.getY() + vector.getY()));
      vector.getYoX().addVariableChangedListener(v -> endPointX.set(base.getX() + vector.getX()));
      vector.getYoY().addVariableChangedListener(v -> endPointY.set(base.getY() + vector.getY()));

      return new YoArtifactLineSegment2d(getName(),
                                         new YoFrameLineSegment2D(base.getYoX(), base.getYoY(), endPointX, endPointY, ReferenceFrame.getWorldFrame()),
                                         color3f.get());
   }

   @Override
   public boolean containsNaN()
   {
      return base.containsNaN() || vector.containsNaN();
   }

   public void hide()
   {
      base.setToNaN();
      vector.setToNaN();
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.VECTOR_DGO;
   }

   @Override
   public YoDouble[] getVariables()
   {
      return new YoDouble[] {base.getYoX(), base.getYoY(), base.getYoZ(), vector.getYoX(), vector.getYoY(), vector.getYoZ()};
   }

   @Override
   public double[] getConstants()
   {
      return new double[] {scaleFactor};
   }

   public boolean getDrawArrowhead()
   {
      return drawArrowhead;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();

      if (drawArrowhead)
      {
         linkGraphics.addCylinder(0.9, lineRadiusWhenOneMeterLong, appearance);
         linkGraphics.translate(0.0, 0.0, 0.9);
         linkGraphics.addCone(0.1, (0.05 / 0.02) * lineRadiusWhenOneMeterLong, appearance);
      }
      else
      {
         linkGraphics.addCylinder(1.0, lineRadiusWhenOneMeterLong, appearance);
      }

      return linkGraphics;
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }
}
