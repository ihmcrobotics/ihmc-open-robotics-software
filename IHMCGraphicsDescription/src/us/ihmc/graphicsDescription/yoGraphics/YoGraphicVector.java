package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.robotics.dataStructures.MutableColor;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.gui.GraphicsUpdatable;

public class YoGraphicVector extends YoGraphic implements RemoteYoGraphic, GraphicsUpdatable
{
   private double lineRadiusWhenOneMeterLong = 0.015;
   private double minRadiusScaleFactor = 0.3;
   private double maxRadiusScaleFactor = 3.0;

   private final DoubleYoVariable baseX, baseY, baseZ, x, y, z;
   protected final double scaleFactor;
   private boolean drawArrowhead;
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

   public YoGraphicVector(String name, YoFramePoint startPoint, YoFrameVector directionVector, double scale, AppearanceDefinition appearance,
                          boolean drawArrowhead, double lineRadiusWhenOneMeterLong)
   {
      this(name, startPoint, directionVector, scale, appearance, drawArrowhead);
      this.setLineRadiusWhenOneMeterLong(lineRadiusWhenOneMeterLong);
   }

   public YoGraphicVector(String name, YoFramePoint startPoint, YoFrameVector frameVector, double scale, AppearanceDefinition appearance, boolean drawArrowhead)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), startPoint.getYoZ(), frameVector.getYoX(), frameVector.getYoY(), frameVector.getYoZ(), scale,
           appearance, drawArrowhead);

      if ((!startPoint.getReferenceFrame().isWorldFrame()) || (!frameVector.getReferenceFrame().isWorldFrame()))
      {
         System.err.println("Warning: Should be in a World Frame to create a YoGraphicVector. startPoint = " + startPoint + ", frameVector = "
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
      vector3d.setX(x.getDoubleValue());
      vector3d.setY(y.getDoubleValue());
      vector3d.setZ(z.getDoubleValue());
   }

   public void getVector(FrameVector frameVector)
   {
      frameVector.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
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

   //   @Override
   //   public void update()
   //   {
   //      if (hasChanged.getAndSet(false))
   //      {
   //         if ((!pointOne.containsNaN()) && (!pointTwo.containsNaN()) && (!pointThree.containsNaN()))
   //         {
   //            instruction.setMesh(MeshDataGenerator.Polygon(new Point3D[] { pointOne.getPoint3dCopy(), pointTwo.getPoint3dCopy(), pointThree.getPoint3dCopy() }));
   //         }
   //         else
   //         {
   //            instruction.setMesh(null);
   //         }
   //      }
   //   }

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

   @Override
   public Artifact createArtifact()
   {
      MutableColor color3f = appearance.getColor();
      return new YoArtifactLineSegment2d(getName(), new YoFrameLineSegment2d(baseX, baseY, x, y, ReferenceFrame.getWorldFrame()), color3f.get());
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

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.VECTOR_DGO;
   }

   @Override
   public DoubleYoVariable[] getVariables()
   {
      return new DoubleYoVariable[] {baseX, baseY, baseZ, x, y, z};
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
