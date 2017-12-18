package us.ihmc.graphicsDescription.yoGraphics;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePoint2dInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoGraphicPosition extends YoGraphic implements RemoteYoGraphic
{
   public enum GraphicType
   {
      BALL, SOLID_BALL, CROSS, BALL_WITH_CROSS, ROTATED_CROSS, BALL_WITH_ROTATED_CROSS, DIAMOND, DIAMOND_WITH_CROSS, SQUARE, SQUARE_WITH_CROSS, ELLIPSOID
   };

   protected final YoDouble x, y, z;
   private final double scale;

   private final GraphicType type;
   private final AppearanceDefinition appearance;

   private final ArrayList<Graphics3DInstruction> linkGraphicInstructions = new ArrayList<Graphics3DInstruction>();

   public YoGraphicPosition(String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale, AppearanceDefinition appearance)
   {
      this(namePrefix + nameSuffix, new YoFramePoint(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), scale, appearance);
   }

   public YoGraphicPosition(String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale, AppearanceDefinition appearance, GraphicType type)
   {
      this(namePrefix + nameSuffix, new YoFramePoint(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), scale, appearance, type);
   }

   public YoGraphicPosition(String name, YoDouble x, YoDouble y, YoDouble z, double scale, AppearanceDefinition appearance)
   {
      this(name, x, y, z, scale, appearance, GraphicType.BALL);
   }

   public YoGraphicPosition(String name, YoDouble x, YoDouble y, double scale, AppearanceDefinition appearance, GraphicType type)
   {
      this(name, x, y, null, scale, appearance, type);
   }

   public YoGraphicPosition(String name, YoDouble x, YoDouble y, YoDouble z, double scale, AppearanceDefinition appearance, GraphicType type)
   {
      super(name);

      this.x = x;
      this.y = y;
      this.z = z;

      this.scale = scale;
      this.type = type;
      this.appearance = appearance;
   }

   public YoGraphicPosition(String name, YoFramePointInMultipleFrames framePoint, double scale, AppearanceDefinition appearance)
   {
      this(name, framePoint.buildUpdatedYoFramePointForVisualizationOnly(), scale, appearance, GraphicType.BALL);
   }

   public YoGraphicPosition(String name, YoFramePoint framePoint, double scale, AppearanceDefinition appearance)
   {
      this(name, framePoint, scale, appearance, GraphicType.BALL);
   }

   public YoGraphicPosition(String name, YoFramePointInMultipleFrames framePoint, double scale, AppearanceDefinition appearance, GraphicType type)
   {
      this(name, framePoint.buildUpdatedYoFramePointForVisualizationOnly(), scale, appearance, type);
   }

   public YoGraphicPosition(String name, YoFramePoint framePoint, double scale, AppearanceDefinition appearance, GraphicType type)
   {
      super(name);

      framePoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      this.x = framePoint.getYoX();
      this.y = framePoint.getYoY();
      this.z = framePoint.getYoZ();

      this.scale = scale;

      this.type = type;
      this.appearance = appearance;
   }

   public YoGraphicPosition(String name, YoFramePoint2dInMultipleFrames framePoint, double scale, AppearanceDefinition appearance)
   {
      this(name, framePoint.buildUpdatedYoFramePointForVisualizationOnly(), scale, appearance, GraphicType.BALL);
   }

   public YoGraphicPosition(String name, YoFramePoint2d framePoint, double scale, AppearanceDefinition appearance)
   {
      this(name, framePoint, scale, appearance, GraphicType.BALL);
   }

   public YoGraphicPosition(String name, YoFramePoint2dInMultipleFrames framePoint, double scale, AppearanceDefinition appearance, GraphicType type)
   {
      this(name, framePoint.buildUpdatedYoFramePointForVisualizationOnly(), scale, appearance, type);
   }

   public YoGraphicPosition(String name, YoFramePoint2d framePoint, double scale, AppearanceDefinition appearance, GraphicType type)
   {
      super(name);

      framePoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      this.x = framePoint.getYoX();
      this.y = framePoint.getYoY();
      this.z = null;

      this.scale = scale;

      this.type = type;
      this.appearance = appearance;
   }

   public void setPositionToNaN()
   {
      setPosition(Double.NaN, Double.NaN, Double.NaN);
   }

   public void setPosition(FramePoint3DReadOnly position)
   {
      position.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setPosition((Tuple3DReadOnly) position);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(ReferenceFrame.getWorldFrame());
      getPosition((Tuple3DBasics) positionToPack);
   }

   public void setPosition(double x, double y, double z)
   {
      this.x.set(x);
      this.y.set(y);
      if (this.z != null)
         this.z.set(z);
   }

   public void getPosition(Tuple3DBasics positionToPack)
   {
      positionToPack.setX(this.getX());
      positionToPack.setY(this.getY());
      if (this.z != null)
         positionToPack.setZ(this.getZ());
      else
         positionToPack.setZ(0.0);
   }

   public double getX()
   {
      return x.getDoubleValue();
   }

   public double getY()
   {
      return y.getDoubleValue();
   }

   public double getZ()
   {
      return z.getDoubleValue();
   }

   public void setAppearance(AppearanceDefinition appearance)
   {
      for (int i = 0; i < linkGraphicInstructions.size(); i++)
      {
         linkGraphicInstructions.get(i).setAppearance(appearance);
      }
   }

   public GraphicType getType()
   {
      return type;
   }

   public double getScale()
   {
      return scale;
   }

   public Color getColor()
   {
      if (this.appearance instanceof YoAppearanceRGBColor)
      {
         YoAppearanceRGBColor yoAppearanceRGBColor = (YoAppearanceRGBColor) this.appearance;
         Color color = new Color(yoAppearanceRGBColor.getRed(), yoAppearanceRGBColor.getGreen(), yoAppearanceRGBColor.getBlue());
         return color;
      }
      else
      {
         throw new RuntimeException("No color defined");
      }
   }

   public Graphics3DObject getLinkGraphics()
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.setChangeable(true);
      switch (type)
      {
      case BALL:
      case SOLID_BALL:
      case BALL_WITH_CROSS:
      case BALL_WITH_ROTATED_CROSS:
      {
         double radius = 1.0;
         linkGraphicInstructions.add(linkGraphics.addSphere(radius, appearance));
         break;
      }
      case ELLIPSOID:
      {
         linkGraphicInstructions.add(linkGraphics.addEllipsoid(0.50, 1.0, 0.20, appearance));
         break;
      }
      case CROSS:
      {
         double R = 1.0, r = 0.16;

         linkGraphicInstructions.add(linkGraphics.addArcTorus(0.0, 2.0 * Math.PI, R, r, appearance));

         linkGraphics.rotate(Math.PI / 2.0, Axis.X);
         linkGraphics.translate(0.0, 0.0, -R);
         linkGraphicInstructions.add(linkGraphics.addCylinder(2.0 * R, r, appearance));

         linkGraphics.identity();
         linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
         linkGraphics.translate(0.0, 0.0, -R);
         linkGraphicInstructions.add(linkGraphics.addCylinder(2.0 * R, r, appearance));
         break;
      }

      case ROTATED_CROSS:
      {

         double R = 1.0, r = 0.16;

         linkGraphicInstructions.add(linkGraphics.addArcTorus(0.0, 2.0 * Math.PI, R, r, appearance));

         linkGraphics.rotate(Math.PI / 4.0, Axis.Z);
         linkGraphics.rotate(Math.PI / 2.0, Axis.X);
         linkGraphics.translate(0.0, 0.0, -R);
         linkGraphicInstructions.add(linkGraphics.addCylinder(2.0 * R, r, appearance));

         linkGraphics.identity();
         linkGraphics.rotate(Math.PI / 4.0, Axis.Z);
         linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
         linkGraphics.translate(0.0, 0.0, -R);
         linkGraphicInstructions.add(linkGraphics.addCylinder(2.0 * R, r, appearance));
         break;
      }
      default:
         throw new RuntimeException("Shouldn't get here. Seems you set an invalid YoGraphicPosition type! type = " + type);
      }

      return linkGraphics;
   }

   private Vector3D translationVector = new Vector3D();

   protected void computeRotationTranslation(AffineTransform transform3D)
   {
      transform3D.setIdentity();

      if (Double.isNaN(x.getDoubleValue()) || Double.isNaN(y.getDoubleValue()) || ((z != null) && (Double.isNaN(z.getDoubleValue()))))
      {
         translationVector.set(-1000.0, -1000.0, -1000.0);
         transform3D.setTranslation(translationVector);
      }
      else
      {
         if (z != null)
            translationVector.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
         else
            translationVector.set(x.getDoubleValue(), y.getDoubleValue(), 0.0);

         double globalScale = 1.0;
         if (globalScaleProvider != null)
         {
            globalScale = globalScaleProvider.getValue();
         }

         transform3D.setScale(scale * globalScale);
         transform3D.setTranslation(translationVector);
      }
   }

   public YoArtifactPosition createArtifact()
   {
      return new YoArtifactPosition(getName(), x, y, type, this.getColor(), scale);
   }

   @Override
   protected boolean containsNaN()
   {
      if (x.isNaN())
         return true;
      if (y.isNaN())
         return true;
      if ((z != null) && (z.isNaN()))
         return true;

      return false;
   }

   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.POSITION_DGO;
   }

   public YoVariable<?>[] getVariables()
   {
      if (z != null)
      {
         return new YoDouble[] {x, y, z};
      }
      else
      {
         return new YoDouble[] {x, y};
      }
   }

   public double[] getConstants()
   {
      return new double[] {scale, type.ordinal()};
   }

   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }

   public YoVariable<?> getYoX()
   {
      return x;
   }

   public YoVariable<?> getYoY()
   {
      return y;
   }
}
