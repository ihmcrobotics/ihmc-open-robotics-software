package us.ihmc.graphicsDescription.yoGraphics;

import java.awt.Color;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;

public class YoGraphicCoordinateSystem extends YoGraphic implements RemoteYoGraphic
{
   protected final YoDouble x, y, z, yaw, pitch, roll;
   protected final double scale;
   protected AppearanceDefinition arrowColor = YoAppearance.Gray();
   private double colorRGB32BitInt = arrowColor.getAwtColor().getRGB();
   private double transparency = arrowColor.getTransparency();

   private final double[] tempYawPitchRoll = new double[3];

   public YoGraphicCoordinateSystem(String name, YoDouble x, YoDouble y, YoDouble z, YoDouble yaw, YoDouble pitch,
         YoDouble roll, double scale)
   {
      super(name);

      this.x = x;
      this.y = y;
      this.z = z;

      this.yaw = yaw;
      this.pitch = pitch;
      this.roll = roll;

      this.scale = scale;
   }

   YoGraphicCoordinateSystem(String name, YoDouble x, YoDouble y, YoDouble z, YoDouble yaw, YoDouble pitch,
         YoDouble roll, double[] constants)
   {
      super(name);

      this.x = x;
      this.y = y;
      this.z = z;

      this.yaw = yaw;
      this.pitch = pitch;
      this.roll = roll;

      this.scale = constants[0];
      // Ensuring backward compatibility
      if (constants.length == 3)
         setArrowColor(new YoAppearanceRGBColor(new Color((int) constants[1]), constants[2]));
   }

   public YoGraphicCoordinateSystem(String name, YoFramePoint framePoint, YoFrameOrientation orientation, double scale)
   {
      super(name);

      ReferenceFrame.getWorldFrame().checkReferenceFrameMatch(framePoint);
      framePoint.checkReferenceFrameMatch(orientation.getReferenceFrame());

      x = framePoint.getYoX();
      y = framePoint.getYoY();
      z = framePoint.getYoZ();

      yaw = orientation.getYaw();
      pitch = orientation.getPitch();
      roll = orientation.getRoll();

      this.scale = scale;
   }

   public YoGraphicCoordinateSystem(String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale)
   {
      this(namePrefix, nameSuffix, registry, scale, YoAppearance.Gray());
   }

   public YoGraphicCoordinateSystem(String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale, AppearanceDefinition arrowColor)
   {
      this(namePrefix + nameSuffix, new YoFramePoint(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), new YoFrameOrientation(namePrefix,
            nameSuffix, ReferenceFrame.getWorldFrame(), registry), scale);
      setArrowColor(arrowColor);
   }

   public YoGraphicCoordinateSystem(String name, YoFramePoint framePoint, YoFrameOrientation orientation, double scale, AppearanceDefinition arrowColor)
   {
      this(name, framePoint, orientation, scale);
      setArrowColor(arrowColor);
   }

   public YoGraphicCoordinateSystem(String name, YoFramePose yoFramePose, double scale)
   {
      this(name, yoFramePose.getPosition(), yoFramePose.getOrientation(), scale);
   }
   
   public YoGraphicCoordinateSystem(String name, YoFramePose yoFramePose, double scale, AppearanceDefinition arrowColor)
   {
      this(name, yoFramePose.getPosition(), yoFramePose.getOrientation(), scale, arrowColor);
   }

   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      if (referenceFrame == null)
      {
         throw new RuntimeException("referenceFrame == null");
      }

      referenceFrame.getTransformToDesiredFrame(transformToWorld, ReferenceFrame.getWorldFrame());
      setTransformToWorld(transformToWorld);
   }

   private final Vector3D translationToWorld = new Vector3D();
   private final FrameQuaternion orientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());

   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      transformToWorld.getTranslation(translationToWorld);

      x.set(translationToWorld.getX());
      y.set(translationToWorld.getY());
      z.set(translationToWorld.getZ());

      orientation.setIncludingFrame(ReferenceFrame.getWorldFrame(), transformToWorld.getRotationMatrix());
      setOrientation(orientation);
   }

   public double getScale()
   {
      return scale;
   }

   public void getPosition(Vector3D position)
   {
      position.setX(x.getDoubleValue());
      position.setY(y.getDoubleValue());
      position.setZ(z.getDoubleValue());
   }

   public void getYawPitchRoll(Vector3D yawPitchRoll)
   {
      yawPitchRoll.setX(yaw.getDoubleValue());
      yawPitchRoll.setY(pitch.getDoubleValue());
      yawPitchRoll.setZ(roll.getDoubleValue());
   }

   public void setPosition(double x, double y, double z)
   {
      this.x.set(x);
      this.y.set(y);
      this.z.set(z);
   }

   public void setPosition(FramePoint3D position)
   {
      x.set(position.getX());
      y.set(position.getY());
      z.set(position.getZ());
   }

   public void setOrientation(FrameQuaternion orientation)
   {
      orientation.getYawPitchRoll(tempYawPitchRoll);
      setYawPitchRoll(tempYawPitchRoll);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      this.yaw.set(yaw);
      this.pitch.set(pitch);
      this.roll.set(roll);
   }

   public void setPose(FramePose3D pose)
   {
      pose.getOrientationYawPitchRoll(tempYawPitchRoll);
      setYawPitchRoll(tempYawPitchRoll);

      x.set(pose.getX());
      y.set(pose.getY());
      z.set(pose.getZ());
   }

   public void setArrowColor(AppearanceDefinition arrowColor)
   {
      this.arrowColor = arrowColor;
      this.colorRGB32BitInt = arrowColor.getAwtColor().getRGB();
      this.transparency = arrowColor.getTransparency();
   }

   public void hide()
   {
      x.set(Double.NaN);
      y.set(Double.NaN);
      z.set(Double.NaN);

      yaw.set(Double.NaN);
      pitch.set(Double.NaN);
      roll.set(Double.NaN);
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCoordinateSystem(1.0, arrowColor);

      return linkGraphics;
   }

   protected Vector3D translationVector = new Vector3D();

   @Override
   protected void computeRotationTranslation(AffineTransform transform3D)
   {
      transform3D.setIdentity();
      translationVector.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());

      double globalScale = 1.0;
      if (globalScaleProvider != null)
      {
         globalScale = globalScaleProvider.getValue();
      }

      transform3D.setScale(scale * globalScale);
      transform3D.setRotationEuler(roll.getDoubleValue(), pitch.getDoubleValue(), yaw.getDoubleValue());
      transform3D.setTranslation(translationVector);
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

      if (yaw.isNaN())
         return true;
      if (pitch.isNaN())
         return true;
      if (roll.isNaN())
         return true;

      return false;
   }

   @Override
   public Artifact createArtifact()
   {
      throw new RuntimeException("Implement Me!");
   }

   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.COORDINATE_SYSTEM_DGO;
   }

   public YoDouble[] getVariables()
   {
      return new YoDouble[] { x, y, z, yaw, pitch, roll };
   }

   public double[] getConstants()
   {
      return new double[] { scale, colorRGB32BitInt, transparency };
   }

   public AppearanceDefinition getAppearance()
   {
      return arrowColor;
   }
}
