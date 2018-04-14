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
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class YoGraphicCoordinateSystem extends YoGraphic implements RemoteYoGraphic
{
   protected final YoFramePoseUsingYawPitchRoll pose;
   protected final double scale;
   protected AppearanceDefinition arrowColor = YoAppearance.Gray();
   private double colorRGB32BitInt = arrowColor.getAwtColor().getRGB();
   private double transparency = arrowColor.getTransparency();

   private final double[] tempYawPitchRoll = new double[3];

   public YoGraphicCoordinateSystem(String name, YoDouble x, YoDouble y, YoDouble z, YoDouble yaw, YoDouble pitch, YoDouble roll, double scale)
   {
      super(name);

      pose = new YoFramePoseUsingYawPitchRoll(new YoFramePoint3D(x, y, z, ReferenceFrame.getWorldFrame()),
                                              new YoFrameYawPitchRoll(yaw, pitch, roll, ReferenceFrame.getWorldFrame()));
      this.scale = scale;
   }

   YoGraphicCoordinateSystem(String name, YoDouble x, YoDouble y, YoDouble z, YoDouble yaw, YoDouble pitch, YoDouble roll, double[] constants)
   {
      super(name);

      pose = new YoFramePoseUsingYawPitchRoll(new YoFramePoint3D(x, y, z, ReferenceFrame.getWorldFrame()),
                                              new YoFrameYawPitchRoll(yaw, pitch, roll, ReferenceFrame.getWorldFrame()));

      scale = constants[0];
      // Ensuring backward compatibility
      if (constants.length == 3)
         setArrowColor(new YoAppearanceRGBColor(new Color((int) constants[1]), constants[2]));
   }

   public YoGraphicCoordinateSystem(String name, YoFramePoint3D framePoint, YoFrameYawPitchRoll orientation, double scale)
   {
      super(name);

      ReferenceFrame.getWorldFrame().checkReferenceFrameMatch(framePoint);
      framePoint.checkReferenceFrameMatch(orientation.getReferenceFrame());

      pose = new YoFramePoseUsingYawPitchRoll(framePoint, orientation);

      this.scale = scale;
   }

   public YoGraphicCoordinateSystem(String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale)
   {
      this(namePrefix, nameSuffix, registry, scale, YoAppearance.Gray());
   }

   public YoGraphicCoordinateSystem(String namePrefix, String nameSuffix, YoVariableRegistry registry, double scale, AppearanceDefinition arrowColor)
   {
      this(namePrefix + nameSuffix, new YoFramePoint3D(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry),
           new YoFrameYawPitchRoll(namePrefix, nameSuffix, ReferenceFrame.getWorldFrame(), registry), scale);
      setArrowColor(arrowColor);
   }

   public YoGraphicCoordinateSystem(String name, YoFramePoint3D framePoint, YoFrameYawPitchRoll orientation, double scale, AppearanceDefinition arrowColor)
   {
      this(name, framePoint, orientation, scale);
      setArrowColor(arrowColor);
   }

   public YoGraphicCoordinateSystem(String name, YoFramePoseUsingYawPitchRoll yoFramePose, double scale)
   {
      this(name, yoFramePose.getPosition(), yoFramePose.getOrientation(), scale);
   }

   public YoGraphicCoordinateSystem(String name, YoFramePoseUsingYawPitchRoll yoFramePose, double scale, AppearanceDefinition arrowColor)
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

      pose.setPosition(transformToWorld.getTranslationVector());
      orientation.setIncludingFrame(ReferenceFrame.getWorldFrame(), transformToWorld.getRotationMatrix());
      setOrientation(orientation);
   }

   public double getScale()
   {
      return scale;
   }

   public void getPosition(Vector3D position)
   {
      position.setX(pose.getX());
      position.setY(pose.getY());
      position.setZ(pose.getZ());
   }

   public void getYawPitchRoll(Vector3D yawPitchRoll)
   {
      yawPitchRoll.setX(pose.getYaw());
      yawPitchRoll.setY(pose.getPitch());
      yawPitchRoll.setZ(pose.getRoll());
   }

   public void setPosition(double x, double y, double z)
   {
      pose.setPosition(x, y, z);
   }

   public void setPosition(FramePoint3D position)
   {
      pose.setPosition(position);
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
      pose.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setPose(FramePose3D pose)
   {
      this.pose.set(pose);
   }

   public void setArrowColor(AppearanceDefinition arrowColor)
   {
      this.arrowColor = arrowColor;
      colorRGB32BitInt = arrowColor.getAwtColor().getRGB();
      transparency = arrowColor.getTransparency();
   }

   public void hide()
   {
      pose.setToNaN();
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
      translationVector.set(pose.getX(), pose.getY(), pose.getZ());

      double globalScale = 1.0;
      if (globalScaleProvider != null)
      {
         globalScale = globalScaleProvider.getValue();
      }

      transform3D.setScale(scale * globalScale);
      transform3D.setRotationEuler(pose.getRoll(), pose.getPitch(), pose.getYaw());
      transform3D.setTranslation(translationVector);
   }

   @Override
   public boolean containsNaN()
   {
      return pose.containsNaN();
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

   @Override
   public YoDouble[] getVariables()
   {
      return new YoDouble[] {pose.getYoX(), pose.getYoY(), pose.getYoZ(), pose.getYoYaw(), pose.getYoPitch(), pose.getYoRoll()};
   }

   @Override
   public double[] getConstants()
   {
      return new double[] {scale, colorRGB32BitInt, transparency};
   }

   @Override
   public YoGraphicCoordinateSystem duplicate(YoVariableRegistry newRegistry)
   {
      return new YoGraphicCoordinateSystem(getName(), pose.duplicate(newRegistry), scale, arrowColor);
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      return arrowColor;
   }
}
