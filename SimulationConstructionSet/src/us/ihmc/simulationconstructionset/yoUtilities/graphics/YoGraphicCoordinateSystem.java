package us.ihmc.simulationconstructionset.yoUtilities.graphics;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.plotting.Artifact;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.Transform3d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;

public class YoGraphicCoordinateSystem extends YoGraphic implements RemoteYoGraphic
{
   protected final DoubleYoVariable x, y, z, yaw, pitch, roll;
   protected final double scale;
   protected AppearanceDefinition arrowColor = YoAppearance.Gray();

   private final double[] tempYawPitchRoll = new double[3];

   public YoGraphicCoordinateSystem(String name, DoubleYoVariable x, DoubleYoVariable y, DoubleYoVariable z, DoubleYoVariable yaw, DoubleYoVariable pitch,
         DoubleYoVariable roll, double scale)
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

   public YoGraphicCoordinateSystem(String name, YoFramePoint framePoint, YoFrameOrientation orientation, double scale)
   {
      super(name);

      framePoint.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
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
      this.arrowColor = arrowColor;
   }

   public YoGraphicCoordinateSystem(String name, YoFramePoint framePoint, YoFrameOrientation orientation, double scale, AppearanceDefinition arrowColor)
   {
      this(name, framePoint, orientation, scale);
      this.arrowColor = arrowColor;
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

   private final Vector3d translationToWorld = new Vector3d();
   private final FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame());

   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      transformToWorld.get(translationToWorld);

      x.set(translationToWorld.x);
      y.set(translationToWorld.y);
      z.set(translationToWorld.z);

      orientation.setIncludingFrame(ReferenceFrame.getWorldFrame(), transformToWorld);
      setOrientation(orientation);
   }

   public double getScale()
   {
      return scale;
   }

   public void getPosition(Vector3d position)
   {
      position.x = x.getDoubleValue();
      position.y = y.getDoubleValue();
      position.z = z.getDoubleValue();
   }

   public void getYawPitchRoll(Vector3d yawPitchRoll)
   {
      yawPitchRoll.x = yaw.getDoubleValue();
      yawPitchRoll.y = pitch.getDoubleValue();
      yawPitchRoll.z = roll.getDoubleValue();
   }

   public void setPosition(double x, double y, double z)
   {
      this.x.set(x);
      this.y.set(y);
      this.z.set(z);
   }

   public void setPosition(FramePoint position)
   {
      x.set(position.getX());
      y.set(position.getY());
      z.set(position.getZ());
   }

   public void setOrientation(FrameOrientation orientation)
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

   public void setPose(FramePose pose)
   {
      pose.getOrientation(tempYawPitchRoll);
      setYawPitchRoll(tempYawPitchRoll);

      x.set(pose.getX());
      y.set(pose.getY());
      z.set(pose.getZ());
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

      linkGraphics.addCoordinateSystem(scale, arrowColor);

      return linkGraphics;
   }

   protected Vector3d translationVector = new Vector3d();

   @Override
   protected void computeRotationTranslation(Transform3d transform3D)
   {
      transform3D.setIdentity();
      translationVector.set(x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
      transform3D.setScale(scale);
      transform3D.setEuler(roll.getDoubleValue(), pitch.getDoubleValue(), yaw.getDoubleValue());
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

   public DoubleYoVariable[] getVariables()
   {
      return new DoubleYoVariable[] { x, y, z, yaw, pitch, roll };
   }

   public double[] getConstants()
   {
      return new double[] { scale };
   }

   public AppearanceDefinition getAppearance()
   {
      return arrowColor;
   }
}
