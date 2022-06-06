package us.ihmc.avatar.scs2;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoYawPitchRollDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicArrow3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCoordinateSystem3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint3DDefinition;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoTuple2D;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoGraphicDefinitionFactory
{
   public static YoGraphicArrow3DDefinition newYoGraphicArrow3DDefinition(String name,
                                                                          YoFrameTuple3D origin,
                                                                          YoFrameTuple3D direction,
                                                                          double scale,
                                                                          ColorDefinition color)
   {
      boolean scaleLength = true;
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      boolean scaleRadius = true;
      double bodyRadius = scale * 0.015;
      double headRadius = bodyRadius * 2.5;
      return newYoGraphicArrow3DDefinition(name, origin, direction, scaleLength, bodyLength, headLength, scaleRadius, bodyRadius, headRadius, color);
   }

   public static YoGraphicArrow3DDefinition newYoGraphicArrow3DDefinition(String name,
                                                                          YoFrameTuple3D origin,
                                                                          YoFrameTuple3D direction,
                                                                          boolean scaleLength,
                                                                          double bodyLength,
                                                                          double headLength,
                                                                          boolean scaleRadius,
                                                                          double bodyRadius,
                                                                          double headRadius,
                                                                          ColorDefinition color)
   {
      YoGraphicArrow3DDefinition definition = new YoGraphicArrow3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setOrigin(newYoTuple3DDefinition(origin));
      definition.setDirection(newYoTuple3DDefinition(direction));
      definition.setScaleLength(scaleLength);
      definition.setBodyLength(bodyLength);
      definition.setHeadLength(headLength);
      definition.setScaleRadius(scaleRadius);
      definition.setBodyRadius(bodyRadius);
      definition.setHeadRadius(headRadius);
      definition.setColor(color);
      return definition;
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                YoFramePose3D pose,
                                                                                                double scale,
                                                                                                ColorDefinition color)
   {
      return newYoGraphicCoordinateSystem3DDefinition(name, pose.getPosition(), pose.getOrientation(), scale, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                YoFramePoint3D position,
                                                                                                YoFrameQuaternion orientation,
                                                                                                double scale,
                                                                                                ColorDefinition color)
   {
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      double bodyRadius = scale * 0.02;
      double headRadius = bodyRadius * 2.0;
      return newYoGraphicCoordinateSystem3DDefinition(name, position, orientation, bodyLength, headLength, bodyRadius, headRadius, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                YoFramePoint3D position,
                                                                                                YoFrameQuaternion orientation,
                                                                                                double bodyLength,
                                                                                                double headLength,
                                                                                                double bodyRadius,
                                                                                                double headRadius,
                                                                                                ColorDefinition color)
   {
      YoGraphicCoordinateSystem3DDefinition definition = new YoGraphicCoordinateSystem3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(position));
      definition.setOrientation(newYoQuaternionDefinition(orientation));
      definition.setBodyLength(bodyLength);
      definition.setHeadLength(headLength);
      definition.setBodyRadius(bodyRadius);
      definition.setHeadRadius(headRadius);
      definition.setColor(color);
      return definition;
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                FramePose3DReadOnly constantFramePose,
                                                                                                double scale,
                                                                                                ColorDefinition color)
   {
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      double bodyRadius = scale * 0.02;
      double headRadius = bodyRadius * 2.0;
      return newYoGraphicCoordinateSystem3DDefinition(name,
                                                      constantFramePose.getPosition(),
                                                      constantFramePose.getOrientation(),
                                                      constantFramePose.getReferenceFrame(),
                                                      bodyLength,
                                                      headLength,
                                                      bodyRadius,
                                                      headRadius,
                                                      color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                FramePose3DReadOnly constantFramePose,
                                                                                                double bodyLength,
                                                                                                double headLength,
                                                                                                double bodyRadius,
                                                                                                double headRadius,
                                                                                                ColorDefinition color)
   {
      return newYoGraphicCoordinateSystem3DDefinition(name,
                                                      constantFramePose.getPosition(),
                                                      constantFramePose.getOrientation(),
                                                      constantFramePose.getReferenceFrame(),
                                                      bodyLength,
                                                      headLength,
                                                      bodyRadius,
                                                      headRadius,
                                                      color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                Pose3DReadOnly constantFramePose,
                                                                                                double scale,
                                                                                                ColorDefinition color)
   {
      double bodyLength = scale * 0.9;
      double headLength = scale * 0.1;
      double bodyRadius = scale * 0.02;
      double headRadius = bodyRadius * 2.0;
      return newYoGraphicCoordinateSystem3DDefinition(name,
                                                      constantFramePose.getPosition(),
                                                      constantFramePose.getOrientation(),
                                                      null,
                                                      bodyLength,
                                                      headLength,
                                                      bodyRadius,
                                                      headRadius,
                                                      color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                Pose3DReadOnly constantPose,
                                                                                                double bodyLength,
                                                                                                double headLength,
                                                                                                double bodyRadius,
                                                                                                double headRadius,
                                                                                                ColorDefinition color)
   {
      return newYoGraphicCoordinateSystem3DDefinition(name,
                                                      constantPose.getPosition(),
                                                      null,
                                                      constantPose.getOrientation(),
                                                      null,
                                                      bodyLength,
                                                      headLength,
                                                      bodyRadius,
                                                      headRadius,
                                                      color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                Pose3DReadOnly constantPose,
                                                                                                ReferenceFrame poseFrame,
                                                                                                double bodyLength,
                                                                                                double headLength,
                                                                                                double bodyRadius,
                                                                                                double headRadius,
                                                                                                ColorDefinition color)
   {
      return newYoGraphicCoordinateSystem3DDefinition(name,
                                                      constantPose.getPosition(),
                                                      poseFrame,
                                                      constantPose.getOrientation(),
                                                      poseFrame,
                                                      bodyLength,
                                                      headLength,
                                                      bodyRadius,
                                                      headRadius,
                                                      color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                Point3DReadOnly constantPoint,
                                                                                                Orientation3DReadOnly constantOrientation,
                                                                                                ReferenceFrame poseFrame,
                                                                                                double bodyLength,
                                                                                                double headLength,
                                                                                                double bodyRadius,
                                                                                                double headRadius,
                                                                                                ColorDefinition color)
   {
      return newYoGraphicCoordinateSystem3DDefinition(name,
                                                      constantPoint,
                                                      poseFrame,
                                                      constantOrientation,
                                                      poseFrame,
                                                      bodyLength,
                                                      headLength,
                                                      bodyRadius,
                                                      headRadius,
                                                      color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3DDefinition(String name,
                                                                                                Point3DReadOnly constantPoint,
                                                                                                ReferenceFrame pointFrame,
                                                                                                Orientation3DReadOnly constantOrientation,
                                                                                                ReferenceFrame orientationFrame,
                                                                                                double bodyLength,
                                                                                                double headLength,
                                                                                                double bodyRadius,
                                                                                                double headRadius,
                                                                                                ColorDefinition color)
   {
      YoGraphicCoordinateSystem3DDefinition definition = new YoGraphicCoordinateSystem3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(constantPoint, pointFrame));
      definition.setOrientation(newYoOrientation3DDefinition(constantOrientation, orientationFrame));
      definition.setBodyLength(bodyLength);
      definition.setHeadLength(headLength);
      definition.setBodyRadius(bodyRadius);
      definition.setHeadRadius(headRadius);
      definition.setColor(color);
      return definition;
   }

   public static YoGraphicPoint3DDefinition newYoGraphicPoint3DDefinition(String name, YoFrameTuple3D position, double size, ColorDefinition color)
   {
      YoGraphicPoint3DDefinition definition = new YoGraphicPoint3DDefinition();
      definition.setName(name);
      definition.setVisible(true);
      definition.setPosition(newYoTuple3DDefinition(position));
      definition.setSize(size);
      definition.setColor(color);
      return definition;
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(Tuple2DReadOnly tuple2D)
   {
      return newYoTuple2DDefinition(tuple2D, null);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(Tuple2DReadOnly tuple2D, ReferenceFrame frame)
   {
      return newYoTuple2DDefinition(tuple2D == null ? 0 : tuple2D.getX(), tuple2D == null ? 0 : tuple2D.getY(), frame);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(FrameTuple2DReadOnly frameTuple2D)
   {
      return newYoTuple2DDefinition(frameTuple2D, frameTuple2D == null ? null : frameTuple2D.getReferenceFrame());
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoTuple2D tuple2D)
   {
      return newYoTuple2DDefinition(tuple2D, null);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoTuple2D tuple2D, ReferenceFrame frame)
   {
      return newYoTuple2DDefinition(tuple2D == null ? null : tuple2D.getYoX(), tuple2D == null ? null : tuple2D.getYoY(), frame);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoFrameTuple2D frameTuple2D)
   {
      return newYoTuple2DDefinition(frameTuple2D, frameTuple2D == null ? null : frameTuple2D.getReferenceFrame());
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoDouble yoX, YoDouble yoY, ReferenceFrame frame)
   {
      return newYoTuple2DDefinition(yoX, 0, yoY, 0, frame);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(double x, double y, ReferenceFrame frame)
   {
      return newYoTuple2DDefinition(null, x, null, y, frame);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoDouble yoX, double defaultX, YoDouble yoY, double defaultY, ReferenceFrame frame)
   {
      YoTuple2DDefinition definition = new YoTuple2DDefinition();
      definition.setX(toPropertyName(yoX, defaultX));
      definition.setY(toPropertyName(yoY, defaultY));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoTuple3D tuple3D)
   {
      return newYoTuple3DDefinition(tuple3D, null);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoTuple3D tuple3D, ReferenceFrame frame)
   {
      return newYoTuple3DDefinition(tuple3D == null ? null : tuple3D.getYoX(),
                                    tuple3D == null ? null : tuple3D.getYoY(),
                                    tuple3D == null ? null : tuple3D.getYoZ(),
                                    frame);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoFrameTuple3D frameTuple3D)
   {
      return newYoTuple3DDefinition(frameTuple3D, frameTuple3D == null ? null : frameTuple3D.getReferenceFrame());
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(Tuple3DReadOnly tuple3D)
   {
      return newYoTuple3DDefinition(tuple3D, null);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(Tuple3DReadOnly tuple3D, ReferenceFrame frame)
   {
      return newYoTuple3DDefinition(tuple3D == null ? 0 : tuple3D.getX(), tuple3D == null ? 0 : tuple3D.getY(), tuple3D == null ? 0 : tuple3D.getZ(), frame);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(FrameTuple3DReadOnly frameTuple3D)
   {
      return newYoTuple3DDefinition(frameTuple3D, frameTuple3D == null ? null : frameTuple3D.getReferenceFrame());
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoDouble yoX, YoDouble yoY, YoDouble yoZ, ReferenceFrame frame)
   {
      return newYoTuple3DDefinition(yoX, 0.0, yoY, 0.0, yoZ, 0.0, frame);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(double x, double y, double z, ReferenceFrame frame)
   {
      return newYoTuple3DDefinition(null, x, null, y, null, z, frame);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoDouble yoX,
                                                            double defaultX,
                                                            YoDouble yoY,
                                                            double defaultY,
                                                            YoDouble yoZ,
                                                            double defaultZ,
                                                            ReferenceFrame frame)
   {
      YoTuple3DDefinition definition = new YoTuple3DDefinition();
      definition.setX(toPropertyName(yoX, defaultX));
      definition.setY(toPropertyName(yoY, defaultY));
      definition.setZ(toPropertyName(yoZ, defaultZ));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   public static YoOrientation3DDefinition newYoOrientation3DDefinition(Orientation3DReadOnly orientation3D)
   {
      return newYoOrientation3DDefinition(orientation3D, null);
   }

   public static YoOrientation3DDefinition newYoOrientation3DDefinition(Orientation3DReadOnly orientation3D, ReferenceFrame frame)
   {
      if (orientation3D == null)
         return newYoYawPitchRollDefinition(null, frame);
      if (orientation3D instanceof QuaternionReadOnly)
         return newYoQuaternionDefinition((QuaternionReadOnly) orientation3D, frame);
      if (orientation3D instanceof YawPitchRollReadOnly)
         return newYoYawPitchRollDefinition((YawPitchRollReadOnly) orientation3D, frame);
      throw new UnsupportedOperationException("Orientation type [" + orientation3D.getClass().getSimpleName() + "] is not supported yet.");
   }

   public static YoOrientation3DDefinition newYoOrientation3DDefinition(FrameOrientation3DReadOnly frameOrientation3D)
   {
      return newYoOrientation3DDefinition(frameOrientation3D, frameOrientation3D == null ? null : frameOrientation3D.getReferenceFrame());
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YawPitchRollReadOnly yawPitchRoll)
   {
      return newYoYawPitchRollDefinition(yawPitchRoll, null);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YawPitchRollReadOnly yawPitchRoll, ReferenceFrame frame)
   {
      return newYoYawPitchRollDefinition(yawPitchRoll == null ? 0 : yawPitchRoll.getYaw(),
                                         yawPitchRoll == null ? 0 : yawPitchRoll.getPitch(),
                                         yawPitchRoll == null ? 0 : yawPitchRoll.getRoll(),
                                         frame);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(FrameYawPitchRollReadOnly frameYawPitchRoll)
   {
      return newYoYawPitchRollDefinition(frameYawPitchRoll, frameYawPitchRoll == null ? null : frameYawPitchRoll.getReferenceFrame());
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoFrameYawPitchRoll frameYawPitchRoll)
   {
      return newYoYawPitchRollDefinition(frameYawPitchRoll == null ? null : frameYawPitchRoll.getYoYaw(),
                                         frameYawPitchRoll == null ? null : frameYawPitchRoll.getYoPitch(),
                                         frameYawPitchRoll == null ? null : frameYawPitchRoll.getYoRoll(),
                                         frameYawPitchRoll == null ? null : frameYawPitchRoll.getReferenceFrame());
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoDouble yoYaw, YoDouble yoPitch, YoDouble yoRoll, ReferenceFrame frame)
   {
      return newYoYawPitchRollDefinition(yoYaw, 0.0, yoPitch, 0.0, yoRoll, 0.0, frame);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(double yaw, double pitch, double roll, ReferenceFrame frame)
   {
      return newYoYawPitchRollDefinition(null, yaw, null, pitch, null, roll, frame);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoDouble yoYaw,
                                                                      double defaultYaw,
                                                                      YoDouble yoPitch,
                                                                      double defaultPitch,
                                                                      YoDouble yoRoll,
                                                                      double defaultRoll,
                                                                      ReferenceFrame frame)
   {
      YoYawPitchRollDefinition definition = new YoYawPitchRollDefinition();
      definition.setYaw(toPropertyName(yoYaw, defaultYaw));
      definition.setPitch(toPropertyName(yoPitch, defaultPitch));
      definition.setRoll(toPropertyName(yoRoll, defaultRoll));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(QuaternionReadOnly quaternion)
   {
      return newYoQuaternionDefinition(quaternion, null);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(QuaternionReadOnly quaternion, ReferenceFrame frame)
   {
      return newYoQuaternionDefinition(quaternion == null ? null : quaternion.getX(),
                                       quaternion == null ? null : quaternion.getY(),
                                       quaternion == null ? null : quaternion.getZ(),
                                       quaternion == null ? null : quaternion.getS(),
                                       frame);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(FrameQuaternionReadOnly frameQuaternion)
   {
      return newYoQuaternionDefinition(frameQuaternion, frameQuaternion == null ? null : frameQuaternion.getReferenceFrame());
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoQuaternion quaternion)
   {
      return newYoQuaternionDefinition(quaternion, null);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoQuaternion quaternion, ReferenceFrame frame)
   {
      return newYoQuaternionDefinition(quaternion == null ? null : quaternion.getYoQx(),
                                       quaternion == null ? null : quaternion.getYoQy(),
                                       quaternion == null ? null : quaternion.getYoQz(),
                                       quaternion == null ? null : quaternion.getYoQs(),
                                       frame);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoFrameQuaternion frameQuaternion)
   {
      return newYoQuaternionDefinition(frameQuaternion, frameQuaternion == null ? null : frameQuaternion.getReferenceFrame());
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoDouble yoX, YoDouble yoY, YoDouble yoZ, YoDouble yoS, ReferenceFrame frame)
   {
      return newYoQuaternionDefinition(yoX, 0, yoY, 0, yoZ, 0, yoS, 0, frame);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(double x, double y, double z, double s, ReferenceFrame frame)
   {
      return newYoQuaternionDefinition(null, x, null, y, null, z, null, s, frame);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoDouble yoX,
                                                                  double defaultX,
                                                                  YoDouble yoY,
                                                                  double defaultY,
                                                                  YoDouble yoZ,
                                                                  double defaultZ,
                                                                  YoDouble yoS,
                                                                  double defaultS,
                                                                  ReferenceFrame frame)
   {
      YoQuaternionDefinition definition = new YoQuaternionDefinition();
      definition.setX(toPropertyName(yoX, defaultX));
      definition.setY(toPropertyName(yoY, defaultY));
      definition.setZ(toPropertyName(yoZ, defaultZ));
      definition.setS(toPropertyName(yoS, defaultS));
      definition.setReferenceFrame(toPropertyName(frame));
      return definition;
   }

   public static String toPropertyName(ReferenceFrame referenceFrame)
   {
      return referenceFrame == null ? null : referenceFrame.getNameId();
   }

   public static String toPropertyName(YoVariable yoVariable)
   {
      return yoVariable == null ? null : yoVariable.getFullNameString();
   }

   public static String toPropertyName(YoVariable yoVariable, double defaultValue)
   {
      return yoVariable == null ? Double.toString(defaultValue) : yoVariable.getFullNameString();
   }
}
