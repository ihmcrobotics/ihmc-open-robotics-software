package us.ihmc.avatar.scs2;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicText;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicText3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicTriangle;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVRML;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLine2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.yoComposite.YoOrientation3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoQuaternionDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoTuple3DDefinition;
import us.ihmc.scs2.definition.yoComposite.YoYawPitchRollDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicArrow3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicCoordinateSystem3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPoint3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolygonExtruded3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicPolynomial3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoListDefinition;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoTuple2D;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoGraphicDefinitionTools
{
   //////////////////////////////////////////////////////////////////////////////////
   /////////////////// Redirections to SCS1GraphicConversionTools ///////////////////
   //////////////////////////////////////////////////////////////////////////////////

   public static List<YoGraphicDefinition> toYoGraphicDefinitions(YoGraphicsListRegistry registry)
   {
      return YoGraphicConversionTools.toYoGraphicDefinitions(registry);
   }

   public static YoGraphicGroupDefinition toYoGraphicGroupDefinition(YoGraphicsList yoGraphicsList)
   {
      return YoGraphicConversionTools.toYoGraphicGroupDefinition(yoGraphicsList);
   }

   public static YoGraphicGroupDefinition toYoGraphicGroupDefinition(ArtifactList artifactList)
   {
      return YoGraphicConversionTools.toYoGraphicGroupDefinition(artifactList);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphic yoGraphic)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphic);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicPolygon yoGraphicPolygon)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicPolygon);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicShape yoGraphicShape)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicShape);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicText yoGraphicText)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicText);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicText3D yoGraphicText3D)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicText3D);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicCoordinateSystem yoGraphicCoordinateSystem)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicCoordinateSystem);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicEllipsoid yoGraphicEllipsoid)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicEllipsoid);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicReferenceFrame yoGraphicReferenceFrame)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicReferenceFrame);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicVRML yoGraphicVRML)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicVRML);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicCylinder yoGraphicCylinder)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicCylinder);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicPolygon3D yoGraphicPolygon3D)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicPolygon3D);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicPolynomial3D yoGraphicPolynomial3D)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicPolynomial3D);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicPosition yoGraphicPosition)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicPosition);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicTriangle yoGraphicTriangle)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicTriangle);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicVector yoGraphicVector)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicVector);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoGraphicLineSegment yoGraphicLineSegment)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoGraphicLineSegment);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(Artifact artifact)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(artifact);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactLine2d yoArtifactLine2d)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoArtifactLine2d);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactLineSegment2d yoArtifactLineSegment2d)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoArtifactLineSegment2d);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactOval yoArtifactOval)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoArtifactOval);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactPolygon yoArtifactPolygon)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoArtifactPolygon);
   }

   public static YoGraphicDefinition toYoGraphicDefinition(YoArtifactPosition yoArtifactPosition)
   {
      return YoGraphicConversionTools.toYoGraphicDefinition(yoArtifactPosition);
   }

   public static ColorDefinition toColorDefinition(AppearanceDefinition appearanceDefinition)
   {
      return YoGraphicConversionTools.toColorDefinition(appearanceDefinition);
   }

   //////////////////////////////////////////////////////////////////////////////////
   /////////////////// Redirections to YoGraphicDefinitionFactory ///////////////////
   //////////////////////////////////////////////////////////////////////////////////

   public static YoGraphicArrow3DDefinition newYoGraphicArrow3D(String name,
                                                                YoFrameTuple3D origin,
                                                                YoFrameTuple3D direction,
                                                                double scale,
                                                                ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicArrow3D(name, origin, direction, scale, color);
   }

   public static YoGraphicArrow3DDefinition newYoGraphicArrow3D(String name,
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
      return YoGraphicDefinitionFactory.newYoGraphicArrow3D(name,
                                                            origin,
                                                            direction,
                                                            scaleLength,
                                                            bodyLength,
                                                            headLength,
                                                            scaleRadius,
                                                            bodyRadius,
                                                            headRadius,
                                                            color);
   }

   public static YoGraphicArrow3DDefinition newYoGraphicArrow3D(String name,
                                                                YoTuple3D origin,
                                                                ReferenceFrame originFrame,
                                                                YoTuple3D direction,
                                                                ReferenceFrame directionFrame,
                                                                double scale,
                                                                ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicArrow3D(name, origin, originFrame, direction, directionFrame, scale, color);
   }

   public static YoGraphicArrow3DDefinition newYoGraphicArrow3D(String name,
                                                                YoTuple3D origin,
                                                                ReferenceFrame originFrame,
                                                                YoTuple3D direction,
                                                                ReferenceFrame directionFrame,
                                                                boolean scaleLength,
                                                                double bodyLength,
                                                                double headLength,
                                                                boolean scaleRadius,
                                                                double bodyRadius,
                                                                double headRadius,
                                                                ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicArrow3D(name,
                                                            origin,
                                                            originFrame,
                                                            direction,
                                                            directionFrame,
                                                            scaleLength,
                                                            bodyLength,
                                                            headLength,
                                                            scaleRadius,
                                                            bodyRadius,
                                                            headRadius,
                                                            color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name, YoFramePose3D pose, double scale, ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name, pose, scale, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      YoFramePoint3D position,
                                                                                      YoFrameQuaternion orientation,
                                                                                      double scale,
                                                                                      ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name, position, orientation, scale, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      YoFramePoint3D position,
                                                                                      YoFrameQuaternion orientation,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name, position, orientation, bodyLength, headLength, bodyRadius, headRadius, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      FramePose3DReadOnly constantFramePose,
                                                                                      double scale,
                                                                                      ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name, constantFramePose, scale, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      FramePose3DReadOnly constantFramePose,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name, constantFramePose, bodyLength, headLength, bodyRadius, headRadius, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      Pose3DReadOnly constantFramePose,
                                                                                      double scale,
                                                                                      ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name, constantFramePose, scale, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      Pose3DReadOnly constantPose,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name, constantPose, bodyLength, headLength, bodyRadius, headRadius, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      Pose3DReadOnly constantPose,
                                                                                      ReferenceFrame poseFrame,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name, constantPose, bodyLength, headLength, bodyRadius, headRadius, color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
                                                                                      Point3DReadOnly constantPoint,
                                                                                      Orientation3DReadOnly constantOrientation,
                                                                                      ReferenceFrame poseFrame,
                                                                                      double bodyLength,
                                                                                      double headLength,
                                                                                      double bodyRadius,
                                                                                      double headRadius,
                                                                                      ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name,
                                                                       constantPoint,
                                                                       constantOrientation,
                                                                       poseFrame,
                                                                       bodyLength,
                                                                       headLength,
                                                                       bodyRadius,
                                                                       headRadius,
                                                                       color);
   }

   public static YoGraphicCoordinateSystem3DDefinition newYoGraphicCoordinateSystem3D(String name,
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
      return YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(name,
                                                                       constantPoint,
                                                                       pointFrame,
                                                                       constantOrientation,
                                                                       orientationFrame,
                                                                       bodyLength,
                                                                       headLength,
                                                                       bodyRadius,
                                                                       headRadius,
                                                                       color);
   }

   public static YoGraphicPoint3DDefinition newYoGraphicPoint3D(String name, YoFrameTuple3D position, double size, ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPoint3D(name, position, size, color);
   }

   public static YoGraphicPoint3DDefinition newYoGraphicPoint3D(String name,
                                                                YoTuple3D position,
                                                                ReferenceFrame positionFrame,
                                                                double size,
                                                                ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPoint3D(name, position, positionFrame, size, color);
   }

   public static YoGraphicPoint2DDefinition newYoGraphicPoint2D(String name,
                                                                YoFrameTuple2D position,
                                                                double size,
                                                                ColorDefinition strokeColor,
                                                                DefaultPoint2DGraphic graphicType)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPoint2D(name, position, size, strokeColor, graphicType);
   }

   public static YoGraphicPoint2DDefinition newYoGraphicPoint2D(String name,
                                                                YoTuple2D position,
                                                                ReferenceFrame positionFrame,
                                                                double size,
                                                                ColorDefinition strokeColor,
                                                                DefaultPoint2DGraphic graphicType)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPoint2D(name, position, positionFrame, size, strokeColor, graphicType);
   }

   public static YoGraphicPolygonExtruded3DDefinition newYoGraphicPolygonExtruded3DDefinition(String name,
                                                                                              YoFramePose3D pose,
                                                                                              YoFrameConvexPolygon2D polygon,
                                                                                              double thickness,
                                                                                              ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPolygonExtruded3DDefinition(name, pose, polygon, thickness, color);
   }

   public static YoGraphicPolygonExtruded3DDefinition newYoGraphicPolygonExtruded3DDefinition(String name,
                                                                                              YoFramePose3D pose,
                                                                                              List<? extends Point2DReadOnly> vertices,
                                                                                              double thickness,
                                                                                              ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPolygonExtruded3DDefinition(name, pose, vertices, thickness, color);
   }

   public static YoGraphicPolynomial3DDefinition newYoGraphicPolynomial3D(String name,
                                                                          YoVariable[] coefficientsX,
                                                                          YoInteger numberOfCoefficientsX,
                                                                          YoVariable[] coefficientsY,
                                                                          YoInteger numberOfCoefficientsY,
                                                                          YoVariable[] coefficientsZ,
                                                                          YoInteger numberOfCoefficientsZ,
                                                                          YoDouble startTime,
                                                                          YoDouble endTime,
                                                                          double size,
                                                                          ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPolynomial3D(name,
                                                                 coefficientsX,
                                                                 numberOfCoefficientsX,
                                                                 coefficientsY,
                                                                 numberOfCoefficientsY,
                                                                 coefficientsZ,
                                                                 numberOfCoefficientsZ,
                                                                 startTime,
                                                                 endTime,
                                                                 size,
                                                                 color);
   }

   public static YoGraphicPolynomial3DDefinition newYoGraphicPolynomial3D(String name,
                                                                          YoVariable[] coefficientsX,
                                                                          YoInteger numberOfCoefficientsX,
                                                                          YoVariable[] coefficientsY,
                                                                          YoInteger numberOfCoefficientsY,
                                                                          YoVariable[] coefficientsZ,
                                                                          YoInteger numberOfCoefficientsZ,
                                                                          YoDouble startTime,
                                                                          YoDouble endTime,
                                                                          double size,
                                                                          int timeResolution,
                                                                          int numberOfDivisions,
                                                                          ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPolynomial3D(name,
                                                                 coefficientsX,
                                                                 numberOfCoefficientsX,
                                                                 coefficientsY,
                                                                 numberOfCoefficientsY,
                                                                 coefficientsZ,
                                                                 numberOfCoefficientsZ,
                                                                 startTime,
                                                                 endTime,
                                                                 size,
                                                                 timeResolution,
                                                                 numberOfDivisions,
                                                                 color);
   }

   public static YoGraphicPolynomial3DDefinition newYoGraphicPolynomial3D(String name,
                                                                          YoListDefinition coefficientsX,
                                                                          YoListDefinition coefficientsY,
                                                                          YoListDefinition coefficientsZ,
                                                                          YoDouble startTime,
                                                                          double defaultStartTime,
                                                                          YoDouble endTime,
                                                                          double defaultEndTime,
                                                                          double size,
                                                                          ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPolynomial3D(name,
                                                                 coefficientsX,
                                                                 coefficientsY,
                                                                 coefficientsZ,
                                                                 startTime,
                                                                 defaultStartTime,
                                                                 endTime,
                                                                 defaultEndTime,
                                                                 size,
                                                                 color);
   }

   public static YoGraphicPolynomial3DDefinition newYoGraphicPolynomial3D(String name,
                                                                          YoListDefinition coefficientsX,
                                                                          YoListDefinition coefficientsY,
                                                                          YoListDefinition coefficientsZ,
                                                                          YoDouble startTime,
                                                                          double defaultStartTime,
                                                                          YoDouble endTime,
                                                                          double defaultEndTime,
                                                                          double size,
                                                                          int timeResolution,
                                                                          int numberOfDivisions,
                                                                          ColorDefinition color)
   {
      return YoGraphicDefinitionFactory.newYoGraphicPolynomial3D(name,
                                                                 coefficientsX,
                                                                 coefficientsY,
                                                                 coefficientsZ,
                                                                 startTime,
                                                                 defaultStartTime,
                                                                 endTime,
                                                                 defaultEndTime,
                                                                 size,
                                                                 timeResolution,
                                                                 numberOfDivisions,
                                                                 color);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(Tuple2DReadOnly tuple2D)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(tuple2D);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(Tuple2DReadOnly tuple2D, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(tuple2D, frame);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(FrameTuple2DReadOnly frameTuple2D)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(frameTuple2D);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoTuple2D tuple2D)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(tuple2D);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoTuple2D tuple2D, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(tuple2D, frame);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoFrameTuple2D frameTuple2D)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(frameTuple2D);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoVariable[] yoVariables, int startIndex)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(yoVariables, startIndex);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoVariable yoX, YoVariable yoY)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(yoX, yoY);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoDouble yoX, YoDouble yoY, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(yoX, yoY, frame);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(double x, double y, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(x, y, frame);
   }

   public static YoTuple2DDefinition newYoTuple2DDefinition(YoDouble yoX, double defaultX, YoDouble yoY, double defaultY, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple2DDefinition(yoX, defaultX, yoY, defaultY, frame);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoTuple3D tuple3D)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(tuple3D);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoTuple3D tuple3D, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(tuple3D, frame);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoFrameTuple3D frameTuple3D)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(frameTuple3D);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(Tuple3DReadOnly tuple3D)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(tuple3D);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(Tuple3DReadOnly tuple3D, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(tuple3D, frame);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(FrameTuple3DReadOnly frameTuple3D)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(frameTuple3D);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoVariable[] yoVariables, int startIndex)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(yoVariables, startIndex);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoVariable yoX, YoVariable yoY, YoVariable yoZ)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(yoX, yoY, yoZ);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoDouble yoX, YoDouble yoY, YoDouble yoZ, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(yoX, yoY, yoZ, frame);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(double x, double y, double z, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(x, y, z, frame);
   }

   public static YoTuple3DDefinition newYoTuple3DDefinition(YoDouble yoX,
                                                            double defaultX,
                                                            YoDouble yoY,
                                                            double defaultY,
                                                            YoDouble yoZ,
                                                            double defaultZ,
                                                            ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoTuple3DDefinition(yoX, defaultX, yoY, defaultY, yoZ, defaultZ, frame);
   }

   public static YoOrientation3DDefinition newYoOrientation3DDefinition(Orientation3DReadOnly orientation3D)
   {
      return YoGraphicDefinitionFactory.newYoOrientation3DDefinition(orientation3D);
   }

   public static YoOrientation3DDefinition newYoOrientation3DDefinition(Orientation3DReadOnly orientation3D, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoOrientation3DDefinition(orientation3D, frame);
   }

   public static YoOrientation3DDefinition newYoOrientation3DDefinition(FrameOrientation3DReadOnly frameOrientation3D)
   {
      return YoGraphicDefinitionFactory.newYoOrientation3DDefinition(frameOrientation3D);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YawPitchRollReadOnly yawPitchRoll)
   {
      return YoGraphicDefinitionFactory.newYoYawPitchRollDefinition(yawPitchRoll);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YawPitchRollReadOnly yawPitchRoll, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoYawPitchRollDefinition(yawPitchRoll, frame);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(FrameYawPitchRollReadOnly frameYawPitchRoll)
   {
      return YoGraphicDefinitionFactory.newYoYawPitchRollDefinition(frameYawPitchRoll);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoFrameYawPitchRoll frameYawPitchRoll)
   {
      return YoGraphicDefinitionFactory.newYoYawPitchRollDefinition(frameYawPitchRoll);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoVariable[] yoVariables, int startIndex)
   {
      return YoGraphicDefinitionFactory.newYoYawPitchRollDefinition(yoVariables, startIndex);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoVariable yoYaw, YoVariable yoPitch, YoVariable yoRoll)
   {
      return YoGraphicDefinitionFactory.newYoYawPitchRollDefinition(yoYaw, yoPitch, yoRoll);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoDouble yoYaw, YoDouble yoPitch, YoDouble yoRoll, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoYawPitchRollDefinition(yoYaw, yoPitch, yoRoll, frame);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(double yaw, double pitch, double roll, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoYawPitchRollDefinition(yaw, pitch, roll, frame);
   }

   public static YoYawPitchRollDefinition newYoYawPitchRollDefinition(YoDouble yoYaw,
                                                                      double defaultYaw,
                                                                      YoDouble yoPitch,
                                                                      double defaultPitch,
                                                                      YoDouble yoRoll,
                                                                      double defaultRoll,
                                                                      ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoYawPitchRollDefinition(yoYaw, defaultYaw, yoPitch, defaultPitch, yoRoll, defaultRoll, frame);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(QuaternionReadOnly quaternion)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(quaternion);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(QuaternionReadOnly quaternion, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(quaternion, frame);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(FrameQuaternionReadOnly frameQuaternion)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(frameQuaternion);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoQuaternion quaternion)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(quaternion);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoQuaternion quaternion, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(quaternion, frame);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoFrameQuaternion frameQuaternion)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(frameQuaternion);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoVariable[] yoVariables, int startIndex)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(yoVariables, startIndex);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoVariable yoX, YoVariable yoY, YoVariable yoZ, YoVariable yoS)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(yoX, yoY, yoZ, yoS);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(YoDouble yoX, YoDouble yoY, YoDouble yoZ, YoDouble yoS, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(yoX, yoY, yoZ, yoS, frame);
   }

   public static YoQuaternionDefinition newYoQuaternionDefinition(double x, double y, double z, double s, ReferenceFrame frame)
   {
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(x, y, z, s, frame);
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
      return YoGraphicDefinitionFactory.newYoQuaternionDefinition(yoX, defaultX, yoY, defaultY, yoZ, defaultZ, yoS, defaultS, frame);
   }

   public static YoListDefinition toYoListDefinition(YoVariable[] yoVariables, YoInteger size)
   {
      return YoGraphicDefinitionFactory.toYoListDefinition(yoVariables, size);
   }

   public static YoListDefinition toYoListDefinition(YoVariable[] yoVariables, double[] defaultValues, YoInteger size)
   {
      return YoGraphicDefinitionFactory.toYoListDefinition(yoVariables, defaultValues, size);
   }

   public static List<String> toPropertyNames(YoVariable[] yoVariables)
   {
      return YoGraphicDefinitionFactory.toPropertyNames(yoVariables);
   }

   public static List<String> toPropertyNames(YoVariable[] yoVariables, double[] defaultValues)
   {
      return YoGraphicDefinitionFactory.toPropertyNames(yoVariables, defaultValues);
   }

   public static String toPropertyName(ReferenceFrame referenceFrame)
   {
      return YoGraphicDefinitionFactory.toPropertyName(referenceFrame);
   }

   public static String toPropertyName(YoVariable yoVariable)
   {
      return YoGraphicDefinitionFactory.toPropertyName(yoVariable);
   }

   public static String toPropertyName(YoVariable yoVariable, double defaultValue)
   {
      return YoGraphicDefinitionFactory.toPropertyName(yoVariable, defaultValue);
   }
}
