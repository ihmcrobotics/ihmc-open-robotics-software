package us.ihmc.graphicsDescription.yoGraphics;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class allows constructs RemoteYoGraphic objects from parsed data retrieved by the
 * YoVariableHandshakeClient.
 *
 * @author Alex Lesman
 *
 */
public class RemoteYoGraphicFactory
{
   public interface YoGraphicFromMessageBuilder<T extends RemoteYoGraphic>
   {
      T yoGraphicFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance);
   }

   private final List<Class<? extends RemoteYoGraphic>> classList = new ArrayList<>();
   private final TObjectIntMap<Class<? extends RemoteYoGraphic>> registrationIDs = new TObjectIntHashMap<>();
   private final TIntObjectMap<Class<? extends RemoteYoGraphic>> registrationClasses = new TIntObjectHashMap<>();
   private final TIntObjectMap<YoGraphicFromMessageBuilder<?>> registrationBuilders = new TIntObjectHashMap<>();

   public RemoteYoGraphicFactory()
   {
      registerBuilder(YoGraphicCylinder.class, (name, vars, consts, appearance) -> yoGraphicCylinderFromMessage(name, vars, consts, appearance));
      registerBuilder(YoGraphicTriangle.class, (name, vars, consts, appearance) -> yoGraphicTriangleFromMessage(name, vars, consts, appearance));
      registerBuilder(YoGraphicVector.class, (name, vars, consts, appearance) -> yoGraphicVectorFromMessage(name, vars, consts, appearance));
      registerBuilder(YoGraphicPosition.class, (name, vars, consts, appearance) -> yoGraphicPositionFromMessage(name, vars, consts, appearance));
      registerBuilder(YoGraphicCoordinateSystem.class,
                      (name, vars, consts, appearance) -> yoGraphicCoordinateSystemFromMessage(name, vars, consts, appearance));
      registerBuilder(YoGraphicLineSegment.class, (name, vars, consts, appearance) -> yoGraphicLineSegmentFromMessage(name, vars, consts, appearance));
      registerBuilder(YoGraphicPolygon.class, (name, vars, consts, appearance) -> yoGraphicPolygonFromMessage(name, vars, consts, appearance));
      registerBuilder(YoArtifactPosition.class, (name, vars, consts, appearance) -> yoArtifactPositionFromMessage(name, vars, consts, appearance));
      registerBuilder(YoArtifactOval.class, (name, vars, consts, appearance) -> yoArtifactOvalFromMessage(name, vars, consts, appearance));
      registerBuilder(YoArtifactLineSegment2d.class, (name, vars, consts, appearance) -> yoArtifactLineSegment2DFromMessage(name, vars, consts, appearance));
      registerBuilder(YoArtifactPolygon.class, (name, vars, consts, appearance) -> yoArtifactPolygonFromMessage(name, vars, consts, appearance));
      registerBuilder(YoGraphicReferenceFrame.class, (name, vars, consts, appearance) -> yoGraphicReferenceFrameFromMessage(name, vars, consts, appearance));
   }

   public <T extends RemoteYoGraphic> void registerBuilder(Class<T> clazz, YoGraphicFromMessageBuilder<T> builder)
   {
      int id = clazz.getName().hashCode();

      if (registrationIDs.containsKey(clazz))
         throw new RuntimeException("The class: " + clazz.getSimpleName() + " has already been registered.");

      classList.add(clazz);
      registrationIDs.put(clazz, id);
      registrationClasses.put(id, clazz);
      registrationBuilders.put(id, builder);
   }

   public int getRegistrationID(Class<? extends RemoteYoGraphic> clazz)
   {
      if (!registrationIDs.containsKey(clazz))
         throw new RuntimeException("The class: " + clazz.getSimpleName() + " is not registered.");
      return registrationIDs.get(clazz);
   }

   public RemoteYoGraphic yoGraphicFromMessage(int registrationID, String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      YoGraphicFromMessageBuilder<?> builder = registrationBuilders.get(registrationID);

      if (builder == null)
         throw new RuntimeException("Unhandled registrion ID: " + registrationID + ", yoGraphic name: " + name);

      return builder.yoGraphicFromMessage(name, vars, consts, appearance);
   }

   private static YoArtifactPolygon yoArtifactPolygonFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      int i = 0;
      YoInteger yoNumVertices = (YoInteger) vars[i++];
      ArrayList<YoFramePoint2D> yoFramePoints = new ArrayList<YoFramePoint2D>();
      while (i < vars.length)
      {
         yoFramePoints.add(new YoFramePoint2D((YoDouble) vars[i++], (YoDouble) vars[i++], referenceFrame));
      }

      YoFrameConvexPolygon2D convexPolygon2d = new YoFrameConvexPolygon2D(yoFramePoints, yoNumVertices, referenceFrame);

      return new YoArtifactPolygon(name, convexPolygon2d, appearance.getColor().get(), consts[0] > 0);
   }

   private static YoArtifactLineSegment2d yoArtifactLineSegment2DFromMessage(String name, YoVariable<?>[] vars, double[] consts,
                                                                             AppearanceDefinition appearance)
   {
      YoFrameLineSegment2D segment = new YoFrameLineSegment2D((YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], (YoDouble) vars[3],
                                                              ReferenceFrame.getWorldFrame());

      return new YoArtifactLineSegment2d(name, segment, appearance.getColor().get());
   }

   private static YoArtifactOval yoArtifactOvalFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      return new YoArtifactOval(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], appearance.getColor().get());
   }

   private static YoArtifactPosition yoArtifactPositionFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      return new YoArtifactPosition(name, (YoDouble) vars[0], (YoDouble) vars[1], YoGraphicPosition.GraphicType.values()[(int) (double) consts[1]],
                                    appearance.getColor().get(), consts[0]);
   }

   private static YoGraphicPolygon yoGraphicPolygonFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      int i = 0;
      YoInteger yoNumVertices = (YoInteger) vars[i++];
      ArrayList<YoFramePoint2D> yoFramePoints = new ArrayList<YoFramePoint2D>();
      while (i < vars.length - 6)
      {
         yoFramePoints.add(new YoFramePoint2D((YoDouble) vars[i++], (YoDouble) vars[i++], referenceFrame));
      }

      YoFrameConvexPolygon2D convexPolygon2d = new YoFrameConvexPolygon2D(yoFramePoints, yoNumVertices, referenceFrame);

      YoFramePoint3D framePoint = new YoFramePoint3D((YoDouble) vars[i++], (YoDouble) vars[i++], (YoDouble) vars[i++], referenceFrame);
      YoFrameYawPitchRoll frameOrientation = new YoFrameYawPitchRoll((YoDouble) vars[i++], (YoDouble) vars[i++], (YoDouble) vars[i++], referenceFrame);

      return new YoGraphicPolygon(name, convexPolygon2d, framePoint, frameOrientation, consts[0], appearance);
   }

   private static YoGraphicLineSegment yoGraphicLineSegmentFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      return new YoGraphicLineSegment(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], (YoDouble) vars[3], (YoDouble) vars[4],
                                      (YoDouble) vars[5], consts[0], appearance);
   }

   private static YoGraphicCoordinateSystem yoGraphicCoordinateSystemFromMessage(String name, YoVariable<?>[] vars, double[] consts,
                                                                                 AppearanceDefinition appearance)
   {
      return new YoGraphicCoordinateSystem(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], (YoDouble) vars[3], (YoDouble) vars[4],
                                           (YoDouble) vars[5], consts);
   }

   private static YoGraphicReferenceFrame yoGraphicReferenceFrameFromMessage(String name, YoVariable<?>[] vars, double[] consts,
                                                                             AppearanceDefinition appearance)
   {
      return new YoGraphicReferenceFrame(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], (YoDouble) vars[3], (YoDouble) vars[4],
                                         (YoDouble) vars[5], consts);
   }

   private static YoGraphicPosition yoGraphicPositionFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      return new YoGraphicPosition(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) getVariableOrNull(vars, 2), consts[0], appearance,
                                   YoGraphicPosition.GraphicType.values()[(int) (double) consts[1]]);
   }

   private static YoGraphicVector yoGraphicVectorFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      return new YoGraphicVector(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], (YoDouble) vars[3], (YoDouble) vars[4], (YoDouble) vars[5],
                                 consts[0], appearance, true);
   }

   private static YoGraphicTriangle yoGraphicTriangleFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      return new YoGraphicTriangle(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], (YoDouble) vars[3], (YoDouble) vars[4], (YoDouble) vars[5],
                                   (YoDouble) vars[6], (YoDouble) vars[7], (YoDouble) vars[8], appearance);
   }

   private static YoGraphicCylinder yoGraphicCylinderFromMessage(String name, YoVariable<?>[] vars, double[] consts, AppearanceDefinition appearance)
   {
      return new YoGraphicCylinder(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], (YoDouble) vars[3], (YoDouble) vars[4], (YoDouble) vars[5],
                                   appearance, consts[0]);
   }

   private static YoVariable<?> getVariableOrNull(YoVariable<?>[] vars, int i)
   {
      if (i < vars.length)
      {
         return vars[i];
      }
      else
      {
         return null;
      }
   }
}
