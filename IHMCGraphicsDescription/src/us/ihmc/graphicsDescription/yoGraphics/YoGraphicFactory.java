package us.ihmc.graphicsDescription.yoGraphics;

import java.util.ArrayList;

import org.apache.commons.lang3.NotImplementedException;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphic.RemoteGraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;

/**
 * This class allows constructs RemoteYoGraphic objects from parsed
 * data retrieved by the YoVariableHandshakeClient.
 *
 * @author Alex Lesman
 *
 */
public class YoGraphicFactory
{
   private static YoVariable<?> getVariableOrNull(YoVariable<?>[] vars, int i)
   {
      if(i < vars.length)
      {
         return vars[i];
      }
      else
      {
         return null;
      }
   }
   
   public static RemoteYoGraphic yoGraphicFromMessage(RemoteGraphicType type, String name, YoVariable<?>[] vars, double[] consts,
         AppearanceDefinition appearance)
   {
      switch (type)
      {
      case CYLINDER_DGO:
         return new YoGraphicCylinder(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2],
               (YoDouble) vars[3], (YoDouble) vars[4], (YoDouble) vars[5], appearance, consts[0]);

      case TRIANGLE_DGO:
         return new YoGraphicTriangle(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2],
               (YoDouble) vars[3], (YoDouble) vars[4], (YoDouble) vars[5],
               (YoDouble) vars[6], (YoDouble) vars[7], (YoDouble) vars[8],
               appearance);

      case VECTOR_DGO:
         return new YoGraphicVector(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], (YoDouble) vars[3],
               (YoDouble) vars[4], (YoDouble) vars[5], consts[0], appearance, true);

      case POSITION_DGO:
         return new YoGraphicPosition(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) getVariableOrNull(vars, 2), consts[0], appearance,
               YoGraphicPosition.GraphicType.values()[(int) (double) consts[1]]);

      case COORDINATE_SYSTEM_DGO:
         return new YoGraphicCoordinateSystem(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2],
               (YoDouble) vars[3], (YoDouble) vars[4], (YoDouble) vars[5], consts);

      case LINE_SEGMENT_DGO:
         return new YoGraphicLineSegment(name, (YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2],
               (YoDouble) vars[3], (YoDouble) vars[4], (YoDouble) vars[5], consts[0], appearance);

      case YO_FRAME_POLYGON_DGO:
      {
         ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
         int i = 0;
         YoInteger yoNumVertices = (YoInteger) vars[i++];
         ArrayList<YoFramePoint2d> yoFramePoints = new ArrayList<YoFramePoint2d>();
         while (i < vars.length - 6)
         {
            yoFramePoints.add(new YoFramePoint2d((YoDouble) vars[i++], (YoDouble) vars[i++], referenceFrame));
         }

         YoFrameConvexPolygon2d convexPolygon2d = new YoFrameConvexPolygon2d(yoFramePoints, yoNumVertices, referenceFrame);

         YoFramePoint framePoint = new YoFramePoint((YoDouble) vars[i++], (YoDouble) vars[i++], (YoDouble) vars[i++], referenceFrame);
         YoFrameOrientation frameOrientation = new YoFrameOrientation((YoDouble) vars[i++], (YoDouble) vars[i++], (YoDouble) vars[i++],
               referenceFrame);

         return new YoGraphicPolygon(name, convexPolygon2d, framePoint, frameOrientation, consts[0], appearance);
      }

      case POSITION_ARTIFACT:
         return new YoArtifactPosition(name, (YoDouble) vars[0], (YoDouble) vars[1],
               YoGraphicPosition.GraphicType.values()[(int) (double) consts[1]], appearance.getColor().get(), consts[0]);

      case CIRCLE_ARTIFACT:
         return new YoArtifactOval(name, (YoDouble) vars[0], (YoDouble) vars[1],
               (YoDouble) vars[2], appearance.getColor().get());

      case LINE_SEGMENT_2D_ARTIFACT:

         YoFrameLineSegment2d segment = new YoFrameLineSegment2d((YoDouble) vars[0], (YoDouble) vars[1], (YoDouble) vars[2], (YoDouble) vars[3],
               ReferenceFrame.getWorldFrame());

         return new YoArtifactLineSegment2d(name, segment, appearance.getColor().get());

      case POLYGON_ARTIFACT:
      {
         ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
         int i = 0;
         YoInteger yoNumVertices = (YoInteger) vars[i++];
         ArrayList<YoFramePoint2d> yoFramePoints = new ArrayList<YoFramePoint2d>();
         while (i < vars.length)
         {
            yoFramePoints.add(new YoFramePoint2d((YoDouble) vars[i++], (YoDouble) vars[i++], referenceFrame));
         }

         YoFrameConvexPolygon2d convexPolygon2d = new YoFrameConvexPolygon2d(yoFramePoints, yoNumVertices, referenceFrame);

         return new YoArtifactPolygon(name, convexPolygon2d, appearance.getColor().get(), consts[0] > 0);
      }
      case PLANAR_REGIONS_LIST_DGO:
      {
         return YoGraphicPlanarRegionsList.createAsRemoteYoGraphic(name, vars, consts);
      }
      case POLYNOMIAL_3D_DGO:
         return YoGraphicPolynomial3D.createAsRemoteYoGraphic(name, vars, consts);

      default:
         throw new NotImplementedException(type.toString() + " is not implemented");
      }
   }
}
