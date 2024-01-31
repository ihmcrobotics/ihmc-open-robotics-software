package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.input.ImGui3DViewPickResult;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDX3DPanelTooltip;
import us.ihmc.robotics.interaction.BoxRayIntersection;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;

public class RDXFootstepGraphic implements RenderableProvider
{
   public static final Color LEFT_FOOT_RED_COLOR = new Color(1.0f, 0.0f, 0.0f, 1.0f);
   public static final Color RIGHT_FOOT_GREEN_COLOR = new Color(0.0f, 0.5019608f, 0.0f, 1.0f);
   public static final SideDependentList<Color> FOOT_COLORS = new SideDependentList<>(LEFT_FOOT_RED_COLOR, RIGHT_FOOT_GREEN_COLOR);
   private static final AtomicInteger INDEX = new AtomicInteger();

   private final Color color;
   private final ConvexPolygon2D defaultContactPoints = new ConvexPolygon2D();
   private RDXModelInstance modelInstance;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final BoxRayIntersection boxRayIntersection = new BoxRayIntersection();
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();
   private boolean mouseHovering = false;
   private RDX3DPanelTooltip tooltip;

   public RDXFootstepGraphic(SegmentDependentList<RobotSide, ArrayList<Point2D>> controllerFootGroundContactPoints, RobotSide side)
   {
      this(controllerFootGroundContactPoints.get(side), new Color(FOOT_COLORS.get(side)));
   }

   public RDXFootstepGraphic(ArrayList<Point2D> controllerFootGroundContactPoints, Color color)
   {
      this.color = color;
      controllerFootGroundContactPoints.forEach(defaultContactPoints::addVertex);
      defaultContactPoints.update();
   }

   public void create()
   {
      Point3D[] vertices = new Point3D[defaultContactPoints.getNumberOfVertices()];
      for (int j = 0; j < vertices.length; j++)
      {
         vertices[j] = new Point3D(defaultContactPoints.getVertex(j).getX(),
                                   defaultContactPoints.getVertex(j).getY(),
                                   0.0);
      }

      modelInstance = new RDXModelInstance(RDXModelBuilder.buildModelInstance(meshBuilder ->
      {
         meshBuilder.addMultiLine(vertices, 0.01, color, true);
      }, "footstepGraphic" + INDEX.getAndIncrement()));
      LibGDXTools.setOpacity(modelInstance, color.a);
   }

   public void setupTooltip(RDX3DPanel panel3D, String text)
   {
      tooltip = new RDX3DPanelTooltip(panel3D);
      panel3D.addImGuiOverlayAddition(() ->
      {
         if (mouseHovering)
            tooltip.render(text);
      });
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      LibGDXTools.toEuclid(modelInstance.transform, tempTransform);
      boxRayIntersection.intersect(0.2, 0.1, 0.02, tempTransform, input.getPickRayInWorld());
      if (boxRayIntersection.getIntersects())
      {
         pickResult.setDistanceToCamera(boxRayIntersection.getFirstIntersectionToPack().distance(input.getPickRayInWorld().getPoint()));
         input.addPickResult(pickResult);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      mouseHovering = pickResult == input.getClosestPick();
      modelInstance.setOpacity(mouseHovering ? 0.5f : 1.0f);
      if (tooltip != null)
         tooltip.setInput(input);
   }

   public void setOpacity(double opacity)
   {
      color.a = (float) opacity;
   }

   public void setColor(Color color)
   {
      this.color.r = color.r;
      this.color.g = color.g;
      this.color.b = color.b;
   }

   public void setPose(Pose3DReadOnly pose)
   {
      LibGDXTools.toLibGDX(pose, tempTransform, modelInstance.transform);
   }

   public void setPoseFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      modelInstance.setTransformToReferenceFrame(referenceFrame);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public void destroy()
   {

   }

   public boolean getMouseHovering()
   {
      return mouseHovering;
   }
}
