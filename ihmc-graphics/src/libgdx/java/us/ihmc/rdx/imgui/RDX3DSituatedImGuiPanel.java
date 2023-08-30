package us.ihmc.rdx.imgui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.ImGuiPlatformIO;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiWindowFlags;
import imgui.gl3.ImGuiImplGl3;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRPickResult;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;
import static com.badlogic.gdx.graphics.VertexAttributes.Usage.TextureCoordinates;

public class RDX3DSituatedImGuiPanel
{
   private final String name;
   private final Runnable renderImGuiWidgets;
   private RDXModelInstance modelInstance = null;
   private ImGuiImplGl3 imGuiGl3;
   private long imGuiContext;
   private float pixelsWidth;
   private float pixelsHeight;
   private float panelWidthInMeters;
   private float panelHeightInMeters;
   private float halfPanelWidthInMeters;
   private float halfPanelHeightInMeters;
   private float metersToPixels;
   private FrameBuffer frameBuffer;
   private boolean useTransparentBackground = false;
   private int backgroundColor;
   private float mousePosX = -20.0f;
   private float mousePosY = -20.0f;
   private boolean leftMouseDown = false;
   private final AtomicLong INDEX = new AtomicLong(0);
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final RigidBodyTransform graphicsXRightYDownToCenterXThroughZUpTransform = new RigidBodyTransform();
   private final ReferenceFrame centerXThroughZUpFrame
      = ReferenceFrameTools.constructFrameWithChangingTransformToParent("centerXThroughZUpFrame" + INDEX.getAndIncrement(),
                                                                        ReferenceFrame.getWorldFrame(),
                                                                        transform);
   private final ReferenceFrame graphicsXRightYDownFrame
         = ReferenceFrameTools.constructFrameWithChangingTransformToParent("graphicsXRightYDownFrame" + INDEX.getAndIncrement(),
                                                                           centerXThroughZUpFrame,
                                                                           graphicsXRightYDownToCenterXThroughZUpTransform);
   private final Plane3D plane = new Plane3D();
   private final FramePoint3D pickIntersection = new FramePoint3D();
   private final SideDependentList<RDXVRPickResult> pickResult = new SideDependentList<>(RDXVRPickResult::new);

   public RDX3DSituatedImGuiPanel(String name, Runnable renderImGuiWidgets)
   {
      this.name = name;
      this.renderImGuiWidgets = renderImGuiWidgets;

      for (RobotSide side : RobotSide.values)
         pickResult.get(side).setPickedObjectID(this, name + " ImGui Panel");
   }

   public void create(ImGuiImplGl3 imGuiGl3, double panelWidthInMeters, double panelHeightInMeters, int pixelsPerCentimeter)
   {
      this.imGuiGl3 = imGuiGl3;
      this.panelWidthInMeters = (float) panelWidthInMeters;
      this.panelHeightInMeters = (float) panelHeightInMeters;
      halfPanelWidthInMeters = this.panelWidthInMeters / 2.0f;
      halfPanelHeightInMeters = this.panelHeightInMeters / 2.0f;
      metersToPixels = pixelsPerCentimeter * 100;

      pixelsWidth = Math.round(panelWidthInMeters * metersToPixels);
      pixelsHeight = Math.round(panelHeightInMeters * metersToPixels);

      imGuiContext = ImGuiTools.createContext(ImGuiTools.getFontAtlas());
      ImGuiTools.setCurrentContext(imGuiContext);

      ImGuiIO io = ImGui.getIO();
      io.setIniFilename(null); // We don't want to save .ini file
      io.setMouseDrawCursor(true);

      ImGuiTools.initializeColorStyle();

      buildModel();

      // set up graphicsXRightYDownToCenterXThroughZUpTransform
      graphicsXRightYDownToCenterXThroughZUpTransform.appendYawRotation(-Math.toRadians(90.0));
      graphicsXRightYDownToCenterXThroughZUpTransform.appendPitchRotation(Math.toRadians(0.0));
      graphicsXRightYDownToCenterXThroughZUpTransform.appendRollRotation(-Math.toRadians(90.0));
      graphicsXRightYDownToCenterXThroughZUpTransform.appendTranslation(-halfPanelWidthInMeters, -halfPanelHeightInMeters, 0.0f);
      graphicsXRightYDownFrame.update();

      plane.getNormal().set(Axis3D.X);
   }

   private void buildModel()
   {
      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      // Counter clockwise order
      Vector3 topLeftPosition = new Vector3(0.0f, halfPanelWidthInMeters, halfPanelHeightInMeters);
      Vector3 bottomLeftPosition = new Vector3(0.0f, halfPanelWidthInMeters, -halfPanelHeightInMeters);
      Vector3 bottomRightPosition = new Vector3(0.0f, -halfPanelWidthInMeters, -halfPanelHeightInMeters);
      Vector3 topRightPosition = new Vector3(0.0f, -halfPanelWidthInMeters, halfPanelHeightInMeters);
      Vector3 topLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 bottomLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 bottomRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 topRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector2 topLeftUV = new Vector2(0.0f, 1.0f);
      Vector2 bottomLeftUV = new Vector2(0.0f, 0.0f);
      Vector2 bottomRightUV = new Vector2(1.0f, 0.0f);
      Vector2 topRightUV = new Vector2(1.0f, 1.0f);
      meshBuilder.vertex(topLeftPosition, topLeftNormal, Color.WHITE, topLeftUV);
      meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, Color.WHITE, bottomLeftUV);
      meshBuilder.vertex(bottomRightPosition, bottomRightNormal, Color.WHITE, bottomRightUV);
      meshBuilder.vertex(topRightPosition, topRightNormal, Color.WHITE, topRightUV);
      meshBuilder.triangle((short) 3, (short) 0, (short) 1);
      meshBuilder.triangle((short) 1, (short) 2, (short) 3);

      Mesh mesh = meshBuilder.end();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();

      GLFrameBuffer.FrameBufferBuilder frameBufferBuilder = new GLFrameBuffer.FrameBufferBuilder((int) pixelsWidth, (int) pixelsHeight);
      frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
      frameBuffer = frameBufferBuilder.build();
      Texture colorBufferTexture = frameBuffer.getColorBufferTexture();
      material.set(TextureAttribute.createDiffuse(colorBufferTexture));

      material.set(ColorAttribute.createDiffuse(Color.WHITE));
      modelBuilder.part(meshPart, material);

      Model model = modelBuilder.end();
      modelInstance = new RDXModelInstance(model);
   }

   public void setMouseState(float mousePosX, float mousePosY, boolean leftMouseDown)
   {
      this.mousePosX = mousePosX;
      this.mousePosY = mousePosY;
      this.leftMouseDown = leftMouseDown;
   }

   public void update()
   {
      ImGuiTools.setCurrentContext(imGuiContext);

      ImGuiIO io = ImGui.getIO();
      io.setDisplaySize(pixelsWidth, pixelsHeight);
      io.setDisplayFramebufferScale(1.0f, 1.0f);
      io.setMousePos(mousePosX, mousePosY);
      io.setMouseDown(ImGuiMouseButton.Left, leftMouseDown);

      ImGuiPlatformIO platformIO = ImGui.getPlatformIO();
      // Sets the ImVector of monitors to 0; clearing them
      platformIO.resizeMonitors(0);
      // Adding a virtual monitor
      platformIO.pushMonitors(0.0f, 0.0f, pixelsWidth, pixelsHeight, 0.0f, 0.0f, pixelsWidth, pixelsHeight, 1.0f);

      float deltaTime = Gdx.app.getGraphics().getDeltaTime();
      io.setDeltaTime(deltaTime > 0.0f ? deltaTime : 1.0f / 60.0f);

      ImGui.newFrame();
      ImGui.pushFont(ImGuiTools.getSmallFont());

      ImGui.setNextWindowPos(0.0f, 0.0f);
      ImGui.setNextWindowSize(pixelsWidth, pixelsHeight);

      int windowFlags = ImGuiWindowFlags.None;
//      windowFlags |= ImGuiWindowFlags.NoBackground;

      if (useTransparentBackground)
         ImGui.pushStyleColor(ImGuiCol.WindowBg, backgroundColor);
      if (ImGui.begin(name, windowFlags))
      {
         renderImGuiWidgets.run();
         ImGui.end();
      }
      if (useTransparentBackground)
         ImGui.popStyleColor();

      ImGui.popFont();
      ImGui.render();

      frameBuffer.begin();
      GL41.glClearColor(0.3f, 0.3f, 0.3f, 0.0f);
      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT);
      imGuiGl3.renderDrawData(ImGui.getDrawData());
      frameBuffer.end();
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
         // Check not facing the back of the panel
         if (controller.getPickRay().getDirection().dot(plane.getNormal()) > 0.0)
         {
            pickIntersection.setToZero(ReferenceFrame.getWorldFrame());
            plane.intersectionWith(controller.getPickRay(), pickIntersection);
            double distance = controller.getPickRay().getPoint().distance(pickIntersection);

            pickIntersection.changeFrame(graphicsXRightYDownFrame);

            float scaledX = Math.round((float) pickIntersection.getX() * metersToPixels);
            float scaledY = Math.round((float) pickIntersection.getY() * metersToPixels);

            boolean xInBounds = MathTools.intervalContains(scaledX, 0, pixelsWidth, true, false);
            boolean yInBounds = MathTools.intervalContains(scaledY, 0, pixelsHeight, true, false);
            if (xInBounds && yInBounds)
            {
               mousePosX = scaledX;
               mousePosY = scaledY;

               pickResult.get(RobotSide.RIGHT).setPointingAtCollision(distance);
               controller.addPickResult(pickResult.get(RobotSide.RIGHT));
            }
         }
      });
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
         if (controller.getSelectedPick() == pickResult.get(RobotSide.RIGHT))
         {
            leftMouseDown = controller.getClickTriggerActionData().bState();
         }
         else
         {
            mousePosX = -20.0f;
            mousePosY = -20.0f;
            leftMouseDown = false;
         }
      });
   }

   public void setBackgroundTransparency(Color backgroundColor)
   {
      useTransparentBackground = true;
      this.backgroundColor = backgroundColor.toIntBits();
      for (Material material : modelInstance.materials)
      {
         material.set(new BlendingAttribute(true, GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA, 1.0f));
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      frameBuffer.dispose();
   }

   public void updateDesiredPose(Consumer<RigidBodyTransform> transformUpdater)
   {
      updateCurrentPose(transformUpdater);
   }

   private void updateCurrentPose(Consumer<RigidBodyTransform> transformUpdater)
   {
      transform.setToZero();
      transformUpdater.accept(transform);

      modelInstance.setTransformToWorldFrame(transform);

      plane.setToZero();
      plane.getNormal().set(Axis3D.X);
      plane.applyTransform(transform);
      LibGDXTools.toLibGDX(transform, modelInstance.transform);
      centerXThroughZUpFrame.update();
   }

   public void setTransformToReferenceFrame(ReferenceFrame referenceFrame)
   {
      modelInstance.setTransformToReferenceFrame(referenceFrame);
   }

   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public final String getPanelName()
   {
      return name;
   }
}
