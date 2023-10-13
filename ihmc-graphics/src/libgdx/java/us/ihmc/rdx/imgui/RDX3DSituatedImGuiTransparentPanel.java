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
import imgui.flag.ImGuiWindowFlags;
import imgui.gl3.ImGuiImplGl3;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelInstance;

import java.util.concurrent.atomic.AtomicLong;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;

public class RDX3DSituatedImGuiTransparentPanel
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
   private float pixelsToMeters;
   private FrameBuffer frameBuffer;
   private int backgroundColor;
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

   public RDX3DSituatedImGuiTransparentPanel(String name, Runnable renderImGuiWidgets)
   {
      this.name = name;
      this.renderImGuiWidgets = renderImGuiWidgets;
   }

   public void create(ImGuiImplGl3 imGuiGl3, double panelWidthInMeters, double panelHeightInMeters, int pixelsPerCentimeter)
   {
      this.imGuiGl3 = imGuiGl3;
      this.panelWidthInMeters = (float) panelWidthInMeters;
      this.panelHeightInMeters = (float) panelHeightInMeters;
      halfPanelWidthInMeters = this.panelWidthInMeters / 2.0f;
      halfPanelHeightInMeters = this.panelHeightInMeters / 2.0f;
      metersToPixels = pixelsPerCentimeter * 100;
      pixelsToMeters = 1.0f / metersToPixels;

      pixelsWidth = Math.round(panelWidthInMeters * metersToPixels);
      pixelsHeight = Math.round(panelHeightInMeters * metersToPixels);

      imGuiContext = ImGuiTools.createContext(ImGuiTools.getFontAtlas());
      ImGuiTools.setCurrentContext(imGuiContext);

      ImGuiIO io = ImGui.getIO();
      io.setIniFilename(null); // We don't want to save .ini file
      ImGuiTools.initializeColorStyle();
      buildModel();

      // set up graphicsXRightYDownToCenterXThroughZUpTransform
      graphicsXRightYDownToCenterXThroughZUpTransform.appendYawRotation(-Math.toRadians(90.0));
      graphicsXRightYDownToCenterXThroughZUpTransform.appendPitchRotation(Math.toRadians(0.0));
      graphicsXRightYDownToCenterXThroughZUpTransform.appendRollRotation(-Math.toRadians(90.0));
      graphicsXRightYDownToCenterXThroughZUpTransform.appendTranslation(-halfPanelWidthInMeters, -halfPanelHeightInMeters, 0.0f);
      graphicsXRightYDownFrame.update();
   }

   private void buildModel()
   {
      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      // Counterclockwise order
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
      material.set(new BlendingAttribute(true, GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA, 0.0f));
      modelBuilder.part(meshPart, material);

      Model model = modelBuilder.end();
      modelInstance = new RDXModelInstance(model);
      setBackgroundTransparency(new Color(0f,0f, 0f,0f));
   }

   public void update()
   {
      ImGuiTools.setCurrentContext(imGuiContext);

      ImGuiIO io = ImGui.getIO();
      io.setDisplaySize(pixelsWidth, pixelsHeight);
      io.setDisplayFramebufferScale(1.0f, 1.0f);

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

      int windowFlags = ImGuiWindowFlags.NoDecoration;
      ImGui.pushStyleColor(ImGuiCol.WindowBg, backgroundColor);
      if (ImGui.begin(name, windowFlags))
      {
         renderImGuiWidgets.run();
         ImGui.end();
      }
      ImGui.popStyleColor();

      ImGui.popFont();
      ImGui.render();

      frameBuffer.begin();
      GL41.glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT);
      imGuiGl3.renderDrawData(ImGui.getDrawData());
      frameBuffer.end();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      frameBuffer.dispose();
   }

   public void setBackgroundTransparency(Color backgroundColor)
   {
      this.backgroundColor = backgroundColor.toIntBits();
      for (Material material : modelInstance.materials)
      {
         material.set(new BlendingAttribute(true, GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA, 1.0f));
      }
   }

   public void setTransformToReferenceFrame(ReferenceFrame referenceFrame)
   {
      modelInstance.setTransformToReferenceFrame(referenceFrame);
   }

   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }
}