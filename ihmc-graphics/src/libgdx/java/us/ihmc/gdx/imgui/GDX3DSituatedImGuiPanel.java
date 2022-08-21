package us.ihmc.gdx.imgui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
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
import imgui.gl3.ImGuiImplGl3;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXModelInstance;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;
import static com.badlogic.gdx.graphics.VertexAttributes.Usage.TextureCoordinates;

public class GDX3DSituatedImGuiPanel
{
   private final String name;
   private final Runnable renderImGuiWidgets;
   private GDXModelInstance modelInstance = null;
   private ImGuiImplGl3 imGuiGl3;
   private long imGuiContext;
   private float pixelsWidth;
   private float pixelsHeight;
   private float metersPerPixel = 0.001f;
   private FrameBuffer frameBuffer;

   public GDX3DSituatedImGuiPanel(String name, Runnable renderImGuiWidgets)
   {
      this.name = name;
      this.renderImGuiWidgets = renderImGuiWidgets;
   }

   public void create(ImGuiImplGl3 imGuiGl3, double width, double height)
   {
      this.imGuiGl3 = imGuiGl3;

      pixelsWidth = Math.round(width / metersPerPixel);
      pixelsHeight = Math.round(height / metersPerPixel);

      imGuiContext = ImGuiTools.createContext(ImGuiTools.getFontAtlas());
      ImGuiTools.setCurrentContext(imGuiContext);

      ImGuiIO io = ImGui.getIO();
      io.setIniFilename(null); // We don't want to save .ini file
      io.setMouseDrawCursor(true);

      ImGuiTools.initializeColorStyle();

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      // Counter clockwise order
      // Draw so thumb faces away and index right
      Vector3 topLeftPosition = new Vector3(0.0f, 0.0f, 0.0f);
      Vector3 bottomLeftPosition = new Vector3(0.0f, metersPerPixel * pixelsHeight, 0.0f);
      Vector3 bottomRightPosition = new Vector3(metersPerPixel * pixelsWidth, metersPerPixel * pixelsHeight, 0.0f);
      Vector3 topRightPosition = new Vector3(metersPerPixel * pixelsWidth, 0.0f, 0.0f);
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
      modelInstance = new GDXModelInstance(model);
   }

   public void update()
   {
      ImGuiTools.setCurrentContext(imGuiContext);

      ImGuiIO io = ImGui.getIO();
      io.setDisplaySize(pixelsWidth, pixelsHeight);
      io.setDisplayFramebufferScale(1.0f, 1.0f);
//      io.setMousePos(mousePosX, mousePosY);
//      io.setMouseDown(ImGuiMouseButton.Left, leftMouseDown);

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

      if (ImGui.begin(name))
      {
         renderImGuiWidgets.run();
         ImGui.end();
      }

      ImGui.popFont();
      ImGui.render();

      frameBuffer.begin();
      ImGuiTools.glClearDarkGray();
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

   public void setTransformToReferenceFrame(ReferenceFrame referenceFrame)
   {
      modelInstance.setTransformToReferenceFrame(referenceFrame);
//      modelInstance.transform.scale(0.005f, 0.005f, 0.005f);
   }

   public GDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public final String getPanelName()
   {
      return name;
   }
}
