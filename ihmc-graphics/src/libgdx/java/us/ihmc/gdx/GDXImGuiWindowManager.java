package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.*;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiConfigFlags;
import imgui.internal.ImGuiContext;
import org.lwjgl.opengl.GL30;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.util.ArrayList;

public class GDXImGuiWindowManager implements RenderableProvider
{
   private static GDXImGuiWindowManager instance = null;

   public static GDXImGuiWindowManager getInstance() {
      if (instance == null)
         instance = new GDXImGuiWindowManager();

      return instance;
   }

   private final ArrayList<GDXImGuiWindow> panels = new ArrayList<>();
   private ImGuiContext CTX = null;
   private ImFont font = null;

   private final ModelBuilder BUILDER = new ModelBuilder();
   private ModelInstance modelInstance = null;

   private GDXImGuiWindowManager() {}

   public void update() {
      ImGuiContext CTX_EXTERNAL = ImGui.getCurrentContext();

      if (CTX == null)
      {
         CTX = ImGui.createContext();

         final ImGuiIO io = ImGui.getIO();

         io.setIniFilename(null); // We don't want to save .ini file
         io.addConfigFlags(ImGuiConfigFlags.DockingEnable);
         io.addConfigFlags(ImGuiConfigFlags.ViewportsEnable);
         io.setConfigViewportsNoTaskBarIcon(true);
         io.setConfigWindowsMoveFromTitleBarOnly(true);
         io.setConfigViewportsNoDecoration(false);
         io.setConfigDockingTransparentPayload(false);

         ImGui.styleColorsLight();

         font = ImGuiTools.setupFonts(io);

         if (io.hasConfigFlags(ImGuiConfigFlags.ViewportsEnable))
         {
            final ImGuiStyle style = imgui.ImGui.getStyle();
            style.setWindowRounding(0.0f);
            style.setColor(ImGuiCol.WindowBg, imgui.ImGui.getColorU32(ImGuiCol.WindowBg, 1));
         }

         io.setDisplaySize(3000, 2000);
      }

      ImGui.setCurrentContext(CTX);
      ImGui.newFrame();

      for (GDXImGuiWindow panel : panels) {
         if (ImGui.begin(panel.getName())) {
            panel.renderImGuiWidgets();
            ImGui.end();
         }
      }

      ImGui.render();
      drawToRenderable(ImGui.getDrawData());

      if (CTX_EXTERNAL.ptr != 0)
         ImGui.setCurrentContext(CTX_EXTERNAL);
   }

   private final ImVec2 displaySize = new ImVec2();
   private final ImVec2 framebufferScale = new ImVec2();
   private final ImVec2 displayPos = new ImVec2();
   private final ImVec4 clipRect = new ImVec4();

   //maybe functional, maybe not
   private void drawToRenderable(ImDrawData drawData) {
      if (drawData.getCmdListsCount() <= 0) {
         return;
      }

      // Will project scissor/clipping rectangles into framebuffer space
      drawData.getDisplaySize(displaySize);           // (0,0) unless using multi-viewports
      drawData.getFramebufferScale(framebufferScale); // (1,1) unless using retina display which are often (2,2)

      // Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
      final int fbWidth = (int) (displaySize.x * framebufferScale.x);
      final int fbHeight = (int) (displaySize.y * framebufferScale.y);

      if (fbWidth <= 0 || fbHeight <= 0) {
         return;
      }

      drawData.getDisplayPos(displayPos);

      FrameBuffer buffer = new FrameBuffer(Pixmap.Format.RGB888, fbWidth, fbHeight, false);
      buffer.bind();

      // Render command lists
      for (int cmdListIdx = 0; cmdListIdx < drawData.getCmdListsCount(); cmdListIdx++) {
         // Upload vertex/index buffers
         GL30.glBufferData(GL30.GL_ARRAY_BUFFER, drawData.getCmdListVtxBufferData(cmdListIdx), GL30.GL_STREAM_DRAW);
         GL30.glBufferData(GL30.GL_ELEMENT_ARRAY_BUFFER, drawData.getCmdListIdxBufferData(cmdListIdx), GL30.GL_STREAM_DRAW);

         for (int cmdBufferIdx = 0; cmdBufferIdx < drawData.getCmdListCmdBufferSize(cmdListIdx); cmdBufferIdx++) {
            drawData.getCmdListCmdBufferClipRect(cmdListIdx, cmdBufferIdx, clipRect);

            final float clipRectX = (clipRect.x - displayPos.x) * framebufferScale.x;
            final float clipRectY = (clipRect.y - displayPos.y) * framebufferScale.y;
            final float clipRectZ = (clipRect.z - displayPos.x) * framebufferScale.x;
            final float clipRectW = (clipRect.w - displayPos.y) * framebufferScale.y;

            if (clipRectX < fbWidth && clipRectY < fbHeight && clipRectZ >= 0.0f && clipRectW >= 0.0f) {
               // Apply scissor/clipping rectangle
               GL30.glScissor((int) clipRectX, (int) (fbHeight - clipRectW), (int) (clipRectZ - clipRectX), (int) (clipRectW - clipRectY));

               // Bind texture, Draw
               final int textureId = drawData.getCmdListCmdBufferTextureId(cmdListIdx, cmdBufferIdx);
               final int elemCount = drawData.getCmdListCmdBufferElemCount(cmdListIdx, cmdBufferIdx);
               final int idxBufferOffset = drawData.getCmdListCmdBufferIdxOffset(cmdListIdx, cmdBufferIdx);
               final int vtxBufferOffset = drawData.getCmdListCmdBufferVtxOffset(cmdListIdx, cmdBufferIdx);
               final int indices = idxBufferOffset * ImDrawData.SIZEOF_IM_DRAW_IDX;

               GL30.glBindTexture(GL30.GL_TEXTURE_2D, textureId);

               GL30.glDrawElements(GL30.GL_TRIANGLES, elemCount, GL30.GL_UNSIGNED_SHORT, indices); //EXCEPTION_ACCESS_VIOLATION (0xc0000005) probably this https://stackoverflow.com/questions/18803921/calling-gldrawelements-causes-access-violation but I don't know how to fix it
            }
         }
      }

      FrameBuffer.unbind();

      Texture texture = buffer.getColorBufferTexture();
      Material material = new Material(TextureAttribute.createDiffuse(texture),
                                       ColorAttribute.createSpecular(1, 1, 1, 1),
                                       new BlendingAttribute(GL30.GL_SRC_ALPHA, GL30.GL_ONE_MINUS_SRC_ALPHA));
      long attributes = VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal | VertexAttributes.Usage.TextureCoordinates;

      Model model = BUILDER.createRect(0,
                                       0,
                                       0,
                                       texture.getWidth(),
                                       0,
                                       0,
                                       texture.getWidth(),
                                       texture.getHeight(),
                                       0,
                                       0,
                                       texture.getHeight(),
                                       0,
                                       0,
                                       0,
                                       1,
                                       material,
                                       attributes);
      modelInstance = new ModelInstance(model);

      buffer.dispose();
   }

   public void addPanel(GDXImGuiWindow panel) {
      panels.add(panel);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }

   public void dispose() {
      for (GDXImGuiWindow panel : panels)
         panel.dispose();
   }

   @Override
   protected void finalize() throws Throwable
   {
      this.dispose();
      super.finalize();
   }
}
