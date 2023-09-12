package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDX3DSituatedTextDemo
{
   public RDX3DSituatedTextDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private RDX3DSituatedText text;
         private RDX3DSituatedText rapidlyChangingText;
         private RDX3DSituatedTextData previousTextData;
         private final Stopwatch stopwatch = new Stopwatch().start();
         private ModelInstance markerModelInstance;
         private final FrameBox3D selectionCollisionBox = new FrameBox3D();
         private final Vector3 topLeftPosition = new Vector3();
         private final Vector3 bottomLeftPosition = new Vector3();
         private final Vector3 bottomRightPosition = new Vector3();
         private final Vector3 topRightPosition = new Vector3();
         private final Color BOX_EDGE_COLOR = new Color(Color.WHITE);

         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            text = new RDX3DSituatedText("test");
            baseUI.getPrimaryScene().addRenderableProvider(text);

            rapidlyChangingText = new RDX3DSituatedText("rapidly changing");
            baseUI.getPrimaryScene().addRenderableProvider(rapidlyChangingText);
         }

         @Override
         public void render()
         {
            text.getModelTransform().rotate(0.0f, 0.0f, 1.0f, 1.0f);

            if (previousTextData != null)
               previousTextData.dispose();

            previousTextData = rapidlyChangingText.setTextWithoutCache("Time: " + stopwatch.totalElapsed());
            rapidlyChangingText.getModelTransform().rotate(1.0f, 0.0f, 0.0f, 200.0f * (float) stopwatch.totalElapsed());

            double panelWidth = 0.5 * Math.sin(0.0001 * ((float) stopwatch.totalElapsed()));
            double panelHeight = 0.5 * Math.sin(0.0001 * ((float) stopwatch.totalElapsed()));

            float halfPanelHeight = (float) panelHeight / 2.0f;
            float halfPanelWidth = (float) panelWidth / 2.0f;

            topLeftPosition.set(halfPanelHeight, halfPanelWidth, 0.0f);
            bottomLeftPosition.set(-halfPanelHeight, halfPanelWidth, 0.0f);
            bottomRightPosition.set(-halfPanelHeight, -halfPanelWidth, 0.0f);
            topRightPosition.set(halfPanelHeight, -halfPanelWidth, 0.0f);

            selectionCollisionBox.getSize().set(0.05, Math.abs(topRightPosition.y - topLeftPosition.y), Math.abs(topRightPosition.y - bottomLeftPosition.y));
            FramePoint3DBasics[] vertices = selectionCollisionBox.getVertices();
            if (markerModelInstance != null)
            {
               markerModelInstance.model.dispose();
            }
            Model markerModel = RDXModelBuilder.buildModel(boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(vertices,
                                                                                                            0.0005,
                                                                                                            BOX_EDGE_COLOR));
            markerModelInstance = new RDXModelInstance(markerModel);
//            baseUI.getPrimaryScene().addRenderableProvider(markerModelInstance);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDX3DSituatedTextDemo();
   }
}
