package us.ihmc.rdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Skin;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;

/**
 * Not working anymore.
 */
public class RDX3DWith2DUIDemo
{
   private Stage stage;
   private Table table;

   public RDX3DWith2DUIDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

//            baseUI.getPrimaryScene().setViewportBounds(0,
//                              (int) (baseUI.getPrimaryScene().getCurrentWindowHeight() * 1.0 / 4.0),
//                              (int) (baseUI.getPrimaryScene().getCurrentWindowWidth() * 1.0),
//                              (int) (baseUI.getPrimaryScene().getCurrentWindowHeight() * 3.0 / 4.0));

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            stage = new Stage(new ScreenViewport());
            baseUI.getPrimary3DPanel().addLibGDXInputProcessor(stage);

            table = new Table();
            table.setFillParent(true);
            //      table.setDebug(true);
            stage.addActor(table);

            Skin skin = new Skin(Gdx.files.internal("uiskin.json"));

            table.left();
            table.top();
            TextButton button1 = new TextButton("Button 1", skin);
            table.add(button1);

            TextButton button2 = new TextButton("Button 2", skin);
            table.add(button2);
         }

         @Override
         public void render()
         {
            GL41.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight() * 1 / 4);
            stage.getViewport().update(baseUI.getPrimary3DPanel().getCurrentWindowWidth(), baseUI.getPrimary3DPanel().getCurrentWindowHeight() * 1 / 4, true);

            stage.act(Gdx.graphics.getDeltaTime());
            stage.draw();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            stage.dispose();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDX3DWith2DUIDemo();
   }
}
