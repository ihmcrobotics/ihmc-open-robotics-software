package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalShadowLight;
import com.badlogic.gdx.graphics.g3d.environment.ShadowMap;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.graphics.g3d.utils.TextureDescriptor;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.math.Vector3;

public class GDXMultiSourceShadowMap implements ShadowMap
{
   private Environment environment;

   private DirectionalShadowLight shadowLight;
   private ModelBatch shadowBatch;

   public GDXMultiSourceShadowMap(Environment environment) {
      this.environment = environment;
   }

   public void create() {
      this.shadowBatch = new ModelBatch(new DepthShaderProvider());
   }

   public <T extends RenderableProvider> void update(Camera camera3D, final Iterable<T> renderables) {
      shadowLight.begin(Vector3.Zero, camera3D.direction);
      shadowBatch.begin(shadowLight.getCamera());

      shadowBatch.render(renderables);

      shadowBatch.end();
      shadowLight.end();
   }

   public void add(DirectionalShadowLight shadowLight) {
      environment.add(shadowLight);
      this.shadowLight = shadowLight;
   }

   @Override
   public Matrix4 getProjViewTrans()
   {
      return shadowLight.getProjViewTrans();
   }

   @Override
   public TextureDescriptor getDepthMap()
   {
      return shadowLight.getDepthMap();
   }
}
