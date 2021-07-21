package us.ihmc.gdx.lighting;

import com.badlogic.gdx.graphics.glutils.ShaderProgram;

public abstract class GDXLight
{
   protected abstract void apply(final ShaderProgram shader);
}
