package us.ihmc.gdx.simulation.environment.object.objects;

import us.ihmc.gdx.lighting.GDXLight;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;

public abstract class GDXLightObject extends GDXEnvironmentObject
{
   public abstract GDXLight getLight();
}
