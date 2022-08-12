package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

public class GDXPastFootSteps implements RenderableProvider
{
    private final ArrayList<ImGuiGDXManuallyPlacedFootstep> pastSteps = new ArrayList<>();
    private GDXImGuiBasedUI baseUI;
    private boolean render = false;

    public GDXPastFootSteps(GDXImGuiBasedUI baseUI)
    {
        this.baseUI = baseUI;
    }

    public void logStepsTaken(ArrayList<ImGuiGDXManuallyPlacedFootstep> stepsTaken)
    {
        for (int i = 0; i < stepsTaken.size(); ++i)
        {
            ImGuiGDXManuallyPlacedFootstep step = stepsTaken.get(i);
            pastSteps.add(step);
            baseUI.getPrimaryScene().addModelInstance(step.getFootstepModelInstance(), GDXSceneLevel.VIRTUAL);
        }

        if(pastSteps.size()>0)
        {
            LogTools.info("current pastSteps list size: " + pastSteps.size());
        }

    }

    @Override
    public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
    {
        for (int i = 0; i < pastSteps.size(); ++i)
        {
            pastSteps.get(i).getFootstepModelInstance().getRenderables(renderables,pool);
        }
    }

    public void clear()
    {
        pastSteps.clear();
    }
}
