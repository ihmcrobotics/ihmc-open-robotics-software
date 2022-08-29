package us.ihmc.gdx.tools;

import us.ihmc.tools.io.WorkspaceDirectory;

import java.util.ArrayList;

public class GDXToolButton
{
   private final String buttonName;
   private ArrayList<GDXIconTexture> icons = new ArrayList<>();
   private ArrayList<Runnable> runnables = new ArrayList<>();
   private boolean togglable = false;
   private boolean depends = false;
   private int stateIndex = 0;
   private String toolTipText = new String();

   public GDXToolButton(String buttonName, WorkspaceDirectory iconDirectory, String iconFileName, Runnable runnable)
   {
      this(buttonName,iconDirectory,iconFileName,runnable,false);
   }

   public GDXToolButton(String buttonName, WorkspaceDirectory iconDirectory, String iconFileName, Runnable runnable, boolean depends)
   {
      this.buttonName = buttonName;
      icons.add(new GDXIconTexture(iconDirectory.file(iconFileName)));
      if (runnable == null)
      {
         runnables = null;
      }
      else
      {
         runnables.add(runnable);
      }
      this.depends = depends;
   }

   public GDXToolButton(String buttonName,
                        WorkspaceDirectory iconDirectory,
                        String iconFileName,
                        ArrayList<Runnable> runnables,
                        boolean togglable,
                        boolean depends)
   {
      this.buttonName = buttonName;
      icons.add(new GDXIconTexture(iconDirectory.file(iconFileName)));
      this.runnables = runnables;
      this.togglable = togglable;
      this.depends = depends;
   }

   public GDXToolButton(String buttonName,
                        WorkspaceDirectory iconDirectory,
                        ArrayList<String> iconFileNames,
                        ArrayList<Runnable> runnables,
                        boolean togglable,
                        boolean depends)
   {
      this.buttonName = buttonName;
      for (String iconFileName : iconFileNames)
      {
         icons.add(new GDXIconTexture(iconDirectory.file(iconFileName)));
      }
      this.runnables = runnables;
      this.togglable = togglable;
      this.depends = depends;
   }

   public void isClicked()
   {
      if (togglable)
      {
         stateIndex = (stateIndex + 1) % icons.size();
      }
   }

   public void setState(int index)
   {
      this.stateIndex = index;
   }

   public int getStateIndex()
   {
      return stateIndex;
   }

   public void execute()
   {
      if (runnables == null)
         return;
      if (runnables.size() == 1)
         runnables.get(0).run();
      else
         runnables.get(getStateIndex()).run();
   }

   public GDXIconTexture getIcon()
   {
      if(icons.size()==1)
         return icons.get(0);
      return icons.get(getStateIndex());
   }

   public String getButtonName()
   {
      return buttonName;
   }

   public boolean isTogglable()
   {
      return togglable;
   }

   public boolean doesDepend()
   {
      return depends;
   }

   public void setToolTipText(String text)
   {
      toolTipText = text;
   }

   public String getToolTipText()
   {
      return toolTipText;
   }
}