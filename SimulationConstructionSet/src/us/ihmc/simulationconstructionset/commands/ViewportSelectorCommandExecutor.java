package us.ihmc.simulationconstructionset.commands;


public interface ViewportSelectorCommandExecutor
{
   public abstract void selectViewport(String name);
   public abstract void hideViewport();
   public abstract void showViewport();
   public abstract boolean isViewportHidden();
   
   public abstract void registerViewportSelectorCommandListener(ViewportSelectorCommandListener commandListener);
   public abstract void closeAndDispose();
}
