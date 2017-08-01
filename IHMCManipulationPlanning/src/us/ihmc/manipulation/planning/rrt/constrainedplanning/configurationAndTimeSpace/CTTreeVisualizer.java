package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

public class CTTreeVisualizer
{
   private CTNodeVisualizer nodeVisualizer;
   
   public CTTreeVisualizer(int configurationDimension)
   {
      nodeVisualizer = new CTNodeVisualizer(1);
   }
   
   public void initialize()
   {
      nodeVisualizer.initialize();
   }
   
   public void update(CTTaskNode newNode)
   {
      nodeVisualizer.updateVisualizer(newNode);
   }
}
