package us.ihmc.plotting.plotter2d;

public class Plotter2dDemo
{
   public void showPlotter()
   {
      Plotter2d plotter = new Plotter2d();
      plotter.setPreferredSize(800, 600);
      
      plotter.setScale(10.0, 10.0);
      plotter.setShowLabels(true);
      
//      plotter.addArtifact(new LineArtifact("01", new Point2d(0, 0), new Point2d(1, 1)));
//      plotter.addArtifact(new LineArtifact("02", new Point2d(1, 1), new Point2d(2, 0)));
//      plotter.addArtifact(new LineArtifact("03", new Point2d(2, 0), new Point2d(3, 1)));
      
      plotter.showInNewWindow();
      
      plotter.setScale(40.0, 20.0);
      plotter.setFocusPointX(-5.0);
      plotter.setFocusPointY(10.0);
   }
   
   public static void main(String[] args)
   {
      Plotter2dDemo plotterDemo = new Plotter2dDemo();
      plotterDemo.showPlotter();
   }
}
