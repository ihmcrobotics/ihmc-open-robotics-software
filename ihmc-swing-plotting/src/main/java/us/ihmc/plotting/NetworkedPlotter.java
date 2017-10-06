package us.ihmc.plotting;
 
import java.util.Optional;
 
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.PlotterColors;
import us.ihmc.graphicsDescription.plotting.QualitativeColors;
import us.ihmc.graphicsDescription.plotting.artifact.PixelLineArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.PixelPointArtifact;
 
public class NetworkedPlotter
{
  private final Plotter plotter;
  private final QualitativeColors qualitativeColors;
  private int colorIndex;
  private long id;
 
  private Optional<Point2D> lastPoint;
 
  private Optional<Point2D> adjustingViewRangeMax;
  private Optional<Point2D> adjustingViewRangeMin;
 
  public NetworkedPlotter()
  {
     plotter = new Plotter(PlotterColors.javaFXStyle(), true);
 
     plotter.setXYZoomEnabled(true);
     plotter.setViewRangeX(1000.0);
     plotter.setViewRangeY(1000.0);
 
     qualitativeColors = new QualitativeColors();
     colorIndex = -1;
     id = 0;
     adjustingViewRangeMax = Optional.empty();
     adjustingViewRangeMin = Optional.empty();
 
     incrementSeries();
 
     plotter.showInNewWindow("Networked Plotter", false);
  }
 
  public void add2dPoint(Point2D point2D)
  {
     adjustViewRange(point2D);
 
     if (lastPoint.isPresent())
     {
        plotter.addArtifact(new PixelLineArtifact(String.valueOf(id++), new Point2D(lastPoint.get().getX(), lastPoint.get().getY()),
                                                  new Point2D(point2D.getX(), point2D.getY()), qualitativeColors.getColor(colorIndex), 2));
     }
     else
     {
        plotter.addArtifact(new PixelPointArtifact(String.valueOf(id++), new Point2D(point2D.getX(), point2D.getY()), qualitativeColors.getColor(colorIndex),
                                                   2));
     }
 
     lastPoint = Optional.of(point2D);
  }
 
  private void adjustViewRange(Point2D point2D)
  {
     if (!adjustingViewRangeMax.isPresent())
     {
        adjustingViewRangeMax = Optional.of(new Point2D(point2D));
        adjustingViewRangeMin = Optional.of(new Point2D(point2D));
        plotter.setFocusPointX(point2D.getX());
        plotter.setFocusPointY(point2D.getY());
        return;
     }
 
     if (point2D.getX() > adjustingViewRangeMax.get().getX())
     {
        adjustingViewRangeMax.get().setX(point2D.getX());
     }
     if (point2D.getY() > adjustingViewRangeMax.get().getY())
     {
        adjustingViewRangeMax.get().setY(point2D.getY());
     }
     if (point2D.getX() < adjustingViewRangeMin.get().getX())
     {
        adjustingViewRangeMin.get().setX(point2D.getX());
     }
     if (point2D.getY() < adjustingViewRangeMin.get().getY())
     {
        adjustingViewRangeMin.get().setY(point2D.getY());
     }
 
     plotter.setFocusPointX(adjustingViewRangeMin.get().getX() + ((adjustingViewRangeMax.get().getX() - adjustingViewRangeMin.get().getX()) / 2.0));
     plotter.setFocusPointY(adjustingViewRangeMin.get().getY() + ((adjustingViewRangeMax.get().getY() - adjustingViewRangeMin.get().getY()) / 2.0));
 
     plotter.setViewRangeX((adjustingViewRangeMax.get().getX() - adjustingViewRangeMin.get().getX()) * 1.10);
     plotter.setViewRangeY((adjustingViewRangeMax.get().getY() - adjustingViewRangeMin.get().getY()) * 1.10);
  }
 
  public void incrementSeries()
  {
     lastPoint = Optional.empty();
     ++colorIndex;
  }
 
  public static void main(String[] args)
  {
     NetworkedPlotter networkedPlotter = new NetworkedPlotter();
 
     networkedPlotter.add2dPoint(new Point2D(0, 28224263));
     networkedPlotter.add2dPoint(new Point2D(1, 17704197));
     networkedPlotter.add2dPoint(new Point2D(2, 15344388));
     networkedPlotter.add2dPoint(new Point2D(3, 16182020));
     networkedPlotter.add2dPoint(new Point2D(4, 16072452));
     networkedPlotter.add2dPoint(new Point2D(5, 5502209));
     networkedPlotter.add2dPoint(new Point2D(6, 576001));
     networkedPlotter.add2dPoint(new Point2D(7, 630016));
     networkedPlotter.add2dPoint(new Point2D(8, 635904));
     networkedPlotter.add2dPoint(new Point2D(9, 598272));
     networkedPlotter.add2dPoint(new Point2D(10, 650240));
     networkedPlotter.add2dPoint(new Point2D(11, 444416));
     networkedPlotter.add2dPoint(new Point2D(12, 426496));
     networkedPlotter.add2dPoint(new Point2D(13, 367617));
     networkedPlotter.add2dPoint(new Point2D(14, 397568));
     networkedPlotter.add2dPoint(new Point2D(15, 365056));
     networkedPlotter.add2dPoint(new Point2D(16, 443392));
     networkedPlotter.add2dPoint(new Point2D(17, 365056));
     networkedPlotter.add2dPoint(new Point2D(18, 372224));
     networkedPlotter.add2dPoint(new Point2D(19, 390144));
 
     networkedPlotter.incrementSeries();
 
     networkedPlotter.add2dPoint(new Point2D(0, 39331338));
     networkedPlotter.add2dPoint(new Point2D(1, 1155328));
     networkedPlotter.add2dPoint(new Point2D(2, 1130753));
     networkedPlotter.add2dPoint(new Point2D(3, 1124608));
     networkedPlotter.add2dPoint(new Point2D(4, 1140992));
     networkedPlotter.add2dPoint(new Point2D(5, 1118208));
     networkedPlotter.add2dPoint(new Point2D(6, 1004545));
     networkedPlotter.add2dPoint(new Point2D(7, 917504));
     networkedPlotter.add2dPoint(new Point2D(8, 916224));
     networkedPlotter.add2dPoint(new Point2D(9, 1424128));
     networkedPlotter.add2dPoint(new Point2D(10, 1789952));
     networkedPlotter.add2dPoint(new Point2D(11, 1289984));
     networkedPlotter.add2dPoint(new Point2D(12, 909569));
     networkedPlotter.add2dPoint(new Point2D(13, 910336));
     networkedPlotter.add2dPoint(new Point2D(14, 914688));
     networkedPlotter.add2dPoint(new Point2D(15, 911360));
     networkedPlotter.add2dPoint(new Point2D(16, 909569));
     networkedPlotter.add2dPoint(new Point2D(17, 909056));
     networkedPlotter.add2dPoint(new Point2D(18, 910336));
     networkedPlotter.add2dPoint(new Point2D(19, 936192));
  }
}