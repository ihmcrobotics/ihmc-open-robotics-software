package us.ihmc.robotEnvironmentAwareness.updaters;

import geometry_msgs.Point;
import map_sense.RawGPUPlanarRegion;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class GPUPlanarRegionUpdater {

    private ConcaveHullFactoryParameters concaveHullFactoryParameters;
    private PolygonizerParameters polygonizerParameters;

    public GPUPlanarRegionUpdater() {
        concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
        polygonizerParameters = new PolygonizerParameters();
    }

    public void logSegmentationRawRegions(List<PlanarRegionSegmentationRawData> rawData) {
        for (int i = 0; i < rawData.size(); i++) {
            PlanarRegionSegmentationRawData rawRegion = rawData.get(i);
            LogTools.info("Region: {}, Size:{}", rawRegion.getRegionId(), rawRegion.getPointCloudInWorld().size());
            LogTools.info("Normal: ({},{},{})", rawRegion.getNormal().getX(), rawRegion.getNormal().getY(), rawRegion.getNormal().getZ());
            for (int j = 0; j < rawRegion.getPointCloudInWorld().size(); j++) {
                Point3D point = rawRegion.getPointCloudInWorld().get(j);
                System.out.println(point.getX() + "," + point.getY());
            }
        }
    }

    public void logRawGPURegions(RawGPUPlanarRegionList rawGPUPlanarRegionList) {
        for (int i = 0; i < rawGPUPlanarRegionList.getNumOfRegions(); i++) {
            RawGPUPlanarRegion rawRegion = rawGPUPlanarRegionList.getRegions().get(i);
            LogTools.info("Region: {}, Size:{}", rawRegion.getId(), rawRegion.getVertices().size());
            LogTools.info("Normal: ({},{},{})", rawRegion.getNormal().getX(), rawRegion.getNormal().getY(), rawRegion.getNormal().getZ());
            for (int j = 0; j < rawRegion.getVertices().size(); j++) {
                Point point = rawRegion.getVertices().get(j);
                System.out.println(point.getX() + "," + point.getY());
            }
        }
    }

    public PlanarRegionsList generatePlanarRegions(RawGPUPlanarRegionList rawGPUPlanarRegionList) {
        List<PlanarRegionSegmentationRawData> rawData = new ArrayList<>();
        getSegmentationRawData(rawGPUPlanarRegionList, rawData);
        return updatePolygons(rawData);
    }

    private Point3D toPoint3D(Point point) {
        return new Point3D(point.getX(), point.getY(), point.getZ());
    }

    public void getSegmentationRawData(RawGPUPlanarRegionList rawGPUPlanarRegionList, List<PlanarRegionSegmentationRawData> rawDataToPack) {

        for (int i = 0; i < rawGPUPlanarRegionList.getNumOfRegions(); i++) {
            RawGPUPlanarRegion region = rawGPUPlanarRegionList.getRegions().get(i);
            List<Point3D> pointCloud = region.getVertices().stream().map(this::toPoint3D).collect(Collectors.toList());
            rawDataToPack.add(new PlanarRegionSegmentationRawData(region.getId(),
                    new Vector3D(region.getNormal().getX(), region.getNormal().getY(), region.getNormal().getZ()),
                    new Point3D(region.getNormal().getX(), region.getNormal().getY(), region.getNormal().getZ()), pointCloud));
        }
    }

    private PlanarRegionsList updatePolygons(List<PlanarRegionSegmentationRawData> rawData) {
        return PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
    }

}
