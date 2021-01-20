package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class GPUModuleAPI {
    private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
    private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("GPUBasedREARoot");
    private static final MessagerAPIFactory.CategoryTheme GPUBasedREA = apiFactory.createCategoryTheme("GPUBasedREA");

    // REA data
    public static final MessagerAPIFactory.Topic<PlanarRegionsList> PlanarRegionData = topic("PlanarRegionData");
    public static final MessagerAPIFactory.Topic<Boolean> AcceptNewPlanarRegions = topic("AcceptNewPlanarRegions");
    public static final MessagerAPIFactory.Topic<Boolean> ShowPlanarRegions = topic("ShowPlanarRegions");
    public static final MessagerAPIFactory.Topic<Boolean> ShowCoordinateSystem = topic("ShowCoordinateSystem");

    public static final MessagerAPIFactory.Topic<Boolean> RequestEntireModuleState = topic("RequestEntireModuleState");

    public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

    private static <T> MessagerAPIFactory.Topic<T> topic(String name) {
        return Root.child(GPUBasedREA).topic(apiFactory.createTypedTopicTheme(name));
    }
}
