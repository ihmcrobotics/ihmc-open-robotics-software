package us.ihmc.robotbuilder.model;

import javaslang.collection.List;
import javaslang.concurrent.Future;
import javaslang.control.Option;
import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.robotics.robotDescription.RobotDescription;

import java.io.File;
import java.io.FileInputStream;
import java.util.Collections;
import java.util.function.Function;

/**
 *
 */
public class Loader {
    public static Future<Option<RobotDescription>> loadFile(File file, Function<List<String>, Future<Option<String>>> modelSelector) {
        JaxbSDFLoader[] rawLoader = new JaxbSDFLoader[1];
        RobotDescriptionFromSDFLoader sdfLoader = new RobotDescriptionFromSDFLoader();
        return Future.of(() -> {
                    JaxbSDFLoader jaxbSDFLoader = new JaxbSDFLoader(new FileInputStream(file), Collections.emptyList(), null);
                    rawLoader[0] = jaxbSDFLoader;
                    return List.ofAll(jaxbSDFLoader.getGeneralizedSDFRobotModels())
                            .map(GeneralizedSDFRobotModel::getName);
                })
                .flatMap(modelSelector)
                .map(selectedModel -> selectedModel
                        .flatMap((selected) -> Option.of(rawLoader[0].getGeneralizedSDFRobotModel(selected)))
                        .map(model -> sdfLoader.loadRobotDescriptionFromSDF(model, null, true)));
    }
}
